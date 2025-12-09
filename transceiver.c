/*
 * transceiver.c
 */

#include "transceiver.h"
#include <rtems.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* =========================================================================
 * 1. DEFINICIONES DE REGISTROS (Privado - Hardware Abstraction Layer)
 * ========================================================================= */

/* Direcciones Base */
#define GPIO0_BASE       0xA0000000u  /* Configuración Serial */
#define GPIO1_BASE       0xA0010000u  /* Control de Flujo RX */
#define GPIO2_BASE       0xA0020000u  /* Datos TX */
#define GPIO3_BASE       0xA0030000u  /* Datos RX */
#define INTC_BASE_ADDR   0xA0040000u  /* AXI Interrupt Controller */

/* Offsets AXI INTC */
#define INTC_ISR_OFFSET  0x00u
#define INTC_IER_OFFSET  0x08u
#define INTC_IAR_OFFSET  0x0Cu
#define INTC_SIE_OFFSET  0x10u /* Set Enable */
#define INTC_CIE_OFFSET  0x14u /* Clear Enable */
#define INTC_MER_OFFSET  0x1Cu

/* Máscaras y Bits */
#define INTR_RX_BIT      1
#define INTR_TX_BIT      0
#define IRQ_ID_PL_PS     121   /* ZynqMP PL-PS IRQ */

/* Máscaras GPIO */
#define PS_OUT_TX_RDY_MASK    0x2000u
#define PS_OUT_EMPTY_MASK     0x0200u
#define PS_OUT_DATA_MASK      0x01FFu

#define GPIO1_DATA_READ_MASK  0x1u
#define GPIO2_DATA_IN_MASK    0x01FFu
#define GPIO2_TX_SEND_MASK    0x0200u

/* Máscaras Configuración Serial */
#define SERIAL_BAUD_MASK      0x003FFFFFu
#define SERIAL_STOP_MASK      0x01C00000u
#define SERIAL_PARITY_MASK    0x0E000000u
#define SERIAL_DATA_BITS_MASK 0x70000000u
#define SERIAL_BIT_ORDER_MASK 0x80000000u

/* Configuraciones de Software */
#define RX_QUEUE_SIZE    (4 * 1024)
#define TX_BUF_SIZE      256
#define WORKER_PRIORITY  50
#define WORKER_STACK     (RTEMS_MINIMUM_STACK_SIZE * 2)

/* =========================================================================
 * 2. VARIABLES GLOBALES (Privadas - Static)
 * ========================================================================= */

/* Contexto RX */
static uint8_t  rx_queue[RX_QUEUE_SIZE];
static size_t   rx_head = 0;
static size_t   rx_tail = 0;
static size_t   rx_count = 0;
static rtems_id rx_mutex_id = 0;
static rtems_id rx_worker_id = 0;

/* Contexto TX */
static uint8_t  tx_buf[TX_BUF_SIZE];
static size_t   tx_head = 0;
static size_t   tx_tail = 0;
static volatile bool tx_active = false;

/* Callbacks */
static Transceiver_Event_Cb_t user_rx_cb = NULL;
static void *user_rx_arg = NULL;

/* =========================================================================
 * 3. FUNCIONES HELPER (Low Level)
 * ========================================================================= */

static inline void mmio_write32(uintptr_t addr, uint32_t val) {
    *(volatile uint32_t *)addr = val;
    __asm__ volatile("dmb sy" ::: "memory");
}

static inline uint32_t mmio_read32(uintptr_t addr) {
    __asm__ volatile("dmb sy" ::: "memory");
    return *(volatile uint32_t *)addr;
}

static void gpio_rmw(uintptr_t base, uint32_t mask, uint32_t val) {
    uint32_t reg = mmio_read32(base);
    reg &= ~mask;
    reg |= (val & mask);
    mmio_write32(base, reg);
}

/* Pulso para confirmar lectura al HW */
static void fifo_consume_pulse(void) {
    gpio_rmw(GPIO1_BASE, GPIO1_DATA_READ_MASK, GPIO1_DATA_READ_MASK);
    /* Pequeña espera para asegurar que el HW lo pilla */
    for (volatile int i = 0; i < 50; ++i) __asm__ volatile("nop");
    gpio_rmw(GPIO1_BASE, GPIO1_DATA_READ_MASK, 0);
}

/* Control de Interrupción RX (Enable/Disable) */
static void hw_int_enable_rx(bool enable) {
    if (enable) {
        mmio_write32(INTC_BASE_ADDR + INTC_SIE_OFFSET, (1 << INTR_RX_BIT));
    } else {
        mmio_write32(INTC_BASE_ADDR + INTC_CIE_OFFSET, (1 << INTR_RX_BIT));
    }
}

/* =========================================================================
 * 4. RUTINA DE SERVICIO DE INTERRUPCIÓN (ISR - Top Half)
 * ========================================================================= */

static rtems_isr Transceiver_ISR(rtems_vector_number vector) {
    (void)vector;
    uint32_t pending = mmio_read32(INTC_BASE_ADDR + INTC_ISR_OFFSET);

    /* --- Manejo de Recepción (RX) --- */
    if (pending & (1 << INTR_RX_BIT)) {
        /* 1. Confirmar interrupción (ACK) */
        mmio_write32(INTC_BASE_ADDR + INTC_IAR_OFFSET, (1 << INTR_RX_BIT));

        /* 2. SILENCIAR: Deshabilitar interrupción RX para evitar bucle infinito.
           El Worker la volverá a habilitar cuando limpie la FIFO. */
        mmio_write32(INTC_BASE_ADDR + INTC_CIE_OFFSET, (1 << INTR_RX_BIT));

        /* 3. Despertar al Worker (Bottom Half) */
        if (rx_worker_id != 0) {
            rtems_event_send(rx_worker_id, RTEMS_EVENT_0);
        }
    }

    /* --- Manejo de Transmisión (TX) --- */
    if (pending & (1 << INTR_TX_BIT)) {
        mmio_write32(INTC_BASE_ADDR + INTC_IAR_OFFSET, (1 << INTR_TX_BIT));
        
        /* Lógica TX simple: enviar siguiente byte si hay */
        if (tx_active) {
             if (tx_head != tx_tail) {
                /* Hay más datos, enviar siguiente (Implementación básica) */
                uint8_t b = tx_buf[tx_tail];
                tx_tail = (tx_tail + 1) % TX_BUF_SIZE;
                
                gpio_rmw(GPIO2_BASE, GPIO2_DATA_IN_MASK, b);
                gpio_rmw(GPIO2_BASE, GPIO2_TX_SEND_MASK, GPIO2_TX_SEND_MASK);
                for (volatile int k=0; k<50; ++k) __asm__ volatile("nop");
                gpio_rmw(GPIO2_BASE, GPIO2_TX_SEND_MASK, 0);
             } else {
                 tx_active = false; /* Buffer vacío */
             }
        }
    }
}

/* =========================================================================
 * 5. TAREA WORKER (Bottom Half)
 * ========================================================================= */

static rtems_task Rx_Worker_Task(rtems_task_argument arg) {
    (void)arg;
    rtems_event_set events;

    /* Inicio seguro: Habilitar RX */
    hw_int_enable_rx(true);

    for (;;) {
        /* 1. Dormir hasta ser despertado por la ISR */
        rtems_event_receive(RTEMS_EVENT_0, RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &events);

        /* 2. Bucle de Vaciado de FIFO Hardware */
        /* Mientras la señal EMPTY (bit 9) sea 0 (Lógica Negativa: 0 = Hay dato) */
        while ((mmio_read32(GPIO3_BASE) & PS_OUT_EMPTY_MASK) == 0) {
            
            /* Leer dato crudo */
            uint32_t raw = mmio_read32(GPIO3_BASE);
            uint8_t  byte = (uint8_t)(raw & PS_OUT_DATA_MASK);

            /* Guardar en buffer circular (Protegido por Mutex) */
            rtems_semaphore_obtain(rx_mutex_id, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
            if (rx_count < RX_QUEUE_SIZE) {
                rx_queue[rx_tail] = byte;
                rx_tail = (rx_tail + 1) % RX_QUEUE_SIZE;
                rx_count++;
            }
            rtems_semaphore_release(rx_mutex_id);

            /* Confirmar lectura al HW */
            fifo_consume_pulse();
        }

        /* 3. Notificar al usuario (si hay callback registrado) */
        if (user_rx_cb) {
            user_rx_cb(user_rx_arg);
        }

        /* 4. Reactivar Interrupción RX */
        hw_int_enable_rx(true);
    }
}

/* =========================================================================
 * 6. API PÚBLICA
 * ========================================================================= */

rtems_status_code Transceiver_Init(const Transceiver_Config_t *cfg) {
    rtems_status_code sc;

    /* 1. Mapeo de memoria (externo o interno, asumimos hecho o llamar aquí) */
    /* mmu_map_pl_axi_early(); */ 

    /* 2. Configuración de Hardware (Baud, Parity, etc.) */
    if (cfg) {
        /* Leer estado actual del registro de configuración (GPIO0) */
        uint32_t val = mmio_read32(GPIO0_BASE);

        /* 2.1 Baud Rate (Bits 0-21) */
        if (cfg->baud != 0) {
            val &= ~SERIAL_BAUD_MASK;
            val |= (cfg->baud & SERIAL_BAUD_MASK);
        }

        /* 2.2 Stop Bits (Bits 22-24) */
        if (cfg->stop_bits != 0) {
            val &= ~SERIAL_STOP_MASK;
            val |= ((cfg->stop_bits & 0x7) << 22);
        }

        /* 2.3 Parity (Bits 25-27) */
        if (cfg->parity != 0) {
            val &= ~SERIAL_PARITY_MASK;
            val |= ((cfg->parity & 0x7) << 25);
        }

        /* 2.4 Data Bits (Bits 28-30) */
        if (cfg->data_bits != 0) {
            val &= ~SERIAL_DATA_BITS_MASK;
            val |= ((cfg->data_bits & 0x7) << 28);
        }

        /* 2.5 Bit Order (Bit 31) */
        /* Nota: Limpiamos el bit (ponemos a 0) y si cfg->bit_order es 1, lo activamos */
        val &= ~SERIAL_BIT_ORDER_MASK;
        if (cfg->bit_order) {
            val |= (1u << 31);
        }

        /* Escribir configuración de vuelta al hardware */
        mmio_write32(GPIO0_BASE, val);
        
        /* Opcional: Forzar una espera pequeña si el hardware necesita asimilar el cambio de baudios */
        for (volatile int i = 0; i < 1000; i++) __asm__("nop");
    }

    /* 3. Crear Mutex y Tarea Worker */
    if (rx_mutex_id == 0) {
        sc = rtems_semaphore_create(rtems_build_name('T','R','M','X'), 1,
                                    RTEMS_PRIORITY | RTEMS_BINARY_SEMAPHORE, 0, &rx_mutex_id);
        if (sc != RTEMS_SUCCESSFUL) return sc;
    }

    sc = rtems_task_create(rtems_build_name('T','R','W','K'), WORKER_PRIORITY, 
                           WORKER_STACK, RTEMS_DEFAULT_MODES, RTEMS_DEFAULT_ATTRIBUTES, &rx_worker_id);
    if (sc != RTEMS_SUCCESSFUL) return sc;

    sc = rtems_task_start(rx_worker_id, Rx_Worker_Task, 0);
    if (sc != RTEMS_SUCCESSFUL) return sc;

    /* 4. Inicializar Controlador de Interrupciones (AXI INTC) */
    mmio_write32(INTC_BASE_ADDR + INTC_MER_OFFSET, 0x3); /* Master Enable */
    mmio_write32(INTC_BASE_ADDR + INTC_IER_OFFSET, (1<<INTR_RX_BIT) | (1<<INTR_TX_BIT));

    /* 5. Instalar ISR en RTEMS */
    sc = rtems_interrupt_handler_install(IRQ_ID_PL_PS, "TRANS_ISR", 
                                         RTEMS_INTERRUPT_UNIQUE, Transceiver_ISR, NULL);
    
    return sc;
}

void Transceiver_SetRxCallback(Transceiver_Event_Cb_t cb, void *arg) {
    user_rx_cb = cb;
    user_rx_arg = arg;
}

size_t Transceiver_Read(uint8_t *buf, size_t maxlen) {
    size_t transferred = 0;
    
    rtems_semaphore_obtain(rx_mutex_id, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
    while (transferred < maxlen && rx_count > 0) {
        buf[transferred++] = rx_queue[rx_head];
        rx_head = (rx_head + 1) % RX_QUEUE_SIZE;
        rx_count--;
    }
    rtems_semaphore_release(rx_mutex_id);
    
    return transferred;
}

size_t Transceiver_Available(void) {
    size_t c;
    rtems_semaphore_obtain(rx_mutex_id, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
    c = rx_count;
    rtems_semaphore_release(rx_mutex_id);
    return c;
}

/* Envío simple (bloqueante o con buffer TX, aquí simplificado directo para ejemplo) */
int Transceiver_Send(const uint8_t *buf, size_t len) {
    if (!buf || len == 0) return 0;
    
    /* Nota: Esta implementación básica usa busy-wait corto. 
       Para producción real, usar buffer circular TX + interrupción TX similar a RX */
    for (size_t i = 0; i < len; ++i) {
        gpio_rmw(GPIO2_BASE, GPIO2_DATA_IN_MASK, buf[i]);
        gpio_rmw(GPIO2_BASE, GPIO2_TX_SEND_MASK, GPIO2_TX_SEND_MASK);
        for (volatile int k=0; k<100; ++k) __asm__ volatile("nop");
        gpio_rmw(GPIO2_BASE, GPIO2_TX_SEND_MASK, 0);
        
        /* Esperar TX RDY (simple polling para evitar overflow) */
        /* Omitido para brevedad, añadir timeout idealmente */
        rtems_task_wake_after(1); 
    }
    return 0;
}

int Transceiver_SendString(const char *s) {
    if (!s) return -1;
    return Transceiver_Send((const uint8_t *)s, strlen(s));
}

void Transceiver_Shutdown(void) {
    /* Deshabilitar IRQs HW */
    mmio_write32(INTC_BASE_ADDR + INTC_IER_OFFSET, 0);
    /* Borrar tarea worker */
    if (rx_worker_id) rtems_task_delete(rx_worker_id);
}