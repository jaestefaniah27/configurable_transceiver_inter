/* transceiver.c */
#include "transceiver.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* === Mapa de Memoria Global === */
#define TRANSCEIVER_BASE_START  0xA0000000
#define TRANSCEIVER_STRIDE      0x100000     /* 1MB por bloque */
#define INTC_GLOBAL_ADDR        0xA0E00000  /* Ubicado tras el último TRX (si son 14) */
#define IRQ_VECTOR_ID           121         /* IRQ física PL->PS */

/* Offsets internos del bloque RTL */
#define OFF_SETUP  0x00000
#define OFF_RX     0x10000
#define OFF_TX     0x20000
#define OFF_STATUS 0x30000

/* Registros AXI INTC */
#define INTC_ISR   0x00
#define INTC_IER   0x08
#define INTC_IAR   0x0C
#define INTC_SIE   0x10
#define INTC_CIE   0x14
#define INTC_MER   0x1C

/* Máscaras de Bits del Hardware RTL */
#define HW_RX_EMPTY_MASK  0x0200
#define HW_RX_DATA_MASK   0x01FF
#define HW_TX_RDY_MASK    0x2000

/* Máscaras (Asegúrate de tener esto arriba en transceiver.c) */
#define SERIAL_BAUD_MASK      0x003FFFFFu
#define SERIAL_STOP_MASK      0x01C00000u
#define SERIAL_PARITY_MASK    0x0E000000u
#define SERIAL_DATA_BITS_MASK 0x70000000u
#define SERIAL_BIT_ORDER_MASK 0x80000000u

/* Array global de punteros a transceptores para que la ISR los encuentre */
#define MAX_TRANSCEIVERS 16
static Transceiver *g_instances[MAX_TRANSCEIVERS] = { NULL };

/* --- Helpers de Memoria --- */
static inline uint32_t reg_read(uintptr_t addr) {
    return *(volatile uint32_t *)addr;
}
static inline void reg_write(uintptr_t addr, uint32_t val) {
    *(volatile uint32_t *)addr = val;
}

/* --- Control de Interrupciones (Global INTC) --- */
static void intc_enable_line(Transceiver *dev, uint32_t mask) {
    reg_write(dev->intc_base + INTC_SIE, mask);
}
static void intc_disable_line(Transceiver *dev, uint32_t mask) {
    reg_write(dev->intc_base + INTC_CIE, mask);
}

/* --- Worker Task (Bottom Half) --- */
static rtems_task Rx_Worker_Task(rtems_task_argument arg) {
    Transceiver *dev = (Transceiver *)arg;
    rtems_event_set events;

    /* Habilitar interrupción RX al arrancar */
    intc_enable_line(dev, dev->mask_rx);

    for (;;) {
        /* Dormir hasta aviso de la ISR */
        rtems_event_receive(RTEMS_EVENT_0, RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &events);

        /* Vaciado de FIFO Hardware */
        /* Leemos mientras el bit EMPTY (bit 9 de Status) sea 0 */
        while ((reg_read(dev->addr_status) & HW_RX_EMPTY_MASK) == 0) {
            
            uint32_t raw = reg_read(dev->addr_status);
            uint8_t byte = (uint8_t)(raw & HW_RX_DATA_MASK);

            /* Guardar en Ring Buffer protegido */
            rtems_semaphore_obtain(dev->mutex_id, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
            if (dev->rx_count < dev->rx_buf_size) {
                dev->rx_buffer[dev->rx_tail] = byte;
                dev->rx_tail = (dev->rx_tail + 1) % dev->rx_buf_size;
                dev->rx_count++;
            }
            rtems_semaphore_release(dev->mutex_id);

            /* ACK de lectura al HW (Pulso en registro RX) */
            /* Escribir 1 y luego 0 en el bit 0 del registro RX */
            uint32_t current_rx_reg = reg_read(dev->addr_rx);
            reg_write(dev->addr_rx, current_rx_reg | 0x1);
            /* Pequeña espera si es necesaria */
            for(volatile int k=0; k<10; k++); 
            reg_write(dev->addr_rx, current_rx_reg & ~0x1);
        }

        /* Notificar usuario */
        if (dev->rx_callback) {
            dev->rx_callback(dev->rx_callback_arg);
        }

        /* Reactivar Interrupción RX en el INTC Global */
        intc_enable_line(dev, dev->mask_rx);
    }
}

/* --- ISR Maestra (Top Half) --- */
/* Esta ISR atiende a TODOS los transceptores */
static rtems_isr Master_ISR(void *arg) {
    (void)arg;
    /* Asumimos INTC Global en dirección fija o pasada por arg. Usamos la define por simplicidad aquí */
    uintptr_t intc = INTC_GLOBAL_ADDR; 
    
    /* Leer estado de interrupciones pendientes */
    uint32_t pending = reg_read(intc + INTC_ISR);

    /* Recorrer posibles transceptores activos */
    for (int i = 0; i < MAX_TRANSCEIVERS; i++) {
        Transceiver *dev = g_instances[i];
        if (dev == NULL) continue;

        /* Verificar RX (Bit par: 2*i) */
        if (pending & dev->mask_rx) {
            /* 1. ACK */
            reg_write(intc + INTC_IAR, dev->mask_rx);
            /* 2. DISABLE (Silenciar línea para evitar bucle, el worker la reactiva) */
            reg_write(intc + INTC_CIE, dev->mask_rx);
            /* 3. WAKE WORKER */
            rtems_event_send(dev->worker_id, RTEMS_EVENT_0);
        }

        /* Verificar TX (Bit impar: 2*i + 1) */
        /* (Implementar lógica TX similar si se desea) */
        if (pending & dev->mask_tx) {
            reg_write(intc + INTC_IAR, dev->mask_tx);
            /* Notificar fin de transmisión si usas TX por interrupción */
        }
    }
}

/* --- Inicialización Global del INTC --- */
void Transceiver_Global_INTC_Init(void) {
    /* Reset y Enable Master del INTC Global */
    reg_write(INTC_GLOBAL_ADDR + INTC_MER, 0x3); // Hardware Enable | Master Enable
    reg_write(INTC_GLOBAL_ADDR + INTC_IER, 0x0); // Deshabilitar todo inicialmente
    reg_write(INTC_GLOBAL_ADDR + INTC_IAR, 0xFFFFFFFF); // Limpiar pendientes

    /* Instalar la ISR Maestra en el vector del Zynq */
    rtems_interrupt_handler_install(IRQ_VECTOR_ID, "UART_Master", 
                                    RTEMS_INTERRUPT_UNIQUE, 
                                    Master_ISR, 
                                    NULL);
}

/* --- Inicialización de Instancia --- */
rtems_status_code Transceiver_Init(Transceiver *dev, uint32_t id, const Transceiver_Config_t *cfg) {
    rtems_status_code sc;

    if (id >= MAX_TRANSCEIVERS) return RTEMS_INVALID_ID;

    /* 1. Calcular Direcciones según el ID */
    /* Base: 0xA0000000 + (ID * 0x10000) */
    dev->id = id;
    dev->base_addr = TRANSCEIVER_BASE_START + (id * TRANSCEIVER_STRIDE);
    dev->intc_base = INTC_GLOBAL_ADDR;

    dev->addr_setup  = dev->base_addr + OFF_SETUP;
    dev->addr_rx     = dev->base_addr + OFF_RX;
    dev->addr_tx     = dev->base_addr + OFF_TX;
    dev->addr_status = dev->base_addr + OFF_STATUS;

    /* 2. Calcular Máscaras de IRQ */
    /* RX es bit 2*id (0, 2, 4...) */
    /* TX es bit 2*id + 1 (1, 3, 5...) */
    dev->mask_rx = (1 << (2 * id));
    dev->mask_tx = (1 << (2 * id + 1));

    /* 3. Inicializar Buffer Software */
    dev->rx_buf_size = 4096;
    dev->rx_buffer = malloc(dev->rx_buf_size); // O usar buffer estático si prefieres
    dev->rx_head = 0; 
    dev->rx_tail = 0; 
    dev->rx_count = 0;

    /* 4. Crear Recursos RTEMS */
    sc = rtems_semaphore_create(rtems_build_name('T','R','X', '0'+id), 1, 
                                RTEMS_PRIORITY | RTEMS_BINARY_SEMAPHORE, 0, &dev->mutex_id);
    if (sc != RTEMS_SUCCESSFUL) return sc;

    sc = rtems_task_create(rtems_build_name('W','K','R', '0'+id), 50, 
                           RTEMS_MINIMUM_STACK_SIZE * 2,
                           RTEMS_DEFAULT_MODES, RTEMS_DEFAULT_ATTRIBUTES, &dev->worker_id);
    if (sc != RTEMS_SUCCESSFUL) return sc;

    /* 5. Registrar instancia globalmente */
    g_instances[id] = dev;

    /* 6. Configurar Hardware (Baudios, etc.) usando dev->addr_setup */
/* 6. Configurar Hardware (Baudios, etc.) usando dev->addr_setup */
    if (cfg) {
        /* Leemos el estado actual para no machacar bits reservados si los hubiera */
        uint32_t val = reg_read(dev->addr_setup);

        /* --- 1. Baud Rate (Bits 0-21) --- */
        if (cfg->baud) {
            val &= ~SERIAL_BAUD_MASK;           /* Limpiar bits viejos */
            val |= (cfg->baud & SERIAL_BAUD_MASK); /* Poner nuevos */
        }

        /* --- 2. Stop Bits (Bits 22-24) --- */
        /* Se asume que cfg->stop_bits contiene el código de hardware (ej: 2) */
        val &= ~SERIAL_STOP_MASK;
        val |= ((cfg->stop_bits & 0x7) << 22);

        /* --- 3. Parity (Bits 25-27) --- */
        val &= ~SERIAL_PARITY_MASK;
        val |= ((cfg->parity & 0x7) << 25);

        /* --- 4. Data Bits (Bits 28-30) --- */
        val &= ~SERIAL_DATA_BITS_MASK;
        val |= ((cfg->data_bits & 0x7) << 28);

        /* --- 5. Bit Order (Bit 31) --- */
        /* 0 = LSB First (Normal), 1 = MSB First */
        if (cfg->bit_order) {
            val |= SERIAL_BIT_ORDER_MASK;
        } else {
            val &= ~SERIAL_BIT_ORDER_MASK;
        }

        /* Escribir la configuración en el hardware */
        reg_write(dev->addr_setup, val);

        /* Espera técnica para que el divisor de frecuencia interno se estabilice */
        for(volatile int k=0; k<1000; k++);
    }

    /* 7. Arrancar Worker (pasándole 'dev' como argumento) */
    sc = rtems_task_start(dev->worker_id, Rx_Worker_Task, (rtems_task_argument)dev);

    return sc;
}

size_t Transceiver_Read(Transceiver *dev, uint8_t *buf, size_t maxlen) {
    size_t got = 0;
    rtems_semaphore_obtain(dev->mutex_id, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
    while (got < maxlen && dev->rx_count > 0) {
        buf[got++] = dev->rx_buffer[dev->rx_head];
        dev->rx_head = (dev->rx_head + 1) % dev->rx_buf_size;
        dev->rx_count--;
    }
    rtems_semaphore_release(dev->mutex_id);
    return got;
}

int Transceiver_SendString(Transceiver *dev, const char *s) {
    /* Implementación básica de envío usando dev->addr_tx */
    while (*s) {
        /* Escribir dato */
        uint32_t current_val = reg_read(dev->addr_tx);
        // Limpiar bits de datos previos y poner nuevo
        current_val &= ~0x1FF; 
        current_val |= (*s & 0xFF);
        reg_write(dev->addr_tx, current_val);
        
        /* Pulso de envío (Bit 9) */
        reg_write(dev->addr_tx, current_val | 0x200); 
        for(volatile int k=0; k<50; k++); 
        reg_write(dev->addr_tx, current_val & ~0x200);
        
        /* Esperar TX RDY (Bit 13 en Status) */
        while( (reg_read(dev->addr_status) & 0x2000) == 0 );
        
        s++;
    }
    return 0;
}

void Transceiver_SetRxCallback(Transceiver *dev, void (*cb)(void *), void *arg) {
    dev->rx_callback = cb;
    dev->rx_callback_arg = arg;
}