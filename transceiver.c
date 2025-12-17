/* transceiver.c */
#include "transceiver.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* === Mapa de Memoria Global (Compacto) === */
#define TRANSCEIVER_BASE_START  0xA0000000
#define TRANSCEIVER_STRIDE      0x1000      /* 4KB por bloque */

/* Dirección Fija del Bloque de Información del Sistema (Privado del Driver) */
#define SYS_INFO_ADDR           0xA0020000

#define IRQ_VECTOR_ID           121

/* === Offsets del AXI GPIO (Dual Channel) === */
#define REG_CH1_DATA  0x00  /* Output: Config + Control + TX Data */
#define REG_CH2_DATA  0x08  /* Input:  Status + RX Data */

/* Registros AXI INTC */
#define INTC_ISR   0x00
#define INTC_IER   0x08
#define INTC_IAR   0x0C
#define INTC_SIE   0x10
#define INTC_CIE   0x14
#define INTC_MER   0x1C

/* === Mapeo de Bits CANAL 1 (Output -> FPGA) === */
/* [Order(1) | DataBits(3) | Parity(3) | Stop(2) | Baud(6) | Read(1) | ErrOK(1) | Send(1) | DataIn(9)] */
#define MASK_DATA_IN        0x000001FF
#define BIT_TX_SEND         (1 << 9)
#define BIT_ERROR_OK        (1 << 10)
#define BIT_DATA_READ       (1 << 11)
#define SHIFT_BAUD          12
#define SHIFT_STOP          18
#define SHIFT_PARITY        20
#define SHIFT_DATA_BITS     23
#define BIT_ORDER           (1 << 26)

/* === Mapeo de Bits CANAL 2 (Input <- FPGA) === */
/* [TxRdy(1) | FrameErr(1) | ParErr(1) | Full(1) | Empty(1) | DataOut(9)] */
#define MASK_DATA_OUT       0x000001FF
#define BIT_RX_EMPTY        (1 << 9)
#define BIT_RX_FULL         (1 << 10)
#define BIT_PAR_ERROR       (1 << 11)
#define BIT_FRAME_ERROR     (1 << 12)
#define BIT_TX_RDY          (1 << 13)

#define MAX_TRANSCEIVERS 32
static Transceiver *g_instances[MAX_TRANSCEIVERS] = { NULL };

/* Cache para la dirección del INTC calculada dinámicamente */
static uintptr_t g_intc_addr = 0;

/* --- Helpers --- */
static inline uint32_t reg_read(uintptr_t addr) {
    return *(volatile uint32_t *)addr;
}
static inline void reg_write(uintptr_t addr, uint32_t val) {
    *(volatile uint32_t *)addr = val;
}

/* === NUEVA FUNCIÓN: Obtener número de transceptores desde HW === */
uint32_t Transceiver_GetHardwareCount(void) {
    /* Leemos la constante cableada en la FPGA */
    return reg_read(SYS_INFO_ADDR);
}

/* === Helper Interno: Calcular dirección del INTC === */
static uintptr_t get_intc_addr(void) {
    /* Si ya la calculamos, retornarla */
    if (g_intc_addr != 0) return g_intc_addr;

    /* Si no, leer count y calcular: Base + (N * Stride) */
    uint32_t n = Transceiver_GetHardwareCount();
    g_intc_addr = TRANSCEIVER_BASE_START + (n * TRANSCEIVER_STRIDE);
    printf("Detected %u transceivers. INTC at 0x%08lx\n", n, (unsigned long)g_intc_addr);
    return g_intc_addr;
}

/* --- Control Global INTC --- */
static void intc_enable_line(Transceiver *dev, uint32_t mask) {
    reg_write(dev->intc_base + INTC_SIE, mask);
}

/* --- Worker Task (RX) --- */
static rtems_task Rx_Worker_Task(rtems_task_argument arg) {
    Transceiver *dev = (Transceiver *)arg;
    rtems_event_set events;
    uintptr_t reg_out = dev->base_addr + REG_CH1_DATA;
    uintptr_t reg_in  = dev->base_addr + REG_CH2_DATA;

    intc_enable_line(dev, dev->mask_rx);

    for (;;) {
        rtems_event_receive(RTEMS_EVENT_0, RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &events);

        while ((reg_read(reg_in) & BIT_RX_EMPTY) == 0) {
            uint32_t status_val = reg_read(reg_in);
            uint8_t byte = (uint8_t)(status_val & MASK_DATA_OUT);

            rtems_semaphore_obtain(dev->mutex_id, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
            if (dev->rx_count < dev->rx_buf_size) {
                dev->rx_buffer[dev->rx_tail] = byte;
                dev->rx_tail = (dev->rx_tail + 1) % dev->rx_buf_size;
                dev->rx_count++;
            }
            rtems_semaphore_release(dev->mutex_id);

            uint32_t current_ctrl = reg_read(reg_out);
            reg_write(reg_out, current_ctrl | BIT_DATA_READ);
            for(volatile int k=0; k<10; k++); 
            reg_write(reg_out, current_ctrl & ~BIT_DATA_READ);
        }

        if (dev->rx_callback) dev->rx_callback(dev->rx_callback_arg);
        intc_enable_line(dev, dev->mask_rx);
    }
}

/* --- ISR Maestra --- */
static rtems_isr Master_ISR(void *arg) {
    (void)arg;
    /* Usamos la dirección calculada dinámicamente */
    uintptr_t intc = get_intc_addr(); 
    uint32_t pending = reg_read(intc + INTC_ISR);

    /* Recorremos hasta el máximo posible o guardamos 'count' en una variable global */
    for (int i = 0; i < MAX_TRANSCEIVERS; i++) {
        Transceiver *dev = g_instances[i];
        if (!dev) continue; /* Saltamos los no inicializados */

        if (pending & dev->mask_rx) {
            reg_write(intc + INTC_IAR, dev->mask_rx);
            reg_write(intc + INTC_CIE, dev->mask_rx);
            rtems_event_send(dev->worker_id, RTEMS_EVENT_0);
        }
    }
}

/* --- Inicialización Global (Auto-Detect) --- */
void Transceiver_Global_INTC_Init(void) {
    /* 1. Detectar dirección del INTC automáticamente */
    uintptr_t intc = get_intc_addr();
    
    /* 2. Inicializar Hardware */
    reg_write(intc + INTC_MER, 0x3); 
    reg_write(intc + INTC_IER, 0x0);
    reg_write(intc + INTC_IAR, 0xFFFFFFFF);

    /* 3. Instalar ISR */
    rtems_interrupt_handler_install(IRQ_VECTOR_ID, "UART_Master", 
                                    RTEMS_INTERRUPT_UNIQUE, Master_ISR, NULL);
}

/* --- Init Transceiver --- */
rtems_status_code Transceiver_Init(Transceiver *dev, uint32_t id, const Transceiver_Config_t *cfg) {
    rtems_status_code sc;
    if (id >= MAX_TRANSCEIVERS) return RTEMS_INVALID_ID;

    dev->id = id;
    dev->base_addr = TRANSCEIVER_BASE_START + (id * TRANSCEIVER_STRIDE);
    /* Asignar dirección del INTC calculada */
    dev->intc_base = get_intc_addr();
    
    
    dev->mask_rx = (1 << (2 * id));
    dev->mask_tx = (1 << (2 * id + 1));

    dev->rx_buf_size = 4096;
    dev->rx_buffer = malloc(dev->rx_buf_size);
    dev->rx_head = dev->rx_tail = dev->rx_count = 0;

    sc = rtems_semaphore_create(rtems_build_name('T','R','X', '0'+id), 1, 
                                RTEMS_PRIORITY | RTEMS_BINARY_SEMAPHORE, 0, &dev->mutex_id);
    if (sc != RTEMS_SUCCESSFUL) return sc;

    sc = rtems_task_create(rtems_build_name('W','K','R', '0'+id), 50, 
                           RTEMS_MINIMUM_STACK_SIZE * 2,
                           RTEMS_DEFAULT_MODES, RTEMS_DEFAULT_ATTRIBUTES, &dev->worker_id);
    if (sc != RTEMS_SUCCESSFUL) return sc;

    g_instances[id] = dev;

    if (cfg) {
        /* Leer estado actual para no borrar bits de control en vuelo */
        uint32_t val = reg_read(dev->base_addr + REG_CH1_DATA);
        
        /* Borrar Configuración Previa (Bits 12-31) */
        val &= 0x00000FFF; 
        
        val |= ((cfg->baud & 0x3F) << SHIFT_BAUD);
        val |= ((cfg->stop_bits & 0x3) << SHIFT_STOP);
        val |= ((cfg->parity & 0x7) << SHIFT_PARITY);
        val |= ((cfg->data_bits & 0x7) << SHIFT_DATA_BITS);
        if (cfg->bit_order) val |= BIT_ORDER;

        reg_write(dev->base_addr + REG_CH1_DATA, val);
    }

    sc = rtems_task_start(dev->worker_id, Rx_Worker_Task, (rtems_task_argument)dev);
    return sc;
}

/* ... Las funciones Read/SendString siguen igual ... */
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
    uintptr_t reg_out = dev->base_addr + REG_CH1_DATA;
    uintptr_t reg_in  = dev->base_addr + REG_CH2_DATA;

    while (*s) {
        while ((reg_read(reg_in) & BIT_TX_RDY) == 0);

        uint32_t val = reg_read(reg_out);
        val &= ~(MASK_DATA_IN | BIT_TX_SEND | BIT_DATA_READ);
        val |= (*s & MASK_DATA_IN);
        
        reg_write(reg_out, val);
        reg_write(reg_out, val | BIT_TX_SEND);
        for(volatile int k=0; k<20; k++);
        reg_write(reg_out, val & ~BIT_TX_SEND);
        s++;
    }
    return 0;
}

void Transceiver_SetRxCallback(Transceiver *dev, void (*cb)(void *), void *arg) {
    dev->rx_callback = cb;
    dev->rx_callback_arg = arg;
}