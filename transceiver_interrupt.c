// transceiver_interrupt.c
#include <rtems.h>
#include <stdint.h>
#include <stdbool.h>    /* Necesario para bool */

#include "transceiver_interrupt.h"

#define INTC_BASE_ADDR       0xa0040000
#define INTC_ISR_OFFSET      0x00
#define INTC_IPR_OFFSET      0x04
#define INTC_IER_OFFSET      0x08
#define INTC_IAR_OFFSET      0x0C
#define INTC_SIE_OFFSET      0x10
#define INTC_CIE_OFFSET      0x14
#define INTC_IVR_OFFSET      0x18
#define INTC_MER_OFFSET      0x1C

#define IRQ_ID_PL_PS_IRQ0 121  // O el que corresponda según la documentación para ZynqMP

#define INTR_RX_BIT 1
#define INTR_TX_BIT 0

static void (*rx_handler)(void) = NULL;
static void (*tx_handler)(void) = NULL;
static rtems_id rx_worker_id = 0;

static inline void write_reg(uint32_t offset, uint32_t value) {
    volatile uint32_t *addr = (uint32_t *)(INTC_BASE_ADDR + offset);
    *addr = value;
}

static inline uint32_t read_reg(uint32_t offset) {
    volatile uint32_t *addr = (uint32_t *)(INTC_BASE_ADDR + offset);
    return *addr;
}
/* Función segura para activar/desactivar RX */
void transceiver_int_enable_rx(bool enable) {
    if (enable) {
        write_reg(INTC_SIE_OFFSET, (1 << INTR_RX_BIT)); /* Escribe 1 en SIE activa */
    } else {
        write_reg(INTC_CIE_OFFSET, (1 << INTR_RX_BIT)); /* Escribe 1 en CIE desactiva */
    }
}
static rtems_isr interrupt_service_routine(rtems_vector_number vector) {
    (void)vector;
    uint32_t pending = read_reg(INTC_ISR_OFFSET);

    if (pending & (1 << INTR_RX_BIT)) {
        write_reg(INTC_IAR_OFFSET, (1 << INTR_RX_BIT));
        if (rx_handler) rx_handler();
        if (rx_worker_id != 0) {
            rtems_event_send(rx_worker_id, RTEMS_EVENT_0);
        }
        //printf("interrupt_service_routine RX\n");
    }
    if (pending & (1 << INTR_TX_BIT)) {
        write_reg(INTC_IAR_OFFSET, (1 << INTR_TX_BIT));
        if (tx_handler) tx_handler();
        //printf("TX INTR handled\n");
    }
}

void transceiver_interrupt_init(void) {

    volatile uint32_t *ier = (uint32_t *)(INTC_BASE_ADDR + INTC_IER_OFFSET);
    volatile uint32_t *mer = (uint32_t *)(INTC_BASE_ADDR + INTC_MER_OFFSET);

    *ier = 0x3;    // Habilita entradas 0 (TX_RDY) y 1 (EMPTY)
    *mer = 0x3;    // Habilita interrupciones globales y locales

    //write_reg(INTC_IER_OFFSET, (1 << INTR_RX_BIT) | (1 << INTR_TX_BIT));
    //write_reg(INTC_MER_OFFSET, 0x3); // ME = 1, HIE = 1
    //rtems_interrupt_catch(interrupt_service_routine, 0x10, NULL); // Vector 0x10 como en IVAR_RST_VAL
    rtems_interrupt_handler_install(IRQ_ID_PL_PS_IRQ0, "TX_RDY_IRQ", RTEMS_INTERRUPT_UNIQUE, 
        interrupt_service_routine, NULL );
}

void transceiver_register_rx_callback(void (*cb)(void)) {
    rx_handler = cb;
}
void transceiver_register_rx_worker_id(rtems_id task_id) { rx_worker_id = task_id; }

void transceiver_register_tx_callback(void (*cb)(void)) {
    tx_handler = cb;
}
