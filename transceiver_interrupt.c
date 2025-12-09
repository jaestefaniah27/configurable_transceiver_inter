/* transceiver_interrupt.c */
#include <rtems.h>
#include <stdio.h>
#include <stdbool.h>
#include "transceiver_interrupt.h"

/* Registros AXI INTC */
#define INTC_BASE_ADDR       0xa0040000
#define INTC_ISR_OFFSET      0x00
#define INTC_IER_OFFSET      0x08
#define INTC_IAR_OFFSET      0x0C
#define INTC_SIE_OFFSET      0x10  /* Set Interrupt Enable */
#define INTC_CIE_OFFSET      0x14  /* Clear Interrupt Enable */
#define INTC_MER_OFFSET      0x1C

#define IRQ_ID_PL_PS_IRQ0 121
#define INTR_RX_BIT 1
#define INTR_TX_BIT 0

static void (*tx_handler)(void) = NULL;
static rtems_id rx_worker_id = 0;

static inline void write_reg(uint32_t offset, uint32_t value) {
    *(volatile uint32_t *)(INTC_BASE_ADDR + offset) = value;
}

static inline uint32_t read_reg(uint32_t offset) {
    return *(volatile uint32_t *)(INTC_BASE_ADDR + offset);
}

/* Control de interrupción RX para el Worker */
void transceiver_int_enable_rx(bool enable) {
    if (enable) {
        write_reg(INTC_SIE_OFFSET, (1 << INTR_RX_BIT)); /* Habilitar */
    } else {
        write_reg(INTC_CIE_OFFSET, (1 << INTR_RX_BIT)); /* Deshabilitar */
    }
}

static rtems_isr interrupt_service_routine(rtems_vector_number vector) {
    (void)vector;
    uint32_t pending = read_reg(INTC_ISR_OFFSET);

    /* --- RX (Recepción) --- */
    if (pending & (1 << INTR_RX_BIT)) {
        /* 1. Confirmar al controlador (ACK) */
        write_reg(INTC_IAR_OFFSET, (1 << INTR_RX_BIT));

        /* 2. SILENCIAR: Apagamos RX para que deje de interrumpir mientras procesamos */
        write_reg(INTC_CIE_OFFSET, (1 << INTR_RX_BIT));

        /* 3. Despertar al Worker (Bottom Half) */
        if (rx_worker_id != 0) {
            rtems_event_send(rx_worker_id, RTEMS_EVENT_0);
        }
    }

    /* --- TX (Transmisión) --- */
    if (pending & (1 << INTR_TX_BIT)) {
        write_reg(INTC_IAR_OFFSET, (1 << INTR_TX_BIT));
        if (tx_handler) tx_handler();
    }
}

void transceiver_interrupt_init(void) {
    /* Master Enable */
    write_reg(INTC_MER_OFFSET, 0x3);
    
    /* Habilitar interrupciones inicialmente */
    write_reg(INTC_IER_OFFSET, (1 << INTR_RX_BIT) | (1 << INTR_TX_BIT));

    rtems_interrupt_handler_install(IRQ_ID_PL_PS_IRQ0, "TRANS_IRQ", 
                                    RTEMS_INTERRUPT_UNIQUE, 
                                    interrupt_service_routine, NULL);
}

void transceiver_register_tx_callback(void (*cb)(void)) { tx_handler = cb; }
void transceiver_register_rx_worker_id(rtems_id id) { rx_worker_id = id; }