// transceiver_interrupt.h
#ifndef TRANSCEIVER_INTERRUPT_H
#define TRANSCEIVER_INTERRUPT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el controlador de interrupciones AXI_INTC.
 *        Habilita RX y TX, configura MER y vector base 0x10.
 */
void transceiver_interrupt_init(void);

/**
 * @brief Registra el callback para interrupción de recepción.
 * @param cb Función que se llama cuando RX genera una interrupción.
 */
void transceiver_register_rx_worker_id(rtems_id task_id);

/* Función para habilitar/deshabilitar interrupción RX (Usada por el Worker) */
void transceiver_int_enable_rx(bool enable);
/**
 * @brief Registra el callback para interrupción de transmisión.
 * @param cb Función que se llama cuando TX genera una interrupción.
 */
void transceiver_register_tx_callback(void (*cb)(void));


#ifdef __cplusplus
}
#endif

#endif // TRANSCEIVER_INTERRUPT_H
