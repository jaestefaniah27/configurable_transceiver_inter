/* transceiver.h */
#ifndef TRANSCEIVER_H
#define TRANSCEIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <rtems.h>

#ifdef __cplusplus
extern "C" {
#endif

/* === Configuración === */
typedef struct {
    uint32_t baud;
    uint32_t data_bits;
    uint32_t parity;
    uint32_t stop_bits;
    uint32_t bit_order;
} Transceiver_Config_t;

/* === Objeto Transceiver (Handle) === */
typedef struct {
    /* -- Hardware Address Map -- */
    uint32_t id;                /* Índice del transceptor (0, 1, ... 13) */
    uintptr_t base_addr;        /* Dirección base de esta instancia (ej: 0xA0000000) */
    uintptr_t intc_base;        /* Dirección del INTC Global compartido */
    
    /* Offsets calculados */
    uintptr_t addr_setup;       /* Base + 0x0000 */
    uintptr_t addr_rx;          /* Base + 0x1000 */
    uintptr_t addr_tx;          /* Base + 0x2000 */
    uintptr_t addr_status;      /* Base + 0x3000 */

    /* -- Interrupt info -- */
    uint32_t mask_rx;           /* Bitmask para RX en el INTC Global */
    uint32_t mask_tx;           /* Bitmask para TX en el INTC Global */

    /* -- Software State -- */
    rtems_id worker_id;         /* ID de la tarea worker */
    rtems_id mutex_id;          /* ID del mutex para el buffer RX */
    
    uint8_t *rx_buffer;         /* Puntero al buffer circular (allocado dinámicamente o estático) */
    size_t rx_buf_size;
    volatile size_t rx_head;
    size_t rx_tail;
    volatile size_t rx_count;

    /* Callback de usuario */
    void (*rx_callback)(void *arg);
    void *rx_callback_arg;

} Transceiver;

/* === API Pública === */

/**
 * @brief Inicializa una instancia del transceptor.
 * @param dev Puntero a la estructura Transceiver a inicializar.
 * @param id ID del transceptor (0 a 13). Calcula direcciones automáticamente.
 * @param cfg Configuración de baudios, paridad, etc.
 */
rtems_status_code Transceiver_Init(Transceiver *dev, uint32_t id, const Transceiver_Config_t *cfg);

/**
 * @brief Lee datos del buffer de recepción.
 */
size_t Transceiver_Read(Transceiver *dev, uint8_t *buf, size_t maxlen);

/**
 * @brief Envía una cadena (bloqueante o no, según implementación).
 */
int Transceiver_SendString(Transceiver *dev, const char *s);

/**
 * @brief Registra callback de recepción.
 */
void Transceiver_SetRxCallback(Transceiver *dev, void (*cb)(void *), void *arg);

/**
 * @brief Función maestra para inicializar el controlador de interrupciones global.
 * Se debe llamar UNA sola vez al inicio, antes de init los transceptores.
 */
void Transceiver_Global_INTC_Init(void);

#ifdef __cplusplus
}
#endif

#endif