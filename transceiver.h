/*
 * transceiver.h
 *
 * Driver profesional para Transceptor PL-UART sobre RTEMS (ZynqMP).
 * Implementación: Interrupción + Worker Task (Top Half / Bottom Half).
 */

#ifndef TRANSCEIVER_H
#define TRANSCEIVER_H

#include <stdint.h>
#include <stddef.h>
#include <rtems.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- Configuración --- */

typedef struct {
    uint32_t baud;         /* Ej: 115200 */
    uint32_t data_bits;    /* Ej: 0..3 -> 5..9mapping */
    uint32_t parity;       /* Ej: 4 = None */
    uint32_t stop_bits;    /* Ej: 2 */
    uint32_t bit_order;    /* 0 o 1 */
} Transceiver_Config_t;

/* --- API de Inicialización --- */

/**
 * @brief Inicializa el hardware, interrupciones y tareas del transceptor.
 * @param cfg Puntero a estructura de configuración (puede ser NULL para defaults).
 * @return RTEMS_SUCCESSFUL si todo fue bien.
 */
rtems_status_code Transceiver_Init(const Transceiver_Config_t *cfg);

/**
 * @brief Cierra el driver, deshabilita interrupciones y limpia recursos.
 */
void Transceiver_Shutdown(void);


/* --- API de Transmisión (TX) --- */

/**
 * @brief Envía un buffer de bytes.
 * @return 0 en éxito, -1 en error/timeout.
 */
int Transceiver_Send(const uint8_t *buf, size_t len);

/**
 * @brief Wrapper para enviar cadenas terminadas en null.
 */
int Transceiver_SendString(const char *s);


/* --- API de Recepción (RX) --- */

/* Tipo de callback para notificación (no recibe datos, solo avisa) */
typedef void (*Transceiver_Event_Cb_t)(void *arg);

/**
 * @brief Registra un callback que será llamado cuando lleguen nuevos datos.
 * El callback se ejecuta en contexto de tarea (seguro).
 */
void Transceiver_SetRxCallback(Transceiver_Event_Cb_t cb, void *arg);

/**
 * @brief Lee datos del buffer interno del driver.
 * @param buf Buffer de destino.
 * @param maxlen Tamaño máximo a leer.
 * @return Número de bytes leídos.
 */
size_t Transceiver_Read(uint8_t *buf, size_t maxlen);

/**
 * @brief Consulta cuántos bytes hay disponibles en el buffer.
 */
size_t Transceiver_Available(void);

#ifdef __cplusplus
}
#endif

#endif /* TRANSCEIVER_H */