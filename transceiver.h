#ifndef TRANSCEIVER_H
#define TRANSCEIVER_H

#include <stdint.h>
#include <stddef.h>

/* === Bases (ajusta si hace falta) === */
#define GPIO0_BASE   0xA0000000u   /* SERIAL_CONFIG */
#define GPIO1_BASE   0xA0010000u   /* PS_RX_DataRead_ErrorOk */
#define GPIO2_BASE   0xA0020000u   /* PS_TX_DataIn_Send */
#define GPIO3_BASE   0xA0030000u   /* PS_OUT */
/* Offsets canal 1 (si usas channel 2, cambiar a 0x08) */
#define GPIO0_DATA_OFF 0x00u
#define GPIO1_DATA_OFF 0x00u
#define GPIO2_DATA_OFF 0x00u
#define GPIO3_DATA_OFF 0x00u

/* INTERRUPTS */
#define INTR_TX_RDY_MASK    0x1u
#define INTR_RX_NOT_EMPTY_MASK   0x2u



/* Masks */
#define PS_OUT_TX_RDY_MASK   0x2000u
#define PS_OUT_FRAME_ERR_MASK 0x1000u
#define PS_OUT_PAR_ERR_MASK   0x0800u
#define PS_OUT_FULL_MASK      0x0400u
#define PS_OUT_EMPTY_MASK     0x0200u
#define PS_OUT_DATA_MASK      0x01FFu

#define GPIO1_DATA_READ_MASK  0x1u
#define GPIO1_ERROR_OK_MASK   0x2u

#define GPIO2_DATA_IN_MASK    0x01FFu
#define GPIO2_TX_SEND_MASK    0x0200u

/* SERIAL_CONFIG masks */
#define SERIAL_BAUD_MASK      0x003FFFFFu
#define SERIAL_STOP_MASK      0x01C00000u
#define SERIAL_PARITY_MASK    0x0E000000u
#define SERIAL_DATA_BITS_MASK 0x70000000u
#define SERIAL_BIT_ORDER_MASK 0x80000000u

/* Prototipos: mmio y helpers */
void mmu_map_pl_axi_early(void); /* prototipo que sigue en tu Init */

/* MMIO */
void mmio_write32(uintptr_t a, uint32_t v);
uint32_t mmio_read32(uintptr_t a);

/* GPIO raw access */
uint32_t gpio0_read_raw(void);
void     gpio0_write_raw(uint32_t v);
uint32_t gpio1_read_raw(void);
uint32_t gpio2_read_raw(void);
uint32_t gpio3_read_raw(void);

/* RMW helpers */
void gpio1_write_rmw(uint32_t mask, uint32_t value_masked);
void gpio2_write_rmw(uint32_t mask, uint32_t value_masked);
void gpio0_write_field(uint32_t mask, unsigned shift, uint32_t val);

/* Field helpers convenient (previous inline ones) */
void gpio1_write_data_read(uint32_t v);
void gpio1_write_error_ok(uint32_t v);
void gpio2_write_data_in(uint32_t v);
void gpio2_write_tx_send(uint32_t v);

/* SERIAL getters/setters */
uint32_t serial_get_baudrate(void);
uint32_t serial_get_stop_bits(void);
uint32_t serial_get_parity(void);
uint32_t serial_get_data_bits(void);
uint32_t serial_get_bit_order(void);

void serial_set_baudrate(uint32_t br);
void serial_set_stop_bits(uint32_t sb);
void serial_set_parity(uint32_t p);
void serial_set_data_bits(uint32_t db);
void serial_set_bit_order(uint32_t bo);

/* Estructura para configurar el transceiver */
typedef struct {
  uint32_t baud;         /* 0 = no tocar */
  uint32_t data_bits;    /* código (0..n) -> mapping interno; 0 = no tocar */
  uint32_t parity;       /* código de paridad (p.ej. 4 -> no parity). 0 = no tocar */
  uint32_t stop_bits;    /* nº de stop bits o código (1..). 0 = no tocar */
  uint32_t bit_order;    /* 0/1 o 0 = no tocar si pones 0xFFFFFFFF como "no tocar" */
} transceiver_cfg_t;

/* Configura el transceiver: aplica los campos != 0 (o != 0xFFFFFFFF para bit_order).
 * Devuelve 0 si OK.
 * Además actualiza internamente el tiempo de frame cacheado.
 *
 * Convención simple:
 *  - Si cfg->baud == 0 => no se cambia el baud
 *  - Si cfg->data_bits == 0 => no se cambia data bits (usa el encoding que ya tenías)
 *  - Si cfg->parity == 0 => no se cambia
 *  - Si cfg->stop_bits == 0 => no se cambia
 *  - Si cfg->bit_order == 0xFFFFFFFF => no se cambia (porque 0 es valor válido)
 */
int transceiver_configure(const transceiver_cfg_t *cfg);

/* Devuelve el tiempo de transmisión cacheado en ms (0 si no cacheado) */
uint32_t transceiver_get_frame_time_ms(void);

/* Fuerza recalcular el tiempo de frame a partir de los registros actuales */
void transceiver_update_cached_frame_time(void);

/* TX helper */
void tx_write_data_and_send(uint16_t data9);

/* FIFO consume pulse */
void fifo_consume_pulse(void);

/* Higher-level send (envía varios bytes) */
/* devuelve 0 si OK, -1 si timeout en algún byte */
int send_bytes(const uint8_t *buf, size_t len);
/* wrapper para cadenas C (no envía el '\0') */
int send_string(const char *s);

typedef void (*transceiver_rx_cb_t)(const uint8_t *data, size_t len, void *arg);

/* Inicializa RX en modo polling (solo PS).
 * poll_interval_ms: intervalo de sondeo (ms). Recomendado 1..10 ms.
 * Devuelve 0 si OK, -1 si error.
 */
int transceiver_rx_init_polling(uint32_t poll_interval_ms);
void transceiver_rx_init_interrupt(void);
/* Registrar callback (se ejecuta dentro de la tarea rx worker).
 * Si cb == NULL se desactiva el callback.
 */
void transceiver_set_rx_callback(transceiver_rx_cb_t cb, void *arg);
void register_tx_callback(void);
/* Consultas / lectura de la cola RX */
size_t transceiver_rx_available(void);
size_t transceiver_rx_read(uint8_t *buf, size_t maxlen);

/* Apaga/limpia el subsistema RX (no necesariamente mata tareas de forma segura) */
void transceiver_rx_shutdown(void);

#endif /* TRANSCEIVER_H */
