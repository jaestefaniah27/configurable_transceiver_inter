/* transceiver.h */
#ifndef TRANSCEIVER_H
#define TRANSCEIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <rtems.h>
/* =========================================================================
 * STOP BITS)
 * ========================================================================= */

#define TRANSCEIVER_STOP_BITS_1     0 /* 1 Stop Bit */
#define TRANSCEIVER_STOP_BITS_1_5   2 /* 1.5 Stop Bits */
#define TRANSCEIVER_STOP_BITS_2     3 /* 2 Stop Bits */
/* =========================================================================
 * PARITY MODES
 * ========================================================================= */
#define TRANSCEIVER_PARITY_EVEN    0  /* Paridad Par */
#define TRANSCEIVER_PARITY_ODD     1  /* Paridad Impar */
#define TRANSCEIVER_PARITY_MARK    2  /* Mark (Bit de paridad siempre 1) */
#define TRANSCEIVER_PARITY_SPACE   3  /* Space (Bit de paridad siempre 0) */
#define TRANSCEIVER_PARITY_NONE    4  /* Sin Paridad (Desactivada) */
/* =========================================================================
 * BAUD RATE SELECTORS (NCO LUT INDICES)
 * ========================================================================= */
/* --- Low Frequencies --- */
#define TRANSCEIVER_BAUD_50        0
#define TRANSCEIVER_BAUD_75        1
#define TRANSCEIVER_BAUD_110       2
#define TRANSCEIVER_BAUD_134       3   /* ~134.5 */
#define TRANSCEIVER_BAUD_150       4
#define TRANSCEIVER_BAUD_200       5
#define TRANSCEIVER_BAUD_300       6
#define TRANSCEIVER_BAUD_600       7
#define TRANSCEIVER_BAUD_1200      8
#define TRANSCEIVER_BAUD_1800      9
#define TRANSCEIVER_BAUD_2000      10
#define TRANSCEIVER_BAUD_2400      11
#define TRANSCEIVER_BAUD_3600      12
#define TRANSCEIVER_BAUD_4800      13
#define TRANSCEIVER_BAUD_7200      14

/* --- Common Standards --- */
#define TRANSCEIVER_BAUD_9600      15  /* Arduino / Standard */
#define TRANSCEIVER_BAUD_12000     16
#define TRANSCEIVER_BAUD_14400     17
#define TRANSCEIVER_BAUD_19200     18
#define TRANSCEIVER_BAUD_28800     19

/* --- Special Protocols --- */
#define TRANSCEIVER_BAUD_31250     20  /* MIDI */
#define TRANSCEIVER_BAUD_38400     21
#define TRANSCEIVER_BAUD_50000     22
#define TRANSCEIVER_BAUD_56000     23  /* Legacy Modems */
#define TRANSCEIVER_BAUD_57600     24

/* --- High Speed --- */
#define TRANSCEIVER_BAUD_64000     25
#define TRANSCEIVER_BAUD_74400     26
#define TRANSCEIVER_BAUD_74880     27  /* ESP8266 Boot */
#define TRANSCEIVER_BAUD_76800     28
#define TRANSCEIVER_BAUD_115200    29  /* PC Serial Standard */
#define TRANSCEIVER_BAUD_128000    30
#define TRANSCEIVER_BAUD_153600    31
#define TRANSCEIVER_BAUD_200000    32
#define TRANSCEIVER_BAUD_230400    33
#define TRANSCEIVER_BAUD_250000    34  /* DMX512 Lighting */
#define TRANSCEIVER_BAUD_256000    35
#define TRANSCEIVER_BAUD_312500    36
#define TRANSCEIVER_BAUD_400000    37
#define TRANSCEIVER_BAUD_460800    38
#define TRANSCEIVER_BAUD_500000    39
#define TRANSCEIVER_BAUD_576000    40
#define TRANSCEIVER_BAUD_614400    41
#define TRANSCEIVER_BAUD_750000    42
#define TRANSCEIVER_BAUD_921600    43

/* --- Megabaud Rates --- */
#define TRANSCEIVER_BAUD_1M        44
#define TRANSCEIVER_BAUD_1_152M    45
#define TRANSCEIVER_BAUD_1_5M      46
#define TRANSCEIVER_BAUD_1_8432M   47
#define TRANSCEIVER_BAUD_2M        48
#define TRANSCEIVER_BAUD_2_5M      49
#define TRANSCEIVER_BAUD_3M        50
#define TRANSCEIVER_BAUD_3_5M      51
#define TRANSCEIVER_BAUD_3_6864M   52
#define TRANSCEIVER_BAUD_4M        53  /* Max Recommended */

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

/**
 * @brief Obtiene el número de transceptores presentes en el hardware (FPGA).
 * Lee el registro de identificación del sistema en 0xA0020000.
 * @return Número de canales disponibles (ej: 14).
 */
uint32_t Transceiver_GetHardwareCount(void);
#ifdef __cplusplus
}
#endif

#endif