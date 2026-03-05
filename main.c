/*
 * main.c
 * Sistema Multi-Transceptor sobre RTEMS.
 *
 * Funcionalidad:
 * 1. Inicializa transceptores en paralelo (Base 0xA0000000).
 * 2. RX: Muestra por consola lo que llega, indicando de qué UART vino.
 * 3. TX: Permite enviar mensajes a una UART específica desde la consola USB.
 */

#include <rtems.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "transceiver.h"

/* Objeto global para los transceptores */
static uint32_t num_transceivers;
static Transceiver uarts[MAX_TRANSCEIVERS];

/* Configuración común para todos (115200 8N1) */
static const Transceiver_Config_t cfg = {
    .baud = TRANSCEIVER_BAUD_115200, 
    .data_bits = TRANSCEIVER_DATA_BITS_8, 
    .parity = TRANSCEIVER_PARITY_NONE, 
    .stop_bits = TRANSCEIVER_STOP_BITS_1, 
    .bit_order = TRANSCEIVER_BIT_ORDER_DEFAULT,
    .slo_mode = TRANSCEIVER_SLO_DEFAULT
};

/* Configuración común para SLO ON (115200 8N1) */
static const Transceiver_Config_t cfg_slo_on = {
    .baud = TRANSCEIVER_BAUD_115200, 
    .data_bits = TRANSCEIVER_DATA_BITS_8, 
    .parity = TRANSCEIVER_PARITY_NONE, 
    .stop_bits = TRANSCEIVER_STOP_BITS_1, 
    .bit_order = TRANSCEIVER_BIT_ORDER_DEFAULT,
    .slo_mode = TRANSCEIVER_SLO_ON
};

/* Configuración común para SLO OFF (115200 8N1) */
static const Transceiver_Config_t cfg_slo_off = {
    .baud = TRANSCEIVER_BAUD_115200, 
    .data_bits = TRANSCEIVER_DATA_BITS_8, 
    .parity = TRANSCEIVER_PARITY_NONE, 
    .stop_bits = TRANSCEIVER_STOP_BITS_1, 
    .bit_order = TRANSCEIVER_BIT_ORDER_DEFAULT,
    .slo_mode = TRANSCEIVER_SLO_OFF
};
/* =========================================================================
 * 1. CALLBACK DE RECEPCIÓN (Se ejecuta cuando llega algo a CUALQUIER UART)
 * ========================================================================= */
/* Añadir esto al principio de main.c, junto a los otros defines */
#define APP_LINE_BUF_SIZE 1024

/* Estructura para gestionar el buffer de cada canal independientemente */
typedef struct {
    char buf[APP_LINE_BUF_SIZE];
    size_t idx;
} AppLineBuffer;

/* Array estático para guardar el estado de los 14 canales */
static AppLineBuffer rx_lines[MAX_TRANSCEIVERS]; 

static void on_rx_data(void *arg) {
    Transceiver *dev = (Transceiver *)arg;
    
    /* 1. Obtenemos el buffer correspondiente a ESTE transceptor */
    AppLineBuffer *line = &rx_lines[dev->id];
    
    uint8_t temp_buf[64]; /* Buffer temporal para sacar datos del driver */
    size_t n;

    /* 2. Drenamos todo lo que tenga el driver disponible */
    do {
        n = Transceiver_Read(dev, temp_buf, sizeof(temp_buf));
        
        for (size_t i = 0; i < n; i++) {
            char c = (char)temp_buf[i];

            /* A. Si es salto de línea: Terminar y Mostrar */
            if (c == '\n' || c == '\r') {
                if (line->idx > 0) { // Solo imprimir si hay algo acumulado
                    line->buf[line->idx] = '\0'; // Null-termination
                    printf("[RX UART %02d]: %s\n", (unsigned int)dev->id, line->buf);
                    fflush(stdout);
                    line->idx = 0; // Resetear índice para la siguiente frase
                }
            } 
            /* B. Si es un carácter normal: Acumular */
            else {
                if (line->idx < (APP_LINE_BUF_SIZE - 1)) {
                    line->buf[line->idx++] = c;
                } 
                /* C. Protección Anti-Overflow: Si se llena sin \n, forzamos impresión */
                else {
                    line->buf[line->idx] = '\0';
                    printf("[RX UART %02d PARCIAL]: %s\n", (unsigned int)dev->id, line->buf);
                    line->idx = 0; // Reset y guardamos el carácter actual en el nuevo buffer
                    line->buf[line->idx++] = c;
                }
            }
        }
    } while (n > 0);
}

/* =========================================================================
 * 2. TAREA DE CONSOLA (Parsea comandos y envía)
 * ========================================================================= */
static rtems_task Tx_Console_Task(rtems_task_argument arg) {
    (void)arg;
    char input_buf[512];
    char *cmd_ptr;
    char *msg_ptr;
    int target_id;

    printf("\n=======================================================\n");
    printf(" CONSOLA DE CONTROL MULTI-TRANSCEPTOR\n");
    printf("-------------------------------------------------------\n");
    printf(" Formato: <ID> <MENSAJE> | <ID> SLO ON | <ID> SLO OFF\n");
    printf(" Ejemplos:\n");
    printf("   '0 Hola'      -> Envia 'Hola' por UART 0\n");
    printf("   'ALL Reset'   -> Envia 'Reset' por TODAS las UARTs\n");
    printf("   '2 SLO ON'    -> Activa el modo Slow Rate en UART 2\n");
    printf("   'ALL SLO OFF' -> Desactiva el modo Slow en TODAS\n");
    printf("=======================================================\n\n");

    for (;;) {
        /* Prompt */
        printf("CMD> ");
        fflush(stdout);

        /* Leer línea de la consola USB */
        if (fgets(input_buf, sizeof(input_buf), stdin) == NULL) {
            rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(100));
            continue;
        }

        /* Ignorar líneas vacías */
        if (strlen(input_buf) == 0) continue;

        /* Separar ID del Mensaje */
        cmd_ptr = strtok(input_buf, " "); // Primer token (ID o ALL)
        msg_ptr = strtok(NULL, "");       // Resto de la línea (Mensaje o Comando)

        if (msg_ptr == NULL) {
            printf("Error: Falta el mensaje o comando.\n");
            continue;
        }

        /* --- CASO 1: Enviar a TODOS --- */
        if (strcasecmp(cmd_ptr, "ALL") == 0) {
            if (strcasecmp(msg_ptr, "SLO ON") == 0) {
                printf("Activando SLO en todas las UARTs...\n");
                for (int i = 0; i < num_transceivers; i++) {
                    Transceiver_Init(&uarts[i], i, &cfg_slo_on);
                    Transceiver_SetRxCallback(&uarts[i], on_rx_data, &uarts[i]);
                }
            } else if (strcasecmp(msg_ptr, "SLO OFF") == 0) {
                printf("Desactivando SLO en todas las UARTs...\n");
                for (int i = 0; i < num_transceivers; i++) {
                    Transceiver_Init(&uarts[i], i, &cfg_slo_off);
                    Transceiver_SetRxCallback(&uarts[i], on_rx_data, &uarts[i]);
                }
            } else {
                printf("Enviando a las %d UARTs...\n", num_transceivers);
                for (int i = 0; i < num_transceivers; i++) {
                    Transceiver_SendString(&uarts[i], msg_ptr);
                }
            }
            continue;
        }

        /* --- CASO 2: Enviar a ID específico --- */
        char *endptr;
        target_id = strtoul(cmd_ptr, &endptr, 10);

        if (*endptr != '\0') {
            printf("Error: ID '%s' no valido.\n", cmd_ptr);
            continue;
        }

        if (target_id >= 0 && target_id < num_transceivers) {
            
            /* Interceptar comandos de configuración de Slew Rate */
            if (strcasecmp(msg_ptr, "SLO ON") == 0) {
                Transceiver_Init(&uarts[target_id], target_id, &cfg_slo_on);
                Transceiver_SetRxCallback(&uarts[target_id], on_rx_data, &uarts[target_id]);
                printf("SLO -> UART %d: ON\n", target_id);
            } 
            else if (strcasecmp(msg_ptr, "SLO OFF") == 0) {
                Transceiver_Init(&uarts[target_id], target_id, &cfg_slo_off);
                Transceiver_SetRxCallback(&uarts[target_id], on_rx_data, &uarts[target_id]);
                printf("SLO -> UART %d: OFF\n", target_id);
            } 
            else {
                /* ¡ENVÍO REAL! */
                Transceiver_SendString(&uarts[target_id], msg_ptr);
                printf("Tx -> UART %d: OK\n", target_id);
            }
        } else {
            printf("Error: ID %d fuera de rango (0-%d)\n", target_id, num_transceivers - 1);
        }
    }
}

/* =========================================================================
 * 3. INICIALIZACIÓN DEL SISTEMA
 * ========================================================================= */
rtems_task Init(rtems_task_argument arg) {
    (void)arg;
    rtems_status_code sc;

    /* Mapeo de Memoria (Necesario para acceder al PL) */
    extern void mmu_map_pl_axi_early(void);
    mmu_map_pl_axi_early();

    
    /* 1. Inicializar el INTC Global (Paso Crítico Único) */
    /* Esto habilita el controlador maestro que escucha a las 14 UARTs */
    num_transceivers = Transceiver_Global_INIT();
    printf("\n=== ARRANQUE SISTEMA ZCU102 (%d UARTs) ===\n", num_transceivers);
    /* 2. Bucle de Inicialización de Transceptores */
    for (int i = 0; i < num_transceivers; i++) {
        /* Esta función calcula las direcciones automáticamente basándose en el ID */
        sc = Transceiver_Init(&uarts[i], i, &cfg);

        if (sc == RTEMS_SUCCESSFUL) {
            /* Registrar callback para recibir datos */
            Transceiver_SetRxCallback(&uarts[i], on_rx_data, &uarts[i]);
            
            /* Saludo opcional por el cable al arrancar */
            char boot_msg[64];
            sprintf(boot_msg, "UART %d Lista.\r\n", i);
            Transceiver_SendString(&uarts[i], boot_msg);
            
            printf("UART %02d [OK] Base: 0x%08lX\n", i, (unsigned long)uarts[i].base_addr);
        } else {
            printf("UART %02d [FALLO] Error código: %d\n", i, sc);
        }
    }

    /* 3. Arrancar la Consola de Usuario */
    rtems_id console_tid;
    sc = rtems_task_create(rtems_build_name('C','M','D','T'), 
                           100, /* Prioridad baja */
                           RTEMS_MINIMUM_STACK_SIZE * 4,
                           RTEMS_DEFAULT_MODES, 
                           RTEMS_DEFAULT_ATTRIBUTES, 
                           &console_tid);
    if (sc == RTEMS_SUCCESSFUL) {
        rtems_task_start(console_tid, Tx_Console_Task, 0);
    }

    /* Init muere o duerme */
    rtems_task_delete(RTEMS_SELF);
}