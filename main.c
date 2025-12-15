/*
 * main.c
 * Sistema Multi-Transceptor (14 Canales) sobre RTEMS.
 *
 * Funcionalidad:
 * 1. Inicializa 14 transceptores en paralelo (Base 0xA0000000).
 * 2. RX: Muestra por consola lo que llega, indicando de qué UART vino.
 * 3. TX: Permite enviar mensajes a una UART específica desde la consola USB.
 */

#include <rtems.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "transceiver.h"

/* Definiciones del Hardware (Coinciden con tu script TCL) */
#define NUM_TRANSCEIVERS      14
// #define TRANSCEIVER_BASE_ADDR 0xA0000000
// #define TRANSCEIVER_STRIDE    0x100000     /* 1MB por bloque */
// #define INTC_GLOBAL_BASE      (TRANSCEIVER_BASE_ADDR + (NUM_TRANSCEIVERS * TRANSCEIVER_STRIDE))

/* Objeto global para los 14 transceptores */
static Transceiver uarts[NUM_TRANSCEIVERS];

/* Configuración común para todos (115200 8N1) */
static const Transceiver_Config_t cfg = {
    .baud = 115200, .data_bits = 3, .parity = 4, .stop_bits = 2, .bit_order = 0
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
static AppLineBuffer rx_lines[NUM_TRANSCEIVERS]; 

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
    printf(" Formato: <ID> <MENSAJE>\n");
    printf(" Ejemplos:\n");
    printf("   '0 Hola'      -> Envia 'Hola' por UART 0\n");
    printf("   '12 Status'   -> Envia 'Status' por UART 12\n");
    printf("   'ALL Reset'   -> Envia 'Reset' por TODAS las UARTs\n");
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

        /* Limpiar salto de línea al final */
        input_buf[strcspn(input_buf, "\r")] = 0;

        /* Ignorar líneas vacías */
        if (strlen(input_buf) == 0) continue;

        /* Separar ID del Mensaje */
        cmd_ptr = strtok(input_buf, " "); // Primer token (ID)
        msg_ptr = strtok(NULL, "");       // Resto de la línea (Mensaje)

        if (msg_ptr == NULL) {
            printf("Error: Falta el mensaje. Uso: <ID> <MENSAJE>\n");
            continue;
        }

        /* --- CASO 1: Enviar a TODOS --- */
        if (strcasecmp(cmd_ptr, "ALL") == 0) {
            printf("Enviando a las %d UARTs...\n", NUM_TRANSCEIVERS);
            for (int i = 0; i < NUM_TRANSCEIVERS; i++) {
                Transceiver_SendString(&uarts[i], msg_ptr);
                // Transceiver_SendString(&uarts[i], "\r\n"); // Opcional: añadir CR/LF
            }
            continue;
        }

        /* --- CASO 2: Enviar a ID específico --- */
        /* Validar que el ID sea un número */
        char *endptr;
        target_id = strtoul(cmd_ptr, &endptr, 10);

        if (*endptr != '\0') {
            printf("Error: ID '%s' no valido.\n", cmd_ptr);
            continue;
        }

        if (target_id >= 0 && target_id < NUM_TRANSCEIVERS) {
            /* ¡ENVÍO REAL! */
            Transceiver_SendString(&uarts[target_id], msg_ptr);
            // Transceiver_SendString(&uarts[target_id], "\r\n"); 
            printf("Tx -> UART %d: OK\n", target_id);
        } else {
            printf("Error: ID %d fuera de rango (0-%d)\n", target_id, NUM_TRANSCEIVERS - 1);
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

    printf("\n=== ARRANQUE SISTEMA ZCU102 (14 UARTs) ===\n");

    /* 1. Inicializar el INTC Global (Paso Crítico Único) */
    /* Esto habilita el controlador maestro que escucha a las 14 UARTs */
    Transceiver_Global_INTC_Init();
    printf("INTC Global en 0xA0E00000: Inicializado.\n");

    /* 2. Bucle de Inicialización de Transceptores */
    for (int i = 0; i < NUM_TRANSCEIVERS; i++) {
        /* Esta función calcula las direcciones automáticamente basándose en el ID */
        /* Base = 0xA0000000 + (i * 0x10000) */
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