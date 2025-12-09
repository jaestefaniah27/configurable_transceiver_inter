/*
 * main.c
 *
 * Aplicación de prueba para el Driver Profesional del Transceptor.
 *
 * Funcionalidad:
 * 1. RX: Lo que llega por el Transceptor (PL) se imprime en la consola RTEMS.
 * 2. TX: Lo que escribes en la consola RTEMS (USB) se envía por el Transceptor.
 */

#include <rtems.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "transceiver.h"

#define CMD_BUF_SZ 512

/* Helper para asegurar que el texto sale por consola inmediatamente */
static void print_safe(const char *s) {
    fputs(s, stdout);
    fflush(stdout);
}

/* =========================================================================
 * 1. CALLBACK DE RECEPCIÓN (Se ejecuta cuando llega algo al Transceptor)
 * ========================================================================= */
static void On_Transceiver_Data_Received(void *arg) {
    (void)arg;
    uint8_t buffer[128];
    size_t n;

    /* Drenamos el buffer del driver hasta que esté vacío */
    do {
        /* Usamos la API pública para leer del buffer interno */
        n = Transceiver_Read(buffer, sizeof(buffer));
        
        if (n > 0) {
            /* Imprimimos lo recibido. 
               Nota: Si recibes binario puro, considera usar un hexdump. 
               Aquí asumimos texto para ver "Hola Mundo". */
            //printf("\n[RX PL -> PS] (%d bytes): ", (int)n);
            fwrite(buffer, 1, n, stdout);
            //printf("\n");
            fflush(stdout);
        }
    } while (n > 0);
}

/* =========================================================================
 * 2. TAREA DE TRANSMISIÓN (Lee Consola USB -> Envía a Transceptor)
 * ========================================================================= */
static rtems_task Tx_Console_Task(rtems_task_argument arg) {
    (void)arg;
    char buf[CMD_BUF_SZ];

    print_safe("\n--------------------------------------------------\n");
    print_safe(" TAREA TX ARRANCADA\n");
    print_safe(" Escribe texto y pulsa ENTER para enviarlo por el PL.\n");
    print_safe(" Escribe 'EXIT' para detener esta tarea.\n");
    print_safe("--------------------------------------------------\n\n");

    for (;;) {
        /* 1. Bloqueante: Espera a que el usuario escriba algo en la consola USB */
        if (fgets(buf, sizeof(buf), stdin) == NULL) {
            rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(100));
            continue;
        }

        /* 2. Limpieza: Eliminar saltos de línea (\n o \r) al final */
        size_t len = strlen(buf);
        while (len > 0 && buf[len-1] == '\r') {
            buf[--len] = '\0';
        }

        /* Si era una línea vacía, ignorar */
        if (len == 0) continue;

        /* 3. Comando especial de salida */
        if (strcmp(buf, "EXIT") == 0) {
            print_safe("Finalizando tarea de consola...\n");
            rtems_task_delete(RTEMS_SELF);
        }

        /* 4. Enviar usando el nuevo Driver Profesional */
        int res = Transceiver_SendString(buf);

        if (res == 0) {
            printf("[TX PS -> PL] Enviado: '%s'", buf);
        } else {
            printf("[TX ERROR] Fallo al enviar.\n");
        }
        
        fflush(stdout);
    }
}

/* Helper para arrancar la tarea TX */
static void Start_Console_Task(void) {
    rtems_status_code sc;
    rtems_id tid;

    sc = rtems_task_create(rtems_build_name('C','O','N','S'), 
                           80, /* Prioridad menor que el driver (50) */
                           RTEMS_MINIMUM_STACK_SIZE * 4,
                           RTEMS_DEFAULT_MODES, 
                           RTEMS_DEFAULT_ATTRIBUTES, 
                           &tid);
    
    if (sc == RTEMS_SUCCESSFUL) {
        rtems_task_start(tid, Tx_Console_Task, 0);
    } else {
        printf("Error creando tarea de consola: %s\n", rtems_status_text(sc));
    }
}

/* =========================================================================
 * 3. TAREA INIT (Inicialización del Sistema)
 * ========================================================================= */
rtems_task Init(rtems_task_argument arg) {
    (void)arg;
    rtems_status_code sc;

    /* 1. Mapeo de memoria (Crítico para ZynqMP) */
    /* Asegúrate de tener el prototipo o el include donde esté definida */
    extern void mmu_map_pl_axi_early(void); 
    mmu_map_pl_axi_early();

    /* Espera de estabilización */
    rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(50));

    printf("\n=== INICIANDO SISTEMA RTEMS + TRANSCEIVER ===\n");

    /* 2. Configuración del Driver */
    Transceiver_Config_t cfg = {
        .baud = 115200,
        .data_bits = 3, /* Según tu lógica: 3 map a 8 bits */
        .parity = 4,    /* 4 map a None */
        .stop_bits = 2, /* 2 map a 1 stop bit (o lo que tengas definido) */
        .bit_order = 0
    };

    /* 3. Inicializar Driver (Internamente crea Worker, Mutex e Interrupciones) */
    sc = Transceiver_Init(&cfg);
    if (sc != RTEMS_SUCCESSFUL) {
        printf("PANIC: No se pudo iniciar el Transceptor. Error: %d\n", sc);
        exit(1);
    }
    printf("Driver Transceiver: OK (IRQ + Worker)\n");

    /* 4. Registrar Callback de RX */
    /* Cuando llegue un dato, el driver llamará a esta función automáticamente */
    Transceiver_SetRxCallback(On_Transceiver_Data_Received, NULL);

    /* 5. Arrancar la tarea que lee de la consola USB */
    Start_Console_Task();

    /* 6. La tarea Init se duerme para siempre (o actúa como watchdog) */
    printf("Sistema Listo. Escribe en la consola.\n");
    
    for (;;) {
        rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(1000));
    }
}