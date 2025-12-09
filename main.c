/* main.c
 *
 * App mínima para probar la transmisión: lee de la consola USB (stdin)
 * y retransmite por el transceptor PL usando la API en transceiver.h.
 *
 * Compila junto a transceiver.c / transceiver.h en tu proyecto RTEMS.
 */

#include <rtems.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "transceiver.h"
#include "transceiver_interrupt.h"
#define CMD_BUF_SZ 512

/* Para imprimir y forzar flush a la consola */
static void send_line_stdout(const char *s) {
  puts(s);
  fflush(stdout);
}

/* Tarea que lee líneas desde stdin y las envía por el transceptor */
static rtems_task tx_task(rtems_task_argument arg) {
  (void)arg;
  char buf[CMD_BUF_SZ];

  send_line_stdout("TX task arrancada. Escribe líneas para enviarlas por el transceptor.");
  send_line_stdout("La línea se enviará sin el caracter de nueva línea final.");
  send_line_stdout("Escribe EXIT para salir de la tarea.");

  for (;;) {
    /* Leer línea (bloqueante sobre stdin) */
    if (fgets(buf, sizeof(buf), stdin) == NULL) {
      /* EOF o error: esperar y seguir intentando */
      rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(200));
      continue;
    }

    /* eliminar \r finales */
    size_t L = strlen(buf);
    if (L > 0 && buf[L-1] == '\r') buf[--L] = 0;

    if (L == 0) {
      send_line_stdout("NOTHING (línea vacía) — nada enviado.");
      continue;
    }

    /* comando para salir de la tarea (útil en pruebas) */
    if (strcmp(buf, "EXIT") == 0) {
      send_line_stdout("TX task finalizando (EXIT recibido).");
      return;
    }

    /* Intentar enviar la cadena (timeout por byte = 50 ms) */
    int r = send_string(buf); /* 0 = OK, -1 = timeout */
    if (r == 0) {
      send_line_stdout("OK");
    } else {
      send_line_stdout("ERR TX_TIMEOUT");
    }
  }
}

/* Crea y arranca la tarea tx_task */
static void start_tx_task(void) {
  rtems_status_code sc;
  rtems_id tid;
  rtems_name name = rtems_build_name('T','X','1',' ');
  sc = rtems_task_create(name, 80, RTEMS_MINIMUM_STACK_SIZE * 4,
                         RTEMS_DEFAULT_MODES, RTEMS_DEFAULT_ATTRIBUTES, &tid);
  if (sc != RTEMS_SUCCESSFUL) {
    printf("ERR: tx_task create %d\n", (int)sc);
    return;
  }
  sc = rtems_task_start(tid, tx_task, 0);
  if (sc != RTEMS_SUCCESSFUL) {
    printf("ERR: tx_task start %d\n", (int)sc);
  }
}
/* Callback sencillo: imprime en stdout los bytes recibidos */
static void my_rx_cb(const uint8_t *data, size_t len, void *arg) {
  (void)arg;
  /* imprimir como texto (si son ASCII) */
  fwrite(data, 1, len, stdout);
  fflush(stdout);
}


/* Init RTEMS */
rtems_task Init(rtems_task_argument arg) {
  (void)arg;

  /* Mapear PL antes de acceder a los GPIO AXI (tu implementación) */
  mmu_map_pl_axi_early();

  /* Pequeña espera para que todo se estabilice */
  rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(10));

  printf("Main: app de prueba TX arrancando. Consola: USB-PS.\n");

    transceiver_cfg_t cfg = {
    .baud = 115200,
    .data_bits = 3,   /* según tu encoding: 3 -> 8 bits */
    .parity = 4,      /* 0 = EVEN (tu convención) */
    .stop_bits = 2,   /* 3 = 1.5 stop bits (tu convención) */
    .bit_order = 0
    };
    transceiver_configure(&cfg);


  /* Arrancar la tarea de transmisión */
  transceiver_interrupt_init();
  register_tx_callback();
  /* En tu Init(), después de mapear PL etc. */
  //transceiver_rx_init_polling(5); /* poll cada 5 ms */
  transceiver_rx_init_interrupt();
  transceiver_set_rx_callback(my_rx_cb, NULL);
  //printf("RX iniciado. Callback registrado.\n");
  
  start_tx_task();
  /* Init no hace nada más; dormir en bucle. */
  for (;;) {
    rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(100));
  }
}
