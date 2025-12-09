#include <rtems.h>   /* para rtems_task_wake_after en send_bytes */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "transceiver.h"
#include "transceiver_interrupt.h"
// /* Ajusta tamaño de buffer RX según necesidades */
#ifndef RX_QUEUE_SIZE
#define RX_QUEUE_SIZE (4 * 1024)
#endif

#define TX_BUF_SIZE 256
static uint8_t tx_buf[TX_BUF_SIZE];
static size_t tx_head = 0;
static size_t tx_tail = 0;
static volatile bool tx_active = false;

/* --- Estructuras RX --- */
static uint8_t rx_queue[RX_QUEUE_SIZE];
static size_t rx_head = 0;
static size_t rx_tail = 0;
static size_t rx_count = 0;
/* sincronización */
static rtems_id rx_mutex = 0;   /* mutex para proteger la cola */
static rtems_id rx_sem = 0;     /* semáforo para notificar al worker (si se usa) */
static rtems_id rx_poll_tid = 0;
static int rx_poll_running = 0;
static rtems_id rx_worker_tid = 0;

/* Callback del usuario */
static transceiver_rx_cb_t rx_callback = NULL;
static void *rx_callback_arg = NULL;


/* MMIO helpers */
void mmio_write32(uintptr_t a, uint32_t v) {
  *(volatile uint32_t *)a = v;
  __asm__ volatile("dmb sy" ::: "memory");
}
uint32_t mmio_read32(uintptr_t a) {
  __asm__ volatile("dmb sy" ::: "memory");
  return *(volatile uint32_t *)a;
}

/* ---- GPIO helpers (lectura/escritura por campos) ---- */
uint32_t gpio0_read_raw(void) {
  return mmio_read32(GPIO0_BASE + GPIO0_DATA_OFF);
}
void gpio0_write_raw(uint32_t v) {
  mmio_write32(GPIO0_BASE + GPIO0_DATA_OFF, v);
  __asm__ volatile("dmb sy" ::: "memory");
}
uint32_t gpio1_read_raw(void) {
  return mmio_read32(GPIO1_BASE + GPIO1_DATA_OFF);
}
uint32_t gpio2_read_raw(void) {
  return mmio_read32(GPIO2_BASE + GPIO2_DATA_OFF);
}
uint32_t gpio3_read_raw(void) {
  return mmio_read32(GPIO3_BASE + GPIO3_DATA_OFF);
}

/* RMW helpers */
void gpio1_write_rmw(uint32_t mask, uint32_t value_masked) {
  uintptr_t addr = GPIO1_BASE + GPIO1_DATA_OFF;
  uint32_t v = mmio_read32(addr);
  v &= ~mask;
  v |= (value_masked & mask);
  mmio_write32(addr, v);
  __asm__ volatile("dmb sy" ::: "memory");
}
void gpio2_write_rmw(uint32_t mask, uint32_t value_masked) {
  uintptr_t addr = GPIO2_BASE + GPIO2_DATA_OFF;
  uint32_t v = mmio_read32(addr);
  v &= ~mask;
  v |= (value_masked & mask);
  mmio_write32(addr, v);
  __asm__ volatile("dmb sy" ::: "memory");
}
void gpio0_write_field(uint32_t mask, unsigned shift, uint32_t val) {
  uint32_t v = gpio0_read_raw();
  v &= ~mask;
  v |= ((val << shift) & mask);
  gpio0_write_raw(v);
}

/* SERIAL getters/setters */
uint32_t serial_get_baudrate(void) { return gpio0_read_raw() & SERIAL_BAUD_MASK; }
uint32_t serial_get_stop_bits(void) { return (gpio0_read_raw() & SERIAL_STOP_MASK) >> 22; }
uint32_t serial_get_parity(void) { return (gpio0_read_raw() & SERIAL_PARITY_MASK) >> 25; }
uint32_t serial_get_data_bits(void) { return (gpio0_read_raw() & SERIAL_DATA_BITS_MASK) >> 28; }
uint32_t serial_get_bit_order(void) { return (gpio0_read_raw() & SERIAL_BIT_ORDER_MASK) ? 1u : 0u; }

void serial_set_baudrate(uint32_t br) { gpio0_write_field(SERIAL_BAUD_MASK, 0u, br & 0x3FFFFFu); }
void serial_set_stop_bits(uint32_t sb) { gpio0_write_field(SERIAL_STOP_MASK, 22u, sb & 0x7u); }
void serial_set_parity(uint32_t p) { gpio0_write_field(SERIAL_PARITY_MASK, 25u, p & 0x7u); }
void serial_set_data_bits(uint32_t db) { gpio0_write_field(SERIAL_DATA_BITS_MASK, 28u, db & 0x7u); }
void serial_set_bit_order(uint32_t bo) { gpio0_write_field(SERIAL_BIT_ORDER_MASK, 31u, bo & 0x1u); }

/* TX helper: pulse TX_SEND in GPIO2 (RMW) */
void tx_write_data_and_send(uint16_t data9) {
  gpio2_write_rmw(GPIO2_DATA_IN_MASK, (uint32_t)(data9 & GPIO2_DATA_IN_MASK));
  /* pulse TX_SEND (bit 9) */
  gpio2_write_rmw(GPIO2_TX_SEND_MASK, GPIO2_TX_SEND_MASK);
  for (volatile int i = 0; i < 200; ++i) __asm__ volatile("nop");
  gpio2_write_rmw(GPIO2_TX_SEND_MASK, 0u);
}

/* DATA_READ pulse on GPIO1 */
void fifo_consume_pulse(void) {
  gpio1_write_rmw(GPIO1_DATA_READ_MASK, GPIO1_DATA_READ_MASK);
  for (volatile int i = 0; i < 200; ++i) __asm__ volatile("nop");
  gpio1_write_rmw(GPIO1_DATA_READ_MASK, 0u);
}

/* Field-style helpers */
void gpio1_write_data_read(uint32_t v)
{
    uint32_t reg = mmio_read32(GPIO1_BASE);
    reg &= ~0x1u;          // limpiar bit0
    reg |= (v & 0x1u);     // escribir nuevo valor
    mmio_write32(GPIO1_BASE, reg);
}

void gpio1_write_error_ok(uint32_t v)
{
    uint32_t reg = mmio_read32(GPIO1_BASE);
    reg &= ~0x2u;              // limpiar bit1
    reg |= ((v & 0x1u) << 1);  // escribir bit1
    mmio_write32(GPIO1_BASE, reg);
}
void gpio2_write_data_in(uint32_t v)
{
    v &= 0x1FFu;           // limitar a 9 bits
    uint32_t reg = mmio_read32(GPIO2_BASE);
    reg &= ~0x1FFu;        // limpiar bits 0–8
    reg |= v;              // escribir DATA_IN
    mmio_write32(GPIO2_BASE, reg);
}

void gpio2_write_tx_send(uint32_t v)
{
    uint32_t reg = mmio_read32(GPIO2_BASE);
    reg &= ~(1u << 9);         // limpiar bit9
    reg |= ((v & 0x1u) << 9);  // escribir TX_SEND
    mmio_write32(GPIO2_BASE, reg);
}

/* --- variables estáticas para cachear el tiempo de frame --- */
static uint32_t cached_frame_time_ms = 0;    /* 0 = no calculado aún */
static uint64_t cached_frame_time_us = 0ULL; /* valor en microsegundos */

/* Helper interno: calcula frame_time (us, ms) a partir de los parámetros actuales leídos
   de los registros SERIAL (gpio0). Usa las mismas asunciones que antes:
     data_bits = 5 + data_bits_code
     parity_code == 4 -> sin paridad
     stop_bits_code >=1 -> número de stop bits, si es 0 -> 1
*/
static void transceiver_calc_frame_time_from_regs(void)
{
  uint32_t baud = serial_get_baudrate();
  if (baud == 0) baud = 115200u; /* fallback */

  uint32_t data_bits_code = serial_get_data_bits(); /* 0..n -> mapping asumido */
  uint32_t data_bits = 5u + data_bits_code;        /* asumimos mapping 5..8 */

  uint32_t parity_code = serial_get_parity();
  int parity_enabled = (parity_code == 4u) ? 0 : 1;

  uint32_t stop_bits_code = serial_get_stop_bits();
  uint32_t stop_bits = (stop_bits_code >= 1u) ? stop_bits_code : 1u;

  uint32_t frame_bits = 1u /* start */ + data_bits + (parity_enabled ? 1u : 0u) + stop_bits;

  uint64_t frame_time_us_local = (uint64_t)frame_bits * 1000000ULL;
  /* ceil division: (frame_bits*1e6 + baud -1) / baud) */
  frame_time_us_local = (frame_time_us_local + (uint64_t)baud - 1ULL) / (uint64_t)baud;

  uint32_t frame_time_ms_local = (uint32_t)((frame_time_us_local + 999ULL) / 1000ULL);
  if (frame_time_ms_local == 0) frame_time_ms_local = 1;

  cached_frame_time_us = frame_time_us_local;
  cached_frame_time_ms = frame_time_ms_local;
}

/* Pública: fuerza recálculo usando registros actuales */
void transceiver_update_cached_frame_time(void)
{
  transceiver_calc_frame_time_from_regs();
}

/* Pública: devuelve frame time cacheado en ms */
uint32_t transceiver_get_frame_time_ms(void)
{
  return cached_frame_time_ms;
}

/* Pública: configurar el transceiver y recálculo si tocas el baud (o siempre) */
int transceiver_configure(const transceiver_cfg_t *cfg)
{
  if (!cfg) return -1;

  /* Aplicar únicamente los campos solicitados (convención: 0 = no tocar, salvo bit_order) */
  if (cfg->baud != 0) {
    serial_set_baudrate(cfg->baud);
  }
  if (cfg->data_bits != 0) {
    /* asumimos que cfg->data_bits viene codificado igual que serial_set_data_bits espera */
    serial_set_data_bits(cfg->data_bits);
  }
  if (cfg->parity != 0) {
    serial_set_parity(cfg->parity);
  }
  if (cfg->stop_bits != 0) {
    serial_set_stop_bits(cfg->stop_bits);
  }
  if (cfg->bit_order != 0xFFFFFFFFu) {
    serial_set_bit_order(cfg->bit_order);
  }

  /* Siempre recalculamos la cache porque cualquier cambio en config puede afectar al tiempo */
  transceiver_calc_frame_time_from_regs();

  return 0;
}

static void tx_kick(void) {
  if (tx_tail == tx_head) {
    tx_active = false;
    return;  // Nada que enviar
  }

  uint8_t byte = tx_buf[tx_tail];
  tx_tail = (tx_tail + 1) % TX_BUF_SIZE;

  tx_active = true;

  /* Transmitir el byte */
  gpio2_write_rmw(GPIO2_DATA_IN_MASK, (uint32_t)(byte & GPIO2_DATA_IN_MASK));
  gpio2_write_rmw(GPIO2_TX_SEND_MASK, GPIO2_TX_SEND_MASK);
  for (volatile int j = 0; j < 100; ++j) __asm__ volatile("nop");
  gpio2_write_rmw(GPIO2_TX_SEND_MASK, 0u);
}

void register_tx_callback(void){
  transceiver_register_tx_callback( (void (*)(void))tx_kick );
}
/* ---- Higher-level send_bytes ---- */
/* Devuelve 0 OK, -1 timeout -- OLD */
// int send_bytes(const uint8_t *buf, size_t len, uint32_t timeout_ms_per_byte) {
//   if (!buf || len == 0) return 0;

//   for (size_t i = 0; i < len; ++i) {
//     uint16_t data9 = (uint16_t)(buf[i] & GPIO2_DATA_IN_MASK);

//     /* 1) escribir DATA_IN (9 bits) y pulsar TX_SEND inmediatamente */
//     gpio2_write_rmw(GPIO2_DATA_IN_MASK, (uint32_t)data9);
//     gpio2_write_rmw(GPIO2_TX_SEND_MASK, GPIO2_TX_SEND_MASK);
//     for (volatile int j = 0; j < 100; ++j) __asm__ volatile("nop");
//     gpio2_write_rmw(GPIO2_TX_SEND_MASK, 0u);

//     /* 2) obtener tiempo estimado de transmisión del fotograma en ms (usamos cache si existe) */
//     uint32_t frame_time_ms = cached_frame_time_ms;
//     if (frame_time_ms == 0) {
//       /* si no está cacheado, calculamos (pero también actualizamos cache) */
//       transceiver_calc_frame_time_from_regs();
//       frame_time_ms = cached_frame_time_ms;
//     }
//     rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(frame_time_ms));

//     /* 3) comprobar TX_RDY; si está listo -> siguiente byte */
//     if (gpio3_read_raw() & PS_OUT_TX_RDY_MASK) {
//       /* listo, siguiente iteración (siguiente byte) */
//       continue;
//     }

//     /* 4) Si no está listo, fallback híbrido: busy-wait corto + sleep por pasos */
//     {
//       const unsigned AFTER_WAKE_NOP_ITERS = 2000u; /* busy-wait corto tras cada wake (tuneable) */
//       const uint32_t STEP_MS = 1u;

//       int ready = 0;

//       if (timeout_ms_per_byte == 0) {
//         /* espera indefinida: combinamos ventanas de busy-wait y sleeps para no saturar CPU */
//         for (;;) {
//           /* busy-wait corto */
//           for (unsigned k = 0; k < AFTER_WAKE_NOP_ITERS; ++k) {
//             if (gpio3_read_raw() & PS_OUT_TX_RDY_MASK) { ready = 1; break; }
//             __asm__ volatile("nop");
//           }
//           if (ready) break;

//           /* dormir 1 ms (no bloquea CPU intensivamente) */
//           rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(STEP_MS));
//         }
//       } else {
//         uint32_t waited = 0;

//         /* primer intento con busy-wait corto */
//         for (unsigned k = 0; k < AFTER_WAKE_NOP_ITERS; ++k) {
//           if (gpio3_read_raw() & PS_OUT_TX_RDY_MASK) { ready = 1; break; }
//           __asm__ volatile("nop");
//         }
//         if (!ready) {
//           while (!ready) {
//             if (waited >= timeout_ms_per_byte) {
//               return -1; /* timeout */
//             }

//             /* dormir 1 ms */
//             rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(STEP_MS));
//             waited += STEP_MS;

//             /* busy-wait corto tras despertar para atrapar la transición rápido */
//             for (unsigned k = 0; k < AFTER_WAKE_NOP_ITERS; ++k) {
//               if (gpio3_read_raw() & PS_OUT_TX_RDY_MASK) { ready = 1; break; }
//               __asm__ volatile("nop");
//             }
//           }
//         }
//       }

//       /* cuando salimos del bloque, 'ready' es verdadero (TX_RDY) */
//       (void)ready;
//     }

//     /* 5) TX_RDY detectado -> continuar con el siguiente byte */
//   }

//   return 0;
// }
/* new with interrupts */
int send_bytes(const uint8_t *buf, size_t len) {
  if (!buf || len == 0) return 0;

  for (size_t i = 0; i < len; ++i) {
    size_t next_head = (tx_head + 1) % TX_BUF_SIZE;

    if (next_head == tx_tail) {
      // Buffer lleno
      return -1;
    }

    tx_buf[tx_head] = buf[i];
    tx_head = next_head;
  }

  if ((gpio3_read_raw() & PS_OUT_TX_RDY_MASK) && !tx_active) {
    tx_kick();  // lanzar la transmisión si está libre
  }

  return 0;
}


/* Envoltorio para strings C null-terminated (no envía el '\0') */
int send_string(const char *s)
{
  if (!s) return -1;
  return send_bytes((const uint8_t *)s, strlen(s));
}


/* Helpers de cola (protegidos por rx_mutex) */
static void rx_lock(void) { rtems_semaphore_obtain(rx_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT); }
static void rx_unlock(void) { rtems_semaphore_release(rx_mutex); }

static void rx_queue_push_one(uint8_t b) {
    rx_lock();
    if (rx_count < RX_QUEUE_SIZE) {
        rx_queue[rx_tail] = b;
        rx_tail = (rx_tail + 1) % RX_QUEUE_SIZE;
        rx_count++;
    }
    rx_unlock();
}
/* API Exportada: Vaciar Buffer (Leer) */
size_t transceiver_rx_read(uint8_t *buf, size_t maxlen) {
    size_t got = 0;
    rx_lock();
    while (got < maxlen && rx_count > 0) {
        buf[got++] = rx_queue[rx_head];
        rx_head = (rx_head + 1) % RX_QUEUE_SIZE;
        rx_count--;
    }
    rx_unlock();
    return got;
}

static size_t rx_queue_pop_bytes(uint8_t *buf, size_t maxlen) {
  size_t got = 0;
  rx_lock();
  while (got < maxlen && rx_count > 0) {
    buf[got++] = rx_queue[rx_head];
    rx_head = (rx_head + 1) % RX_QUEUE_SIZE;
    rx_count--;
  }
  rx_unlock();
  return got;
}

/* tarea de polling: lee GPIO3 regularmente y encola bytes */
// static rtems_task rx_poll_task(rtems_task_argument arg) {
//   uint32_t poll_interval_ms = (uint32_t)arg;
//   uint8_t tmp_buf[256];

//   rx_poll_running = 1;
//   for (;;) {
//     /* comprobar si hay datos en el bloque PL (EMPTY == 0 => hay dato) */
//     uint32_t g3 = gpio3_read_raw();
//     if ((g3 & PS_OUT_EMPTY_MASK) == 0) {
//       /* leer en bucle todos los bytes disponibles */
//       size_t local_count = 0;
//       while ((g3 & PS_OUT_EMPTY_MASK) == 0) {
//         uint8_t b = (uint8_t)(g3 & PS_OUT_DATA_MASK);
//         if (rx_queue_push_one(b)) {
//           local_count++;
//         } else {
//           /* cola llena: descartamos nuevo byte (podemos cambiar política) */
//         }
//         /* notificar al PL que hemos consumido el dato */
//         fifo_consume_pulse();

//         /* leer siguiente estado */
//         g3 = gpio3_read_raw();
//       }
      
//       /* si hay callback, sacar hasta tmp_buf y llamar (ejecuta en contexto de esta tarea) */
//       if (rx_callback) {
//         size_t n = rx_queue_pop_bytes(tmp_buf, sizeof(tmp_buf));
//         if (n > 0) {
//           rx_callback(tmp_buf, n, rx_callback_arg);
//         }
//       }
//     }

//     /* dormir poll_interval_ms (no bloquea CPU) */
//     rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(poll_interval_ms));
//   }

//   /* nunca debería llegar aquí */
//   rx_poll_running = 0;
//   rtems_task_delete(RTEMS_SELF);
// }

/* API: inicia polling RX */
// int transceiver_rx_init_polling(uint32_t poll_interval_ms) {
//   rtems_status_code sc;

//   /* crear mutex si no existe */
//   if (rx_mutex == 0) {
//     sc = rtems_semaphore_create(rtems_build_name('R','X','M','X'),
//                                 1, /* mutex inicial = 1 */
//                                 RTEMS_PRIORITY | RTEMS_BINARY_SEMAPHORE,
//                                 0, &rx_mutex);
//     if (sc != RTEMS_SUCCESSFUL) return -1;
//   }

//   /* crear semáforo de notificación si lo quieres (no usado actualmente) */
//   if (rx_sem == 0) {
//     sc = rtems_semaphore_create(rtems_build_name('R','X','S','M'),
//                                 0,
//                                 RTEMS_PRIORITY | RTEMS_BINARY_SEMAPHORE,
//                                 0, &rx_sem);
//     if (sc != RTEMS_SUCCESSFUL) return -1;
//   }

//   /* crear tarea polling */
//   sc = rtems_task_create(rtems_build_name('R','X','P','L'),
//                          100, RTEMS_MINIMUM_STACK_SIZE * 2,
//                          RTEMS_DEFAULT_MODES, RTEMS_DEFAULT_ATTRIBUTES, &rx_poll_tid);
//   if (sc != RTEMS_SUCCESSFUL) return -1;

//   /* arrancar la tarea pasando poll_interval_ms como argumento */
//   sc = rtems_task_start(rx_poll_tid, rx_poll_task, (rtems_task_argument)poll_interval_ms);
//   if (sc != RTEMS_SUCCESSFUL) return -1;

//   return 0;
// }

static void rx_read_store_callback(void){
  //printf("rx_read_store_callback\n");
    uint32_t g3 = gpio3_read_raw();
    if ((g3 & PS_OUT_EMPTY_MASK) == 0) {
        uint8_t b = (uint8_t)(g3 & PS_OUT_DATA_MASK);
        rx_queue_push_one(b);
        fifo_consume_pulse();
    }
    uint8_t tmp_buf[256];
    /* si hay callback, sacar hasta tmp_buf y llamar (ejecuta en contexto de esta tarea) */
    if (rx_callback) {
      size_t n = rx_queue_pop_bytes(tmp_buf, sizeof(tmp_buf));
      if (n > 0) {
        //printf("rx_callback called\n");
        rx_callback(tmp_buf, n, rx_callback_arg);
      }
    }
}
/* TAREA WORKER (Bottom Half) */
static rtems_task rx_interrupt_worker_task(rtems_task_argument arg) {
    (void)arg;
    rtems_event_set events;

    /* Nos aseguramos de habilitar la interrupción al nacer */
    transceiver_int_enable_rx(true);

    for (;;) {
        /* 1. DORMIR esperando el evento de la ISR */
        rtems_event_receive(RTEMS_EVENT_0, RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &events);

        /* 2. LEER DATOS: Vaciar FIFO Hardware -> Buffer Software */
        /* Nota: Usamos gpio3_read_raw() y PS_OUT_EMPTY_MASK de tu .h */
        while ((gpio3_read_raw() & PS_OUT_EMPTY_MASK) == 0) {
            uint8_t b = (uint8_t)(gpio3_read_raw() & PS_OUT_DATA_MASK);
            
            rx_queue_push_one(b); /* Guardar en buffer circular */
            fifo_consume_pulse(); /* Avisar al HW que leímos */
        }

        /* 3. AVISAR AL USUARIO (Main) */
        /* El buffer ya tiene los datos, el callback solo avisa "hay datos" */
        if (rx_callback) {
            rx_callback(NULL, 0, rx_callback_arg); 
            /* Pasamos NULL/0 porque el usuario usará transceiver_rx_read para leer todo lo que quiera */
        }

        /* 4. REACTIVAR interrupción para el siguiente paquete */
        transceiver_int_enable_rx(true);
    }
}


void transceiver_rx_init_interrupt(void) {
    rtems_status_code sc;

    if (rx_mutex == 0) {
        rtems_semaphore_create(rtems_build_name('R','X','M','X'), 1, 
                               RTEMS_PRIORITY | RTEMS_BINARY_SEMAPHORE, 0, &rx_mutex);
    }

    sc = rtems_task_create(rtems_build_name('R','X','W','K'),
                           50, RTEMS_MINIMUM_STACK_SIZE * 2,
                           RTEMS_DEFAULT_MODES, RTEMS_DEFAULT_ATTRIBUTES, &rx_worker_tid);

    if (sc == RTEMS_SUCCESSFUL) {
        transceiver_register_rx_worker_id(rx_worker_tid);
        rtems_task_start(rx_worker_tid, rx_interrupt_worker_task, 0);
    }
}
/* Registrar callback (puede ser NULL para desregistrar) */
void transceiver_set_rx_callback(transceiver_rx_cb_t cb, void *arg)
{
  rx_callback = cb;
  rx_callback_arg = arg;
}

/* Consultas/lectura simples */
size_t transceiver_rx_available(void)
{
  size_t c;
  rx_lock();
  c = rx_count;
  rx_unlock();
  return c;
}

// size_t transceiver_rx_read(uint8_t *buf, size_t maxlen)
// {
//   return rx_queue_pop_bytes(buf, maxlen);
// }

/* Shutdown básico (no mata la tarea de forma elegante) */
void transceiver_rx_shutdown(void)
{
  /* Podríamos rtems_task_delete(rx_poll_tid) si queremos forzar stop.
     Por simplicidad aquí solo desactivamos callback y limpiamos buffer. */
  rx_callback = NULL;
  rx_lock();
  rx_head = rx_tail = rx_count = 0;
  rx_unlock();
}

/* EOF */