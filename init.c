/* init.c — configuración robusta para ZynqMP */

#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER    /* necesario para wake_after */
#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER  /* consola/UART para printf */

/* Tick: 10 ms (100 Hz). Puedes variar si quieres. */
#define CONFIGURE_MICROSECONDS_PER_TICK 10000

/* Recursos y memoria */
#define CONFIGURE_UNLIMITED_OBJECTS
#define CONFIGURE_UNIFIED_WORK_AREAS
#define CONFIGURE_MAXIMUM_FILE_DESCRIPTORS 32

/* Seguridad extra: más stack y chequeo de stack */
#define CONFIGURE_INIT_TASK_STACK_SIZE   (64 * 1024)
#define CONFIGURE_EXTRA_TASK_STACKS      (256 * 1024)
#define CONFIGURE_STACK_CHECKER_ENABLED

/* Si usas printf con floats en el futuro, esto ayuda */
#define CONFIGURE_INIT_TASK_ATTRIBUTES   RTEMS_FLOATING_POINT

/* Tarea Init declarada por el usuario */
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_INIT

#include <rtems/printer.h>
#include <rtems/bspIo.h>   /* rtems_printk */
#include <rtems/confdefs.h>
