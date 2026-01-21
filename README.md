# Zynq UltraScale+ Configurable Transceiver BSP

Este proyecto proporciona un **Board Support Package (BSP)** completo (Hardware y Software) para la placa **Xilinx ZCU102**, diseñado para gestionar múltiples transceptores serie configurables desde RTEMS 7.

El sistema permite la instanciación dinámica de hasta 14 transceptores UART personalizados en la FPGA (PL), controlados por el sistema de procesamiento (PS) a través de un driver optimizado.

## Estructura del Repositorio

```text
.
├── hardware/                 # Diseño de Vivado (Scripts y Fuentes)
│   ├── src/                  # VHDL (IPs, Top level) y Constraints (ZCU102)
│   └── scripts/              # Scripts TCL para generar el proyecto y Block Design
├── wscript                   # Script de construcción Waf para la app RTEMS
├── make_img.sh               # Script helper para compilar y generar la imagen
├── automate_ymodem_update.py # Herramienta de actualización de firmware (PC -> SD Card)
├── *.c / *.h                 # Código fuente de la aplicación y el driver
└── README.md                 # Este archivo

```

## 1. Requisitos del Sistema

### Hardware

* **Placa:** Xilinx Zynq UltraScale+ MPSoC ZCU102.
* **Almacenamiento:** Tarjeta SD formateada con una partición FAT (arranque).
* **Conexión:** Cable USB-UART (para consola y carga de binarios).

### Software

* **Vivado Design Suite 2025.1** (Requerido para compatibilidad con scripts TCL).
* **RTEMS 7 Toolchain** (Compilador `aarch64-rtems7-gcc`).
* **Python 3** (Para scripts de automatización).
* **Git** (Para gestión de submódulos).

## 2. Generación del Hardware (PL)

El diseño se genera mediante scripts TCL que crean el proyecto, el Block Design y mapean la memoria automáticamente.

1. Abra Vivado 2025.1.
2. En la consola TCL, navegue a `hardware/scripts` y ejecute:
```tcl
source rebuild_all.tcl

```


> **Nota:** El script por defecto utiliza el part number configurado en `rebuild_all.tcl`. Si utiliza una ZCU102, asegúrese de que el proyecto coincida con su hardware (`xczu9eg...`) o utilice el constraint file `zcu102_constraints.xdc` incluido.


3. El script `generate_transceivers.tcl` se encargará de:
* Crear 14 instancias de transceptores.
* Configurar el AXI SmartConnect y las interrupciones.
* **Fijar las direcciones de memoria** (ver sección Mapa de Memoria).


4. Genere el **Bitstream** y exporte el archivo `.xsa` para generar el `BOOT.bin` (fsbl + bitstream + u-boot).

## 3. Compilación del Software (PS)

El proyecto utiliza el sistema de construcción **Waf** y depende del submódulo `rtems_waf`.

### Paso 1: Obtener submódulos

Si clonó el repo sin `--recursive`, inicialice `rtems_waf` (si no sabe si lo tiene, ejecute esto por seguridad):

```bash
git submodule update --init --recursive

```

### Paso 2: Compilar

Puede usar el script automático que configura, compila y genera la imagen comprimida:

```bash
# Ajuste la ruta a su toolchain si es necesario (por defecto busca en ~/quick-start/rtems/7)
export RTEMS_PREFIX=$HOME/quick-start/rtems/7
./make_img.sh

```

Esto generará el archivo `rtems.img` (formato U-Boot Legacy).

## 4. Instalación y Actualización (Boot)

El flujo de trabajo principal es arrancar desde la **Tarjeta SD**.

### Método A: Copia Manual

Copie el archivo `rtems.img` (y `BOOT.bin` si cambió el hardware) a la raíz de la tarjeta SD y reinicie la placa.

### Método B: Actualización Automática (YMODEM)

Para agilizar el desarrollo, utilice el script de Python incluido. Este script envía el nuevo firmware por puerto serie a la RAM de la placa y utiliza comandos de U-Boot para escribirlo en la tarjeta SD automáticamente.

**Uso:**

```bash
# Ejemplo: Actualizar rtems.img en la SD conectada a /dev/ttyUSB0
python3 automate_ymodem_update.py

```

*El script interrumpirá el autoboot, transferirá el fichero y ejecutará `fatwrite` para actualizar la SD.*

## 5. Mapa de Memoria y API

El hardware está mapeado en una dirección fija definida en `generate_transceivers.tcl`.

| Componente | Dirección Base | Descripción |
| --- | --- | --- |
| **UARTs Base** | `0xA0000000` | Inicio del bloque de transceptores |
| **Transceiver *n*** | `Base + (n * 0x1000)` | Registros de configuración/datos del canal *n* |
| **INTC Global** | `0xA000E000` | Controlador de interrupciones (si hay 14 canales) |
| **SysInfo** | `0xA0020000` | Bloque de metadatos (Magic Number, Count, Stride) |

### Uso del Driver (`transceiver.h`)

El driver lee automáticamente la configuración desde el bloque **SysInfo**, por lo que no es necesario hardcodear direcciones en el código de usuario si se usa la función de descubrimiento.

```c
#include "transceiver.h"

// 1. Inicialización global (detecta hardware y configura interrupciones)
uint32_t count = Transceiver_INIT();
printf("Detectados %d transceptores en FPGA\n", count);

// 2. Configurar un canal específico (ej. Canal 0 a 9600 baudios)
Transceiver dev;
Transceiver_Config_t config = {
    .baud = TRANSCEIVER_BAUD_9600,
    .data_bits = TRANSCEIVER_DATA_BITS_8,
    .parity = TRANSCEIVER_PARITY_NONE,
    .stop_bits = TRANSCEIVER_STOP_BITS_1,
    .bit_order = TRANSCEIVER_BIT_ORDER_LSB
};

Transceiver_Init(&dev, 0, &config);

// 3. Enviar datos
Transceiver_SendString(&dev, "Prueba ZCU102\n");

```

