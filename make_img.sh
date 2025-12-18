#!/usr/bin/env bash
set -euo pipefail

# --- Auto-detección del directorio del script y nombre de la app ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Si APP_DIR está exportada desde el entorno, la respetamos; si no, la fijamos al directorio del script
APP_DIR="${APP_DIR:-$SCRIPT_DIR}"

# Si APP_NAME no está exportada, la tomamos del nombre del directorio (por ejemplo configurable_transceiver_piton_app)
APP_NAME="${APP_NAME:-$(basename "$APP_DIR")}"
EXE_NAME="${EXE_NAME:-${APP_NAME}.exe}"

# --- Config (RTEMS_PREFIX puede venir del entorno) ---
RTEMS_PREFIX="${RTEMS_PREFIX:-$HOME/quick-start/rtems/7}"
export PATH="$RTEMS_PREFIX/bin:$PATH"

# Informativo
echo "[INFO] APP_DIR = $APP_DIR"
echo "[INFO] APP_NAME = $APP_NAME"
echo "[INFO] EXE_NAME = $EXE_NAME"
echo "[INFO] RTEMS_PREFIX = $RTEMS_PREFIX"

# --- rtems_waf opcional (si tu repo lo usa) ---
if [ ! -d "$APP_DIR/rtems_waf" ] && [ -f "$APP_DIR/.gitmodules" ]; then
  echo "[INFO] Inicializando submódulos (rtems_waf si aplica)..."
  (cd "$APP_DIR" && git submodule update --init --recursive) || true
fi

# --- Build con detección de 'not configured' ---
echo "[INFO] Ejecutando waf build en $APP_DIR ..."
set +e
WAF_OUT="$(cd "$APP_DIR" && ./waf 2>&1)"
WAF_RC=$?
set -e

if [ $WAF_RC -ne 0 ]; then
  if echo "$WAF_OUT" | grep -qi "not configured"; then
    echo "[WARN] Proyecto no configurado. Ejecutando './waf configure' (autodetección BSP)..."
    (cd "$APP_DIR" && ./waf configure)
    echo "[INFO] Reintentando compilación..."
    (cd "$APP_DIR" && ./waf)
  else
    echo "[ERROR] waf falló:"
    echo "$WAF_OUT"
    exit 1
  fi
fi

# --- Crear imagen binaria comprimida ---
BIN_FILE="${APP_NAME}.bin"
IMG_FILE="rtems.img"

echo "[INFO] Buscando ejecutable '${EXE_NAME}' en $APP_DIR/build ..."
# Intentamos localizar el ejecutable dentro del árbol build/
EXE_PATH="$(find "$APP_DIR/build" -type f -name "$EXE_NAME" -print -quit || true)"

if [ -z "$EXE_PATH" ]; then
  echo "[ERROR] No se encontró '${EXE_NAME}' en $APP_DIR/build. Contenido de build/:"
  ls -R "$APP_DIR/build" || true
  exit 1
fi

echo "[INFO] Ejecutable encontrado: $EXE_PATH"

echo "[INFO] Generando binario..."
aarch64-rtems7-objcopy -Obinary "$EXE_PATH" "${BIN_FILE}"
gzip -9f "${BIN_FILE}"

if ! command -v mkimage >/dev/null; then
  echo "[ERROR] mkimage no está instalado (paquete 'u-boot-tools')."
  exit 1
fi

echo "[INFO] Creando ${IMG_FILE}..."
mkimage -A arm64 -O rtems -T kernel -a 0x10000 -e 0x10000 \
  -n RTEMS -d "${BIN_FILE}.gz" "${IMG_FILE}"

echo "[OK] Hecho: ${IMG_FILE}"
