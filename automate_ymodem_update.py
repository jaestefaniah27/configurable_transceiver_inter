#!/usr/bin/env python3
import os, sys, time, re, subprocess, signal, argparse
from pathlib import Path

# ---------- Ajustes ----------
PORT = os.environ.get("RTEMS_SERIAL", "/dev/ttyUSB0")         # tu Silicon Labs
BAUD = int(os.environ.get("RTEMS_BAUD", "115200"))
APP_DIR = Path(__file__).resolve().parent
MAKE = APP_DIR / "make_img.sh"
IMG  = APP_DIR / "rtems.img"

LOAD_ADDR = os.environ.get("RTEMS_LOADADDR", "0x800000")      # alto para evitar solapes
FAT_DEV   = os.environ.get("RTEMS_FATDEV", "mmc 0:1")         # partición FAT
EOL = os.environ.get("RTEMS_EOL", "\n")                       # Line ending LF

# Tiempos
T_AUTOboot = 20.0                                              # ventana de autoboot
T_PROMPT   = 2.5                                               # espera rápida al prompt
T_YMODEM   = 600.0                                             # margen envío YMODEM
READ_SLICE = 0.02                                              # 20 ms por ciclo
# -----------------------------

def run(cmd, **kw):
    print(f"[cmd] {cmd}")
    return subprocess.run(cmd, shell=True, check=True, **kw)

def make_image():
    if not MAKE.exists():
        print(f"ERROR: no existe {MAKE}")
        sys.exit(1)
    run(f"cd '{APP_DIR}' && chmod +x '{MAKE}' && '{MAKE}'")
    if not IMG.exists():
        print(f"ERROR: no se generó {IMG}")
        sys.exit(1)

def open_serial():
    import serial
    # timeout=0 => lectura no bloqueante (reacciona rápido)
    ser = serial.Serial(PORT, BAUD, timeout=0, write_timeout=1)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser

def read_available(ser):
    """Lee todo lo disponible ahora mismo (no bloqueante)."""
    out = b""
    while True:
        n = ser.in_waiting
        if n <= 0:
            break
        out += ser.read(n)
        time.sleep(0.001)
    return out

def wait_for(ser, pattern, timeout, echo=True):
    """Espera regex 'pattern' hasta 'timeout'."""
    regex = re.compile(pattern, re.S)
    buf   = ""
    end   = time.time() + timeout
    while time.time() < end:
        data = read_available(ser)
        if data:
            s = data.decode(errors="ignore")
            buf += s
            if echo and s:
                print(s, end="", flush=True)
            if regex.search(buf):
                return buf
        time.sleep(READ_SLICE)
    raise TimeoutError(f"No apareció el patrón {pattern!r} en {timeout}s")

def send_line(ser, line):
    ser.write((line + EOL).encode())
    ser.flush()

def wait_prompt_once(ser, timeout):
    """Espera SOLO el PRIMER prompt y retorna al instante."""
    regex = re.compile(r"(?:^|\n)\s*(?:=>|Zynq-?MP>)\s*$", re.S)
    end = time.time() + timeout
    buf = ""
    while time.time() < end:
        data = read_available(ser)
        if data:
            s = data.decode(errors="ignore")
            buf += s
            print(s, end="", flush=True)  # eco
            if regex.search(buf):
                return
        time.sleep(READ_SLICE)
    raise TimeoutError("No apareció el prompt en tiempo")

def interrupt_autoboot(ser):
    """
    Si vemos 'Hit any key...' enviamos 1 espacio y esperamos SOLO el primer prompt.
    Si ya hay prompt, seguimos sin enviar nada.
    """
    try:
        buf = wait_for(
            ser,
            r"Hit any key to stop autoboot:|(?:^|\n)\s*(?:=>|Zynq-?MP>)\s*$",
            T_AUTOboot,
            echo=True
        )
        if re.search(r"(?:^|\n)\s*(?:=>|Zynq-?MP>)\s*$", buf):
            return
        ser.write(b" "); ser.flush()      # cortamos autoboot con UN espacio
        wait_prompt_once(ser, 2.0)
    except TimeoutError:
        send_line(ser, "")
        wait_prompt_once(ser, T_PROMPT)

def send_cmd(ser, cmd, wait_prompt=True, timeout=3.0):
    """Envía comando y, si se pide, espera SOLO el primer prompt."""
    print(f"[uboot] {cmd}")
    send_line(ser, cmd)
    if wait_prompt:
        wait_prompt_once(ser, timeout)

def ymodem_send(file_path: Path):
    """
    Envía file_path por YMODEM usando 'sb' (lrzsz).
    Necesita que el puerto esté cerrado (lo abrimos/cerramos en el flujo).
    """
    # usar ruta absoluta para sb
    p = str(file_path)
    # sb escribe/lee desde el dispositivo de serie; redirigimos ambos lados
    run(f"sb -vv --ymodem '{p}' < {PORT} > {PORT}", timeout=T_YMODEM)

def monitor_stream(ser):
    """Deja el puerto abierto mostrando salida hasta Ctrl+C."""
    print("\n--- Monitor serie activo (Ctrl+C para salir) ---\n")
    try:
        while True:
            data = read_available(ser)
            if data:
                try:
                    print(data.decode(errors="ignore"), end="", flush=True)
                except Exception:
                    pass
            time.sleep(READ_SLICE)
    except KeyboardInterrupt:
        print("\n[monitor] Cancelado por usuario (Ctrl+C). Cerrando puerto…")

def fatwrite_via_ymodem(ser, file_path: Path, target_name: str):
    """
    Secuencia completa para enviar un fichero por YMODEM y escribirlo en la FAT con el nombre target_name.
    - envía 'loady LOAD_ADDR'
    - espera los mensajes de 'Ready'
    - cierra el puerto y ejecuta sb para enviar file_path
    - reabre el puerto, espera prompt y ejecuta 'fatwrite FAT_DEV LOAD_ADDR <target_name> ${filesize}'
    """
    print(f"[i] Preparando envío de '{file_path}' como '{target_name}'")
    send_cmd(ser, f"loady {LOAD_ADDR}", wait_prompt=False, timeout=2.0)

    # U-Boot suele imprimir estas marcas al entrar en YMODEM
    try:
        wait_for(ser, r"(?:## Ready for binary|Ready for binary|C{1,})", 3.0, echo=True)
    except TimeoutError:
        pass

    # cerrar puerto y enviar por sb/ymodem
    ser.close()
    print(f"[i] Enviando '{file_path}' por YMODEM…")
    ymodem_send(file_path)

    # reabrir puerto y esperar prompt para fatwrite
    ser = open_serial()
    wait_prompt_once(ser, 10.0)
    send_cmd(ser, f"fatwrite {FAT_DEV} {LOAD_ADDR} {target_name} ${{filesize}}", wait_prompt=True, timeout=20.0)
    return ser

def main():
    parser = argparse.ArgumentParser(description="Enviar rtems.img (y opcionalmente BOOT.bin) por YMODEM y escribir en FAT desde U-Boot.")
    parser.add_argument("-b", "--boot", nargs="?", const="./BOOT.bin", help="Incluir BOOT.bin. Opcionalmente pasar ruta. Si se usa sin ruta, se intenta ./BOOT.bin")
    args = parser.parse_args()

    # 1) build imagen
    make_image()

    # Si se pidió boot, comprobar que exista fichero
    boot_path = None
    if args.boot:
        boot_path = Path(args.boot)
        if not boot_path.exists():
            print(f"ERROR: BOOT.bin solicitado pero no existe '{boot_path}'.")
            sys.exit(1)

    # 2) abrir serie y parar autoboot
    try:
        ser = open_serial()
    except Exception as e:
        print(f"ERROR abriendo {PORT}: {e}")
        sys.exit(2)

    print(f"[i] Abierto {PORT} @ {BAUD} (EOL=LF)")
    try:
        interrupt_autoboot(ser)

        # 3) prepara SD (mmc) y cuando toque, modo YMODEM
        send_cmd(ser, "mmc dev 0; mmc rescan", wait_prompt=True, timeout=5.0)

        # Si pedimos BOOT, enviarlo y escribir como 'BOOT.bin'
        if boot_path is not None:
            ser = fatwrite_via_ymodem(ser, boot_path, "BOOT.bin")

        # Ahora enviar rtems.img como antes
        ser = fatwrite_via_ymodem(ser, IMG, "rtems.img")

        # Tras escribir rtems.img, arrancamos desde la carga
        send_cmd(ser, f"bootm {LOAD_ADDR}", wait_prompt=False, timeout=2.0)

        # 6) monitor serie vivo hasta Ctrl+C
        monitor_stream(ser)
        ser.close()

    except Exception as e:
        try:
            ser.close()
        except Exception:
            pass
        print(f"ERROR: {e}")
        sys.exit(3)

if __name__ == "__main__":
    # salida limpia al SIGINT/SIGTERM
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    main()
