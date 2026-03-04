import re
import os

# =========================================================================
# 1. EL DICCIONARIO DE LA VERDAD (Relleno con UG1182)
# =========================================================================
ZCU102_PINMAP = {
    # --- PMOD0 (J55) - Mapeo completo oficial ---
    "A20": {"conector": "J55", "pin": 1},
    "B21": {"conector": "J55", "pin": 2},
    "B20": {"conector": "J55", "pin": 3},
    "C21": {"conector": "J55", "pin": 4},
    "J55_GND1": {"conector": "J55", "pin": 5, "tipo_fijo": "GND"},
    "J55_VCC1": {"conector": "J55", "pin": 6, "tipo_fijo": "VCC"},
    "A22": {"conector": "J55", "pin": 7},
    "C22": {"conector": "J55", "pin": 8},
    "A21": {"conector": "J55", "pin": 9},
    "D21": {"conector": "J55", "pin": 10},
    "J55_GND2": {"conector": "J55", "pin": 11, "tipo_fijo": "GND"},
    "J55_VCC2": {"conector": "J55", "pin": 12, "tipo_fijo": "VCC"},

    # --- PMOD1 (J87) - Mapeo completo oficial ---
    "D20": {"conector": "J87", "pin": 1},
    "F20": {"conector": "J87", "pin": 2},
    "E20": {"conector": "J87", "pin": 3},
    "G20": {"conector": "J87", "pin": 4},
    "J87_GND1": {"conector": "J87", "pin": 5, "tipo_fijo": "GND"},
    "J87_VCC1": {"conector": "J87", "pin": 6, "tipo_fijo": "VCC"},
    "D22": {"conector": "J87", "pin": 7},
    "J20": {"conector": "J87", "pin": 8},
    "E22": {"conector": "J87", "pin": 9},
    "J19": {"conector": "J87", "pin": 10},
    "J87_GND2": {"conector": "J87", "pin": 11, "tipo_fijo": "GND"},
    "J87_VCC2": {"conector": "J87", "pin": 12, "tipo_fijo": "VCC"},

    # --- HEADER J3 (Pines pares confirmados por tu XDC anterior) ---
    "H14": {"conector": "J3", "pin": 6},
    "J14": {"conector": "J3", "pin": 8},
    "G14": {"conector": "J3", "pin": 10},
    "G15": {"conector": "J3", "pin": 12},
    "J15": {"conector": "J3", "pin": 14},
    "J16": {"conector": "J3", "pin": 16},
    "G16": {"conector": "J3", "pin": 18},
    "H16": {"conector": "J3", "pin": 20},
    "G13": {"conector": "J3", "pin": 22},
    "H13": {"conector": "J3", "pin": 24},
}

# Configuración de los conectores
CONECTORES_CONFIG = {
    "J55 (PMOD0)": {"id": "J55", "total_pines": 12, "columnas": 2}, 
    "J87 (PMOD1)": {"id": "J87", "total_pines": 12, "columnas": 2},
    "J3 (Prototype)": {"id": "J3", "total_pines": 40, "columnas": 2}
}

# =========================================================================
# 2. PARSER DEL XDC
# =========================================================================
def parse_xdc(ruta_xdc):
    pines_asignados = {}
    regex = r"set_property\s+PACKAGE_PIN\s+([A-Z0-9]+)\s+\[get_ports\s+\{([^}]+)\}\]"
    try:
        with open(ruta_xdc, 'r') as f:
            for linea in f:
                match = re.search(regex, linea)
                if match:
                    pines_asignados[match.group(1)] = match.group(2)
        return pines_asignados
    except FileNotFoundError:
        print(f"⚠️ Aviso: No se encontró {ruta_xdc}. Se generará el mapa vacío.")
        return {}

# =========================================================================
# 3. MOTOR DE RENDERIZADO HTML/CSS
# =========================================================================
def generar_html(pines_xdc, archivo_salida="visor_zcu102.html"):
    html = """
    <!DOCTYPE html>
    <html lang="es">
    <head>
        <meta charset="UTF-8">
        <title>ZCU102 Pinout Viewer</title>
        <style>
            body { font-family: 'Segoe UI', sans-serif; background-color: #121212; color: #eee; padding: 20px; }
            h1 { text-align: center; color: #64b5f6; margin-bottom: 40px; }
            .board { display: flex; flex-wrap: wrap; gap: 40px; justify-content: center; }
            .connector { background-color: #1e1e1e; padding: 20px; border-radius: 8px; border: 1px solid #333; box-shadow: 0 8px 16px rgba(0,0,0,0.6); min-width: 250px; }
            .connector h2 { margin-top: 0; text-align: center; font-size: 1.1rem; color: #aaa; border-bottom: 2px solid #333; padding-bottom: 10px; }
            .pin-grid { display: grid; gap: 6px; }
            
            /* Estilos de pines */
            .pin { padding: 8px; text-align: center; border-radius: 4px; font-size: 0.85rem; border: 1px solid #444; background-color: #2c2c2c; color: #888; display: flex; flex-direction: column; justify-content: center;}
            .pin-num { font-size: 0.7rem; color: #aaa; margin-bottom: 2px; }
            .pin-label { font-weight: bold; }
            
            /* Colores automáticos */
            .pin.gnd { background-color: #000000; border-color: #222; color: #fff; }
            .pin.vcc { background-color: #b71c1c; border-color: #e53935; color: #fff; }
            .pin.tx  { background-color: #1b5e20; border-color: #4caf50; color: #fff; }
            .pin.rx  { background-color: #0d47a1; border-color: #1e88e5; color: #fff; }
            .pin.de  { background-color: #e65100; border-color: #fb8c00; color: #fff; }
            .pin.slo { background-color: #4a148c; border-color: #8e24aa; color: #fff; }
            .pin.active { background-color: #37474f; color: #fff; border-color: #546e7a; } /* Para otros pines genéricos en el XDC */
        </style>
    </head>
    <body>
        <h1>Visor Interactivo de Pines - ZCU102</h1>
        <div class="board">
    """

    for nombre_display, config in CONECTORES_CONFIG.items():
        id_conector = config["id"]
        html += f'<div class="connector"><h2>{nombre_display}</h2><div class="pin-grid" style="grid-template-columns: repeat({config["columnas"]}, 1fr);">'
        
        for i in range(1, config["total_pines"] + 1):
            clase_css = ""
            label = "NC" # Not Connected por defecto
            pin_fpga_asociado = ""
            
            # Buscar configuración en el diccionario
            for pin_fpga, data_fisica in ZCU102_PINMAP.items():
                if data_fisica["conector"] == id_conector and data_fisica["pin"] == i:
                    
                    # 1. ¿Es un pin fijo de alimentación/masa?
                    if "tipo_fijo" in data_fisica:
                        if data_fisica["tipo_fijo"] == "GND":
                            label, clase_css = "GND", "gnd"
                        elif data_fisica["tipo_fijo"] == "VCC":
                            label, clase_css = "3.3V", "vcc"
                        break
                        
                    # 2. ¿Es un pin normal de FPGA? Buscamos en el XDC
                    pin_fpga_asociado = pin_fpga
                    if pin_fpga in pines_xdc:
                        label = pines_xdc[pin_fpga]
                        if "TX" in label: clase_css = "tx"
                        elif "RX" in label: clase_css = "rx"
                        elif "DE" in label: clase_css = "de"
                        elif "SLO" in label: clase_css = "slo"
                        else: clase_css = "active"
                    else:
                        label = f"Libre ({pin_fpga})"
                    break
            
            # Formatear el texto a mostrar
            display_text = f'<span class="pin-num">Pin {i} {f"[{pin_fpga_asociado}]" if pin_fpga_asociado else ""}</span><span class="pin-label">{label}</span>'
            
            html += f'<div class="pin {clase_css}">{display_text}</div>'
        
        html += '</div></div>'

    html += """
        </div>
    </body>
    </html>
    """

    with open(archivo_salida, 'w', encoding='utf-8') as f:
        f.write(html)
    print(f"✅ Archivo {archivo_salida} generado. Los PMODs están listos con alimentación y masa.")

if __name__ == "__main__":
    ARCHIVO_XDC = "zcu102_constraints.xdc" 
    pines_parseados = parse_xdc(ARCHIVO_XDC)
    generar_html(pines_parseados)