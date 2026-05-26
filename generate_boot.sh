#!/usr/bin/env bash
#===============================================================================
#  generate_boot.sh  —  Wizard completo: Vivado → Vitis → BOOT.bin (ZCU102)
#
#  Flujo interactivo que te guía desde el proyecto de Vivado hasta BOOT.bin.
#  Tres puntos de entrada:
#    1) Generar bitstream + exportar XSA + crear plataforma + FSBL + BOOT.bin
#    2) Solo exportar XSA (bitstream ya existe) + crear plataforma + FSBL + BOOT.bin
#    3) Usar un XSA ya exportado + crear plataforma + FSBL + BOOT.bin
#
#  Uso:
#    ./generate_boot.sh                   # Wizard interactivo (recomendado)
#    ./generate_boot.sh --help            # Ayuda
#===============================================================================

set -euo pipefail

#----------- COLORES -----------------------------------------------------------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
DIM='\033[2m'
NC='\033[0m'

#----------- RUTAS BASE -------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VITIS_DIR="${SCRIPT_DIR}"
VIVADO_DIR="/home/mpsocv2/vivado_cdhs"
BOOT_COMPONENTS="${VITIS_DIR}/boot_components"

# Xilinx tools
VITIS_BIN="/tools/Xilinx/2025.1/Vitis/bin/vitis"
VIVADO_BIN="/tools/Xilinx/2025.1/Vivado/bin/vivado"
BOOTGEN_PATH="/tools/Xilinx/2025.1/Vitis/bin/bootgen"

# Componentes fijos de boot
PMUFW="${BOOT_COMPONENTS}/pmufw.elf"
BL31="${BOOT_COMPONENTS}/bl31.elf"
UBOOT="${BOOT_COMPONENTS}/u-boot.elf"
SYSTEM_DTB="${BOOT_COMPONENTS}/system.dtb"

#----------- FUNCIONES AUXILIARES ----------------------------------------------

log_info()    { echo -e "${CYAN}[INFO]${NC}  $*"; }
log_ok()      { echo -e "${GREEN}[ OK ]${NC} $*"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_error()   { echo -e "${RED}[ERROR]${NC} $*"; }
log_step()    {
    echo ""
    echo -e "${BOLD}══════════════════════════════════════════════════════${NC}"
    echo -e "${BOLD}  $*${NC}"
    echo -e "${BOLD}══════════════════════════════════════════════════════${NC}"
}

check_file() {
    local filepath="$1"
    local description="$2"
    if [[ ! -f "${filepath}" ]]; then
        log_error "${description} no encontrado: ${filepath}"
        return 1
    fi
    log_ok "${description}: $(basename "${filepath}") ($(du -h "${filepath}" | cut -f1))"
    return 0
}

prompt_confirm() {
    local msg="$1"
    echo -e -n "${BOLD}${msg} [S/n]: ${NC}"
    read -r respuesta
    [[ -z "${respuesta}" || "${respuesta}" =~ ^[sS]$ ]]
}

#----------- VERIFICACIONES ----------------------------------------------------

verify_xilinx_tools() {
    log_step "Verificando herramientas Xilinx"
    local errors=0

    [[ -f "${VIVADO_BIN}" ]]  && log_ok "Vivado: ${VIVADO_BIN}"    || { log_error "Vivado no encontrado"; ((errors++)); }
    [[ -f "${VITIS_BIN}" ]]   && log_ok "Vitis: ${VITIS_BIN}"      || { log_error "Vitis no encontrado"; ((errors++)); }
    [[ -f "${BOOTGEN_PATH}" ]] && log_ok "bootgen: ${BOOTGEN_PATH}" || { log_error "bootgen no encontrado"; ((errors++)); }

    if [[ ${errors} -gt 0 ]]; then
        log_error "Instala Xilinx Vitis/Vivado 2025.1 en /tools/Xilinx/2025.1"
        exit 1
    fi
}

verify_boot_components() {
    local errors=0
    check_file "${PMUFW}"      "pmufw.elf"  || ((errors++))
    check_file "${BL31}"       "bl31.elf"   || ((errors++))
    check_file "${UBOOT}"      "u-boot.elf" || ((errors++))
    check_file "${SYSTEM_DTB}" "system.dtb"  || ((errors++))

    if [[ ${errors} -gt 0 ]]; then
        log_error "Faltan componentes en ${BOOT_COMPONENTS}/"
        log_error "Descárgalos de: github.com/Xilinx/soc-prebuilt-firmware → zcu102-zynqmp"
        exit 1
    fi
}

#==============================================================================
#  PASO 1: ELEGIR PUNTO DE ENTRADA
#==============================================================================

choose_entry_point() {
    echo ""
    echo -e "${BOLD}¿Desde dónde quieres empezar?${NC}"
    echo ""
    echo -e "  ${BOLD}1)${NC} ${CYAN}Proyecto de Vivado${NC} — Generar bitstream + exportar XSA + crear todo"
    echo -e "     ${DIM}(Vivado genera el bitstream, exporta el HW, y luego se crea la plataforma)${NC}"
    echo ""
    echo -e "  ${BOLD}2)${NC} ${CYAN}Proyecto de Vivado (solo exportar XSA)${NC} — El bitstream ya existe"
    echo -e "     ${DIM}(No re-genera bitstream, solo exporta el XSA y busca el .bit existente)${NC}"
    echo ""
    echo -e "  ${BOLD}3)${NC} ${CYAN}Archivo XSA ya exportado${NC} — Solo crear plataforma + FSBL + BOOT.bin"
    echo -e "     ${DIM}(Ya tienes el .xsa listo, necesitas indicar dónde está el .bit)${NC}"
    echo ""
    echo -e -n "${BOLD}Selecciona [1-3]: ${NC}"
    read -r entry_choice

    case "${entry_choice}" in
        1) ENTRY_POINT="vivado_full" ;;
        2) ENTRY_POINT="vivado_xsa_only" ;;
        3) ENTRY_POINT="xsa_existing" ;;
        *) log_error "Opción inválida"; exit 1 ;;
    esac
}

#==============================================================================
#  PASO 2A: SELECCIONAR PROYECTO DE VIVADO
#==============================================================================

select_vivado_project() {
    log_step "Buscando proyectos de Vivado"

    # Buscar .xpr en vivado_cdhs y subdirectorios
    local xpr_files=()
    while IFS= read -r f; do
        xpr_files+=("${f}")
    done < <(find "${VIVADO_DIR}" -maxdepth 3 -name "*.xpr" 2>/dev/null | sort)

    if [[ ${#xpr_files[@]} -eq 0 ]]; then
        log_warn "No se encontraron proyectos .xpr en ${VIVADO_DIR}"
        echo -e -n "  ${BOLD}Introduce la ruta al .xpr: ${NC}"
        read -r XPR_PATH
        if [[ ! -f "${XPR_PATH}" ]]; then
            log_error "Archivo no encontrado: ${XPR_PATH}"
            exit 1
        fi
    elif [[ ${#xpr_files[@]} -eq 1 ]]; then
        XPR_PATH="${xpr_files[0]}"
        log_ok "Proyecto encontrado: ${XPR_PATH}"
    else
        echo -e "${CYAN}Proyectos encontrados:${NC}"
        for i in "${!xpr_files[@]}"; do
            local proj_dir
            proj_dir="$(dirname "${xpr_files[$i]}")"
            local proj_name
            proj_name="$(basename "${xpr_files[$i]}" .xpr)"
            # Buscar bitstreams existentes en este proyecto
            local bit_count
            bit_count=$(find "${proj_dir}" -name "*.bit" 2>/dev/null | wc -l)
            echo -e "  ${BOLD}$((i+1)))${NC} ${proj_name}"
            echo -e "     ${DIM}${xpr_files[$i]}${NC}"
            echo -e "     ${DIM}Bitstreams encontrados: ${bit_count}${NC}"
        done
        echo ""
        echo -e -n "${BOLD}Selecciona proyecto [1-${#xpr_files[@]}]: ${NC}"
        read -r proj_choice
        if [[ "${proj_choice}" =~ ^[0-9]+$ ]] && (( proj_choice >= 1 && proj_choice <= ${#xpr_files[@]} )); then
            XPR_PATH="${xpr_files[$((proj_choice-1))]}"
        else
            log_error "Selección inválida"
            exit 1
        fi
    fi

    XPR_DIR="$(dirname "${XPR_PATH}")"
    XPR_NAME="$(basename "${XPR_PATH}" .xpr)"
    log_ok "Proyecto: ${XPR_NAME} (${XPR_DIR})"
}

#==============================================================================
#  PASO 2B: SELECCIONAR XSA EXISTENTE
#==============================================================================

select_existing_xsa() {
    log_step "Buscando archivos XSA"

    local xsa_files=()
    while IFS= read -r f; do
        xsa_files+=("${f}")
    done < <(find "${VIVADO_DIR}" -maxdepth 3 -name "*.xsa" 2>/dev/null | sort)

    if [[ ${#xsa_files[@]} -eq 0 ]]; then
        log_warn "No se encontraron archivos .xsa en ${VIVADO_DIR}"
        echo -e -n "  ${BOLD}Introduce la ruta al .xsa: ${NC}"
        read -r XSA_PATH
    else
        echo -e "${CYAN}Archivos XSA encontrados:${NC}"
        for i in "${!xsa_files[@]}"; do
            local xsa_size
            xsa_size="$(du -h "${xsa_files[$i]}" | cut -f1)"
            local xsa_date
            xsa_date="$(stat -c '%y' "${xsa_files[$i]}" 2>/dev/null | cut -d. -f1)"
            echo -e "  ${BOLD}$((i+1)))${NC} $(basename "${xsa_files[$i]}")"
            echo -e "     ${DIM}${xsa_files[$i]} (${xsa_size}, ${xsa_date})${NC}"
        done
        echo ""
        echo -e -n "${BOLD}Selecciona XSA [1-${#xsa_files[@]}] (o ruta completa): ${NC}"
        read -r xsa_choice
        if [[ "${xsa_choice}" =~ ^[0-9]+$ ]] && (( xsa_choice >= 1 && xsa_choice <= ${#xsa_files[@]} )); then
            XSA_PATH="${xsa_files[$((xsa_choice-1))]}"
        elif [[ -f "${xsa_choice}" ]]; then
            XSA_PATH="${xsa_choice}"
        else
            log_error "Selección inválida"
            exit 1
        fi
    fi

    if [[ ! -f "${XSA_PATH}" ]]; then
        log_error "XSA no encontrado: ${XSA_PATH}"
        exit 1
    fi
    log_ok "XSA seleccionado: ${XSA_PATH}"
}

#==============================================================================
#  PASO 3: LOCALIZAR / SELECCIONAR BITSTREAM
#==============================================================================

find_or_select_bitstream() {
    log_step "Localizando bitstream (.bit)"

    # Buscar en el directorio del proyecto y en vivado_cdhs
    local search_dirs=("${VIVADO_DIR}")
    [[ -n "${XPR_DIR:-}" ]] && search_dirs=("${XPR_DIR}" "${VIVADO_DIR}")

    local bit_files=()
    for dir in "${search_dirs[@]}"; do
        while IFS= read -r f; do
            # Evitar duplicados
            local dup=false
            for existing in "${bit_files[@]:-}"; do
                [[ "${existing}" == "${f}" ]] && dup=true
            done
            ${dup} || bit_files+=("${f}")
        done < <(find "${dir}" -maxdepth 4 -name "*.bit" 2>/dev/null | sort)
    done

    if [[ ${#bit_files[@]} -eq 0 ]]; then
        log_warn "No se encontraron archivos .bit"
        echo -e -n "  ${BOLD}Introduce la ruta al .bit: ${NC}"
        read -r BIT_PATH
    elif [[ ${#bit_files[@]} -eq 1 ]]; then
        BIT_PATH="${bit_files[0]}"
        log_ok "Bitstream encontrado: ${BIT_PATH}"
        if ! prompt_confirm "¿Usar este bitstream?"; then
            echo -e -n "  ${BOLD}Introduce la ruta al .bit: ${NC}"
            read -r BIT_PATH
        fi
    else
        echo -e "${CYAN}Bitstreams encontrados:${NC}"
        for i in "${!bit_files[@]}"; do
            local bit_size bit_date
            bit_size="$(du -h "${bit_files[$i]}" | cut -f1)"
            bit_date="$(stat -c '%y' "${bit_files[$i]}" 2>/dev/null | cut -d. -f1)"
            # Marcar el de impl_1 como recomendado
            local marker=""
            [[ "${bit_files[$i]}" == *"impl_1"* ]] && marker=" ${GREEN}← impl_1${NC}"
            echo -e "  ${BOLD}$((i+1)))${NC} $(basename "${bit_files[$i]}")${marker}"
            echo -e "     ${DIM}${bit_files[$i]} (${bit_size}, ${bit_date})${NC}"
        done
        echo ""
        echo -e -n "${BOLD}Selecciona bitstream [1-${#bit_files[@]}] (o ruta completa): ${NC}"
        read -r bit_choice
        if [[ "${bit_choice}" =~ ^[0-9]+$ ]] && (( bit_choice >= 1 && bit_choice <= ${#bit_files[@]} )); then
            BIT_PATH="${bit_files[$((bit_choice-1))]}"
        elif [[ -f "${bit_choice}" ]]; then
            BIT_PATH="${bit_choice}"
        else
            log_error "Selección inválida"
            exit 1
        fi
    fi

    if [[ ! -f "${BIT_PATH}" ]]; then
        log_error "Bitstream no encontrado: ${BIT_PATH}"
        exit 1
    fi
    log_ok "Bitstream: ${BIT_PATH} ($(du -h "${BIT_PATH}" | cut -f1))"
}

#==============================================================================
#  PASO 4: PEDIR NOMBRE PARA LA CONFIGURACIÓN
#==============================================================================

ask_config_name() {
    echo ""
    echo -e "${BOLD}Nombre para esta configuración${NC}"
    echo -e "${DIM}  Se usará para nombrar la plataforma y el FSBL.${NC}"
    echo -e "${DIM}  Ejemplo: 'cdhs', 'spi', 'test_pwm'${NC}"
    echo ""
    echo -e -n "${BOLD}Nombre: ${NC}"
    read -r CONFIG_NAME

    if [[ -z "${CONFIG_NAME}" ]]; then
        log_error "El nombre no puede estar vacío"
        exit 1
    fi

    # Sanitizar: solo alfanumérico y guiones bajos
    CONFIG_NAME="${CONFIG_NAME//[^a-zA-Z0-9_]/_}"

    PLATFORM_NAME="platform_${CONFIG_NAME}"
    FSBL_NAME="fsbl_${CONFIG_NAME}"
    FSBL_ELF="${VITIS_DIR}/${FSBL_NAME}/build/${FSBL_NAME}.elf"
    XPFM_PATH="${VITIS_DIR}/${PLATFORM_NAME}/export/${PLATFORM_NAME}/${PLATFORM_NAME}.xpfm"

    log_info "Plataforma: ${PLATFORM_NAME}"
    log_info "FSBL:       ${FSBL_NAME}"
}

#==============================================================================
#  VIVADO: Generar bitstream
#==============================================================================

run_vivado_bitstream() {
    log_step "Vivado — Generando bitstream"

    local vivado_tcl
    vivado_tcl=$(mktemp "${VITIS_DIR}/.vivado_bitstream_XXXXXX.tcl")

    cat > "${vivado_tcl}" << TCLEOF
# Auto-generado por generate_boot.sh
puts "=========================================="
puts " Abriendo proyecto: ${XPR_PATH}"
puts "=========================================="
open_project ${XPR_PATH}

# Lanzar síntesis + implementación + bitstream
puts "Reseteando run anterior para forzar recompilación..."
reset_run synth_1
puts "Lanzando Generate Bitstream (síntesis → impl → bitstream)..."
launch_runs impl_1 -to_step write_bitstream -jobs [exec nproc]
wait_on_run impl_1

# Verificar resultado
set impl_status [get_property STATUS [get_runs impl_1]]
puts "Estado de impl_1: \$impl_status"

if {[string match "*ERROR*" \$impl_status] || [string match "*FAILED*" \$impl_status]} {
    puts "ERROR: La implementación falló"
    exit 1
}

puts ""
puts "=========================================="
puts " Bitstream generado con éxito"
puts "=========================================="

close_project
TCLEOF

    log_info "Ejecutando Vivado en batch (esto puede tardar bastante)..."
    echo ""

    if "${VIVADO_BIN}" -mode batch -source "${vivado_tcl}" -notrace 2>&1 | tee /dev/stderr | tail -1 | grep -qi "error"; then
        log_error "Vivado falló. Revisa los errores arriba."
        rm -f "${vivado_tcl}"
        exit 1
    fi

    rm -f "${vivado_tcl}"
    log_ok "Bitstream generado"

    # Buscar el bitstream recién generado
    local impl_bit
    impl_bit=$(find "${XPR_DIR}" -path "*/impl_1/*.bit" -newer "${XPR_PATH}" 2>/dev/null | head -1)
    if [[ -n "${impl_bit}" ]]; then
        BIT_PATH="${impl_bit}"
        log_ok "Bitstream: ${BIT_PATH}"
    else
        # Fallback: buscar cualquier .bit en impl_1
        impl_bit=$(find "${XPR_DIR}" -path "*/impl_1/*.bit" 2>/dev/null | head -1)
        if [[ -n "${impl_bit}" ]]; then
            BIT_PATH="${impl_bit}"
            log_ok "Bitstream (impl_1): ${BIT_PATH}"
        fi
    fi
}

#==============================================================================
#  VIVADO: Solo exportar XSA
#==============================================================================

run_vivado_export_xsa() {
    log_step "Vivado — Exportando hardware (XSA)"

    # Determinar ruta de salida del XSA
    XSA_PATH="${XPR_DIR}/$(basename "${XPR_DIR}")_wrapper.xsa"

    local vivado_tcl
    vivado_tcl=$(mktemp "${VITIS_DIR}/.vivado_xsa_XXXXXX.tcl")

    cat > "${vivado_tcl}" << TCLEOF
# Auto-generado por generate_boot.sh
puts "=========================================="
puts " Abriendo proyecto: ${XPR_PATH}"
puts "=========================================="
open_project ${XPR_PATH}

puts "Exportando hardware (XSA con bitstream incluido)..."
write_hw_platform -fixed -include_bit -force -file {${XSA_PATH}}

puts ""
puts "=========================================="
puts " XSA exportado: ${XSA_PATH}"
puts "=========================================="

close_project
TCLEOF

    log_info "Ejecutando Vivado en batch..."
    echo ""

    if "${VIVADO_BIN}" -mode batch -source "${vivado_tcl}" -notrace 2>&1; then
        echo ""
        if [[ -f "${XSA_PATH}" ]]; then
            log_ok "XSA exportado: ${XSA_PATH} ($(du -h "${XSA_PATH}" | cut -f1))"
        else
            log_error "El XSA no se generó en la ruta esperada"
            # Buscar alternativas
            local found_xsa
            found_xsa=$(find "${XPR_DIR}" -name "*.xsa" -newer "${XPR_PATH}" 2>/dev/null | head -1)
            if [[ -n "${found_xsa}" ]]; then
                XSA_PATH="${found_xsa}"
                log_ok "XSA encontrado en: ${XSA_PATH}"
            else
                log_error "No se encontró ningún XSA. Abortando."
                rm -f "${vivado_tcl}"
                exit 1
            fi
        fi
    else
        echo ""
        log_error "Vivado falló al exportar XSA"
        rm -f "${vivado_tcl}"
        exit 1
    fi

    rm -f "${vivado_tcl}"
}

#==============================================================================
#  VITIS: Crear plataforma + FSBL
#==============================================================================

run_vitis_setup() {
    log_step "Vitis — Creando plataforma y FSBL"

    # Verificar si ya existen
    if [[ -d "${VITIS_DIR}/${PLATFORM_NAME}" ]] || [[ -d "${VITIS_DIR}/${FSBL_NAME}" ]]; then
        log_warn "Ya existe '${PLATFORM_NAME}' y/o '${FSBL_NAME}'."
        if prompt_confirm "¿Eliminar y recrear?"; then
            log_info "Eliminando componentes anteriores..."
            local cleanup_script
            cleanup_script=$(mktemp "${VITIS_DIR}/.cleanup_XXXXXX.py")
            cat > "${cleanup_script}" << PYEOF
import vitis
client = vitis.create_client()
client.set_workspace(path="${VITIS_DIR}")
try:
    client.delete_component(name="${FSBL_NAME}")
except: pass
try:
    client.delete_component(name="${PLATFORM_NAME}")
except: pass
vitis.dispose()
PYEOF
            "${VITIS_BIN}" -s "${cleanup_script}" 2>&1 | grep -v "^$" | sed 's/^/  /' || true
            rm -f "${cleanup_script}"
            rm -rf "${VITIS_DIR:?}/${PLATFORM_NAME}" "${VITIS_DIR:?}/${FSBL_NAME}"
            log_ok "Limpieza completada"
        else
            log_info "Manteniendo componentes existentes."
            if [[ -f "${FSBL_ELF}" ]]; then
                log_ok "FSBL existente: ${FSBL_ELF}"
                return 0
            fi
        fi
    fi

    # Generar script Python
    local vitis_script
    vitis_script=$(mktemp "${VITIS_DIR}/.setup_XXXXXX.py")

    cat > "${vitis_script}" << PYEOF
import vitis

print("=" * 55)
print("  Vitis: ${PLATFORM_NAME} + ${FSBL_NAME}")
print("=" * 55)

client = vitis.create_client()
client.set_workspace(path="${VITIS_DIR}")

print()
print("[1/5] Creando plataforma '${PLATFORM_NAME}'...")
advanced_options = client.create_advanced_options_dict(dt_overlay="0")
platform = client.create_platform_component(
    name="${PLATFORM_NAME}",
    hw_design="${XSA_PATH}",
    os="standalone",
    cpu="psu_cortexa53_0",
    domain_name="standalone_psu_cortexa53_0",
    no_boot_bsp=True,
    generate_dtb=False,
    advanced_options=advanced_options,
    architecture="64-bit",
    compiler="gcc"
)
print("[OK] Plataforma creada")

print()
print("[2/5] Compilando plataforma...")
platform = client.get_component(name="${PLATFORM_NAME}")
status = platform.build()
print("[OK] Plataforma compilada")

print()
print("[3/5] Añadiendo dominio FSBL...")
domain = platform.add_domain(
    cpu="psu_cortexa53_0",
    os="standalone",
    name="fsbl_domain",
    display_name="fsbl_domain",
    support_app="zynqmp_fsbl",
    generate_dtb=False
)
print("[OK] Dominio FSBL añadido")

print()
print("[3.5/5] Recompilando plataforma para actualizar el .xpfm...")
status = platform.build()
print("[OK] Plataforma actualizada")

print()
print("[4/5] Creando aplicación FSBL '${FSBL_NAME}'...")
comp = client.create_app_component(
    name="${FSBL_NAME}",
    platform="${XPFM_PATH}",
    domain="fsbl_domain",
    template="zynqmp_fsbl"
)
print("[OK] App FSBL creada")

print()
print("[5/5] Compilando todo...")
status = platform.build()
print("[OK] Plataforma recompilada")
comp = client.get_component(name="${FSBL_NAME}")
comp.build()
print("[OK] FSBL compilado")

print()
print("=" * 55)
print("  VITIS SETUP COMPLETADO")
print("=" * 55)

vitis.dispose()
PYEOF

    log_info "Ejecutando Vitis (esto tarda unos minutos)..."
    echo ""

    if "${VITIS_BIN}" -s "${vitis_script}" 2>&1; then
        echo ""
        if [[ -f "${FSBL_ELF}" ]]; then
            log_ok "FSBL ELF: ${FSBL_ELF} ($(du -h "${FSBL_ELF}" | cut -f1))"
        else
            log_warn "FSBL no encontrado en ruta esperada, buscando..."
            local found_elf
            found_elf=$(find "${VITIS_DIR}/${FSBL_NAME}" -name "*.elf" 2>/dev/null | head -1)
            if [[ -n "${found_elf}" ]]; then
                FSBL_ELF="${found_elf}"
                log_ok "FSBL encontrado: ${FSBL_ELF}"
            else
                log_error "No se encontró el FSBL ELF"
                rm -f "${vitis_script}"
                exit 1
            fi
        fi
    else
        echo ""
        log_error "Vitis falló. Revisa los errores arriba."
        rm -f "${vitis_script}"
        exit 1
    fi

    # Guardar script para referencia
    mv "${vitis_script}" "${BOOT_COMPONENTS}/setup_${CONFIG_NAME}.py" 2>/dev/null || rm -f "${vitis_script}"
}

#==============================================================================
#  BOOTGEN: Generar BOOT.bin
#==============================================================================

run_bootgen() {
    log_step "Generando BOOT.bin"

    verify_boot_components

    # Verificar archivos
    local errors=0
    check_file "${FSBL_ELF}" "FSBL ELF"  || ((errors++))
    check_file "${BIT_PATH}" "Bitstream"   || ((errors++))
    if [[ ${errors} -gt 0 ]]; then
        log_error "Faltan archivos para generar BOOT.bin"
        exit 1
    fi

    # Generar BIF
    BIF_FILE="${OUTPUT_DIR}/boot_${CONFIG_NAME}.bif"

    cat > "${BIF_FILE}" << BIFEOF
//arch = zynqmp; split = false; format = BIN
the_ROM_image:
{
	[bootloader, destination_cpu = a53-0]${FSBL_ELF}
	[pmufw_image]${PMUFW}
	[destination_device = pl]${BIT_PATH}
	[destination_cpu = a53-0]${BL31}
	[destination_cpu = a53-0, exception_level = el-2]${UBOOT}
	[load = 0x100000, destination_cpu = a53-0]${SYSTEM_DTB}
}
BIFEOF

    log_ok "BIF generado: $(basename "${BIF_FILE}")"
    echo -e "${DIM}"
    sed 's/^/    /' "${BIF_FILE}"
    echo -e "${NC}"

    # Ejecutar bootgen
    BOOT_BIN="${OUTPUT_DIR}/BOOT.bin"
    [[ -f "${BOOT_BIN}" ]] && rm -f "${BOOT_BIN}"

    log_info "bootgen -arch zynqmp -image $(basename "${BIF_FILE}") -o BOOT.bin -w"
    echo ""

    if "${BOOTGEN_PATH}" -arch zynqmp -image "${BIF_FILE}" -o "${BOOT_BIN}" -w; then
        echo ""
        log_ok "BOOT.bin generado: ${BOOT_BIN} ($(du -h "${BOOT_BIN}" | cut -f1))"
    else
        echo ""
        log_error "bootgen falló"
        exit 1
    fi
}

#==============================================================================
#  COPIA A SD
#==============================================================================

offer_sd_copy() {
    echo ""
    local sd_mounts=()
    while IFS= read -r line; do
        [[ -n "${line}" ]] && sd_mounts+=("${line}")
    done < <(findmnt -rn -o TARGET -t vfat,exfat 2>/dev/null || true)

    for dir in /media/${USER}/* /mnt/*; do
        if [[ -d "${dir}" ]] && mountpoint -q "${dir}" 2>/dev/null; then
            local dup=false
            for existing in "${sd_mounts[@]:-}"; do
                [[ "${existing}" == "${dir}" ]] && dup=true
            done
            ${dup} || sd_mounts+=("${dir}")
        fi
    done

    if [[ ${#sd_mounts[@]} -eq 0 ]]; then
        log_info "No se detectaron tarjetas SD montadas."
        echo -e "  Copia manual: ${BOLD}cp ${BOOT_BIN} /ruta/sd/${NC}"
        return
    fi

    echo -e "${CYAN}Tarjetas SD detectadas:${NC}"
    for i in "${!sd_mounts[@]}"; do
        echo -e "  $((i+1))) ${sd_mounts[$i]}"
    done
    echo -e "  0) No copiar"
    echo -e -n "${BOLD}Selecciona [0-${#sd_mounts[@]}]: ${NC}"
    read -r sd_choice

    if [[ "${sd_choice}" =~ ^[0-9]+$ ]] && (( sd_choice >= 1 && sd_choice <= ${#sd_mounts[@]} )); then
        local target="${sd_mounts[$((sd_choice-1))]}"
        cp "${BOOT_BIN}" "${target}/BOOT.bin"
        sync
        log_ok "BOOT.bin copiado a ${target}/BOOT.bin"
    fi
}

#==============================================================================
#  RESUMEN FINAL
#==============================================================================

print_summary() {
    echo ""
    echo -e "${BOLD}╔═══════════════════════════════════════════════════════╗${NC}"
    echo -e "${BOLD}║           ${GREEN}✓ BOOT.bin GENERADO CON ÉXITO${NC}${BOLD}               ║${NC}"
    echo -e "${BOLD}╠═══════════════════════════════════════════════════════╣${NC}"
    printf "${BOLD}║${NC} %-14s ${CYAN}%s${NC}\n" "Config:" "${CONFIG_NAME}"
    printf "${BOLD}║${NC} %-14s %s\n" "Plataforma:" "${PLATFORM_NAME}"
    printf "${BOLD}║${NC} %-14s %s\n" "FSBL:" "$(basename "${FSBL_ELF}")"
    printf "${BOLD}║${NC} %-14s %s\n" "Bitstream:" "$(basename "${BIT_PATH}")"
    printf "${BOLD}║${NC} %-14s %s\n" "XSA:" "$(basename "${XSA_PATH}")"
    printf "${BOLD}║${NC} %-14s %s\n" "BOOT.bin:" "${BOOT_BIN}"
    printf "${BOLD}║${NC} %-14s %s\n" "Tamaño:" "$(du -h "${BOOT_BIN}" | cut -f1)"
    echo -e "${BOLD}╚═══════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${DIM}  Próximos pasos:${NC}"
    echo -e "${DIM}  1. Copia BOOT.bin a la SD${NC}"
    echo -e "${DIM}  2. Pon la ZCU102 en modo SD boot${NC}"
    echo -e "${DIM}  3. Enciende y conecta la UART${NC}"
}

#==============================================================================
#  AYUDA
#==============================================================================

show_help() {
    cat << 'EOF'

  generate_boot.sh — Wizard: Vivado → Vitis → BOOT.bin (ZCU102)

USO:
  ./generate_boot.sh              Wizard interactivo (recomendado)
  ./generate_boot.sh --help       Esta ayuda

FLUJO INTERACTIVO:
  El wizard te pregunta paso a paso:

  1. ¿Desde dónde empezar?
     a) Proyecto Vivado → genera bitstream + exporta XSA
     b) Proyecto Vivado → solo exporta XSA (bitstream ya existe)
     c) XSA ya exportado

  2. Selecciona el proyecto / XSA (auto-detecta los disponibles)

  3. Selecciona el bitstream .bit (auto-detecta, marca el de impl_1)

  4. Elige un nombre para la configuración

  5. El script hace todo automáticamente:
     - Vivado batch: bitstream + export XSA (si elegiste opción a/b)
     - Vitis API: crear plataforma + dominio FSBL + app FSBL + compilar
     - bootgen: generar BOOT.bin
     - (Opcional) copiar a SD

REQUISITOS:
  - Xilinx Vivado/Vitis 2025.1 en /tools/Xilinx/2025.1
  - Componentes de boot en boot_components/:
    pmufw.elf, bl31.elf, u-boot.elf, system.dtb
    (de github.com/Xilinx/soc-prebuilt-firmware → zcu102-zynqmp)

EOF
}

#==============================================================================
#  MAIN
#==============================================================================

main() {
    # Parse args
    OUTPUT_DIR="${BOOT_COMPONENTS}"
    NO_SD=false

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h) show_help; exit 0 ;;
            --no-sd)   NO_SD=true; shift ;;
            --out|-o)  OUTPUT_DIR="$2"; shift 2 ;;
            *) log_error "Argumento desconocido: $1"; show_help; exit 1 ;;
        esac
    done

    # Banner
    echo ""
    echo -e "${BOLD}╔═══════════════════════════════════════════════════════╗${NC}"
    echo -e "${BOLD}║      ZynqMP BOOT.bin Wizard — ZCU102                 ║${NC}"
    echo -e "${BOLD}║      Vivado → Vitis → BOOT.bin                       ║${NC}"
    echo -e "${BOLD}╚═══════════════════════════════════════════════════════╝${NC}"

    # Verificar herramientas
    verify_xilinx_tools

    # Paso 1: Elegir punto de entrada
    choose_entry_point

    # Paso 2: Según el punto de entrada
    case "${ENTRY_POINT}" in
        vivado_full)
            select_vivado_project
            ask_config_name
            run_vivado_bitstream
            run_vivado_export_xsa
            ;;
        vivado_xsa_only)
            select_vivado_project
            ask_config_name
            # Buscar bitstream existente
            find_or_select_bitstream
            run_vivado_export_xsa
            ;;
        xsa_existing)
            select_existing_xsa
            find_or_select_bitstream
            ask_config_name
            ;;
    esac

    # Crear directorio de salida
    mkdir -p "${OUTPUT_DIR}"

    # Paso 3: Crear plataforma + FSBL en Vitis
    run_vitis_setup

    # Paso 4: Generar BOOT.bin
    run_bootgen

    # Paso 5: Copia a SD
    if ! ${NO_SD}; then
        offer_sd_copy
    fi

    # Resumen
    print_summary
}

main "$@"
