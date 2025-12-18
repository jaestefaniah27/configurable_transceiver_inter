# rebuild_all.tcl
# 1. Configuración de rutas
set project_name "zynq_transceiver_system"
set project_dir  "./vivado_proj"
set src_dir      "./src"
set script_dir   "./scripts"

# 2. Crear proyecto (Ajusta la parte según tu placa)
create_project -force $project_name $project_dir -part xczu3eg-sbva484-1-i

# 3. Añadir fuentes RTL (VHDL/Verilog)
# Asegúrate de que CONFIGURABLE_SERIAL_TOP esté en esta carpeta
add_files [glob $src_dir/*.vhd]
update_compile_order -fileset sources_1

# 4. Crear el Block Design
create_bd_design "system"

# 5. Instanciar y configurar la Zynq UltraScale+
set ps_e [create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e:3.3 zynq_ultra_ps_e_0]
# Aplicar configuración por defecto de la placa (Presests)
apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e -config {apply_board_preset "1"} $ps_e

# 6. Cargar tu script de transceptores
source $script_dir/generate_transceivers.tcl

# 7. Ejecutar la generación (Ejemplo con 14 transceptores)
# El script ahora encontrará la Zynq porque la creamos arriba
create_many_transceivers 14 "zynq_ultra_ps_e_0" "axi_smc"

# 8. Finalizar: Validar y crear Wrapper
validate_bd_design
make_wrapper -files [get_files $project_dir/$project_name.srcs/sources_1/bd/system/system.bd] -top
add_files -norecurse $project_dir/$project_name.srcs/sources_1/bd/system/hdl/system_wrapper.v