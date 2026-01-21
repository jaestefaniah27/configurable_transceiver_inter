# rebuild_all.tcl (Final Correcto: Wrapper Automático + Fixes)
# -------------------------------------------------------------------------
set project_name "zynq_transceiver_system"
set project_dir  "./vivado_proj"
set src_dir      "./src"
set script_dir   "./scripts"

set num_transceivers 13

# 1. Crear proyecto (Part Number ZCU102)
create_project -force $project_name $project_dir -part xczu9eg-ffvb1156-2-e

# 2. Añadir fuentes VHDL y Constraints
add_files [glob $src_dir/*.vhd]
add_files -fileset constrs_1 $src_dir/zcu102_constraints.xdc
update_compile_order -fileset sources_1

# =========================================================================
# PASO 2.5: GENERAR EL IP 'fifo_generator_0'
# =========================================================================
puts "Generando IP Core: fifo_generator_0 (9-bit, Depth 512)..."

create_ip -name fifo_generator -vendor xilinx.com -library ip -module_name fifo_generator_0

set_property -dict [list \
    CONFIG.Interface_Type {Native} \
    CONFIG.Performance_Options {Standard_FIFO} \
    CONFIG.Input_Data_Width {9} \
    CONFIG.Input_Depth {512} \
    CONFIG.Output_Data_Width {9} \
    CONFIG.Reset_Type {Synchronous_Reset} \
    CONFIG.Full_Flags_Reset_Value {0} \
    CONFIG.Use_Dout_Reset {true} \
] [get_ips fifo_generator_0]

generate_target all [get_ips fifo_generator_0]
create_ip_run [get_ips fifo_generator_0]
# =========================================================================

# 3. Crear Block Design
create_bd_design "system"

# 4. Instanciar Zynq (Sin versión fija para máxima compatibilidad)
set ps_e [create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e zynq_ultra_ps_e_0]

# Aplicar presets de ZCU102
apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e -config {apply_board_preset "1"} $ps_e

# 5. CONFIGURACIÓN DE PUERTOS
set_property -dict [list \
    CONFIG.PSU__USE__M_AXI_GP0 {1} \
    CONFIG.PSU__USE__M_AXI_GP1 {0} \
    CONFIG.PSU__USE__M_AXI_GP2 {0} \
    CONFIG.PSU__USE__IRQ0 {1} \
] $ps_e

# 6. Generar Transceptores (13 canales para coincidir con tu XDC actual)
source $script_dir/generate_transceivers.tcl
create_many_transceivers $num_transceivers "zynq_ultra_ps_e_0" "axi_smc"

# =========================================================================
# FIX UNIVERSAL DE RELOJES
# =========================================================================
set pins_to_check [list "maxihpm1_fpd_aclk" "maxihpm0_lpd_aclk"]
foreach pin_name $pins_to_check {
    set pin_obj [get_bd_pins -quiet zynq_ultra_ps_e_0/$pin_name]
    if { $pin_obj ne "" } {
        connect_bd_net [get_bd_pins zynq_ultra_ps_e_0/pl_clk0] $pin_obj
    }
}

# 7. Validar, crear Wrapper y FIJAR COMO TOP
validate_bd_design
make_wrapper -files [get_files $project_dir/$project_name.srcs/sources_1/bd/system/system.bd] -top
add_files -norecurse $project_dir/$project_name.srcs/sources_1/bd/system/hdl/system_wrapper.v

puts "Configurando system_wrapper como Top Module..."
set_property top system_wrapper [current_fileset]
update_compile_order -fileset sources_1

puts "----------------------------------------------------------------"
puts " GENERACIÓN FINALIZADA CON ÉXITO."
puts " Ya puedes ejecutar 'Generate Bitstream'."
puts "----------------------------------------------------------------"