# generate_transceivers.tcl
# Versión 7.0: Generador Autónomo con SysInfo Enriquecido (Plug & Play)
# -----------------------------------------------------------------------------------------
# Características:
# 1. Compacto: 1 AXI GPIO por Transceptor (Dual Channel).
# 2. Autónomo: Crea Reset y SmartConnect si faltan.
# 3. Robusto: Arregla automáticamente el reloj AXI Master del Zynq (maxihpm0_fpd_aclk).
# 4. Inteligente: Genera un bloque SysInfo en 0xA0020000 con metadatos para el driver C.
# 5. Phantom-Safe: Gestiona el transceptor 14 (sin pines) sin errores de implementación.
# -----------------------------------------------------------------------------------------

proc create_transceiver_hier { parent_name index } {
  puts "  -> Generando jerarquía: Transceiver_${index}..."
  
  set hier_name "Transceiver_${index}"
  set current_bd_instance [current_bd_instance .]
  set hier_obj [create_bd_cell -type hier $hier_name]
  current_bd_instance $hier_obj

  set s "_${index}"

  # 1. RTL Core
  set rtl [create_bd_cell -type module -reference CONFIGURABLE_SERIAL_TOP "rtl_core${s}"]

  # 2. AXI GPIO Unificado (Dual Channel)
  # Ch1 (Out): Configuración + TX Data | Ch2 (In): Status + RX Data
  set gpio [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 "axi_gpio${s}"]
  set_property -dict [list CONFIG.C_IS_DUAL {1} CONFIG.C_ALL_OUTPUTS {1} CONFIG.C_GPIO_WIDTH {27} \
                           CONFIG.C_ALL_INPUTS_2 {1} CONFIG.C_GPIO2_WIDTH {14}] $gpio

  # 3. Conexiones Internas
  connect_bd_net [get_bd_pins ${gpio}/gpio_io_o] [get_bd_pins ${rtl}/PS_SERIAL_CONFIG_DataRead_ErrorOk_Send_DataIn]
  connect_bd_net [get_bd_pins ${rtl}/PS_out]     [get_bd_pins ${gpio}/gpio2_io_i]

  # 4. Interfaz AXI
  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:aximm_rtl:1.0 S_AXI
  connect_bd_intf_net [get_bd_intf_pins S_AXI] [get_bd_intf_pins ${gpio}/S_AXI]

  # 5. Pines de Sistema
  create_bd_pin -dir I -type clk aclk
  create_bd_pin -dir I -type rst aresetn
  connect_bd_net [get_bd_pins aclk]    [get_bd_pins ${gpio}/s_axi_aclk] [get_bd_pins ${rtl}/Clk]
  connect_bd_net [get_bd_pins aresetn] [get_bd_pins ${gpio}/s_axi_aresetn] [get_bd_pins ${rtl}/Reset]

  # 6. Pines UART (Físicos)
  create_bd_pin -dir I RD
  create_bd_pin -dir O TD
  connect_bd_net [get_bd_pins RD] [get_bd_pins ${rtl}/RD]
  connect_bd_net [get_bd_pins TD] [get_bd_pins ${rtl}/TD]

  # 7. Interrupción
  create_bd_pin -dir O -from 1 -to 0 irq_raw
  connect_bd_net [get_bd_pins ${rtl}/TX_RDY_EMPTY] [get_bd_pins irq_raw]

  current_bd_instance $current_bd_instance
}

proc create_many_transceivers { count ps_name main_xbar_name } {
  puts "------------------------------------------------"
  puts " INICIANDO GENERACIÓN INTELIGENTE V7 ($count CANALES)"
  puts "------------------------------------------------"

  # === 0. DEFINICIÓN DE PARAMETROS DE MEMORIA ===
  # Estos valores se grabarán en el hardware para que el software los lea.
  set uart_base_addr 0xA0000000
  set uart_stride    0x1000      ;# 4KB
  set sys_info_addr  0xA0020000  ;# Dirección Fija "Ancla"

  # Limpieza previa
  delete_bd_objs [get_bd_cells -quiet irq_concat]
  delete_bd_objs [get_bd_cells -quiet axi_intc_global]
  delete_bd_objs [get_bd_cells -quiet axi_sys_info]
  delete_bd_objs [get_bd_cells -quiet const_sys_meta]
  delete_bd_objs [get_bd_cells -quiet const_sys_base]

  # --- A. INFRAESTRUCTURA Y CONEXIONES ZYNQ ---
  set ps_cell [get_bd_cells $ps_name]
  if { $ps_cell eq "" } { puts "ERROR CRÍTICO: Zynq '$ps_name' no encontrado en el diseño."; return }
  set sys_clk [get_bd_pins $ps_name/pl_clk0]
  
  # 1. Reset System
  set rst_cell [get_bd_cells -quiet *rst*]
  if { $rst_cell eq "" } {
      puts "  -> Creando Processor System Reset..."
      set rst_cell [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 rst_gen]
      connect_bd_net $sys_clk [get_bd_pins $rst_cell/slowest_sync_clk]
      connect_bd_net [get_bd_pins $ps_name/pl_resetn0] [get_bd_pins $rst_cell/ext_reset_in]
  } else { set rst_cell [lindex $rst_cell 0] }
  set peripheral_aresetn [get_bd_pins -of_objects $rst_cell -filter {NAME=~*peripheral_aresetn}]

  # 2. SmartConnect
  set main_xbar_cell [get_bd_cells -quiet $main_xbar_name]
  if { $main_xbar_cell eq "" } {
      puts "  -> Creando SmartConnect '$main_xbar_name'..."
      set main_xbar_cell [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 $main_xbar_name]
      set_property CONFIG.NUM_SI {1} $main_xbar_cell
  }

  # 3. VERIFICACIONES DE SEGURIDAD ZYNQ
  puts "  -> Verificando integridad del puerto AXI Master..."
  
  # Habilitar puerto Master si está apagado
  set zynq_param [get_property CONFIG.PSU__USE__M_AXI_GP0 [get_bd_cells $ps_name]]
  if { $zynq_param != 1 } { 
      puts "     -> Auto-Fix Habilitando M_AXI_HPM0_FPD..."
      set_property CONFIG.PSU__USE__M_AXI_GP0 {1} [get_bd_cells $ps_name] 
  }
  
  # Conectar Reloj al Puerto Master (CRÍTICO)
  if { [get_bd_nets -quiet -of_objects [get_bd_pins $ps_name/maxihpm0_fpd_aclk]] eq "" } {
      puts "     -> Auto-Fix Conectando reloj maxihpm0_fpd_aclk..."
      connect_bd_net $sys_clk [get_bd_pins $ps_name/maxihpm0_fpd_aclk]
  }

  # Conectar Clocks/Resets del SmartConnect
  if { [get_bd_nets -quiet -of_objects [get_bd_pins ${main_xbar_cell}/aclk]] eq "" } {
      connect_bd_net $sys_clk [get_bd_pins ${main_xbar_cell}/aclk]
  }
  if { [get_bd_nets -quiet -of_objects [get_bd_pins ${main_xbar_cell}/aresetn]] eq "" } {
      connect_bd_net $peripheral_aresetn [get_bd_pins ${main_xbar_cell}/aresetn]
  }

  # Conectar Bus (Zynq -> SmartConnect)
  set connected_intf [get_bd_intf_nets -quiet -of_objects [get_bd_intf_pins ${main_xbar_cell}/S00_AXI]]
  if { $connected_intf eq "" } {
      puts "     -> Auto-Fix Enlazando Zynq con SmartConnect..."
      connect_bd_intf_net [get_bd_intf_pins $ps_name/M_AXI_HPM0_FPD] [get_bd_intf_pins ${main_xbar_cell}/S00_AXI]
  }

  # --- B. CONFIGURACIÓN PUERTOS Y CONTROLADOR IRQ ---
  # Puertos necesarios: N Transceptores + 1 INTC + 1 SysInfo
  set needed_ports [expr {$count + 2}]
  set_property CONFIG.NUM_MI $needed_ports $main_xbar_cell

  # IRQ Concat
  set concat [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat:2.1 irq_concat]
  set_property CONFIG.NUM_PORTS $count $concat

  # INTC Global
  set edge_val 0
  for {set k 0} {$k < $count} {incr k} {
      set shift [expr {$k * 2}]
      set pattern [expr {2 << $shift}] 
      set edge_val [expr {$edge_val | $pattern}]
  }
  set edge_hex [format "0x%X" $edge_val]

  set intc_global [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_intc:4.1 axi_intc_global]
  set_property -dict [list CONFIG.C_IRQ_CONNECTION {1} CONFIG.C_NUM_INTR_INPUTS [expr {$count * 2}] \
                           CONFIG.C_KIND_OF_INTR {0xFFFFFFFF} CONFIG.C_KIND_OF_EDGE $edge_hex] $intc_global

  connect_bd_net $sys_clk [get_bd_pins ${intc_global}/s_axi_aclk]
  connect_bd_net $peripheral_aresetn [get_bd_pins ${intc_global}/s_axi_aresetn]
  
  # Conectar INTC al puerto 'count' (penúltimo)
  set intc_idx [format "%02d" $count]
  connect_bd_intf_net [get_bd_intf_pins ${main_xbar_cell}/M${intc_idx}_AXI] [get_bd_intf_pins ${intc_global}/s_axi]

  # --- C. SYSTEM INFO ENRIQUECIDO (Dual Channel) ---
  puts "  -> Creando Bloque SysInfo Enriquecido..."
  
  set sys_info [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 axi_sys_info]
  # Canal 1: Metadata (Count + Stride), Canal 2: Base Address
  set_property -dict [list \
      CONFIG.C_IS_DUAL {1} \
      CONFIG.C_ALL_INPUTS {1}   CONFIG.C_GPIO_WIDTH {32} \
      CONFIG.C_ALL_INPUTS_2 {1} CONFIG.C_GPIO2_WIDTH {32} \
  ] $sys_info
  
  # Constante 1: METADATA (Bits 31-16: Stride, Bits 15-0: Count)
  set meta_val [expr {($uart_stride << 16) | $count}]
  set const_meta [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 const_sys_meta]
  set_property -dict [list CONFIG.CONST_VAL $meta_val CONFIG.CONST_WIDTH {32}] $const_meta
  
  # Constante 2: BASE ADDRESS
  set const_base [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 const_sys_base]
  set_property -dict [list CONFIG.CONST_VAL $uart_base_addr CONFIG.CONST_WIDTH {32}] $const_base
  
  # Conexiones SysInfo
  connect_bd_net [get_bd_pins ${const_meta}/dout] [get_bd_pins ${sys_info}/gpio_io_i]
  connect_bd_net [get_bd_pins ${const_base}/dout] [get_bd_pins ${sys_info}/gpio2_io_i]
  
  connect_bd_net $sys_clk [get_bd_pins ${sys_info}/s_axi_aclk]
  connect_bd_net $peripheral_aresetn [get_bd_pins ${sys_info}/s_axi_aresetn]

  # Conectar SysInfo al último puerto
  set sys_idx [format "%02d" [expr {$count + 1}]]
  connect_bd_intf_net [get_bd_intf_pins ${main_xbar_cell}/M${sys_idx}_AXI] [get_bd_intf_pins ${sys_info}/S_AXI]

  # --- D. BUCLE DE TRANSCEPTORES ---
  for {set i 0} {$i < $count} {incr i} {
    delete_bd_objs [get_bd_cells -quiet "Transceiver_$i"]
    create_transceiver_hier "" $i
    set cell "Transceiver_$i"
    
    # Infraestructura
    connect_bd_net $sys_clk [get_bd_pins ${cell}/aclk]
    connect_bd_net $peripheral_aresetn [get_bd_pins ${cell}/aresetn]
    set mi_idx [format "%02d" $i]
    connect_bd_intf_net [get_bd_intf_pins ${main_xbar_cell}/M${mi_idx}_AXI] [get_bd_intf_pins ${cell}/S_AXI]
    
    # Interrupciones
    connect_bd_net [get_bd_pins ${cell}/irq_raw] [get_bd_pins ${concat}/In${i}]

    # Gestión de Pines (Físico vs Phantom)
    if { $i < 13 } {
        make_bd_pins_external [get_bd_pins ${cell}/RD]
        set_property name "UART_${i}_RX" [get_bd_ports RD_0]
        make_bd_pins_external [get_bd_pins ${cell}/TD]
        set_property name "UART_${i}_TX" [get_bd_ports TD_0]
    } else {
        # Transceptor Phantom (sin salida física, RX tied high)
        set tie [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 "tie_idle_${i}"]
        set_property CONFIG.CONST_VAL {1} $tie
        connect_bd_net [get_bd_pins ${tie}/dout] [get_bd_pins ${cell}/RD]
    }
  }

  # Conexión final de IRQ al procesador
  connect_bd_net [get_bd_pins ${concat}/dout] [get_bd_pins ${intc_global}/intr]
  connect_bd_net [get_bd_pins ${intc_global}/irq] [get_bd_pins ${ps_name}/pl_ps_irq0]

  # --- E. ASIGNACIÓN DE DIRECCIONES ---
  puts "  -> Asignando Direcciones al Mapa de Memoria..."
  set seg "${ps_name}/Data"

  # 1. Transceptores
  for {set i 0} {$i < $count} {incr i} {
      set addr [expr {$uart_base_addr + ($i * $uart_stride)}]
      assign_bd_address -target_address_space $seg [get_bd_addr_segs Transceiver_${i}/*/S_AXI/Reg] -force -offset [format "0x%08X" $addr] -range 4K
  }
  
  # 2. INTC Global
  set intc_addr [expr {$uart_base_addr + ($count * $uart_stride)}]
  assign_bd_address -target_address_space $seg [get_bd_addr_segs axi_intc_global/S_AXI/Reg] -force -offset [format "0x%08X" $intc_addr] -range 4K
  
  # 3. SysInfo (Anchor Address)
  assign_bd_address -target_address_space $seg [get_bd_addr_segs axi_sys_info/S_AXI/Reg] -force -offset [format "0x%08X" $sys_info_addr] -range 4K

  puts "--------------------------------------------------------"
  puts " GENERACIÓN COMPLETADA CON ÉXITO."
  puts " Información para el Driver C:"
  puts "   - SysInfo Addr: [format 0x%X $sys_info_addr]"
  puts "   - UART Base:    [format 0x%X $uart_base_addr]"
  puts "   - UART Stride:  [format 0x%X $uart_stride]"
  puts "   - Count:        $count"
  puts "--------------------------------------------------------"
}
#source /home/mpsocv2/CONFIGURABLE_TRANSCEIVER_SERIAL/CONFIGURABLE_TRANSCEIVER_SERIAL.srcs/sources_1/scripts/generate_transceivers.tcl
#create_many_transceivers 14 "zynq_ultra_ps_e_0" "axi_smc"