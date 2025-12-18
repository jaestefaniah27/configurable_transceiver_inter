# add_transceiver.tcl
# Crea una instancia jerárquica de tu Transceptor (RTL + GPIOs + INTC)
# Uso: source add_transceiver.tcl -> create_transceiver_cell 1

proc create_transceiver_cell { index } {
  puts "Creando Transceptor Instancia $index (RTL Module Reference)..."
  
  # 1. Crear Jerarquía
  set hier_name "Transceiver_${index}"
  set current_bd_instance [current_bd_instance .]
  set hier_obj [create_bd_cell -type hier $hier_name]
  current_bd_instance $hier_obj

  # 2. Crear los AXI GPIOs
  set gpio_setup [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 axi_gpio_0_setup]
  set_property -dict [list CONFIG.C_ALL_OUTPUTS {1} CONFIG.C_GPIO_WIDTH {32}] $gpio_setup

  set gpio_rx [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 axi_gpio_1_rx]
  set_property -dict [list CONFIG.C_ALL_OUTPUTS {1} CONFIG.C_GPIO_WIDTH {2}] $gpio_rx

  set gpio_tx [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 axi_gpio_2_tx]
  set_property -dict [list CONFIG.C_ALL_OUTPUTS {1} CONFIG.C_GPIO_WIDTH {10}] $gpio_tx

  set gpio_out [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 axi_gpio_3_out]
  set_property -dict [list CONFIG.C_ALL_INPUTS {1} CONFIG.C_GPIO_WIDTH {14}] $gpio_out

  # 3. Crear AXI Interrupt Controller
  # Config: IntrType=Edge(1), EdgeType=Rising(1) excepto bit 1 Falling(0) -> 0xFFFFFFFD
  set intc [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_intc:4.1 axi_intc]
  set_property -dict [list \
      CONFIG.C_IRQ_CONNECTION {1} \
      CONFIG.C_KIND_OF_INTR {0xFFFFFFFF} \
      CONFIG.C_KIND_OF_EDGE {0xFFFFFFFD} \
  ] $intc

  # 4. Instanciar tu RTL como REFERENCIA (Sin empaquetar IP)
  # El nombre 'CONFIGURABLE_SERIAL_TOP' debe coincidir con tu entity VHDL
  set rtl_block [create_bd_cell -type module -reference CONFIGURABLE_SERIAL_TOP CONFIGURABLE_SERIAL_0]

  # 5. Conectar Pines Internos
  connect_bd_net [get_bd_pins axi_gpio_0_setup/gpio_io_o] [get_bd_pins CONFIGURABLE_SERIAL_0/PS_SERIAL_CONFIG]
  connect_bd_net [get_bd_pins axi_gpio_1_rx/gpio_io_o] [get_bd_pins CONFIGURABLE_SERIAL_0/PS_RX_DataRead_ErrorOk]
  connect_bd_net [get_bd_pins axi_gpio_2_tx/gpio_io_o] [get_bd_pins CONFIGURABLE_SERIAL_0/PS_TX_DataIn_Send]
  connect_bd_net [get_bd_pins CONFIGURABLE_SERIAL_0/PS_out] [get_bd_pins axi_gpio_3_out/gpio_io_i]
  
  # Interrupciones
  connect_bd_net [get_bd_pins CONFIGURABLE_SERIAL_0/TX_RDY_EMPTY] [get_bd_pins axi_intc/intr]

  # 6. Exponer Pines al Exterior
  create_bd_pin -dir I -type clk s_axi_aclk
  create_bd_pin -dir I -type rst s_axi_aresetn
  
  # Conexión de Relojes
  connect_bd_net [get_bd_pins s_axi_aclk] [get_bd_pins axi_gpio_0_setup/s_axi_aclk]
  connect_bd_net [get_bd_pins s_axi_aclk] [get_bd_pins axi_gpio_1_rx/s_axi_aclk]
  connect_bd_net [get_bd_pins s_axi_aclk] [get_bd_pins axi_gpio_2_tx/s_axi_aclk]
  connect_bd_net [get_bd_pins s_axi_aclk] [get_bd_pins axi_gpio_3_out/s_axi_aclk]
  connect_bd_net [get_bd_pins s_axi_aclk] [get_bd_pins axi_intc/s_axi_aclk]
  connect_bd_net [get_bd_pins s_axi_aclk] [get_bd_pins CONFIGURABLE_SERIAL_0/Clk]

  # Conexión de Resets (AQUÍ ESTÁ LA CORRECCIÓN)
  connect_bd_net [get_bd_pins s_axi_aresetn] [get_bd_pins axi_gpio_0_setup/s_axi_aresetn]
  connect_bd_net [get_bd_pins s_axi_aresetn] [get_bd_pins axi_gpio_1_rx/s_axi_aresetn]
  connect_bd_net [get_bd_pins s_axi_aresetn] [get_bd_pins axi_gpio_2_tx/s_axi_aresetn]
  connect_bd_net [get_bd_pins s_axi_aresetn] [get_bd_pins axi_gpio_3_out/s_axi_aresetn]
  connect_bd_net [get_bd_pins s_axi_aresetn] [get_bd_pins axi_intc/s_axi_aresetn]
  
  # Reset del RTL: Conectamos directo porque tu VHDL dice "Low-level asynchronous reset"
  connect_bd_net [get_bd_pins s_axi_aresetn] [get_bd_pins CONFIGURABLE_SERIAL_0/Reset]

  # Pines Físicos UART
  create_bd_pin -dir I RD
  create_bd_pin -dir O TD
  connect_bd_net [get_bd_pins RD] [get_bd_pins CONFIGURABLE_SERIAL_0/RD]
  connect_bd_net [get_bd_pins TD] [get_bd_pins CONFIGURABLE_SERIAL_0/TD]

  # Salida IRQ al PS
  create_bd_pin -dir O -type intr irq
  connect_bd_net [get_bd_pins axi_intc/irq] [get_bd_pins irq]

  current_bd_instance $current_bd_instance
  puts "Transceptor $index creado (RTL: Reset activo bajo conectado)."
}