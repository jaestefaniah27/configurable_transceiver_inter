delete_bd_objs [get_bd_ports UART_13_RX]
delete_bd_objs [get_bd_ports UART_13_TX]
set c [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 tie_high]
connect_bd_net [get_bd_pins tie_high/dout] [get_bd_pins Transceiver_13/RD]