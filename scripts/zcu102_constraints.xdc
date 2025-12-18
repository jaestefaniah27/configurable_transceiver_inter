# -----------------------------------------------------------------------
# CONSTRAINTS PARA 13 TRANSCEPTORES UART (ZCU102)
# -----------------------------------------------------------------------
# === PROTOTYPE HEADER (J3) -> UARTs 0 a 3 ===
# Banco 50 (3.3V). Pines pares del 6 al 24.
set_property PACKAGE_PIN H14 [get_ports {UART_0_TX}] ;# J3.6
set_property PACKAGE_PIN J14 [get_ports {UART_0_RX}] ;# J3.8

set_property PACKAGE_PIN G14 [get_ports {UART_1_TX}] ;# J3.10
set_property PACKAGE_PIN G15 [get_ports {UART_1_RX}] ;# J3.12

set_property PACKAGE_PIN J15 [get_ports {UART_2_TX}] ;# J3.14
set_property PACKAGE_PIN J16 [get_ports {UART_2_RX}] ;# J3.16

set_property PACKAGE_PIN G16 [get_ports {UART_3_TX}] ;# J3.18
set_property PACKAGE_PIN H16 [get_ports {UART_3_RX}] ;# J3.20

set_property IOSTANDARD LVCMOS33 [get_ports {UART_0_* UART_1_* UART_2_* UART_3_*}]

# === PMOD0 (J55) -> UARTs 4 a 7 ===
# Banco 47 (3.3V)
# set_property PACKAGE_PIN A20 [get_ports {UART_4_TX}] ;# J55.1
# set_property PACKAGE_PIN B21 [get_ports {UART_4_RX}] ;# J55.2
# 
# set_property PACKAGE_PIN B20 [get_ports {UART_5_TX}] ;# J55.3
# set_property PACKAGE_PIN C21 [get_ports {UART_5_RX}] ;# J55.4
# 
# set_property PACKAGE_PIN A22 [get_ports {UART_6_TX}] ;# J55.5
# set_property PACKAGE_PIN C22 [get_ports {UART_6_RX}] ;# J55.6
# 
# set_property PACKAGE_PIN A21 [get_ports {UART_7_TX}] ;# J55.7
# set_property PACKAGE_PIN D21 [get_ports {UART_7_RX}] ;# J55.8
# 
# set_property IOSTANDARD LVCMOS33 [get_ports {UART_4_* UART_5_* UART_6_* UART_7_*}]

# === PMOD1 (J87) -> UARTs 8 a 11 ===
# Banco 47 (3.3V)
# set_property PACKAGE_PIN D20 [get_ports {UART_8_TX}] ;# J87.1
# set_property PACKAGE_PIN F20 [get_ports {UART_8_RX}] ;# J87.2
# 
# set_property PACKAGE_PIN E20 [get_ports {UART_9_TX}] ;# J87.3
# set_property PACKAGE_PIN G20 [get_ports {UART_9_RX}] ;# J87.4
# 
# set_property PACKAGE_PIN D22 [get_ports {UART_10_TX}] ;# J87.5
# set_property PACKAGE_PIN J20 [get_ports {UART_10_RX}] ;# J87.6
# 
# set_property PACKAGE_PIN E22 [get_ports {UART_11_TX}] ;# J87.7
# set_property PACKAGE_PIN J19 [get_ports {UART_11_RX}] ;# J87.8
# 
# set_property IOSTANDARD LVCMOS33 [get_ports {UART_8_* UART_9_* UART_10_* UART_11_*}]

# === UART 12 -> Requiere asignación adicional ===
# (Agregar pines según disponibilidad en tu placa)

