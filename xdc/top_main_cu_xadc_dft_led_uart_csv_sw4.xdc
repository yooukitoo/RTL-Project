## ============================================================
## Basys3 constraints for top_main_cu_xadc_dft_led
##
## Top ports:
##   clk
##   rstn          : active-low reset, mapped to SW1
##   start_btn     : active-high start, mapped to SW0
##   clear_btn     : active-high clear, mapped to SW2
##   vauxp6/vauxn6 : JXADC VAUX6, MAX9814 OUT/GND
##   max7219_*     : JA pins
##   debug_led[15:0]
## ============================================================


## ============================================================
## 100 MHz clock
## ============================================================
set_property -dict { PACKAGE_PIN W5 IOSTANDARD LVCMOS33 } [get_ports clk]
create_clock -name sys_clk_pin -period 10.000 -waveform {0.000 5.000} [get_ports clk]


## ============================================================
## Control inputs
##
## rstn is active-low:
##   SW1 = 0 -> reset
##   SW1 = 1 -> run
##
## start_btn:
##   SW0 rising edge starts one frame
##
## clear_btn:
##   SW2 rising edge clears main_CU status
## ============================================================
set_property -dict { PACKAGE_PIN V16 IOSTANDARD LVCMOS33 } [get_ports rstn]
set_property -dict { PACKAGE_PIN V17 IOSTANDARD LVCMOS33 } [get_ports start_btn]
set_property -dict { PACKAGE_PIN W16 IOSTANDARD LVCMOS33 } [get_ports clear_btn]


## ============================================================
## XADC VAUX6 analog input
##
## Basys3 JXADC:
##   J3 = VAUXP6  <- MAX9814 OUT
##   K3 = VAUXN6  <- MAX9814 GND / common GND
##
## These auxiliary analog pins are in the same I/O bank as other Basys3 3.3V I/O.
## Set IOSTANDARD to LVCMOS33 to keep bank VCCO consistent.
## The pins are still used by the XADC primitive as analog inputs.
## ============================================================
set_property -dict { PACKAGE_PIN J3 IOSTANDARD LVCMOS33 } [get_ports vauxp6]
set_property -dict { PACKAGE_PIN K3 IOSTANDARD LVCMOS33 } [get_ports vauxn6]


## ============================================================
## MAX7219 LED Matrix Pins
##
## Connection:
##   Basys3 JA3(J2) -> MAX7219 DIN
##   Basys3 JA2(L2) -> MAX7219 CS / LOAD
##   Basys3 JA1(J1) -> MAX7219 CLK
##
## Common ground is required:
##   Basys3 GND -> MAX7219 GND
## ============================================================
set_property -dict { PACKAGE_PIN J2 IOSTANDARD LVCMOS33 } [get_ports max7219_din]
set_property -dict { PACKAGE_PIN L2 IOSTANDARD LVCMOS33 } [get_ports max7219_cs]
set_property -dict { PACKAGE_PIN J1 IOSTANDARD LVCMOS33 } [get_ports max7219_clk]


## ============================================================
## Debug LEDs
##
## top_main_cu_xadc_dft_led mapping:
##   debug_led[0]  = main_busy
##   debug_led[1]  = main_done
##   debug_led[2]  = main_error
##   debug_led[3]  = capture_busy
##   debug_led[4]  = capture_done
##   debug_led[5]  = xadc_tvalid
##   debug_led[6]  = xadc_tready
##   debug_led[7]  = xadc_tid[0]
##   debug_led[15:8] = main_state
## ============================================================
set_property -dict { PACKAGE_PIN U16 IOSTANDARD LVCMOS33 } [get_ports {debug_led[0]}]
set_property -dict { PACKAGE_PIN E19 IOSTANDARD LVCMOS33 } [get_ports {debug_led[1]}]
set_property -dict { PACKAGE_PIN U19 IOSTANDARD LVCMOS33 } [get_ports {debug_led[2]}]
set_property -dict { PACKAGE_PIN V19 IOSTANDARD LVCMOS33 } [get_ports {debug_led[3]}]
set_property -dict { PACKAGE_PIN W18 IOSTANDARD LVCMOS33 } [get_ports {debug_led[4]}]
set_property -dict { PACKAGE_PIN U15 IOSTANDARD LVCMOS33 } [get_ports {debug_led[5]}]
set_property -dict { PACKAGE_PIN U14 IOSTANDARD LVCMOS33 } [get_ports {debug_led[6]}]
set_property -dict { PACKAGE_PIN V14 IOSTANDARD LVCMOS33 } [get_ports {debug_led[7]}]
set_property -dict { PACKAGE_PIN V13 IOSTANDARD LVCMOS33 } [get_ports {debug_led[8]}]
set_property -dict { PACKAGE_PIN V3  IOSTANDARD LVCMOS33 } [get_ports {debug_led[9]}]
set_property -dict { PACKAGE_PIN W3  IOSTANDARD LVCMOS33 } [get_ports {debug_led[10]}]
set_property -dict { PACKAGE_PIN U3  IOSTANDARD LVCMOS33 } [get_ports {debug_led[11]}]
set_property -dict { PACKAGE_PIN P3  IOSTANDARD LVCMOS33 } [get_ports {debug_led[12]}]
set_property -dict { PACKAGE_PIN N3  IOSTANDARD LVCMOS33 } [get_ports {debug_led[13]}]
set_property -dict { PACKAGE_PIN P1  IOSTANDARD LVCMOS33 } [get_ports {debug_led[14]}]
set_property -dict { PACKAGE_PIN L1  IOSTANDARD LVCMOS33 } [get_ports {debug_led[15]}]


## ============================================================
## Timing policy for board debug/control I/O
##
## rstn/start_btn/clear_btn are asynchronous board inputs sampled by RTL.
## MAX7219 and LEDs are debug/peripheral outputs, not source-synchronous.
## ============================================================
set_false_path -from [get_ports {rstn start_btn clear_btn}]
set_false_path -to   [get_ports {debug_led[*] max7219_din max7219_cs max7219_clk}]






## ============================================================
## UART logging enable switch
##
## SW4 = 1 -> UART CSV logging ON
## SW4 = 0 -> UART CSV logging OFF
##
## Basys3 Master XDC:
## SW4 is PACKAGE_PIN W15.
## ============================================================
set_property -dict { PACKAGE_PIN W15 IOSTANDARD LVCMOS33 } [get_ports uart_log_enable]

## ============================================================
## Basys3 USB-UART TX
##
## FPGA -> PC serial output.
## Basys3 master XDC names this pin RsTx.
## Top port name here: uart_tx
## ============================================================
set_property -dict { PACKAGE_PIN A18 IOSTANDARD LVCMOS33 } [get_ports uart_tx]
set_false_path -to [get_ports uart_tx]

## ============================================================
## Basys3 configuration voltage
## ============================================================
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
