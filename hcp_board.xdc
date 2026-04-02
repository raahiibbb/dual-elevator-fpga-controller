## ============================================================================
## hcp_board.xdc - Pin Constraints for HCP Board (Nexys A7-100T)
## Hall Call Panel / Outside Controller - 8 Floors
## ============================================================================

## Clock (100 MHz)
set_property -dict {PACKAGE_PIN E3 IOSTANDARD LVCMOS33} [get_ports clk]
create_clock -period 10.000 -name sys_clk_pin -waveform {0.000 5.000} [get_ports clk]

## ============================================================================
## Switches SW[15:0] - Interleaved floor call down/up order
## SW[0]  = Floor 0 DOWN (unused - floor 0 has no DOWN)
## SW[1]  = Floor 0 UP
## SW[2]  = Floor 1 DOWN
## SW[3]  = Floor 1 UP
## SW[4]  = Floor 2 DOWN
## SW[5]  = Floor 2 UP
## SW[6]  = Floor 3 DOWN
## SW[7]  = Floor 3 UP
## SW[8]  = Floor 4 DOWN
## SW[9]  = Floor 4 UP
## SW[10] = Floor 5 DOWN
## SW[11] = Floor 5 UP
## SW[12] = Floor 6 DOWN
## SW[13] = Floor 6 UP
## SW[14] = Floor 7 DOWN
## SW[15] = Floor 7 UP   (unused - floor 7 has no UP)
## ============================================================================
set_property -dict {PACKAGE_PIN J15 IOSTANDARD LVCMOS33} [get_ports {sw[0]}]
set_property -dict {PACKAGE_PIN L16 IOSTANDARD LVCMOS33} [get_ports {sw[1]}]
set_property -dict {PACKAGE_PIN M13 IOSTANDARD LVCMOS33} [get_ports {sw[2]}]
set_property -dict {PACKAGE_PIN R15 IOSTANDARD LVCMOS33} [get_ports {sw[3]}]
set_property -dict {PACKAGE_PIN R17 IOSTANDARD LVCMOS33} [get_ports {sw[4]}]
set_property -dict {PACKAGE_PIN T18 IOSTANDARD LVCMOS33} [get_ports {sw[5]}]
set_property -dict {PACKAGE_PIN U18 IOSTANDARD LVCMOS33} [get_ports {sw[6]}]
set_property -dict {PACKAGE_PIN R13 IOSTANDARD LVCMOS33} [get_ports {sw[7]}]
set_property -dict {PACKAGE_PIN T8  IOSTANDARD LVCMOS33} [get_ports {sw[8]}]
set_property -dict {PACKAGE_PIN U8  IOSTANDARD LVCMOS33} [get_ports {sw[9]}]
set_property -dict {PACKAGE_PIN R16 IOSTANDARD LVCMOS33} [get_ports {sw[10]}]
set_property -dict {PACKAGE_PIN T13 IOSTANDARD LVCMOS33} [get_ports {sw[11]}]
set_property -dict {PACKAGE_PIN H6  IOSTANDARD LVCMOS33} [get_ports {sw[12]}]
set_property -dict {PACKAGE_PIN U12 IOSTANDARD LVCMOS33} [get_ports {sw[13]}]
set_property -dict {PACKAGE_PIN U11 IOSTANDARD LVCMOS33} [get_ports {sw[14]}]
set_property -dict {PACKAGE_PIN V10 IOSTANDARD LVCMOS33} [get_ports {sw[15]}]

## ============================================================================
## Push Button (Reset only)
## ============================================================================
set_property -dict {PACKAGE_PIN N17 IOSTANDARD LVCMOS33} [get_ports btnc]

## ============================================================================
## LEDs [15:0] - Matching interleaved switch order
## LED[0]  = (unused - floor 0 has no DOWN)
## LED[1]  = Floor 0 UP indicator
## LED[2]  = Floor 1 DOWN indicator
## LED[3]  = Floor 1 UP indicator
## LED[4]  = Floor 2 DOWN indicator
## LED[5]  = Floor 2 UP indicator
## LED[6]  = Floor 3 DOWN indicator
## LED[7]  = Floor 3 UP indicator
## LED[8]  = Floor 4 DOWN indicator
## LED[9]  = Floor 4 UP indicator
## LED[10] = Floor 5 DOWN indicator
## LED[11] = Floor 5 UP indicator
## LED[12] = Floor 6 DOWN indicator
## LED[13] = Floor 6 UP indicator
## LED[14] = Floor 7 DOWN indicator
## LED[15] = (unused - floor 7 has no UP)
## ============================================================================
set_property -dict {PACKAGE_PIN H17 IOSTANDARD LVCMOS33} [get_ports {led[0]}]
set_property -dict {PACKAGE_PIN K15 IOSTANDARD LVCMOS33} [get_ports {led[1]}]
set_property -dict {PACKAGE_PIN J13 IOSTANDARD LVCMOS33} [get_ports {led[2]}]
set_property -dict {PACKAGE_PIN N14 IOSTANDARD LVCMOS33} [get_ports {led[3]}]
set_property -dict {PACKAGE_PIN R18 IOSTANDARD LVCMOS33} [get_ports {led[4]}]
set_property -dict {PACKAGE_PIN V17 IOSTANDARD LVCMOS33} [get_ports {led[5]}]
set_property -dict {PACKAGE_PIN U17 IOSTANDARD LVCMOS33} [get_ports {led[6]}]
set_property -dict {PACKAGE_PIN U16 IOSTANDARD LVCMOS33} [get_ports {led[7]}]
set_property -dict {PACKAGE_PIN V16 IOSTANDARD LVCMOS33} [get_ports {led[8]}]
set_property -dict {PACKAGE_PIN T15 IOSTANDARD LVCMOS33} [get_ports {led[9]}]
set_property -dict {PACKAGE_PIN U14 IOSTANDARD LVCMOS33} [get_ports {led[10]}]
set_property -dict {PACKAGE_PIN T16 IOSTANDARD LVCMOS33} [get_ports {led[11]}]
set_property -dict {PACKAGE_PIN V15 IOSTANDARD LVCMOS33} [get_ports {led[12]}]
set_property -dict {PACKAGE_PIN V14 IOSTANDARD LVCMOS33} [get_ports {led[13]}]
set_property -dict {PACKAGE_PIN V12 IOSTANDARD LVCMOS33} [get_ports {led[14]}]
set_property -dict {PACKAGE_PIN V11 IOSTANDARD LVCMOS33} [get_ports {led[15]}]

## ============================================================================
## 7-Segment Display
## Cathodes seg[6:0] = {a, b, c, d, e, f, g}
## ============================================================================
set_property -dict {PACKAGE_PIN T10 IOSTANDARD LVCMOS33} [get_ports {seg[6]}]
set_property -dict {PACKAGE_PIN R10 IOSTANDARD LVCMOS33} [get_ports {seg[5]}]
set_property -dict {PACKAGE_PIN K16 IOSTANDARD LVCMOS33} [get_ports {seg[4]}]
set_property -dict {PACKAGE_PIN K13 IOSTANDARD LVCMOS33} [get_ports {seg[3]}]
set_property -dict {PACKAGE_PIN P15 IOSTANDARD LVCMOS33} [get_ports {seg[2]}]
set_property -dict {PACKAGE_PIN T11 IOSTANDARD LVCMOS33} [get_ports {seg[1]}]
set_property -dict {PACKAGE_PIN L18 IOSTANDARD LVCMOS33} [get_ports {seg[0]}]

## Anodes AN[7:0] (active low)
## AN[0] (J17) = rightmost,  AN[7] (U13) = leftmost
## Display: AN[3]=Lift2 floor, AN[2]=Lift2 status, AN[1]=Lift1 floor, AN[0]=Lift1 status
set_property -dict {PACKAGE_PIN J17 IOSTANDARD LVCMOS33} [get_ports {an[0]}]
set_property -dict {PACKAGE_PIN J18 IOSTANDARD LVCMOS33} [get_ports {an[1]}]
set_property -dict {PACKAGE_PIN T9  IOSTANDARD LVCMOS33} [get_ports {an[2]}]
set_property -dict {PACKAGE_PIN J14 IOSTANDARD LVCMOS33} [get_ports {an[3]}]
set_property -dict {PACKAGE_PIN P14 IOSTANDARD LVCMOS33} [get_ports {an[4]}]
set_property -dict {PACKAGE_PIN T14 IOSTANDARD LVCMOS33} [get_ports {an[5]}]
set_property -dict {PACKAGE_PIN K2  IOSTANDARD LVCMOS33} [get_ports {an[6]}]
set_property -dict {PACKAGE_PIN U13 IOSTANDARD LVCMOS33} [get_ports {an[7]}]

## ============================================================================
## PMOD JA - UART Communication
## JA[1] = C17 = TX to COP board
## JA[2] = D18 = RX from COP board
## Connect: HCP JA[1] -> COP JA[2] (TX -> RX)
##          HCP JA[2] <- COP JA[1] (RX <- TX)
##          GND <-> GND (PMOD pin 5 or 11)
## ============================================================================
set_property -dict {PACKAGE_PIN C17 IOSTANDARD LVCMOS33} [get_ports ja_tx]
set_property -dict {PACKAGE_PIN D18 IOSTANDARD LVCMOS33} [get_ports ja_rx]

## ============================================================================
## Configuration
## ============================================================================
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property CFGBVS VCCO [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 33 [current_design]
