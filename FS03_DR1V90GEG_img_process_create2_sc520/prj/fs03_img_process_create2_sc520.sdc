# ---------------------------------------------------------------------------
# FS03_DR1V90GEG_img_process_create2_sc520 timing constraints
# Device : Anlogic DR1V90GEG484 (-2)   Family : DR1
# Pure-PL: no PS / no ethernet / no DDR. Same clocking as create1.
# ---------------------------------------------------------------------------

# 25 MHz board oscillator
create_clock -name {sys_clk_25m} -period 40.000 -waveform {0.000 20.000} [get_ports {I_sysclk}]

# MIPI D-PHY clock lane: 480 Mbps/lane -> 240 MHz DDR -> 4.167 ns period
create_clock -name {mipi_rx_ck_pad} -period 4.167 -waveform {0.000 2.083} [get_nets {IO_rx_clk_pad_p}]

derive_clocks

# The three main clock domains are asynchronous to each other.
set_clock_groups -asynchronous \
    -group [get_clocks {mipi_rx_ck_pad}] \
    -group [get_clocks {*pll_main*}] \
    -group [get_clocks {*pll_cam24m*}]

# Static user-control signals are exempt from timing:
#   * DIP switch / UART RX: board-static or slow, two-stage synchronized.
#   * LED / module power / buzzer / TM1638 3-wire / UART TX: static or slow.
set_false_path -from [get_ports {I_sw[*]}]
set_false_path -from [get_ports {I_uart_rx}]
set_false_path -to   [get_ports {O_led[*] O_sw_gnd[*] O_sw_vcc[*] O_buzz_vcc O_buzz O_buzz_gnd O_tm_vcc O_tm_gnd O_tm_stb O_tm_clk O_uart_tx}]

# K1 async reset distribution: K1 goes through a 19-bit control bus synchronizer
# into each target clock domain, then ANDed with that domain reset. The
# synchronized K1 is same-domain as the target registers, so recovery budget =
# the whole clock period of that domain; thus reset distribution is not timed.
set_false_path -from [get_clocks {sys_clk_25m}] -to [get_clocks {*pll_main*}]
set_false_path -from [get_clocks {sys_clk_25m}] -to [get_clocks {*pll_cam24m*}]