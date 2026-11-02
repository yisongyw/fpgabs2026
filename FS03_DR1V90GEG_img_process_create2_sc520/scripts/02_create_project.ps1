# =====================================================================
#  02_create_project.ps1
#  Generate FS03_DR1V90GEG_img_process_create2_sc520 TD project (.al), pin
#  constraint (pin.adc) and timing constraint (.sdc), guaranteeing:
#    * all source files are loaded (each is verified to exist on disk)
#    * pin.adc is line-by-line identical to the FS03(DR1V90GEG484) baseline
#      plus the new peripheral pins (NO PS placeholder pins)
#    * pure-PL image processing (no PS / no ethernet / no DDR)
#    * Logo bitmap block-RAM patch (identical to create1)
#
#  Usage: .\02_create_project.ps1
# =====================================================================
[CmdletBinding()]
param(
    [string]$PrjRoot = 'D:\Documents\bc10anlogictd\FS03_DR1V90GEG_img_process_create2_sc520',
    [string]$PlatRoot = 'D:\Documents\bc10anlogictd\FS03_DR1V90GEG_osd_SC500_720P'
)

$ErrorActionPreference = 'Stop'

$New     = 'FS03_DR1V90GEG_img_process_create2_sc520'
$PrjName = 'fs03_img_process_create2_sc520'

$SRC = Join-Path $PrjRoot "${New}_src"
$PRJ = Join-Path $PrjRoot "${New}_prj"
$AL  = Join-Path $PRJ    "$PrjName.al"
$SDC = Join-Path $PRJ    "$PrjName.sdc"
$REL = '..\{0}_src' -f $New

if (-not (Test-Path $PRJ)) { New-Item -ItemType Directory -Force -Path $PRJ | Out-Null }

$PlatSrc = Join-Path $PlatRoot 'FS03_DR1V90GEG_osd_SC500_720P_src'
$RefPin  = Join-Path $PlatSrc 'constraints_source\pin.adc'

if (-not (Test-Path $SRC))    { Write-Error "missing source dir: $SRC (run 01_copy_source.ps1 first)" }
if (-not (Test-Path $RefPin)) { Write-Error "missing FS03 baseline pin.adc: $RefPin" }

# ---------------------------------------------------------------------
# 1. source files to compile (CompileOrder)
# ---------------------------------------------------------------------
$VFiles = @(
    'hdl_source\design_top_wrapper.v',
    'hdl_source\video_stream_bridge.v',
    'hdl_source\dr1_clk\pll_main.v',
    'hdl_source\dr1_clk\pll_cam24m.v',
    'hdl_source\dr1_clk\dr1_mipiio_rx_wrapper.v',
    'hdl_source\dr1_clk\DR1_PHY_PLL_WRAPERR_d2d4a8d4bd40.v',
    'hdl_source\ip_rtl\tdp_eram.v',
    'hdl_source\ip_rtl\blk_mem_gen_awb_delay_signal.v',
    'hdl_source\ip_rtl\blk_mem_gen_zhenghe.v',
    'hdl_source\ip_rtl\blk_mem_gen_demosaic.v',
    'hdl_source\signal_delay.v',
    'hdl_source\isp\csi_unpacket_2lane.v',
    'hdl_source\isp\raw10_unpacket_2lane.v',
    'hdl_source\isp\uial2axis.v',
    'hdl_source\isp\demosaic_4x_2_0\bilinear_interpolation.v',
    'hdl_source\isp\demosaic_4x_2_0\demosaic.v',
    'hdl_source\isp\demosaic_4x_2_0\line_buffer_demosaic.v',
    'hdl_source\isp\demosaic_4x_2_0\mipi_to_raw_converter.v',
    'hdl_source\isp\demosaic_4x_2_0\raw_matrix_3x3_buffer.v',
    'hdl_source\isp\demosaic_4x_2_0\zhenghe.v',
    'hdl_source\isp\data128_96\data128_96.v',
    'hdl_source\isp\data96_128\data96_128.v',
    'hdl_source\uics500_cfg\SC500GainTbl.v',
    'hdl_source\uics500_cfg\uicfgcs500_720p.v',
    'hdl_source\uics500_cfg\uics500regAE.v',
    'hdl_source\uics500_cfg\uii2c.v',
    'hdl_source\uiisp_beta\ae_set.v',
    'hdl_source\anlogic_logo_rom.v',
    'hdl_source\hdmi_mixer.v',
    'hdl_source\hdmi_tx.v',
    'hdl_source\hdmi_1_4b_transmitter_core_wrapper.enc.v',
    'hdl_source\hdmi_phy_warpper.v',
    'hdl_source\lane_lvds_10_1.v',
    'hdl_source\vtc\uivtc.v',
    'hdl_source\osd_char_lib.v',
    'hdl_source\isp\awb_gain_calc.v',
    'hdl_source\isp\awb_gain_mul.v',
    'hdl_source\isp\awb_lite.v',
    'hdl_source\img_proc\sobel3x3.v',
    'hdl_source\img_proc\mask3x3.v',
    'hdl_source\img_proc\ser_math.v',
    'hdl_source\img_proc\img_proc_top.v',
    'hdl_source\img_proc\hist_stat.v',
    'hdl_source\img_proc\hist_eq.v',
    'hdl_source\img_proc\frame_diff3.v',
    'hdl_source\img_proc\motion_eram.v',
    'hdl_source\img_proc\obj_detect.v',
    'hdl_source\img_proc\obj_feature_extract.v',
    'hdl_source\img_proc\mlp_weight_rom.v',
    'hdl_source\img_proc\mlp_classifier.v',
    'hdl_source\isp\isp_top_imgproc.v',
    'hdl_source\osd\osd_text8_lib.v',
    'hdl_source\osd\osd_cls_lut.v',
    'hdl_source\osd\osd_mode_overlay.v',
    'hdl_source\osd\osd_obj_overlay.v',
    'hdl_source\osd\osd_motion_overlay.v',
    'hdl_source\osd\osd_status_overlay.v',
    'hdl_source\ctrl\bin2bcd10.v',
    'hdl_source\ctrl\key_mode_delay_ctrl.v',
    'hdl_source\ctrl\tm1638_led_key.v',
    'hdl_source\ctrl\fps_meter.v',
    'hdl_source\ctrl\latency_meter.v',
    'hdl_source\ctrl\obj_tracker.v',
    'hdl_source\ctrl\buzz_ctrl.v',
    'hdl_source\ctrl\uart_report.v',
    'hdl_source\ctrl\ae_auto.v',
    'hdl_source\innov\cmd_parser.v',
    'hdl_source\innov\uart_cmd_rx.v',
    'hdl_source\innov\auto_demo_ctrl.v',
    'hdl_source\I2S_receiver.v',
    'hdl_source\audio_arc_calculate.v',
    'hdl_source\key_remove_shakes.v',
    'hdl_source\uics500_cfg\uics500reg_720p60.v'
)

$VExcludedCand = @(
    'hdl_source\awb.v',
    'hdl_source\isp\isp_top.v'
)
$VExcluded = @($VExcludedCand | Where-Object { Test-Path (Join-Path $SRC $_) })

$IpFiles = @('ip_source\divider\divider.ipc')

# ---------------------------------------------------------------------
# 2. source file integrity check
# ---------------------------------------------------------------------
$missing = @()
foreach ($f in ($VFiles + $VExcluded + $IpFiles)) {
    if (-not (Test-Path (Join-Path $SRC $f))) { $missing += $f }
}
if ($missing.Count) {
    Write-Error ("[check] missing source files (run 01_copy_source.ps1 first):`n  " + ($missing -join "`n  "))
}
foreach ($bad in @('hdl_source\ps_wrapper.v', 'ip_source\MCU', 'hdl_source\ph1p35_ddr',
                   'hdl_source\mipi_dphy_rx', 'hdl_source\video_in.v',
                   'hdl_source\video_out.v', 'hdl_source\mc_to_user_interface.v',
                   'hdl_source\seg7_display.v', 'hdl_source\ph1p35_324_ddr_wrapper.v')) {
    if (Test-Path (Join-Path $SRC $bad)) {
        Write-Error "[check] violates pure-PL / FS03 constraint: $bad should not exist"
    }
}
Write-Host ("[check] source files complete (Verilog {0} + AutoExcluded {1} + IP {2}), PL-only check passed." -f $VFiles.Count, $VExcluded.Count, $IpFiles.Count)

# ---------------------------------------------------------------------
# 3. generate pin.adc = FS03(DR1V90GEG484) baseline (line-by-line) + new pins
# ---------------------------------------------------------------------
$basePin = [IO.File]::ReadAllText($RefPin, [Text.Encoding]::UTF8)
$basePin = $basePin -replace 'FS03_DR1V90GEG_osd_SC500_720P', $New
if ($basePin -notmatch 'set_inst_assignment') {
    Write-Warning '[pin ] baseline pin.adc has no set_inst_assignment (MIPIIO bank location)'
}

$addPin = @'

# ===========================================================================
# New peripheral pins (beyond the baseline), per
# "在FS03_DR1V90GEG连接外设gpio最终引脚说明.md"
# CEP pin <-> FPGA ball mapping: see
# "外设硬件模块gpio引脚在FS03_DR1V90GEG的连接方式.md"
#
# NOTE: create1 bound 4 PS-ethernet placeholder pins (H19/G15/C17/D18) here.
# create2 is 100% PL (no PS, no ethernet), so those placeholder pins are
# REMOVED -> the 4 DIP-switch upper bits S5~S8 stay electrically unused.
# ===========================================================================

# ---- 1) DIP switch module (12-pin 2x6, PMOD-SWITCH) ----
#     CEP1(J11), module up, normal, start col 2, cols 2~7
#       S1=pin4=H20  S5=pin3=H19   GND=pin11/12=B19/B20   3V3=pin13/14=D22/C22
#       S2=pin6=G16  S6=pin5=G15
#       S3=pin8=C18  S7=pin7=C17
#       S4=pin10=C19 S8=pin9=D18
#     Low 4 bits S1~S4 map to HX SW1~SW4 (SOBEL / GRAY / BIN / status text)
set_pin_assignment	{ I_sw[0] }	{ LOCATION = H20; IOSTANDARD = LVCMOS33; PULLTYPE = NONE; }
set_pin_assignment	{ I_sw[1] }	{ LOCATION = G16; IOSTANDARD = LVCMOS33; PULLTYPE = NONE; }
set_pin_assignment	{ I_sw[2] }	{ LOCATION = C18; IOSTANDARD = LVCMOS33; PULLTYPE = NONE; }
set_pin_assignment	{ I_sw[3] }	{ LOCATION = C19; IOSTANDARD = LVCMOS33; PULLTYPE = NONE; }
# module power: GND driven 0 / 3V3 driven 1 by FPGA GPIO
set_pin_assignment	{ O_sw_gnd[0] }	{ LOCATION = B19; IOSTANDARD = LVCMOS33; DRIVESTRENGTH = 8; PULLTYPE = NONE; }
set_pin_assignment	{ O_sw_gnd[1] }	{ LOCATION = B20; IOSTANDARD = LVCMOS33; DRIVESTRENGTH = 8; PULLTYPE = NONE; }
set_pin_assignment	{ O_sw_vcc[0] }	{ LOCATION = D22; IOSTANDARD = LVCMOS33; DRIVESTRENGTH = 8; PULLTYPE = NONE; }
set_pin_assignment	{ O_sw_vcc[1] }	{ LOCATION = C22; IOSTANDARD = LVCMOS33; DRIVESTRENGTH = 8; PULLTYPE = NONE; }

# ---- 2) active buzzer module (3-pin 1x3, 9012 drive, low-level trigger) ----
#     CEP1(J11) bottom row, normal, start col 23, cols 23~25
#       VCC=pin46=D15  IO=pin48=B22  GND=pin50=G22
set_pin_assignment	{ O_buzz_vcc }	{ LOCATION = D15; IOSTANDARD = LVCMOS33; DRIVESTRENGTH = 8; PULLTYPE = NONE; }
set_pin_assignment	{ O_buzz }	    { LOCATION = B22; IOSTANDARD = LVCMOS33; DRIVESTRENGTH = 8; PULLTYPE = NONE; }
set_pin_assignment	{ O_buzz_gnd }	{ LOCATION = G22; IOSTANDARD = LVCMOS33; DRIVESTRENGTH = 8; PULLTYPE = NONE; }

# ---- 3) TM1638 digit+LED+key module (5-pin 1x5) ----
#     CEP2(J12) bottom row, normal, start col 2, cols 2~6
#       VCC=pin4=W12  GND=pin6=AB12  STB=pin8=Y8  CLK=pin10=AB11  DIO=pin12=Y10
#     Low 4 keys S1~S4 map to HX K1~K4; low 4 LEDs map to LED1~LED4;
#     8-digit tube high->low (left->right) maps to HX 8-digit tube.
set_pin_assignment	{ O_tm_vcc }	{ LOCATION = W12;  IOSTANDARD = LVCMOS33; DRIVESTRENGTH = 8; PULLTYPE = NONE; }
set_pin_assignment	{ O_tm_gnd }	{ LOCATION = AB12; IOSTANDARD = LVCMOS33; DRIVESTRENGTH = 8; PULLTYPE = NONE; }
set_pin_assignment	{ O_tm_stb }	{ LOCATION = Y8;   IOSTANDARD = LVCMOS33; DRIVESTRENGTH = 8; PULLTYPE = NONE; }
set_pin_assignment	{ O_tm_clk }	{ LOCATION = AB11; IOSTANDARD = LVCMOS33; DRIVESTRENGTH = 8; PULLTYPE = NONE; }
set_pin_assignment	{ IO_tm_dio }	{ LOCATION = Y10;  IOSTANDARD = LVCMOS33; DRIVESTRENGTH = 8; PULLTYPE = PULLUP; }

# ---- 4) UART command channel + status report (115200 8N1) ----
#     RX: CEP2(J12) bottom pin14 = AB9 (idle high via pullup)
#     TX: CEP2(J12) bottom pin16 = AA8
set_pin_assignment	{ I_uart_rx }	{ LOCATION = AB9;  IOSTANDARD = LVCMOS33; PULLTYPE = PULLUP; }
set_pin_assignment	{ O_uart_tx }	{ LOCATION = AA8;  IOSTANDARD = LVCMOS33; DRIVESTRENGTH = 8; PULLTYPE = NONE; }
'@

$dstPin = Join-Path $SRC 'constraints_source\pin.adc'
$pinTxt = $basePin.TrimEnd() + "`r`n`r`n" + $addPin.TrimStart("`r`n")
[IO.File]::WriteAllText($dstPin, $pinTxt, (New-Object System.Text.UTF8Encoding($false)))
Write-Host "[pin ] written: $dstPin"

# 3b. line-by-line consistency check vs FS03 baseline
function Get-PinLines([string]$p) {
    return ((Get-Content $p | Where-Object { $_ -match 'set_pin_assignment|set_inst_assignment' }) -join "`n")
}
$refSet = @((Get-PinLines $RefPin) -split "`n")
$dstSet = @((Get-PinLines $dstPin) -split "`n")
$lost = @()
foreach ($l in $refSet) { if ($dstSet -notcontains $l) { $lost += $l } }
if ($lost.Count) {
    Write-Error ("[check] pin.adc lost {0} FS03 baseline pins, first: {1}" -f $lost.Count, $lost[0])
}
$cntBase = $refSet.Count
$cntAll  = $dstSet.Count
Write-Host ("[check] pin.adc line-by-line identical to FS03(DR1V90GEG484) baseline (baseline {0} + new {1} = {2})" -f $cntBase, ($cntAll - $cntBase), $cntAll)

# 3c. verify each new pin
$expectNew = @(
    @{ S = 'I_sw[0]';    P = 'H20'  },
    @{ S = 'I_sw[1]';    P = 'G16'  },
    @{ S = 'I_sw[2]';    P = 'C18'  },
    @{ S = 'I_sw[3]';    P = 'C19'  },
    @{ S = 'O_buzz_vcc'; P = 'D15'  },
    @{ S = 'O_buzz';     P = 'B22'  },
    @{ S = 'O_buzz_gnd'; P = 'G22'  },
    @{ S = 'O_tm_stb';   P = 'Y8'   },
    @{ S = 'O_tm_clk';   P = 'AB11' },
    @{ S = 'IO_tm_dio';  P = 'Y10'  },
    @{ S = 'I_uart_rx';  P = 'AB9'  },
    @{ S = 'O_uart_tx';  P = 'AA8'  }
)
$pinAll = [IO.File]::ReadAllText($dstPin, [Text.Encoding]::UTF8)
$verifyFail = $false
foreach ($e in $expectNew) {
    $pat = '{0}[^{{}}]*\}}\s*\{{[^}}]*LOCATION\s*=\s*{1}\s*;' -f [regex]::Escape($e.S), $e.P
    if ($pinAll -match $pat) {
        Write-Host ("    PASS {0} = {1}" -f $e.S, $e.P)
    }
    else {
        Write-Host ("    FAIL {0} not constrained to {1}" -f $e.S, $e.P)
        $verifyFail = $true
    }
}
if ($verifyFail) { Write-Error '[check] new peripheral pin constraint incomplete' }
# ensure PS placeholder pins are GONE
foreach ($pp in @('H19','G15','C17','D18')) {
    if ($pinAll -match "LOCATION\s*=\s*$pp\s*;") {
        Write-Error ("[check] PS placeholder pin {0} must be removed in pure-PL project" -f $pp)
    }
}
Write-Host '[check] PS placeholder pins (H19/G15/C17/D18) removed -> pure PL confirmed'

# ---------------------------------------------------------------------
# 4. generate .sdc timing constraint
# ---------------------------------------------------------------------
$sdcTxt = @'
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
'@
[IO.File]::WriteAllText($SDC, $sdcTxt, (New-Object System.Text.UTF8Encoding($false)))
Write-Host "[sdc ] written: $SDC"

# 4b. constraint files must be UTF-8 without BOM (TD ADC parser chokes on BOM)
foreach ($cf in @($SDC, $dstPin)) {
    if (Test-Path $cf) {
        $bytes = [IO.File]::ReadAllBytes($cf)
        if ($bytes.Length -ge 3 -and $bytes[0] -eq 0xEF -and $bytes[1] -eq 0xBB -and $bytes[2] -eq 0xBF) {
            $txt = [IO.File]::ReadAllText($cf, [Text.Encoding]::UTF8)
            [IO.File]::WriteAllText($cf, $txt, (New-Object System.Text.UTF8Encoding($false)))
            Write-Host ("[bom ] stripped BOM: {0}" -f (Split-Path $cf -Leaf))
        }
    }
}

# 4c. HDMI TX / PHY device patch (DR1)
$hdmiTx = Join-Path $SRC 'hdl_source\hdmi_tx.v'
if (Test-Path $hdmiTx) {
    $t = Get-Content $hdmiTx -Raw -Encoding UTF8
    $n = $t -replace '(\.DEVICE\s*\(\s*")PH1P("\s*\))', '$1DR1$2'
    if ($n -ne $t) {
        [IO.File]::WriteAllText($hdmiTx, $n, (New-Object System.Text.UTF8Encoding($false)))
        Write-Host '[patch] hdmi_tx.v : DEVICE "PH1P" -> "DR1"'
    }
    else {
        Write-Host '[patch] hdmi_tx.v : DEVICE already DR1'
    }
}

# 4d. Logo bitmap block-RAM patch (identical to create1)
$mix = Join-Path $SRC 'hdl_source\hdmi_mixer.v'
if (-not (Test-Path $mix)) { Write-Error "[patch] missing $mix" }
$m = Get-Content $mix -Raw -Encoding UTF8
$nl = [Environment]::NewLine

if ($m -match 'S_logo_addr_raw' -and $m -match 'u_anlogic_logo_rom\s*\(\s*\.I_clk') {
    Write-Host '[patch] hdmi_mixer.v : Logo block-RAM patch already present, skip'
}
else {
    $m2 = $m -replace '(\r?\n)[ \t]*reg\s+\[14:0\]\s*S_logo_addr\s*;', '$1'
    if ($m2 -ne $m) { $m = $m2; Write-Host '[patch] hdmi_mixer.v : removed reg S_logo_addr declaration' }

    $m2 = $m -replace '(\r?\n)[ \t]*S_logo_addr\s*=\s*15''d0\s*;', '$1'
    if ($m2 -ne $m) { $m = $m2; Write-Host '[patch] hdmi_mixer.v : removed S_logo_addr default assign' }

    $m2 = $m -replace '(\r?\n)[ \t]*S_logo_addr\s*=\s*S_logo_y\s*\*\s*LOGO_W\s*\+\s*S_logo_x\s*;', '$1'
    if ($m2 -ne $m) { $m = $m2; Write-Host '[patch] hdmi_mixer.v : removed S_logo_addr 2d-stage assign' }

    $wireBlk = @(
        '    // create2 : Logo bitmap moved into block RAM; block RAM read is synchronous',
        '    // (address latched on clock edge, data appears next cycle). Pull address one',
        '    // stage early: S_x_1d/S_y_1d are the previous pixel coords, so addr(t) is',
        '    // exactly the address the 2d stage needs at t+1, block RAM outputs at t+1,',
        '    // without changing any pixel.',
        '    wire [14:0] S_logo_addr_raw = (S_y_1d - LOGO_Y) * LOGO_W + (S_x_1d - LOGO_X);',
        '    wire        S_logo_in_win   = (S_x_1d >= LOGO_X) && (S_x_1d < (LOGO_X + LOGO_W)) &&',
        '                                  (S_y_1d >= LOGO_Y) && (S_y_1d < (LOGO_Y + LOGO_H));',
        '    wire [14:0] S_logo_addr     = S_logo_in_win ? S_logo_addr_raw : 15''d0;',
        '',
        '    anlogic_logo_rom u_anlogic_logo_rom(',
        '        .I_clk   ( I_clk        ),   // create2 : block RAM synchronous read',
        '        .I_addr  ( S_logo_addr  ),',
        '        .O_pixel ( S_logo_pixel )',
        '    );'
    ) -join $nl

    $pat = '(?s)[ \t]*anlogic_logo_rom\s+u_anlogic_logo_rom\s*\(.*?\);'
    if ($m -match $pat) {
        $m = [regex]::Replace($m, $pat, ($nl + $wireBlk), 1)
        Write-Host '[patch] hdmi_mixer.v : Logo ROM changed to sync block RAM + address one stage early'
    }
    else {
        Write-Error '[patch] hdmi_mixer.v : anlogic_logo_rom instantiation not found'
    }

    if ($m -match 'reg\s+\[14:0\]\s*S_logo_addr' -or $m -match 'S_logo_addr\s*=\s*S_logo_y') {
        Write-Error '[patch] hdmi_mixer.v : stale S_logo_addr combinational logic remains'
    }

    [IO.File]::WriteAllText($mix, $m, (New-Object System.Text.UTF8Encoding($false)))
}

# ---------------------------------------------------------------------
# 5. generate .al project file
# ---------------------------------------------------------------------
function Esc([string]$p) { return $p -replace '&', '&amp;' }

$sb = New-Object System.Text.StringBuilder
[void]$sb.AppendLine('<?xml version="1.0" encoding="UTF-8"?>')
[void]$sb.AppendLine(('<Project Version="3" Minor="2" Path="{0}">' -f ($PRJ -replace '\\','/')))
[void]$sb.AppendLine('    <Project_Created_Time></Project_Created_Time>')
[void]$sb.AppendLine('    <TD_Encoding>UTF-8</TD_Encoding>')
[void]$sb.AppendLine('    <TD_Version>6.2.168116</TD_Version>')
[void]$sb.AppendLine("    <Name>$PrjName</Name>")
[void]$sb.AppendLine('    <HardWare>')
[void]$sb.AppendLine('        <Family>DR1</Family>')
[void]$sb.AppendLine('        <Device>DR1V90GEG484</Device>')
[void]$sb.AppendLine('        <Speed>-2</Speed>')
[void]$sb.AppendLine('    </HardWare>')
[void]$sb.AppendLine('    <Source_Files>')
[void]$sb.AppendLine('        <Verilog>')

$order = 1
foreach ($f in $VFiles) {
    [void]$sb.AppendLine(('            <File Path="{0}\{1}">' -f $REL, (Esc $f)))
    [void]$sb.AppendLine('                <FileInfo>')
    [void]$sb.AppendLine('                    <Attr Name="UsedInSyn" Val="true"/>')
    [void]$sb.AppendLine('                    <Attr Name="UsedInP&amp;R" Val="true"/>')
    [void]$sb.AppendLine('                    <Attr Name="BelongTo" Val="design_1"/>')
    [void]$sb.AppendLine(('                    <Attr Name="CompileOrder" Val="{0}"/>' -f $order))
    [void]$sb.AppendLine('                </FileInfo>')
    [void]$sb.AppendLine('            </File>')
    $order++
}
foreach ($f in $VExcluded) {
    [void]$sb.AppendLine(('            <File Path="{0}\{1}">' -f $REL, (Esc $f)))
    [void]$sb.AppendLine('                <FileInfo>')
    [void]$sb.AppendLine('                    <Attr Name="AutoExcluded" Val="true"/>')
    [void]$sb.AppendLine('                    <Attr Name="UsedInSyn" Val="true"/>')
    [void]$sb.AppendLine('                    <Attr Name="UsedInP&amp;R" Val="true"/>')
    [void]$sb.AppendLine('                    <Attr Name="BelongTo" Val="design_1"/>')
    [void]$sb.AppendLine(('                    <Attr Name="CompileOrder" Val="{0}"/>' -f $order))
    [void]$sb.AppendLine('                </FileInfo>')
    [void]$sb.AppendLine('            </File>')
    $order++
}
[void]$sb.AppendLine('        </Verilog>')
[void]$sb.AppendLine('        <ADC_FILE>')
[void]$sb.AppendLine(('            <File Path="{0}\constraints_source\pin.adc">' -f $REL))
[void]$sb.AppendLine('                <FileInfo>')
[void]$sb.AppendLine('                    <Attr Name="UsedInSyn" Val="true"/>')
[void]$sb.AppendLine('                    <Attr Name="UsedInP&amp;R" Val="true"/>')
[void]$sb.AppendLine('                    <Attr Name="BelongTo" Val="constraint_1"/>')
[void]$sb.AppendLine('                    <Attr Name="CompileOrder" Val="1"/>')
[void]$sb.AppendLine('                </FileInfo>')
[void]$sb.AppendLine('            </File>')
[void]$sb.AppendLine('        </ADC_FILE>')
[void]$sb.AppendLine('        <SDC_FILE>')
[void]$sb.AppendLine(('            <File Path="{0}.sdc">' -f $PrjName))
[void]$sb.AppendLine('                <FileInfo>')
[void]$sb.AppendLine('                    <Attr Name="UsedInSyn" Val="true"/>')
[void]$sb.AppendLine('                    <Attr Name="UsedInP&amp;R" Val="true"/>')
[void]$sb.AppendLine('                    <Attr Name="BelongTo" Val="constraint_1"/>')
[void]$sb.AppendLine('                    <Attr Name="CompileOrder" Val="2"/>')
[void]$sb.AppendLine('                </FileInfo>')
[void]$sb.AppendLine('            </File>')
[void]$sb.AppendLine('        </SDC_FILE>')
[void]$sb.AppendLine('        <IP_FILE>')
$order = $VFiles.Count + $VExcluded.Count + 1
foreach ($f in $IpFiles) {
    [void]$sb.AppendLine(('            <File Path="{0}\{1}">' -f $REL, (Esc $f)))
    [void]$sb.AppendLine('                <FileInfo>')
    [void]$sb.AppendLine('                    <Attr Name="UsedInSyn" Val="true"/>')
    [void]$sb.AppendLine('                    <Attr Name="UsedInP&amp;R" Val="true"/>')
    [void]$sb.AppendLine('                    <Attr Name="BelongTo" Val="design_1"/>')
    [void]$sb.AppendLine(('                    <Attr Name="CompileOrder" Val="{0}"/>' -f $order))
    [void]$sb.AppendLine('                </FileInfo>')
    [void]$sb.AppendLine('            </File>')
    $order++
}
[void]$sb.AppendLine('        </IP_FILE>')
[void]$sb.AppendLine('    </Source_Files>')
[void]$sb.AppendLine('    <FileSets>')
[void]$sb.AppendLine('        <FileSet Name="design_1" Type="DesignFiles">')
[void]$sb.AppendLine('        </FileSet>')
[void]$sb.AppendLine('        <FileSet Name="constraint_1" Type="ConstrainFiles">')
[void]$sb.AppendLine('        </FileSet>')
[void]$sb.AppendLine('    </FileSets>')
[void]$sb.AppendLine('    <TOP_MODULE>')
[void]$sb.AppendLine('        <LABEL>design_top_wrapper</LABEL>')
[void]$sb.AppendLine('        <MODULE>design_top_wrapper</MODULE>')
[void]$sb.AppendLine('        <CREATEINDEX>user</CREATEINDEX>')
[void]$sb.AppendLine('    </TOP_MODULE>')
[void]$sb.AppendLine('    <Property>')
[void]$sb.AppendLine('    </Property>')
[void]$sb.AppendLine('    <Device_Settings>')
[void]$sb.AppendLine('    </Device_Settings>')
[void]$sb.AppendLine('    <Configurations>')
[void]$sb.AppendLine('        <Control0>')
[void]$sb.AppendLine('            <mclk_freq>33MHz</mclk_freq>')
[void]$sb.AppendLine('        </Control0>')
[void]$sb.AppendLine('        <FeatureRow>')
[void]$sb.AppendLine('            <boot_mode>mspix4</boot_mode>')
[void]$sb.AppendLine('        </FeatureRow>')
[void]$sb.AppendLine('    </Configurations>')
[void]$sb.AppendLine('    <Runs>')
[void]$sb.AppendLine('        <Run Name="syn_1" Type="Synthesis" ConstraintSet="constraint_1" Description="" Active="true" useDefaultScript="false">')
[void]$sb.AppendLine('            <Strategy Name="Default_Synthesis_Strategy">')
[void]$sb.AppendLine('            </Strategy>')
[void]$sb.AppendLine('            <UserParams>')
[void]$sb.AppendLine('            </UserParams>')
[void]$sb.AppendLine('        </Run>')
[void]$sb.AppendLine('        <Run Name="phy_1" Type="PhysicalDesign" ConstraintSet="constraint_1" Description="" SynRun="syn_1" Active="true">')
[void]$sb.AppendLine('            <Strategy Name="Default_PhysicalDesign_Strategy">')
[void]$sb.AppendLine('                <BitgenProperty::GeneralOption>')
[void]$sb.AppendLine('                    <compress>on</compress>')
[void]$sb.AppendLine('                </BitgenProperty::GeneralOption>')
[void]$sb.AppendLine('            </Strategy>')
[void]$sb.AppendLine('            <UserParams>')
[void]$sb.AppendLine('            </UserParams>')
[void]$sb.AppendLine('        </Run>')
[void]$sb.AppendLine('    </Runs>')
[void]$sb.AppendLine('    <Project_Settings>')
[void]$sb.AppendLine('    </Project_Settings>')
[void]$sb.AppendLine('</Project>')

[IO.File]::WriteAllText($AL, $sb.ToString(), (New-Object System.Text.UTF8Encoding($false)))
Write-Host ("[al  ] written: {0}" -f $AL)
Write-Host ("[al  ] verilog = {0}, auto-excluded = {1}, ip = {2}, adc = 1, sdc = 1, top = design_top_wrapper" -f $VFiles.Count, $VExcluded.Count, $IpFiles.Count)

# ---------------------------------------------------------------------
# 6. round-trip check: every file listed in .al really exists
# ---------------------------------------------------------------------
$alText = [IO.File]::ReadAllText($AL)
$lost = @()
foreach ($m in [regex]::Matches($alText, '<File Path="([^"]+)"')) {
    $entry = $m.Groups[1].Value
    if ($entry -match '^[A-Za-z]:' -or $entry.StartsWith('/')) {
        $full = $entry -replace '/', '\'
    }
    else {
        $full = [IO.Path]::GetFullPath((Join-Path $PRJ ($entry -replace '/', '\')))
    }
    if (-not (Test-Path -LiteralPath $full)) { $lost += $entry }
}
if ($lost.Count) {
    $lost | Select-Object -First 10 | ForEach-Object { Write-Host "MISSING $_" }
    Write-Error ("[check] .al lists {0} non-existent files" -f $lost.Count)
}
Write-Host '[check] .al round-trip check: all files exist'

Write-Host ''
Write-Host '== 02_create_project.ps1 done =='
Write-Host 'Next: 03_build_td.ps1 (TD batch synthesis / implementation)'
