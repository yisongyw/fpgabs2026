# =====================================================================
#  01_copy_source.ps1
#  Copy FS03_DR1V90GEG_img_process_create1 (pure-PL image processing) into
#  FS03_DR1V90GEG_img_process_create2_sc520, then:
#    * REMOVE every PS-related artifact (ethernet EMIO GPIO bridge port
#      I_ps_key_evt, the S_key_press merge, the ps_app/ folder, the 4 PS
#      placeholder pin constraints) -> the new project is 100% PL.
#    * KEEP the UART short-string command channel (K1..K4 == physical keys),
#      which already exists in create1 and satisfies the "UART backup for all
#      fpga keys (not DIP switches)" requirement.
#    * ADD a pure-PL innovation: auto_demo_ctrl (Auto-Demo Patrol) toggled by
#      a new UART command "DEMO" (same 115200 8N1 channel). It cycles the
#      16 style levels and sweeps the 16 threshold levels hands-free and
#      shows a LED chase on the TM1638, reusing the existing key-control path.
#
#  Usage: .\01_copy_source.ps1
# =====================================================================
[CmdletBinding()]
param(
    [string]$Root = 'D:\Documents\bc10anlogictd',
    [string]$New  = 'FS03_DR1V90GEG_img_process_create2_sc520',
    [string]$SRC1 = 'FS03_DR1V90GEG_img_process_create1'
)

$ErrorActionPreference = 'Stop'

$NewSrc = "${New}_src"
$DST    = Join-Path $Root $New
$DSTSRC = Join-Path $DST  $NewSrc
$SRC1SRC= Join-Path $Root "$SRC1\$($SRC1)_src"

if (-not (Test-Path $SRC1SRC)) { Write-Error "missing source project: $SRC1SRC" }

# 1. directory skeleton (NO ps_app -> pure PL)
@(
    $DST,
    (Join-Path $DSTSRC 'hdl_source'),
    (Join-Path $DSTSRC 'hdl_source\ctrl'),
    (Join-Path $DSTSRC 'hdl_source\img_proc'),
    (Join-Path $DSTSRC 'hdl_source\osd'),
    (Join-Path $DSTSRC 'hdl_source\isp'),
    (Join-Path $DSTSRC 'hdl_source\uics500_cfg'),
    (Join-Path $DSTSRC 'hdl_source\uiisp_beta'),
    (Join-Path $DSTSRC 'hdl_source\vtc'),
    (Join-Path $DSTSRC 'hdl_source\innov'),
    (Join-Path $DSTSRC 'constraints_source'),
    (Join-Path $DST 'sim')
) | ForEach-Object { New-Item -ItemType Directory -Force -Path $_ | Out-Null }

# 2. copy the entire create1 source tree (hdl_source / constraints_source / ip_source)
Copy-Item -Path (Join-Path $SRC1SRC '*') -Destination $DSTSRC -Recurse -Force

# remove any stale ps_app that might have slipped in (should not exist)
$psApp = Join-Path $DST 'ps_app'
if (Test-Path $psApp) { Remove-Item $psApp -Recurse -Force }

# 3. remove PS ethernet bridge from the top-level design_top_wrapper.v
$top = Join-Path $DSTSRC 'hdl_source\design_top_wrapper.v'
if (-not (Test-Path $top)) { Write-Error "missing top: $top" }
$t = [IO.File]::ReadAllText($top, [Text.Encoding]::UTF8)

# 3.1 title
$t = $t -replace 'FS03_DR1V90GEG_img_process_create1  --  design_top_wrapper\.v',
                'FS03_DR1V90GEG_img_process_create2_sc520  --  design_top_wrapper.v'
$t = $t -replace 'FS03_DR1V90GEG_img_process_create1', 'FS03_DR1V90GEG_img_process_create2_sc520'

# 3.2 remove the PS port (comment + input line), keep the closing ");"
$t = [regex]::Replace($t,
    '\r?\n\s*// ---- PS -> PL ethernet command bridge.*?\r?\n\s*input\s+wire\s+\[3:0\]\s+I_ps_key_evt[^\r\n]*\r?\n',
    "`r`n")
if ($t -match 'input\s+wire\s+\[3:0\]\s+I_ps_key_evt') { Write-Error 'PS port removal failed: I_ps_key_evt port still present' }

# 3.2b the PS port was the last port; after removal the preceding port keeps a
#      trailing comma right before ");" which TD rejects (HDL-8007). Drop it.
$t = $t -replace 'output\s+wire\s+O_uart_tx\s*,(\s*\r?\n\s*)\)', 'output wire        O_uart_tx$1)'

# 3.3 remove PS from S_key_press merge
$reKp = 'wire\s+\[3:0\]\s+S_key_press\s*=\s*\{[\s\S]*?\};'
$newKp = '    wire [3:0] S_key_press = { S_tm_key[3] | S_uh3,' +
         '                               S_tm_key[2] | S_uh2,' +
         '                               S_tm_key[1] | S_uh1 | ~I_button,' +
         '                               S_tm_key[0] | S_uh0 };'
if ($t -match $reKp) { $t = [regex]::Replace($t, $reKp, $newKp) }
else { Write-Error 'S_key_press block not found' }
if ($t -match 'I_ps_key_evt') { Write-Error 'S_key_press still references I_ps_key_evt' }

# 3.4 add DEMO event wire + connect uart_cmd_rx.O_demo_evt
$t = $t -replace 'wire\s+S_uart_rx_act;',
        "wire S_uart_rx_act;`r`n    wire S_demo_evt;"
$t = $t -replace '\.O_rx_act\s*\(\s*S_uart_rx_act\s*\)',
        ".O_rx_act   ( S_uart_rx_act   ),`r`n        .O_demo_evt ( S_demo_evt      )"

# 3.5 OR auto-demo pulses into the existing key-hold registers
$t = $t -replace 'r_hold1 <= S_uart_key_evt\[1\] \? HOLD_MAX',
        'r_hold1 <= (S_uart_key_evt[1] | S_demo_k2) ? HOLD_MAX'
$t = $t -replace 'r_hold2 <= S_uart_key_evt\[2\] \? HOLD_MAX',
        'r_hold2 <= (S_uart_key_evt[2] | S_demo_k3) ? HOLD_MAX'

# 3.6 insert the auto-demo control block at MODULE level (before HOLD_MAX,
#     which is module-level, i.e. OUTSIDE the key-hold always block). ASCII anchor.
$anchor = 'localparam [21:0] HOLD_MAX = 22' + [char]39 + 'd4194303;'
if (-not $t.Contains($anchor)) { Write-Error 'auto-demo insertion anchor not found' }
$ins = @'
    // ===================== Auto-Demo Patrol (create2 new, pure PL) =====================
    // Toggled by UART command "DEMO". When enabled it periodically produces KEY3
    // (style+1) / KEY2 (threshold+1) 1-clock pulses, auto-iterating all algorithm
    // combinations hands-free, and drives a LED chase on the TM1638. It reuses the
    // same key-control path as the physical/UART keys, so no new CDC is introduced.
    reg S_demo_en;
    always @(posedge S_100m_clk or negedge S_sys_rst_n) begin
        if (!S_sys_rst_n) S_demo_en <= 1'b0;
        else if (S_demo_evt) S_demo_en <= ~S_demo_en;
    end

    wire S_demo_k2, S_demo_k3;
    wire [3:0] S_demo_chase;
    auto_demo_ctrl #(
        .CLK_FREQ ( 100000000 )
    ) u_auto_demo (
        .I_clk       ( S_100m_clk  ),
        .I_rst_n     ( S_sys_rst_n ),
        .I_en        ( S_demo_en   ),
        .O_key3_pulse( S_demo_k3   ),
        .O_key2_pulse( S_demo_k2   ),
        .O_led_chase ( S_demo_chase)
    );

'@
$t = $t.Replace($anchor, $ins + $anchor)

# 3.7 expand S_mod_led to 8 bits and show the demo chase on LED5..LED2 when on
$reLed = 'wire\s+\[7:0\]\s+S_mod_led\s*=\s*\{[\s\S]*?S_pll_lock\s*\};'
$newLed = '    wire [7:0] S_mod_led = {' + [Environment]::NewLine +
         '        S_demo_en,                                                            // LED8 auto-demo active (steady)' + [Environment]::NewLine +
         '        r_alarm_100_d1,                                                       // LED7 ROI alarm' + [Environment]::NewLine +
         '        (S_uart_act_led != 24''d0),                                           // LED6 UART command activity' + [Environment]::NewLine +
         '        S_demo_en ? S_demo_chase[3] : (S_en_sobel | S_en_gray | S_en_bin),    // LED5 algorithm on / demo' + [Environment]::NewLine +
         '        S_demo_en ? S_demo_chase[2] : S_dbg_sync_cam_cfg[1],                  // LED4 camera config done' + [Environment]::NewLine +
         '        S_demo_en ? S_demo_chase[1] : S_video_valid,                          // LED3 video link' + [Environment]::NewLine +
         '        S_demo_en ? S_demo_chase[0] : S_pll_lock,                             // LED2 PLL lock' + [Environment]::NewLine +
         '        1''b0                                                                // LED1 reserved' + [Environment]::NewLine +
         '    };'
if ($t -match $reLed) { $t = [regex]::Replace($t, $reLed, $newLed) }
else { Write-Error 'S_mod_led block not found' }

[IO.File]::WriteAllText($top, $t, (New-Object System.Text.UTF8Encoding($false)))

# 4. patch cmd_parser.v to support the "DEMO" command
$cp = Join-Path $DSTSRC 'hdl_source\innov\cmd_parser.v'
$c = [IO.File]::ReadAllText($cp, [Text.Encoding]::UTF8)
$c = $c -replace 'output reg\s+O_rep_evt,', "output reg O_rep_evt,`r`n    output reg O_demo_evt,"
$mDemoRep = 'wire m_rp = (w2 && (c1 == "R") && (c0 == "?"));' + [Environment]::NewLine +
             '    wire m_demo = (w4 && (c3 == "D") && (c2 == "E") && (c1 == "M") && (c0 == "O"));'
$c = $c -replace 'wire\s+m_rp\s*=\s*\(w2 && \(c1 == "R"\) && \(c0 == "\?"\)\);', $mDemoRep
$c = $c -replace 'm_k4 \| m_sd \| m_cam \| m_ld \| m_rp',
        'm_k4 | m_sd | m_cam | m_ld | m_rp | m_demo'
$c = $c -replace 'O_rep_evt\s*<=\s*1''b0;', "O_rep_evt   <= 1'b0;`r`n            O_demo_evt  <= 1'b0;"
$c = $c -replace 'O_rep_evt\s*<=\s*m_rp;', "O_rep_evt   <= m_rp;`r`n                    O_demo_evt  <= m_demo;"
[IO.File]::WriteAllText($cp, $c, (New-Object System.Text.UTF8Encoding($false)))
if (-not ($c -match 'O_demo_evt')) { Write-Error 'cmd_parser DEMO patch failed' }

# 5. patch uart_cmd_rx.v to forward O_demo_evt
$ur = Join-Path $DSTSRC 'hdl_source\innov\uart_cmd_rx.v'
$u = [IO.File]::ReadAllText($ur, [Text.Encoding]::UTF8)
$u = $u -replace 'output wire\s+O_rep_evt,', "output wire O_rep_evt,`r`n    output wire O_demo_evt,"
$u = $u -replace '\.O_rep_evt\s*\(\s*O_rep_evt\s*\)', ".O_rep_evt   ( O_rep_evt  ),`r`n        .O_demo_evt ( O_demo_evt )"
[IO.File]::WriteAllText($ur, $u, (New-Object System.Text.UTF8Encoding($false)))
if (-not ($u -match 'O_demo_evt')) { Write-Error 'uart_cmd_rx DEMO patch failed' }

# 6. write the new auto_demo_ctrl.v innovation module
$adc = Join-Path $DSTSRC 'hdl_source\innov\auto_demo_ctrl.v'
$adcTxt = @'
// ===========================================================================
// FS03_DR1V90GEG_img_process_create2_sc520  --  auto_demo_ctrl.v
//
// Pure-PL innovation module: Auto-Demo Patrol
//   * Toggled by UART command "DEMO" (same 115200 8N1 channel as K1..K4).
//   * When enabled it periodically produces KEY3(style+1) / KEY2(threshold+1)
//     single-clock pulses, auto-iterating the 16 style levels and sweeping the
//     16 threshold levels hands-free, ideal for live demo / unattended showcase.
//   * Shares the same control path as the physical/UART keys
//     (S_key_press -> key_mode_delay_ctrl): no new CDC, no change to the pixel
//     main link, so end-to-end latency is unchanged.
//   * Also outputs O_led_chase[3:0] for a TM1638 LED "chase" status indicator.
// ===========================================================================
`timescale 1ns / 100ps

module auto_demo_ctrl #(
    parameter CLK_FREQ = 100_000_000
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_en,          // 1 = auto-demo on
    output reg         O_key3_pulse,  // style +1 pulse (1 clk)
    output reg         O_key2_pulse,  // threshold +1 pulse (1 clk)
    output reg  [3:0]  O_led_chase    // chase (LED1..LED4)
);

    // style pulse every 2.0 s ; threshold pulse every 1.0 s ; chase every 0.5 s
    localparam integer DIV_STYLE = CLK_FREQ * 2;
    localparam integer DIV_TH    = CLK_FREQ * 1;

    reg [31:0] r_cnt_style;
    reg [31:0] r_cnt_th;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            r_cnt_style <= 32'd0;
            r_cnt_th    <= 32'd0;
            O_key3_pulse<= 1'b0;
            O_key2_pulse<= 1'b0;
            O_led_chase <= 4'b0001;
        end
        else begin
            O_key3_pulse <= 1'b0;
            O_key2_pulse <= 1'b0;

            if (!I_en) begin
                r_cnt_style <= 32'd0;
                r_cnt_th    <= 32'd0;
                O_led_chase <= 4'b0001;
            end
            else begin
                // style pulse (every 2 s)
                if (r_cnt_style == (DIV_STYLE - 1)) begin
                    r_cnt_style <= 32'd0;
                    O_key3_pulse <= 1'b1;
                end
                else
                    r_cnt_style <= r_cnt_style + 32'd1;

                // threshold pulse (every 1 s)
                if (r_cnt_th == (DIV_TH - 1)) begin
                    r_cnt_th <= 32'd0;
                    O_key2_pulse <= 1'b1;
                end
                else
                    r_cnt_th <= r_cnt_th + 32'd1;

                // chase (every 0.5 s)
                if (r_cnt_th == (DIV_TH/2 - 1))
                    O_led_chase <= {O_led_chase[2:0], O_led_chase[3]};
            end
        end
    end

endmodule
'@
[IO.File]::WriteAllText($adc, $adcTxt, (New-Object System.Text.UTF8Encoding($false)))

# 7. copy simulation files + the original 工程说明 (to be updated by hand)
$CfgSim = Join-Path $Root "$SRC1\sim"
if (Test-Path $CfgSim) { Copy-Item -Path (Join-Path $CfgSim '*') -Destination (Join-Path $DST 'sim') -Recurse -Force }

$CfgDoc = Join-Path $Root "$SRC1\工程说明.md"
if (Test-Path $CfgDoc) {
    Copy-Item -Path $CfgDoc -Destination (Join-Path $DST '工程说明_create1原版.md') -Force
    Copy-Item -Path $CfgDoc -Destination (Join-Path $DST '工程说明.md') -Force
}
$CfgHx = Join-Path $Root "$SRC1\工程说明_HX原始.md"
if (Test-Path $CfgHx) { Copy-Item -Path $CfgHx -Destination (Join-Path $DST '工程说明_HX原始.md') -Force }

# 8. health check: forbid any PS / processor / DDR artifact
$bad = @('ps_wrapper.v', 'ph1p35_ddr', 'mipi_dphy_rx', 'video_in.v', 'video_out.v',
          'mc_to_user_interface.v', 'seg7_display.v', 'ph1p35_324_ddr_wrapper.v',
          'I_ps_key_evt', 'ps_app')
foreach ($b in $bad) {
    if (Test-Path (Join-Path $DSTSRC "hdl_source\$b")) { Write-Error "forbidden file leftover: $b" }
}
if (Test-Path $psApp) { Write-Error 'ps_app folder must not exist' }

Write-Host "[copy] sources copied -> $DSTSRC (PS removed, auto_demo added)"
Write-Host "[copy] next run 02_create_project.ps1"
