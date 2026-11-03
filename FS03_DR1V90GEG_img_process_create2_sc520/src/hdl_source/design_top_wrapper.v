// ===========================================================================
// FS03_DR1V90GEG_img_process_create2_sc520  --  design_top_wrapper.v
//
// 由 HX1P35Aimg_process_create1（HX1P35A / PH1P35MDG324，"64 类目标识别 +
// 性能优化"）迁移到 FS03（米联派 DR1V90GEG484）。功能完全保持不变：
//
//   SC520CS (2 lane MIPI RAW10 1280x720@60; board MIPI lane clock 480 Mbps)
//     -> DR1_PHY_MIPIIO -> CSI-2 解包 -> RAW10 解包 -> AXI-Stream
//     -> demosaic -> awb_lite -> img_proc_top
//          layer 1 : SOBEL / GRAY / BIN，由 SW1/SW2/SW3 独立开关
//          layer 2 : 风格化 / 画质增强，由 KEY3 选择（0~3 原 4 档，
//                    4=HE 5=CLAHE 6=STRETCH）
//     -> hist_stat / hist_eq 画质级（模式 0 为逐像素旁路）
//     -> 96bit -> 128bit -> 片上 ERAM 弹性行缓存（跨时钟域，取代 DDR2 三帧）
//     -> hdmi_mixer（安路 Logo + "an lu CNT:"）
//     -> osd_mode_overlay（MODE + TH）
//     -> osd_obj_overlay（目标框 + 类别名 + ID + 轨迹，KEY4 选择 rule / MLP）
//     -> osd_motion_overlay（运动热力图 + 运动包围盒，improve1）
//     -> osd_status_overlay（12 行状态 + 延迟条，左下角）
//     -> HDMI TX 1280x720@60
//
//   目标检测（obj_detect + obj_feature_extract + INT8 MLP 8-32-64）与跟踪
//   （obj_tracker + 时间投票）、三帧差分运动检测（frame_diff3 + motion_eram）
//   均为纯侧路，不修改任何像素、不增加端到端延时。
//
//   控制（与 HX1P35Aimg_process_create1 一一对应）：
//     模块拨码 S1 = SW1 SOBEL / S2 = SW2 GRAY / S3 = SW3 BIN /
//              S4 = SW4 左下状态文字与检测框显隐
//     模块按键 S1 = K1 系统复位（电平型）
//              S2 = K2 阈值 +1（16 档 0/20/.../300，松手 1 s 后生效）
//              S3 = K3 风格档 +1（16 档，松手 1 s 后生效）
//              S4 = K4 分类器切换（0 = 规则，1 = INT8 MLP）
//     模块 LED1 = PLL 锁定 / LED2 = 视频链路就绪（替代 DDR2 初始化完成） /
//          LED3 = 摄像头配置完成 / LED4 = 至少一个基础算法被打开
//     模块 8 位数码管 = 当前阈值（十进制，个位在最右一位）
//     有源蜂鸣器      = ROI 入侵报警
//     UART TX         = 115200 8N1 状态上报（与 improve4 报文一致）
//     UART RX         = 115200 8N1 短字符串命令，"保险"地用 PC 复现
//                       K1~K4 四个按键的全部功能（与
//                       HX1P35Aimg_process_create1 设置完全一致）
//
// 平台适配（相对 FS03_DR1V90GEG_osd_SC500_720P 基线）：
//   1) 帧缓存：DR1V90GEG 的 DDR3 归 PS，本工程只走 PL，沿用基线的片上
//      ERAM 弹性行缓存 video_stream_bridge.v（取代 DDR2 三帧乒乓）。
//      端到端延时由约 3 帧（50 ms）降到十几行（约 0.3 ms）。
//   2) ISP：isp_top.v -> isp_top_imgproc.v（含画质统计 / 画质增强 / 运动检测）。
//   3) 人机交互外设改为外接模块（见
//      D:\Documents\bc10anlogictd\在FS03_DR1V90GEG连接外设gpio最终引脚说明.md）：
//        - 小拨码开关模块（CEP1/J11，模组朝上，起始列 2）S1~S4
//        - TM1638 数码管+LED+按键模块（CEP2/J12，底行，起始列 2）
//        - 有源蜂鸣器模块（CEP1/J11，底行，起始列 23）
//   4) 时钟：25 MHz 晶振 -> pll_main(100/75/375 M) + pll_cam24m(24 M)。
//
// 纯 PL 实现：无 PS、无处理器、无 DDR、无 DDR 控制器。
// ===========================================================================
`timescale 1ns / 100ps

module design_top_wrapper #(
    // ===== SINGLE SOURCE OF TRUTH : 类别数只在这里改 =====
    parameter CLS_W = 6,            // 6 bit 类别索引 -> 64 类
    parameter NOUT  = 64,           // INT8 MLP 输出 64 类
    parameter REC_W = 50 + 2*CLS_W, // 单条目标记录位宽（CLS_W=6 -> 62）
    parameter TRK_W = 58 + 2*CLS_W, // 单条轨迹记录位宽（CLS_W=6 -> 70）
    parameter DYN_W = 72,           // 单条动态指示记录位宽
    parameter NOBJ  = 2,            // 每帧保留的目标数
    parameter NTRK  = 2             // 常驻轨迹槽数
)(
    // ---- 系统时钟 / 复位（FS03 板载）----
    input  wire        I_sysclk,        // 25 MHz, L18
    input  wire        I_rstn,          //        L19
    // ---- SC520 摄像头控制（FS03 MIPI 座）----
    output wire        O_cam_scl,
    inout  wire        IO_cam_sda,
    output wire        O_cam_24m,       // 24 MHz 摄像头基准时钟
    output wire        O_cam_rst,
    // ---- 板载独立按键（备用：与模块 K2 同功能）----
    input  wire        I_button,        // KEY2 @K18，按下 = 0
    // ---- 板载 LED（链路诊断）----
    output wire [7:0]  O_led,
    // ---- HDMI 输出（LVDS33_E 差分对，N 腿由工具自动配对）----
    output wire        O_hdmi_clk_p,
    output wire        O_hdmi_clk_n,
    output wire [2:0]  O_hdmi_tx_p,
    output wire [2:0]  O_hdmi_tx_n,
    // ---- MIPI CSI-2 差分输入 ----
    inout  wire        IO_rx_clk_pad_n,
    inout  wire        IO_rx_clk_pad_p,
    inout  wire [3:0]  IO_rx_data_pad_n,
    inout  wire [3:0]  IO_rx_data_pad_p,
    // ---- 小拨码开关模块（CEP1/J11，模组朝上，正常方向，起始列 2）----
    input  wire [3:0]  I_sw,            // S1..S4 -> SW1..SW4
    output wire [1:0]  O_sw_gnd,        // 模块 GND（FPGA 驱动 0）
    output wire [1:0]  O_sw_vcc,        // 模块 3V3（FPGA 驱动 1）
    // ---- 有源蜂鸣器模块（CEP1/J11，底行，正常方向，起始列 23）----
    output wire        O_buzz_vcc,      // 模块 VCC（FPGA 驱动 1）
    output wire        O_buzz,          // 模块 IO，低电平触发
    output wire        O_buzz_gnd,      // 模块 GND（FPGA 驱动 0）
    // ---- TM1638 数码管+LED+按键模块（CEP2/J12，底行，正常方向，起始列 2）----
    output wire        O_tm_vcc,        // 模块 VCC（FPGA 驱动 1）
    output wire        O_tm_gnd,        // 模块 GND（FPGA 驱动 0）
    output wire        O_tm_stb,
    output wire        O_tm_clk,
    inout  wire        IO_tm_dio,
    // ---- UART 命令通道（115200 8N1，PC -> FPGA）----
    input  wire        I_uart_rx,
    // ---- UART 状态上报（115200 8N1，FPGA -> PC，与 improve4 报文一致）----
    output wire        O_uart_tx
);

    // ===================== 外设供电（由 FPGA GPIO 提供）=====================
    assign O_sw_gnd   = 2'b00;
    assign O_sw_vcc   = 2'b11;
    assign O_tm_vcc   = 1'b1;
    assign O_tm_gnd   = 1'b0;
    assign O_buzz_vcc = 1'b1;
    assign O_buzz_gnd = 1'b0;
    assign O_cam_rst  = 1'b1;
    assign O_cam_24m  = S_24m_clk;

    // ===================== 时钟与复位 =====================
    wire S_100m_clk;                     // 100 MHz（辅助逻辑 / 外设扫描）
    wire S_24m_clk;                      // 24 MHz（摄像头基准 + SCCB）
    wire S_hdmi_pixel_clk;               // 75 MHz（HDMI 像素时钟）
    wire S_hdmi_serial_clk;              // 375 MHz（HDMI 5x 串行时钟）
    wire S_pll_lock;
    wire S_cam_pll_lock;

    pll_main u_pll_main (
        .refclk     ( I_sysclk          ),
        .pllreset   ( ~I_rstn           ),
        .clk0_out   ( S_100m_clk        ),
        .clk4_out   ( S_hdmi_pixel_clk  ),
        .clk5_out   ( S_hdmi_serial_clk ),
        .lock       ( S_pll_lock        )
    );

    pll_cam24m u_pll_cam24m (
        .refclk     ( I_sysclk          ),
        .pllreset   ( ~I_rstn           ),
        .clk0_out   (                   ),
        .clk1_out   ( S_24m_clk         ),
        .lock       ( S_cam_pll_lock    )
    );

    // 只与"上电 / 外部复位"相关的复位：UART 与 TM1638 使用，保证 K1 复位
    // 期间命令通道仍然工作（否则虚拟按键会自锁）。
    wire S_sys_rst_n = S_pll_lock & S_cam_pll_lock & I_rstn;

    // ===================== 按键（TM1638 模块 S1~S4 + UART 虚拟按键）=====
    wire [7:0] S_tm_key;        // S1..S8，1 = 按下
    wire       S_tm_valid;

    // ---- UART 命令前端（115200 8N1，与 create1 完全一致）----
    wire [3:0] S_uart_key_evt;  // K1..K4 虚拟按键脉冲（1 个时钟宽）
    wire S_uart_rx_act;
    wire S_demo_evt;

    uart_cmd_rx #(
        .CLK_FREQ ( 100000000 ),
        .BAUD     ( 115200    )
    ) u_uart_cmd_rx (
        .I_clk      ( S_100m_clk      ),
        .I_rst_n    ( S_sys_rst_n     ),
        .I_uart_rx  ( I_uart_rx       ),
        .O_key_evt  ( S_uart_key_evt  ),
        .O_sd_evt   (                 ),
        .O_cam_evt  (                 ),
        .O_ld_evt   (                 ),
        .O_rep_evt  (                 ),
        .O_cmd_valid(                 ),
        .O_rx_act   ( S_uart_rx_act   ),
        .O_demo_evt ( S_demo_evt      )
    );

    // 把 1 时钟宽的虚拟按键脉冲展宽到约 42 ms，
    // 使其能被 10 ms 采样的消抖逻辑可靠捕获（按下 + 松开都能被看到）。
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
localparam [21:0] HOLD_MAX = 22'd4194303;   // 100 MHz -> 41.9 ms

    reg [21:0] r_hold0;
    reg [21:0] r_hold1;
    reg [21:0] r_hold2;
    reg [21:0] r_hold3;

    always @(posedge S_100m_clk or negedge S_sys_rst_n) begin
        if (!S_sys_rst_n) begin
            r_hold0 <= 22'd0;
            r_hold1 <= 22'd0;
            r_hold2 <= 22'd0;
            r_hold3 <= 22'd0;
        end
        else begin
            r_hold0 <= S_uart_key_evt[0] ? HOLD_MAX :
                       ((r_hold0 != 22'd0) ? r_hold0 - 1'b1 : 22'd0);
            r_hold1 <= (S_uart_key_evt[1] | S_demo_k2) ? HOLD_MAX :
                       ((r_hold1 != 22'd0) ? r_hold1 - 1'b1 : 22'd0);
            r_hold2 <= (S_uart_key_evt[2] | S_demo_k3) ? HOLD_MAX :
                       ((r_hold2 != 22'd0) ? r_hold2 - 1'b1 : 22'd0);
            r_hold3 <= S_uart_key_evt[3] ? HOLD_MAX :
                       ((r_hold3 != 22'd0) ? r_hold3 - 1'b1 : 22'd0);
        end
    end

    wire S_uh0 = (r_hold0 != 22'd0);
    wire S_uh1 = (r_hold1 != 22'd0);
    wire S_uh2 = (r_hold2 != 22'd0);
    wire S_uh3 = (r_hold3 != 22'd0);

    // 物理按键 / 板载备用键 / UART 虚拟按键 合并到同一条控制通路
    //   1 = 按下
        wire [3:0] S_key_press = { S_tm_key[3] | S_uh3,                               S_tm_key[2] | S_uh2,                               S_tm_key[1] | S_uh1 | ~I_button,                               S_tm_key[0] | S_uh0 };

    // K1 为电平型复位（按下即复位，松开恢复），与 improve4 完全一致
    wire S_key1_n = ~S_key_press[0];

    // 全局异步复位只由 PLL lock + 外部复位构成（不含任何寄存器输出），
    // 与 FS03 基线完全一致，复位网不跨越时钟域，因此不产生 recovery 违例。
    //   （若把 100 MHz 域的按键寄存器直接并入复位网，复位/CE 网会扇出到
    //    75 MHz 与 375 MHz 域，TD 给出的 recovery 预算只有 0.664 ns，
    //    实测会产生 89 个违例端点、SWNS -2.545 ns。）
    wire S_rst_n = S_sys_rst_n;

    // ===================== 拨码开关 =====================
    reg [3:0] S_sw_d0;
    reg [3:0] S_sw_d1;

    always @(posedge S_100m_clk or negedge S_sys_rst_n) begin
        if (!S_sys_rst_n) begin
            S_sw_d0 <= 4'd0;
            S_sw_d1 <= 4'd0;
        end
        else begin
            S_sw_d0 <= I_sw;
            S_sw_d1 <= S_sw_d0;
        end
    end

    wire S_en_sobel    = S_sw_d1[0];   // S1 -> SW1 -> SOBEL
    wire S_en_gray     = S_sw_d1[1];   // S2 -> SW2 -> GRAY
    wire S_en_bin      = S_sw_d1[2];   // S3 -> SW3 -> BIN
    wire S_osd_text_en = S_sw_d1[3];   // S4 -> SW4 -> 状态文字 / 检测框显隐

    // ===================== K2 : 阈值档位（16 档，松手 1 s 后生效）=====
    wire [3:0] S_th_sel;
    wire [3:0] S_th_pending;
    wire       S_th_wait;

    key_mode_delay_ctrl #(
        .CNT_MAX   ( 20'd999999   ),    // 100 MHz -> ~10 ms 消抖
        .DELAY_MAX ( 27'd99999999 ),    // 100 MHz -> 1.000 s
        .WIDTH     ( 4            ),
        .MOD       ( 16           )
    ) u_key_th_ctrl (
        .I_clk     ( S_100m_clk        ),
        .I_rst_n   ( S_rst_n & S_key1_n ),
        .I_key     ( ~S_key_press[1]   ),   // 模块 S2 / 板载 KEY2 / UART "K2"
        .O_mode    ( S_th_sel          ),
        .O_pending ( S_th_pending      ),
        .O_waiting ( S_th_wait         )
    );

    // threshold = S_th_sel * 20 = S_th_sel * (16 + 4)
    wire [8:0] S_proc_th = {1'b0, S_th_sel, 4'd0} + {3'd0, S_th_sel, 2'd0};

    // ===================== K3 : 风格 / 画质档位（16 档）=====
    wire [3:0] S_style_sel;
    wire [3:0] S_style_pending;
    wire       S_style_wait;

    key_mode_delay_ctrl #(
        .CNT_MAX   ( 20'd999999   ),
        .DELAY_MAX ( 27'd99999999 ),
        .WIDTH     ( 4            ),
        .MOD       ( 16           )
    ) u_key_style_ctrl (
        .I_clk     ( S_100m_clk        ),
        .I_rst_n   ( S_rst_n & S_key1_n ),
        .I_key     ( ~S_key_press[2]   ),   // 模块 S3 / UART "K3"
        .O_mode    ( S_style_sel       ),
        .O_pending ( S_style_pending   ),
        .O_waiting ( S_style_wait      )
    );

    // ===================== K4 : 分类器选择（0 = 规则，1 = MLP）=====
    wire       S_cls_sel;
    wire       S_cls_pending;
    wire       S_cls_wait;

    key_mode_delay_ctrl #(
        .CNT_MAX   ( 20'd999999   ),
        .DELAY_MAX ( 27'd99999999 ),
        .WIDTH     ( 1            ),
        .MOD       ( 2            )
    ) u_key_cls_ctrl (
        .I_clk     ( S_100m_clk        ),
        .I_rst_n   ( S_rst_n & S_key1_n ),
        .I_key     ( ~S_key_press[3]   ),   // 模块 S4 / UART "K4"
        .O_mode    ( S_cls_sel         ),
        .O_pending ( S_cls_pending     ),
        .O_waiting ( S_cls_wait        )
    );

    // ===================== 控制字跨时钟域分发 =====================
    //   [18]     : K1（按下即复位，借道控制总线同步器进入各时钟域）
    //   [17]     : 分类器选择（KEY4）
    //   [16]     : 左下状态文字 / 检测框使能（SW4）
    //   [15:12]  : 风格档位（KEY3）
    //   [11]     : SOBEL   [10] : GRAY   [9] : BIN
    //   [8:0]    : 阈值
    // ---- 同步寄存器与各时钟域复位的声明必须放在第一次使用之前 ----
    // Verilog 中"未声明即使用"会生成隐式线网(=0)，会把整条视频链路永久
    // 复位并被综合常量传播掉；对应的 always 块仍在文件后面的原位置。
    reg [18:0] S_ctrl_csi_d0;
    reg [18:0] S_ctrl_csi_d1;
    reg [18:0] S_ctrl_hdmi_d0;
    reg [18:0] S_ctrl_hdmi_d1;

    wire [18:0] S_ctrl_bus = {S_key_press[0], S_cls_sel, S_osd_text_en,
                              S_style_sel,
                              S_en_sobel, S_en_gray, S_en_bin, S_proc_th};

    // CSI 域复位 = 干净的 S_rst_n & (本域两级同步后的 K1 取反)
    wire        S_csi_rst_n       = S_rst_n & ~S_ctrl_csi_d1[18];
    wire [3:0]  S_style_csi       = S_ctrl_csi_d1[15:12];
    wire [2:0]  S_qual_mode_csi   = (S_style_csi == 4'd4) ? 3'd1 :
                                    (S_style_csi == 4'd5) ? 3'd2 :
                                    (S_style_csi == 4'd6) ? 3'd3 : 3'd0;
    wire        S_sobel_csi       = S_ctrl_csi_d1[11];
    wire        S_gray_csi        = S_ctrl_csi_d1[10];
    wire        S_bin_csi         = S_ctrl_csi_d1[9];
    wire [8:0]  S_th_csi          = S_ctrl_csi_d1[8:0];

    // 75 MHz 域复位同理
    wire        S_hdmi_rst_n      = S_rst_n & ~S_ctrl_hdmi_d1[18];
    wire        S_video_out_rst_n = S_hdmi_rst_n;
    wire        S_cls_sel_hdmi    = S_ctrl_hdmi_d1[17];
    wire        S_osd_text_hdmi   = S_ctrl_hdmi_d1[16];
    wire [3:0]  S_style_hdmi      = S_ctrl_hdmi_d1[15:12];

    // ===================== 记录字段偏移（由 CLS_W 推导）====================
    localparam B_CONF = 0;
    localparam B_MLP  = B_CONF + 7;
    localparam B_RULE = B_MLP  + CLS_W;
    localparam B_YMAX = B_RULE + CLS_W;
    localparam B_YMIN = B_YMAX + 10;
    localparam B_XMAX = B_YMIN + 10;
    localparam B_XMIN = B_XMAX + 11;
    localparam B_VAL  = B_XMIN + 11;

    localparam T_MISS = 0;
    localparam T_ZERO = T_MISS + 4;
    localparam T_CONF = T_ZERO + 1;
    localparam T_MLP  = T_CONF + 7;
    localparam T_RULE = T_MLP  + CLS_W;
    localparam T_ID   = T_RULE + CLS_W;

    // ROI（报警窗口，像素坐标）
    localparam [11:0] ROI_X = 12'd540;
    localparam [11:0] ROI_Y = 12'd300;
    localparam [11:0] ROI_W = 12'd200;
    localparam [11:0] ROI_H = 12'd140;

    // ===================== 画质统计 / 自动曝光（improve2）==================
    wire [7:0]   S_yavg;          // 上一帧平均亮度
    wire [31:0]  S_sum_y;
    wire [17:0]  S_nsamp;
    wire [17:0]  S_over;
    wire [17:0]  S_under;
    wire         S_stat_valid;    // 帧起始单脉冲（CSI 域）

    // AE 与 HE/CLAHE/STRETCH 互斥：画质增强会改变亮度，AE 同时开会震荡
    wire         S_ae_en = (S_qual_mode_csi == 3'd0);

    wire [15:0]  S_ae_ext;        // 闭环请求的曝光（CSI 域）
    wire [15:0]  S_ag_ext;        // 闭环请求的增益（CSI 域）

    // CSI -> 24 MHz（传感器配置域，每 8 帧才变一次的慢变量）
    reg [15:0] S_ae_ext_24_d0, S_ae_ext_24_d1;
    reg [15:0] S_ag_ext_24_d0, S_ag_ext_24_d1;
    reg        S_ae_en_24_d0,  S_ae_en_24_d1;

    // CSI -> HDMI 像素时钟域（状态 OSD）
    reg [7:0]   S_yavg_h_d0, S_yavg_h_d1;
    reg [15:0]  S_exp_h_d0,  S_exp_h_d1;
    reg [15:0]  S_gain_h_d0, S_gain_h_d1;
    reg         S_ae_en_h_d0, S_ae_en_h_d1;

    // ===================== 运动检测（improve1，三帧差分）==================
    wire [11:0] S_mot_cnt;
    wire [10:0] S_mot_xmin, S_mot_xmax;
    wire [9:0]  S_mot_ymin, S_mot_ymax;
    wire        S_mot_bb_valid;
    wire        S_mot_alarm;
    wire        S_mot_frame;
    wire        S_flag_we;
    wire [13:0] S_flag_addr;
    wire [7:0]  S_flag_data;
    wire [1:0]  S_wr_page_g;

    // 运动统计量每帧锁存一次并整帧稳定 -> 两级同步器足够
    reg  [55:0] S_mot_d0, S_mot_d1;
    wire [11:0] S_mot_cnt_h;
    wire [10:0] S_mot_xmin_h, S_mot_xmax_h;
    wire [9:0]  S_mot_ymin_h, S_mot_ymax_h;
    wire        S_mot_bb_h, S_mot_alarm_h;

    assign { S_mot_alarm_h, S_mot_bb_h, S_mot_ymax_h, S_mot_ymin_h,
             S_mot_xmax_h,  S_mot_xmin_h, S_mot_cnt_h } = S_mot_d1;

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if (!S_hdmi_rst_n) begin
            S_mot_d0 <= 56'd0;
            S_mot_d1 <= 56'd0;
        end
        else begin
            S_mot_d0 <= { S_mot_alarm, S_mot_bb_valid, S_mot_ymax, S_mot_ymin,
                          S_mot_xmax,  S_mot_xmin,     S_mot_cnt  };
            S_mot_d1 <= S_mot_d0;
        end
    end

    // ===================== 跟踪结果 / 目标记录（HDMI 域）==================
    wire [NOBJ*REC_W-1:0] S_obj_bus;    // CSI 域原始目标记录
    wire [3:0]            S_obj_num;

    reg [NOBJ*REC_W-1:0] S_obj_d0, S_obj_d1;
    reg [3:0]            S_obj_num_d0, S_obj_num_d1;

    wire [NOBJ*REC_W-1:0] S_obj56;      // 多数投票后的稳定记录
    wire [NTRK*TRK_W-1:0] S_trk_bus;
    wire [NTRK*DYN_W-1:0] S_trk_dyn;
    wire                  S_trk_alarm;
    wire [3:0]            S_trk_num;

    // improve1 时序修复：跟踪器总线在分发给三层 OSD 之前统一再打一拍。
    // 这些总线都是帧级稳定数据（每帧更新一次），多一拍功能不可见，
    // 但能让长线只到寄存器 D 端，避免进入逐像素组合逻辑。
    reg [NOBJ*REC_W-1:0] S_obj56_r;
    reg [NTRK*TRK_W-1:0] S_trk_bus_r;
    reg [NTRK*DYN_W-1:0] S_trk_dyn_r;
    reg                  S_trk_alarm_r;
    reg [3:0]            S_trk_num_r;

    // ===================== 延时测量（improve3，0.1 ms 分辨率）==============
    wire [7:0] S_latency_ms;
    wire [7:0] S_latency_frames;
    wire [9:0] S_latency_tenth_100;
    wire [9:0] S_latency_jitter_100;
    wire       S_latency_valid_100;
    reg  [9:0] S_latency_tenth_h_d0, S_latency_tenth_h_d1;
    reg        S_latency_valid_h_d0, S_latency_valid_h_d1;
    reg  [7:0] S_latency_frames_d0,  S_latency_frames_d1;

    // ===================== 帧率测量 =====================
    wire [6:0] S_fps;
    reg  [6:0] S_fps_d0, S_fps_d1;

    // ===================== 摄像头 I2C 配置与 AE =====================
    wire [15:0] S_ae;
    wire [15:0] S_ag;
    wire        S_cam_cfg_done;
    wire        S_ae_cfg_done;
    wire        S_ae_req;

    ae_set u_ae_set (
        .I_clk         ( S_24m_clk          ),
        .I_rst         ( ~S_rst_n           ),
        .I_btn         ( 4'b1111            ),  // 按键留给 K1~K4，AE 手动关闭
        .I_cam_cfg_done( S_cam_cfg_done     ),
        .I_ae_cfg_done ( S_ae_cfg_done      ),
        // improve2 : 自动曝光闭环驱动目标值（最小侵入接口）
        .I_ext_en      ( S_ae_en_24_d1      ),
        .I_ae_ext      ( S_ae_ext_24_d1     ),
        .I_ag_ext      ( S_ag_ext_24_d1     ),
        .O_ae_req      ( S_ae_req           ),
        .O_ae          ( S_ae               ),
        .O_ag          ( S_ag               )
    );

    uicfgcs520_720p #(
        .CLK_DIV ( 24000000 / 100000 - 1 )
    ) u_uicfgcs520 (
        .I_clk         ( S_24m_clk      ),
        .I_rst_n       ( S_rst_n        ),
        .I_ae_req      ( S_ae_req       ),
        .I_ae          ( S_ae           ),
        .I_ag          ( S_ag           ),
        .O_cam_scl     ( O_cam_scl      ),
        .IO_cam_sda    ( IO_cam_sda     ),
        .O_cfg_done    ( S_cam_cfg_done ),
        .O_ae_cfg_done ( S_ae_cfg_done  )
    );

    // ===================== MIPI D-PHY RX（DR1 MIPIIO） =====================
    wire        S_csi_rx_clk;
    wire        S_hs_rx_valid;
    wire [15:0] S_hs_rx_data;
    wire [1:0]  S_lane_error;

    dr1_mipiio_rx_wrapper #(
        .LANE_NUM ( 2 )
    ) u_mipi_dphy_rx (
        .I_lp_clk              ( S_100m_clk       ),
        .I_rst                 ( ~S_rst_n         ),
        .I_clk_lane_in_delay   ( 6'd0             ),
        .I_data_lane0_in_delay ( 6'd0             ),
        .I_data_lane1_in_delay ( 6'd0             ),
        .I_data_lane2_in_delay ( 6'd0             ),
        .I_data_lane3_in_delay ( 6'd0             ),
        .I_lane_invert         ( 4'b0000          ),
        .O_hs_rx_clk           ( S_csi_rx_clk     ),
        .O_hs_rx_valid         ( S_hs_rx_valid    ),
        .O_hs_rx_data          ( S_hs_rx_data     ),
        .O_lp_rx_lane0_p       (                  ),
        .O_lp_rx_lane0_n       (                  ),
        .I_lp_tx_en            ( 1'b0             ),
        .I_lp_tx_lane0_p       ( 1'b1             ),
        .I_lp_tx_lane0_n       ( 1'b1             ),
        .O_lane_match_error    (                  ),
        .O_lane_error          ( S_lane_error     ),
        .IO_rx_clk_pad_n       ( IO_rx_clk_pad_n  ),
        .IO_rx_clk_pad_p       ( IO_rx_clk_pad_p  ),
        .IO_rx_data_pad_n      ( IO_rx_data_pad_n ),
        .IO_rx_data_pad_p      ( IO_rx_data_pad_p )
    );

    // ===================== CSI-2 / RAW10 解包 =====================
    wire        S_csi_frame_start;
    wire        S_csi_frame_end;
    wire        S_csi_valid;
    wire [31:0] S_csi_data;

    wire        S_raw10_frame_start;
    wire        S_raw10_frame_end;
    wire        S_raw10_valid;
    wire [39:0] S_raw10_data;

    wire        S_axis_tlast;
    wire        S_axis_tuser;
    wire [39:0] S_axis_tdata;
    wire        S_axis_tvalid;

    csi_unpacket_2lane u_csi_unpacket (
        .I_clk             ( S_csi_rx_clk      ),
        .I_rst_n           ( S_csi_rst_n       ),
        .I_hs_valid        ( S_hs_rx_valid     ),
        .I_hs_data         ( S_hs_rx_data      ),
        .O_csi_frame_start ( S_csi_frame_start ),
        .O_csi_frame_end   ( S_csi_frame_end   ),
        .O_csi_valid       ( S_csi_valid       ),
        .O_csi_data        ( S_csi_data        )
    );

    raw10_unpacket_2lane u_raw10_unpacket (
        .I_clk              ( S_csi_rx_clk        ),
        .I_rst_n            ( S_csi_rst_n         ),
        .I_csi_frame_start  ( S_csi_frame_start   ),
        .I_csi_frame_end    ( S_csi_frame_end     ),
        .I_csi_valid        ( S_csi_valid         ),
        .I_csi_data         ( S_csi_data          ),
        .O_raw10_frame_start( S_raw10_frame_start ),
        .O_raw10_frame_end  ( S_raw10_frame_end   ),
        .O_raw10_valid      ( S_raw10_valid       ),
        .O_raw10_data       ( S_raw10_data        )
    );

    uial2axis #(
        .IMG_WIDTH        ( 1280 ),
        .IMG_HEIGHT       ( 720  ),
        .INPUT_DATA_WIDTH ( 40   )
    ) u_uial2axis (
        .I_native_clk ( S_csi_rx_clk        ),
        .I_rst_n      ( S_csi_rst_n         ),
        .I_data       ( S_raw10_data        ),
        .I_data_valid ( S_raw10_valid       ),
        .I_data_start ( S_raw10_frame_start ),
        .I_data_end   ( S_raw10_frame_end   ),
        .axis_tvalid  ( S_axis_tvalid       ),
        .axis_tdata   ( S_axis_tdata        ),
        .axis_tuser   ( S_axis_tuser        ),
        .axis_tlast   ( S_axis_tlast        )
    );

    wire [39:0] S_raw_tdata  = S_axis_tdata;
    wire        S_raw_tlast  = S_axis_tlast;
    wire        S_raw_tvalid = S_axis_tvalid;
    wire        S_raw_tuser  = S_axis_tuser;

    // ===================== 控制字跨时钟域分发（声明见前文） =====================
    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if (!S_rst_n) begin
            S_ctrl_csi_d0 <= 19'd0;
            S_ctrl_csi_d1 <= 19'd0;
        end
        else begin
            S_ctrl_csi_d0 <= S_ctrl_bus;
            S_ctrl_csi_d1 <= S_ctrl_csi_d0;
        end
    end

    // 同步寄存器本身必须用"干净"的 S_rst_n 复位：若用含 K1 的复位，
    // K1 会先把同步器清零、复位随即释放，形成振荡。
    always @(posedge S_hdmi_pixel_clk or negedge S_rst_n) begin
        if (!S_rst_n) begin
            S_ctrl_hdmi_d0 <= 19'd0;
            S_ctrl_hdmi_d1 <= 19'd0;
        end
        else begin
            S_ctrl_hdmi_d0 <= S_ctrl_bus;
            S_ctrl_hdmi_d1 <= S_ctrl_hdmi_d0;
        end
    end

    // ===================== ISP + 算法流水线 + 检测 / 运动侧路 =============
    wire         S_ISP_O_tready;
    wire         S_isp_up_ready;   // ISP 上游 ready（本工程上游恒可写，未使用）
    wire [127:0] S_ISP_O_tdata;
    wire         S_ISP_O_tlast;
    wire         S_ISP_O_tuser;
    wire         S_ISP_O_tvalid;

    isp_top_imgproc #(
        .CLS_W ( CLS_W ),
        .NOUT  ( NOUT  ),
        .REC_W ( REC_W )
    ) u_isp_top (
        .axi4s_video_aclk ( S_csi_rx_clk      ),
        .I_rst_n          ( S_csi_rst_n       ),
        .I_tlast          ( S_raw_tlast       ),
        .I_tuser          ( S_raw_tuser       ),
        .I_tdata          ( S_raw_tdata       ),
        .I_tvalid         ( S_raw_tvalid      ),
        .I_tdest          ( 10'd0             ),
        .O_tready         ( S_ISP_O_tready    ),
        .I_en_sobel       ( S_sobel_csi       ),
        .I_en_gray        ( S_gray_csi        ),
        .I_en_bin         ( S_bin_csi         ),
        .I_th             ( S_th_csi          ),
        .I_style          ( S_style_csi       ),
        .I_qual_mode      ( S_qual_mode_csi   ),
        // ---- improve2 : 画质统计（侧路，不改像素）----
        .O_yavg           ( S_yavg            ),
        .O_sum_y          ( S_sum_y           ),
        .O_nsamp          ( S_nsamp           ),
        .O_over           ( S_over            ),
        .O_under          ( S_under           ),
        .O_stat_valid     ( S_stat_valid      ),
        // ---- 目标检测 ----
        .O_obj_bus        ( S_obj_bus         ),
        .O_obj_num        ( S_obj_num         ),
        // ---- improve1 : 三帧差分运动检测 ----
        .O_mot_cnt        ( S_mot_cnt         ),
        .O_bb_xmin        ( S_mot_xmin        ),
        .O_bb_xmax        ( S_mot_xmax        ),
        .O_bb_ymin        ( S_mot_ymin        ),
        .O_bb_ymax        ( S_mot_ymax        ),
        .O_bb_valid       ( S_mot_bb_valid    ),
        .O_mot_alarm      ( S_mot_alarm       ),
        .O_mot_frame      ( S_mot_frame       ),
        .O_flag_we        ( S_flag_we         ),
        .O_flag_addr      ( S_flag_addr       ),
        .O_flag_data      ( S_flag_data       ),
        .O_wr_page_g      ( S_wr_page_g       ),
        // ---- 视频主链 ----
        .O_tdata          ( S_ISP_O_tdata     ),
        .O_tlast          ( S_ISP_O_tlast     ),
        .O_tuser          ( S_ISP_O_tuser     ),
        .O_tvalid         ( S_ISP_O_tvalid    ),
        .I_tready         ( S_isp_up_ready    )
    );

    // ===================== 自动曝光闭环（CSI 域，无 CDC 消费统计）==========
    ae_auto #(
        .EXP_MIN   ( 16'd64   ),
        .EXP_MAX   ( 16'd1125 ),   // 1125 x 22.2 us = 25 ms -> 延时上限
        .GAIN_MIN  ( 16'd16   ),
        .GAIN_MAX  ( 16'd176   ),
        .YREF      ( 8'd118   ),
        .DEAD      ( 8'd6     ),
        .FRAMES    ( 4'd8     ),   // 每 8 帧（133 ms）下发一次
        .EXP_INIT  ( 16'd1125 ),
        .GAIN_INIT ( 16'd32   )
    ) u_ae_auto (
        .I_clk    ( S_csi_rx_clk  ),
        .I_rst_n  ( S_csi_rst_n   ),
        .I_en     ( S_ae_en       ),
        .I_sum_y  ( S_sum_y       ),
        .I_nsamp  ( S_nsamp       ),
        .I_over   ( S_over        ),
        .I_under  ( S_under       ),
        .I_valid  ( S_stat_valid  ),
        .O_exp    ( S_ae_ext      ),
        .O_gain   ( S_ag_ext      )
    );

    // CSI -> 24 MHz（传感器配置域）
    always @(posedge S_24m_clk or negedge S_rst_n) begin
        if (!S_rst_n) begin
            S_ae_ext_24_d0 <= 16'd1125; S_ae_ext_24_d1 <= 16'd1125;
            S_ag_ext_24_d0 <= 16'd32;   S_ag_ext_24_d1 <= 16'd32;
            S_ae_en_24_d0  <= 1'b0;     S_ae_en_24_d1  <= 1'b0;
        end
        else begin
            S_ae_ext_24_d0 <= S_ae_ext;  S_ae_ext_24_d1 <= S_ae_ext_24_d0;
            S_ag_ext_24_d0 <= S_ag_ext;  S_ag_ext_24_d1 <= S_ag_ext_24_d0;
            S_ae_en_24_d0  <= S_ae_en;   S_ae_en_24_d1  <= S_ae_en_24_d0;
        end
    end

    // CSI -> HDMI 像素时钟域（状态 OSD）
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if (!S_hdmi_rst_n) begin
            S_yavg_h_d0  <= 8'd0;   S_yavg_h_d1  <= 8'd0;
            S_exp_h_d0   <= 16'd0;  S_exp_h_d1   <= 16'd0;
            S_gain_h_d0  <= 16'd0;  S_gain_h_d1  <= 16'd0;
            S_ae_en_h_d0 <= 1'b0;   S_ae_en_h_d1 <= 1'b0;
        end
        else begin
            S_yavg_h_d0  <= S_yavg;   S_yavg_h_d1  <= S_yavg_h_d0;
            S_exp_h_d0   <= S_ae_ext; S_exp_h_d1   <= S_exp_h_d0;
            S_gain_h_d0  <= S_ag_ext; S_gain_h_d1  <= S_gain_h_d0;
            // AE 开关是电平量，两级同步足够，且避免组合逻辑跨时钟域
            S_ae_en_h_d0 <= S_ae_en;  S_ae_en_h_d1 <= S_ae_en_h_d0;
        end
    end

    // ===================== 帧缓存 / 跨时钟域（片上 ERAM，取代 DDR） =======
    wire        S_bridge_overflow;
    wire        S_bridge_underflow;
    wire [23:0] S_video_rd_data;
    wire        S_hdmi_window_rd_en;

    video_stream_bridge #(
        .FIFO_AWIDTH     ( 12   ),
        .ALMOST_FULL_TH  ( 3584 ),
        .ALMOST_EMPTY_TH ( 256  )
    ) u_video_stream_bridge (
        .I_cam_clk        ( S_csi_rx_clk        ),
        .I_cam_rst_n      ( S_csi_rst_n         ),
        .I_cam_tuser      ( S_ISP_O_tuser       ),
        .I_cam_tvalid     ( S_ISP_O_tvalid      ),
        .I_cam_tdata      ( S_ISP_O_tdata       ),
        .I_cam_tlast      ( S_ISP_O_tlast       ),
        .O_cam_overflow   ( S_bridge_overflow   ),
        .I_disp_clk       ( S_hdmi_pixel_clk    ),
        .I_disp_rst_n     ( S_video_out_rst_n   ),
        .I_disp_rd_en     ( S_hdmi_window_rd_en ),
        .O_disp_data      ( S_video_rd_data     ),
        .O_disp_underflow ( S_bridge_underflow  )
    );

    // ===================== HDMI 输出 =====================
    wire S_hdmi_vsync;
    wire S_hdmi_hsync;
    wire S_hdmi_de;
    wire S_hdmi_user;
    wire S_hdmi_last;

    wire S_hdmi_out_vsync;
    wire S_hdmi_out_hsync;
    wire S_hdmi_out_de;
    wire [23:0] S_hdmi_out_data;

    wire S_hdmi_osd_vsync;
    wire S_hdmi_osd_hsync;
    wire S_hdmi_osd_de;
    wire [23:0] S_hdmi_osd_data;

    wire S_hdmi_obj_vsync;
    wire S_hdmi_obj_hsync;
    wire S_hdmi_obj_de;
    wire [23:0] S_hdmi_obj_data;

    wire S_hdmi_mot_vsync;
    wire S_hdmi_mot_hsync;
    wire S_hdmi_mot_de;
    wire [23:0] S_hdmi_mot_data;

    wire S_hdmi_st_vsync;
    wire S_hdmi_st_hsync;
    wire S_hdmi_st_de;
    wire [23:0] S_hdmi_st_data;

    uivtc #(
        .H_ActiveSize ( 1280 ),
        .H_FrameSize  ( 1650 ),
        .H_SyncStart  ( 1390 ),
        .H_SyncEnd    ( 1430 ),
        .V_ActiveSize ( 720  ),
        .V_FrameSize  ( 750  ),
        .V_SyncStart  ( 725  ),
        .V_SyncEnd    ( 730  )
    ) u_hdmi_vtc (
        .I_vtc_rstn     ( S_hdmi_rst_n     ),
        .I_vtc_clk      ( S_hdmi_pixel_clk ),
        .O_vtc_vs       ( S_hdmi_vsync     ),
        .O_vtc_hs       ( S_hdmi_hsync     ),
        .O_vtc_de_valid ( S_hdmi_de        ),
        .O_vtc_user     ( S_hdmi_user      ),
        .O_vtc_last     ( S_hdmi_last      )
    );

    wire [3:0] S_hdmi_debug_status;

    hdmi_mixer #(
        .H_OFFSET   ( 0    ),
        .V_OFFSET   ( 0    ),
        .IMG_WIDTH  ( 1280 ),
        .IMG_HEIGHT ( 720  ),
        .DEBUG_MODE ( 0    )
    ) u_hdmi_mixer (
        .I_clk           ( S_hdmi_pixel_clk    ),
        .I_rst_n         ( S_hdmi_rst_n        ),
        .I_video_vsync   ( S_hdmi_vsync        ),
        .I_video_hsync   ( S_hdmi_hsync        ),
        .I_video_de      ( S_hdmi_de           ),
        .I_video_user    ( S_hdmi_user         ),
        .I_video_last    ( S_hdmi_last         ),
        .I_debug_status  ( S_hdmi_debug_status ),
        .O_video_rd_en   ( S_hdmi_window_rd_en ),
        .I_video_rd_data ( S_video_rd_data     ),
        .O_hdmi_vsync    ( S_hdmi_out_vsync    ),
        .O_hdmi_hsync    ( S_hdmi_out_hsync    ),
        .O_hdmi_de       ( S_hdmi_out_de       ),
        .O_hdmi_data     ( S_hdmi_out_data     )
    );

    // mixer 把 de/data 延迟 2 拍，user/last 同样延迟，保证后面各层 OSD 时序一致
    reg [1:0] S_hdmi_user_sr;
    reg [1:0] S_hdmi_last_sr;

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if (!S_hdmi_rst_n) begin
            S_hdmi_user_sr <= 2'b00;
            S_hdmi_last_sr <= 2'b00;
        end
        else begin
            S_hdmi_user_sr <= {S_hdmi_user_sr[0], S_hdmi_user};
            S_hdmi_last_sr <= {S_hdmi_last_sr[0], S_hdmi_last};
        end
    end

    // OSD 第一层（"MODE:xxxx" + "TH:nnn"）
    // OSD 显示实际生效的算法（优先级 SOBEL > BIN > GRAY）
    wire [1:0] S_mode_hdmi = S_ctrl_hdmi_d1[11] ? 2'd2 :
                             S_ctrl_hdmi_d1[ 9] ? 2'd3 :
                             S_ctrl_hdmi_d1[10] ? 2'd1 :
                                                  2'd0 ;
    wire [8:0] S_th_hdmi   = S_ctrl_hdmi_d1[8:0];

    osd_mode_overlay #(
        .OSD_X ( 12'd375 ),
        .OSD_Y ( 12'd160 ),
        .OSD_W ( 12'd176 ),
        .OSD_H ( 12'd48  )
    ) u_osd_mode_overlay (
        .I_clk         ( S_hdmi_pixel_clk   ),
        .I_rst_n       ( S_hdmi_rst_n       ),
        .I_video_vsync ( S_hdmi_out_vsync   ),
        .I_video_hsync ( S_hdmi_out_hsync   ),
        .I_video_de    ( S_hdmi_out_de      ),
        .I_video_user  ( S_hdmi_user_sr[1]  ),
        .I_video_last  ( S_hdmi_last_sr[1]  ),
        .I_proc_mode   ( S_mode_hdmi        ),
        .I_th          ( S_th_hdmi          ),
        .I_video_data  ( S_hdmi_out_data    ),
        .O_video_vsync ( S_hdmi_osd_vsync   ),
        .O_video_hsync ( S_hdmi_osd_hsync   ),
        .O_video_de    ( S_hdmi_osd_de      ),
        .O_video_data  ( S_hdmi_osd_data    )
    );

    // ===================== 目标检测结果跨时钟域（CSI -> 75 MHz） ==========
    // 目标列表每帧更新一次并保持整帧稳定（约 16.7 ms = 125 万像素时钟），
    // 两级同步器绰绰有余（无需格雷码）。
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if (!S_hdmi_rst_n) begin
            S_obj_d0     <= {NOBJ*REC_W{1'b0}};
            S_obj_d1     <= {NOBJ*REC_W{1'b0}};
            S_obj_num_d0 <= 4'd0;
            S_obj_num_d1 <= 4'd0;
        end
        else begin
            S_obj_d0     <= S_obj_bus;
            S_obj_d1     <= S_obj_d0;
            S_obj_num_d0 <= S_obj_num;
            S_obj_num_d1 <= S_obj_num_d0;
        end
    end

    // ===================== 跨帧跟踪（时域投票 -> 稳定框 + ROI 报警）======
    obj_tracker #(
        .NOBJ     ( NOBJ  ),
        .NTRK     ( NTRK  ),
        .CLS_W    ( CLS_W ),
        .REC_W    ( REC_W ),
        .TRK_W    ( TRK_W ),
        .DYN_W    ( DYN_W ),
        .GATE     ( 96    ),
        .MAX_MISS ( 8     )
    ) u_obj_tracker (
        .I_clk          ( S_hdmi_pixel_clk  ),
        .I_rst_n        ( S_hdmi_rst_n      ),
        .I_frame_start  ( S_hdmi_user_sr[1] ),
        .I_obj_bus      ( S_obj_d1          ),
        .I_obj_num      ( S_obj_num_d1      ),
        .I_roi_x        ( ROI_X             ),
        .I_roi_y        ( ROI_Y             ),
        .I_roi_w        ( ROI_W             ),
        .I_roi_h        ( ROI_H             ),
        .O_trk_bus      ( S_trk_bus         ),
        .O_obj56        ( S_obj56           ),
        .O_trk_dyn      ( S_trk_dyn         ),
        .O_alarm        ( S_trk_alarm       ),
        .O_trk_num      ( S_trk_num         )
    );

    // improve1 时序修复：跟踪器总线在进入三层 OSD 之前统一再打一拍
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if (!S_hdmi_rst_n) begin
            S_obj56_r     <= {NOBJ*REC_W{1'b0}};
            S_trk_bus_r   <= {NTRK*TRK_W{1'b0}};
            S_trk_dyn_r   <= {NTRK*DYN_W{1'b0}};
            S_trk_alarm_r <= 1'b0;
            S_trk_num_r   <= 4'd0;
        end
        else begin
            S_obj56_r     <= S_obj56;
            S_trk_bus_r   <= S_trk_bus;
            S_trk_dyn_r   <= S_trk_dyn;
            S_trk_alarm_r <= S_trk_alarm;
            S_trk_num_r   <= S_trk_num;
        end
    end

    // 把跟踪结果同步回 100 MHz 域（蜂鸣器 / UART 上报用）
    reg [NTRK*TRK_W-1:0] r_trk_100_d0, r_trk_100_d1;
    reg                  r_alarm_100_d0, r_alarm_100_d1;

    always @(posedge S_100m_clk or negedge S_sys_rst_n) begin
        if (!S_sys_rst_n) begin
            r_trk_100_d0   <= {NTRK*TRK_W{1'b0}};
            r_trk_100_d1   <= {NTRK*TRK_W{1'b0}};
            r_alarm_100_d0 <= 1'b0;
            r_alarm_100_d1 <= 1'b0;
        end
        else begin
            r_trk_100_d0   <= S_trk_bus_r;
            r_trk_100_d1   <= r_trk_100_d0;
            r_alarm_100_d0 <= S_trk_alarm_r;
            r_alarm_100_d1 <= r_alarm_100_d0;
        end
    end

    // ===================== 帧率测量 =====================
    // 相机帧起始脉冲由 CSI 域同步进 100 MHz 域，避免亚稳态
    reg S_cam_fs_s1, S_cam_fs_s2;
    always @(posedge S_100m_clk or negedge S_sys_rst_n) begin
        if (!S_sys_rst_n) begin
            S_cam_fs_s1 <= 1'b0;
            S_cam_fs_s2 <= 1'b0;
        end
        else begin
            S_cam_fs_s1 <= S_ISP_O_tuser & S_ISP_O_tvalid;
            S_cam_fs_s2 <= S_cam_fs_s1;
        end
    end
    wire S_cam_fs_tick = S_cam_fs_s1 & ~S_cam_fs_s2;

    fps_meter #(
        .CLK_FREQ ( 32'd100000000 )
    ) u_fps_meter (
        .I_clk   ( S_100m_clk     ),
        .I_rst_n ( S_rst_n        ),
        .I_frame ( S_cam_fs_tick  ),
        .O_fps   ( S_fps          )
    );

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if (!S_hdmi_rst_n) begin
            S_fps_d0 <= 7'd0;
            S_fps_d1 <= 7'd0;
        end
        else begin
            S_fps_d0 <= S_fps;
            S_fps_d1 <= S_fps_d0;
        end
    end

    // ===================== 相机 -> 显示 延时测量（0.1 ms 格雷码 CDC）======
    latency_meter #(
        .TENTH_MS_TICKS ( 10000 ),     // 100 MHz -> 0.1 ms
        .FRAME_TENTH_MS ( 167   ),     // 60 fps -> 16.7 ms
        .JITTER_SAMPLES ( 100   )
    ) u_latency_meter (
        .I_clk                ( S_100m_clk                       ),
        .I_rst                ( ~S_rst_n                         ),
        .I_cam_clk            ( S_csi_rx_clk                     ),
        .I_cam_frame_pulse    ( S_ISP_O_tuser & S_ISP_O_tvalid   ),
        .I_disp_clk           ( S_hdmi_pixel_clk                 ),
        .I_disp_frame_pulse   ( S_hdmi_user_sr[1]                ),
        .O_latency_ms         ( S_latency_ms                     ),
        .O_latency_frames     ( S_latency_frames                 ),
        .O_latency_tenth_ms   ( S_latency_tenth_100              ),
        .O_jitter_tenth_ms    ( S_latency_jitter_100             ),
        .O_valid              ( S_latency_valid_100              )
    );

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if (!S_hdmi_rst_n) begin
            S_latency_tenth_h_d0  <= 10'd0;
            S_latency_tenth_h_d1  <= 10'd0;
            S_latency_valid_h_d0  <= 1'b0;
            S_latency_valid_h_d1  <= 1'b0;
            S_latency_frames_d0   <= 8'd0;
            S_latency_frames_d1   <= 8'd0;
        end
        else begin
            S_latency_tenth_h_d0  <= S_latency_tenth_100;
            S_latency_tenth_h_d1  <= S_latency_tenth_h_d0;
            S_latency_valid_h_d0  <= S_latency_valid_100;
            S_latency_valid_h_d1  <= S_latency_valid_h_d0;
            S_latency_frames_d0   <= S_latency_frames;
            S_latency_frames_d1   <= S_latency_frames_d0;
        end
    end

    // 帧缓存由 DDR 三帧乒乓改成片上 ERAM 弹性行缓存后，真实端到端延时已小于
    // 一帧；帧计数式延时计的分辨率仍是 1 帧，显示侧偶尔会"先行"一帧，
    // 此时 cam_cnt - disp_cnt 会回绕成 255（折算后 DLY 显示 99.9）。
    // 这里把"显示侧先行"的回绕统一折算为 0，DLY 只显示亚帧相位。
    wire [9:0]  S_lat_tenth_show = (S_latency_frames_d1 > 8'd2) ? 10'd0
                                                                : S_latency_tenth_h_d1;
    wire        S_lat_valid_show = S_latency_valid_h_d1 & (S_latency_frames_d1 <= 8'd2);

    // ===================== ROI 报警蜂鸣器 + UART 状态上报 =================
    buzz_ctrl #(
        .CLK_HZ ( 100_000_000 )
    ) u_buzz_ctrl (
        .I_clk    ( S_100m_clk     ),
        .I_rst    ( ~S_rst_n       ),
        .I_alarm  ( r_alarm_100_d1 ),
        .O_buzz   ( O_buzz         )
    );

    uart_report #(
        .CLK_HZ ( 100_000_000 ),
        .CLS_W  ( CLS_W       ),
        .TRK_W  ( TRK_W       )
    ) u_uart_report (
        .I_clk     ( S_100m_clk     ),
        .I_rst     ( ~S_rst_n       ),
        .I_report  ( 1'b0           ),
        .I_trk_bus ( r_trk_100_d1   ),
        .I_alarm   ( r_alarm_100_d1 ),
        .O_uart_tx ( O_uart_tx      )
    );

    // ===================== OSD 第二层：目标框 + 类别名 + ID + 轨迹 =========
    osd_obj_overlay #(
        .NOBJ  ( NOBJ  ),
        .NTRK  ( NTRK  ),
        .BW    ( 12'd3 ),
        .CLS_W ( CLS_W ),
        .REC_W ( REC_W ),
        .DYN_W ( DYN_W )
    ) u_osd_obj_overlay (
        .I_clk         ( S_hdmi_pixel_clk     ),
        .I_rst_n       ( S_hdmi_rst_n         ),
        .I_video_vsync ( S_hdmi_osd_vsync     ),
        .I_video_hsync ( S_hdmi_osd_hsync     ),
        .I_video_de    ( S_hdmi_osd_de        ),
        .I_video_user  ( S_hdmi_user_sr[1]    ),
        .I_video_last  ( S_hdmi_last_sr[1]    ),
        .I_osd_en      ( S_osd_text_hdmi      ),
        .I_cls_sel     ( S_cls_sel_hdmi       ),
        .I_obj_bus     ( S_obj56_r            ),
        .I_trk_id      ( { S_trk_bus_r[(1*TRK_W + T_ID + 2) -: 3],
                           S_trk_bus_r[(0*TRK_W + T_ID + 2) -: 3] } ),
        .I_trk_dyn     ( S_trk_dyn_r          ),
        .I_alarm       ( S_trk_alarm_r        ),
        .I_roi_x       ( ROI_X                ),
        .I_roi_y       ( ROI_Y                ),
        .I_roi_w       ( ROI_W                ),
        .I_roi_h       ( ROI_H                ),
        .I_video_data  ( S_hdmi_osd_data      ),
        .O_video_vsync ( S_hdmi_obj_vsync     ),
        .O_video_hsync ( S_hdmi_obj_hsync     ),
        .O_video_de    ( S_hdmi_obj_de        ),
        .O_video_data  ( S_hdmi_obj_data      )
    );

    // ===================== OSD 第三层：运动热力图 + 运动包围盒（improve1）==
    //   16x16 像素格中被三帧差分判为运动的格子叠半透明红块，外框是整块
    //   运动区域的外接矩形。同样由 SW4 控制显隐。
    osd_motion_overlay #(
        .IMG_WIDTH  ( 1280 ),
        .IMG_HEIGHT ( 720  ),
        .CELL_W     ( 16   ),
        .CELL_SHIFT ( 4    )
    ) u_osd_motion_overlay (
        .I_clk         ( S_hdmi_pixel_clk     ),
        .I_rst_n       ( S_hdmi_rst_n         ),
        .I_video_vsync ( S_hdmi_obj_vsync     ),
        .I_video_hsync ( S_hdmi_obj_hsync     ),
        .I_video_de    ( S_hdmi_obj_de        ),
        .I_video_user  ( S_hdmi_user_sr[1]    ),
        .I_video_last  ( S_hdmi_last_sr[1]    ),
        .I_osd_en      ( S_osd_text_hdmi      ),
        .I_video_data  ( S_hdmi_obj_data      ),
        // flag 存储写口在 CSI 域（由 frame_diff3 驱动）
        .I_wclk        ( S_csi_rx_clk         ),
        .I_flag_we     ( S_flag_we            ),
        .I_flag_addr   ( S_flag_addr          ),
        .I_flag_data   ( S_flag_data          ),
        .I_wr_page_g   ( S_wr_page_g          ),
        // 上一帧的运动统计（帧稳定）
        .I_bb_xmin     ( S_mot_xmin_h         ),
        .I_bb_xmax     ( S_mot_xmax_h         ),
        .I_bb_ymin     ( S_mot_ymin_h         ),
        .I_bb_ymax     ( S_mot_ymax_h         ),
        .I_bb_valid    ( S_mot_bb_h           ),
        .I_mot_alarm   ( S_mot_alarm_h        ),
        .O_video_vsync ( S_hdmi_mot_vsync     ),
        .O_video_hsync ( S_hdmi_mot_hsync     ),
        .O_video_de    ( S_hdmi_mot_de        ),
        .O_video_data  ( S_hdmi_mot_data      )
    );

    // ===================== OSD 第四层：左下角 12 行状态 + 延迟条 ==========
    osd_status_overlay #(
        .OSD_X ( 12'd24  ),
        .OSD_Y ( 12'd568 ),     // 12 行 -> 144 像素窗口
        .OSD_W ( 12'd120 ),
        .OSD_H ( 12'd144 ),
        .CLS_W ( CLS_W   ),
        .NOUT  ( NOUT    )
    ) u_osd_status_overlay (
        .I_clk              ( S_hdmi_pixel_clk     ),
        .I_rst_n            ( S_hdmi_rst_n         ),
        .I_video_vsync      ( S_hdmi_mot_vsync     ),
        .I_video_hsync      ( S_hdmi_mot_hsync     ),
        .I_video_de         ( S_hdmi_mot_de        ),
        .I_video_user       ( S_hdmi_user_sr[1]    ),
        .I_video_last       ( S_hdmi_last_sr[1]    ),
        .I_osd_en           ( S_osd_text_hdmi      ),
        .I_alg_mode         ( S_mode_hdmi          ),
        .I_style            ( S_style_hdmi         ),
        .I_obj_valid        ( S_obj56_r[B_VAL]                         ),
        .I_rule_cls         ( S_obj56_r[(B_RULE + CLS_W - 1) -: CLS_W] ),
        .I_mlp_cls          ( S_obj56_r[(B_MLP  + CLS_W - 1) -: CLS_W] ),
        .I_mlp_conf         ( S_obj56_r[6:0]       ),
        .I_fps              ( S_fps_d1             ),
        .I_trk_num          ( S_trk_num_r          ),
        .I_alarm            ( S_trk_alarm_r        ),
        .I_latency_tenth_ms ( S_lat_tenth_show     ),
        .I_latency_valid    ( S_lat_valid_show     ),
        .I_mot_cnt          ( S_mot_cnt_h          ),
        // improve2 : 图像质量自动调节
        .I_yavg             ( S_yavg_h_d1          ),
        .I_exp              ( S_exp_h_d1           ),
        .I_gain             ( S_gain_h_d1[6:0]     ),
        .I_ae_on            ( S_ae_en_h_d1         ),
        .I_video_data       ( S_hdmi_mot_data      ),
        .O_video_vsync      ( S_hdmi_st_vsync      ),
        .O_video_hsync      ( S_hdmi_st_hsync      ),
        .O_video_de         ( S_hdmi_st_de         ),
        .O_video_data       ( S_hdmi_st_data       )
    );

    // ===================== HDMI 发送 =====================
    hdmi_tx u_hdmi_tx (
        .I_pixel_clk        ( S_hdmi_pixel_clk  ),
        .I_serial_clk       ( S_hdmi_serial_clk ),
        // hdmi_tx 内部含 375 MHz 串行域，K1 不参与其复位（recovery 预算仅
        // 0.664 ns）；K1 只复位图像处理与 75 MHz 显示链路，HDMI 链路保持。
        .I_rst              ( ~S_rst_n          ),
        .I_key_in           ( 1'b0              ),
        .I_edid_read_trig   ( 1'b0              ),
        .O_edid_read_valid  (                   ),
        .O_edid_read_data   (                   ),
        .I_video_rgb_enable ( 1'b1              ),
        .I_video_in_vs      ( S_hdmi_st_vsync   ),
        .I_video_in_de      ( S_hdmi_st_de      ),
        .I_video_in_user    ( 1'b0              ),
        .I_video_in_valid   ( 1'b0              ),
        .I_video_in_last    ( 1'b0              ),
        .O_video_in_ready   (                   ),
        .I_video_in_data    ( S_hdmi_st_data    ),
        .I_audio_valid      ( 1'b0              ),
        .I_audio_left_data  ( 24'd0             ),
        .I_audio_right_data ( 24'd0             ),
        .I_i2s_BCLK         ( 1'b0              ),
        .I_i2s_LRCK         ( 1'b0              ),
        .I_i2s_DOUT         ( 1'b0              ),
        .O_ddc_scl          (                   ),
        .O_hdmi_clk_p       (                   ),
        .O_hdmi_tx_p        (                   ),
        .O_tmds_ch0_p       ( O_hdmi_tx_p[0]    ),
        .O_tmds_ch1_p       ( O_hdmi_tx_p[1]    ),
        .O_tmds_ch2_p       ( O_hdmi_tx_p[2]    ),
        .O_tmds_clk_p       ( O_hdmi_clk_p      )
    );

    // ===================== 链路诊断 =====================
    wire S_lane_error_any = |S_lane_error;

    reg        S_dbg_cam_cfg_seen;
    reg        S_dbg_hs_seen;
    reg        S_dbg_csi_seen;
    reg        S_dbg_frame_repeat_seen;
    reg        S_dbg_width_1280_seen;
    reg        S_dbg_hs_valid_d;
    reg [10:0] S_dbg_csi_words;
    reg [10:0] S_dbg_good_lines;
    reg        S_dbg_frame_seen_once;
    reg [1:0]  S_dbg_sync_cam_cfg, S_dbg_sync_hs, S_dbg_sync_frame_rep, S_dbg_sync_w1280;
    reg [1:0]  S_dbg_sync_isp_fs,  S_dbg_sync_ovf,  S_dbg_sync_unf,  S_dbg_sync_lane_err;

    always @(posedge S_csi_rx_clk or negedge S_csi_rst_n) begin
        if (!S_csi_rst_n) begin
            S_dbg_hs_seen           <= 1'b0;
            S_dbg_csi_seen          <= 1'b0;
            S_dbg_hs_valid_d        <= 1'b0;
            S_dbg_csi_words         <= 11'd0;
            S_dbg_good_lines        <= 11'd0;
            S_dbg_frame_seen_once   <= 1'b0;
            S_dbg_frame_repeat_seen <= 1'b0;
            S_dbg_width_1280_seen   <= 1'b0;
        end
        else begin
            if (S_hs_rx_valid)                     S_dbg_hs_seen  <= 1'b1;
            if (S_csi_frame_start | S_csi_valid)   S_dbg_csi_seen <= 1'b1;

            S_dbg_hs_valid_d <= S_hs_rx_valid;
            if (!S_dbg_hs_valid_d && S_hs_rx_valid)
                S_dbg_csi_words <= 11'd0;
            else if (S_csi_valid)
                S_dbg_csi_words <= S_dbg_csi_words + 11'd1;

            if (S_dbg_hs_valid_d && !S_hs_rx_valid && (S_dbg_csi_words != 0)
                && (S_dbg_csi_words == 11'd400)) begin
                S_dbg_width_1280_seen <= 1'b1;
                S_dbg_good_lines      <= S_dbg_good_lines + 11'd1;
            end

            if (S_csi_frame_start) begin
                if (S_dbg_frame_seen_once)
                    S_dbg_frame_repeat_seen <= 1'b1;
                else
                    S_dbg_frame_seen_once <= 1'b1;
                S_dbg_good_lines <= 11'd0;
            end
        end
    end

    always @(posedge S_24m_clk or negedge S_rst_n) begin
        if (!S_rst_n)
            S_dbg_cam_cfg_seen <= 1'b0;
        else if (S_cam_cfg_done)
            S_dbg_cam_cfg_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if (!S_hdmi_rst_n) begin
            {S_dbg_sync_cam_cfg, S_dbg_sync_hs, S_dbg_sync_frame_rep,
             S_dbg_sync_w1280, S_dbg_sync_isp_fs, S_dbg_sync_ovf,
             S_dbg_sync_unf, S_dbg_sync_lane_err} <= 16'd0;
        end
        else begin
            S_dbg_sync_cam_cfg   <= {S_dbg_sync_cam_cfg[0],   S_dbg_cam_cfg_seen};
            S_dbg_sync_hs        <= {S_dbg_sync_hs[0],        S_dbg_hs_seen};
            S_dbg_sync_frame_rep <= {S_dbg_sync_frame_rep[0], S_dbg_frame_repeat_seen};
            S_dbg_sync_w1280     <= {S_dbg_sync_w1280[0],     S_dbg_width_1280_seen};
            S_dbg_sync_isp_fs    <= {S_dbg_sync_isp_fs[0],    S_ISP_O_tuser};
            S_dbg_sync_ovf       <= {S_dbg_sync_ovf[0],       S_bridge_overflow};
            S_dbg_sync_unf       <= {S_dbg_sync_unf[0],       S_bridge_underflow};
            S_dbg_sync_lane_err  <= {S_dbg_sync_lane_err[0],  S_lane_error_any};
        end
    end

    assign S_hdmi_debug_status = {
        S_dbg_sync_w1280[1],
        S_dbg_sync_frame_rep[1],
        S_dbg_sync_hs[1],
        S_dbg_sync_cam_cfg[1]
    };

    // ===================== 板载 8 LED（链路诊断，与 FS03 基线一致）========
    reg [24:0] S_led_heartbeat;
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if (!S_hdmi_rst_n)
            S_led_heartbeat <= 25'd0;
        else
            S_led_heartbeat <= S_led_heartbeat + 25'd1;
    end

    assign O_led = {
        S_led_heartbeat[24],          // LED7 心跳(~0.45s)
        S_dbg_sync_lane_err[1],       // LED6 MIPI lane 错误
        S_dbg_sync_unf[1],            // LED5 读侧欠载
        S_dbg_sync_ovf[1],            // LED4 写侧丢弃
        S_dbg_sync_w1280[1],          // LED3 1280 像素行
        S_dbg_sync_frame_rep[1],      // LED2 CSI 帧重复
        S_dbg_sync_hs[1],             // LED1 MIPI HS 活动
        S_dbg_sync_cam_cfg[1]         // LED0 I2C 配置完成
    };

    // ===================== 外设模块：数码管 + LED + 按键（TM1638）========
    // LED1..LED4 与 HX1P35A 板 LED1..LED4 一一对应：
    //   LED1 PLL 锁定 / LED2 视频链路就绪(替代 DDR2 初始化完成) /
    //   LED3 摄像头配置完成 / LED4 至少一个算法被打开
    // LED5 = UART 命令活动指示，LED6 = ROI 报警，LED7~LED8 预留
    wire S_video_valid = S_dbg_sync_frame_rep[1];

    reg [23:0] S_uart_act_led;
    always @(posedge S_100m_clk or negedge S_sys_rst_n) begin
        if (!S_sys_rst_n)
            S_uart_act_led <= 24'd0;
        else if (S_uart_rx_act)
            S_uart_act_led <= 24'hFFFFFF;
        else if (S_uart_act_led != 24'd0)
            S_uart_act_led <= S_uart_act_led - 24'd1;
    end

        wire [7:0] S_mod_led = {
        S_demo_en,                                                            // LED8 auto-demo active (steady)
        r_alarm_100_d1,                                                       // LED7 ROI alarm
        (S_uart_act_led != 24'd0),                                           // LED6 UART command activity
        S_demo_en ? S_demo_chase[3] : (S_en_sobel | S_en_gray | S_en_bin),    // LED5 algorithm on / demo
        S_demo_en ? S_demo_chase[2] : S_dbg_sync_cam_cfg[1],                  // LED4 camera config done
        S_demo_en ? S_demo_chase[1] : S_video_valid,                          // LED3 video link
        S_demo_en ? S_demo_chase[0] : S_pll_lock,                             // LED2 PLL lock
        1'b0                                                                // LED1 reserved
    };

    // ---- CDC: S_mod_led 含 75MHz 域调试状态(cam_cfg/frame_rep) 与 PLL lock，
    //      而 TM1638 运行于 100MHz 域。先做两级同步再送入 I_led，
    //      消除 75->100MHz 跨时钟域组合路径（improve4 同类路径曾贴限，
    //      本工程布局后为 -0.837ns 建立时间违例）。
    (* ASYNC_REG = "TRUE" *)
    reg [7:0] S_mod_led_cdc0, S_mod_led_cdc1;
    always @(posedge S_100m_clk or negedge S_sys_rst_n) begin
        if (!S_sys_rst_n) begin
            S_mod_led_cdc0 <= 8'd0;
            S_mod_led_cdc1 <= 8'd0;
        end
        else begin
            S_mod_led_cdc0 <= S_mod_led;
            S_mod_led_cdc1 <= S_mod_led_cdc0;
        end
    end

    tm1638_led_key #(
        .CLK_HZ     ( 100000000 ),
        .SCK_HZ     ( 1000000   ),
        .REFRESH_HZ ( 200       )
    ) u_tm1638_led_key (
        .I_clk    ( S_100m_clk      ),
        .I_rst_n  ( S_sys_rst_n     ),
        .I_value  ( {1'b0, S_proc_th} ),
        .I_led    ( S_mod_led_cdc1  ),
        .O_key    ( S_tm_key        ),
        .O_valid  ( S_tm_valid      ),
        .O_stb    ( O_tm_stb        ),
        .O_sck    ( O_tm_clk        ),
        .IO_dio   ( IO_tm_dio       )
    );

endmodule
