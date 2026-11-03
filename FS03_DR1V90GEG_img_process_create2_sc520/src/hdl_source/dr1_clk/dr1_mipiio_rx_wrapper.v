// ---------------------------------------------------------------------------
// FS03_DR1V90GEG_mipi_hdmi_SC500_720P : dr1_mipiio_rx_wrapper.v
//
// 替代原 PH1P35 的 mipi_dphy_rx_ph1p_mipiio_wrapper.sv +
// ph1p_mipiio_rx_wrapper.v + byte/channel_aligner 加密核。
// PH1P35 使用 PH1P_LOGIC_DPHY_MIPI_RX 硬核；DR1V90GEG484 使用 DR1 系列
// 通用的 DR1_PHY_MIPIIO 原语（4 数据 lane + 1 时钟 lane，内置 HS 接收、
// 字节 FIFO 与 deskew）。
//
// 对外接口与原 PH1P 封装保持一致，下游 csi_unpacket_2lane 无需修改：
//   O_hs_rx_clk / O_hs_rx_valid / O_hs_rx_data[15:0] / O_lane_error[1:0]
//
// ★ 上板前必须核对 FS03 原理图 / PINLIST：
//   1) MIPI 差分对引脚位置（pin.adc 中以 TBD 标注）；
//   2) 若 FS03 摄像头模组自带 24MHz 晶振，则 O_cam_24m 可不接；
//   3) DR1M90 官方选型表仅 MEG484 标注 MIPI D-PHY RX 硬核，GEG484 通过
//      DR1_PHY_MIPIIO 的 MIPI IO 实现；若上板无法锁定位置，需改用
//      DR1M90MEG484 器件或米联客官方 MIPI 采集工程。
// ---------------------------------------------------------------------------
`timescale 1ns / 100ps

module dr1_mipiio_rx_wrapper #(
    parameter LANE_NUM = 2
)(
    input  wire        I_lp_clk,
    input  wire        I_rst,
    input  wire [5:0]  I_clk_lane_in_delay,
    input  wire [5:0]  I_data_lane0_in_delay,
    input  wire [5:0]  I_data_lane1_in_delay,
    input  wire [5:0]  I_data_lane2_in_delay,
    input  wire [5:0]  I_data_lane3_in_delay,
    input  wire [3:0]  I_lane_invert,
    output wire        O_hs_rx_clk,
    output wire        O_hs_rx_valid,
    output wire [15:0] O_hs_rx_data,
    output wire        O_lp_rx_lane0_p,
    output wire        O_lp_rx_lane0_n,
    input  wire        I_lp_tx_en,
    input  wire        I_lp_tx_lane0_p,
    input  wire        I_lp_tx_lane0_n,
    output wire        O_lane_match_error,
    output wire [1:0]  O_lane_error,
    inout  wire        IO_rx_clk_pad_n,
    inout  wire        IO_rx_clk_pad_p,
    inout  wire [3:0]  IO_rx_data_pad_n,
    inout  wire [3:0]  IO_rx_data_pad_p
);

    wire       S_hsrx_byteclk;
    wire       S_hsrx_clk_detect;
    wire [7:0] S_ch0_data;
    wire [7:0] S_ch1_data;
    wire       S_ch0_empty, S_ch1_empty;
    wire       S_ch0_wfull,  S_ch0_walfull;
    wire       S_ch1_wfull,  S_ch1_walfull;
    wire       S_rst_n = ~I_rst;

    assign O_hs_rx_clk = S_hsrx_byteclk;

    reg S_hs_valid_r;
    always @(posedge S_hsrx_byteclk or negedge S_rst_n) begin
        if (!S_rst_n) S_hs_valid_r <= 1'b0;
        else          S_hs_valid_r <= (~S_ch0_empty) & (~S_ch1_empty) & S_hsrx_clk_detect;
    end
    assign O_hs_rx_valid = S_hs_valid_r;

    assign O_hs_rx_data = {S_ch1_data, S_ch0_data};

    assign O_lane_error[0]    = S_ch0_wfull   | S_ch1_wfull;
    assign O_lane_error[1]    = S_ch0_walfull | S_ch1_walfull;
    // 时钟 lane HS 失锁（clock lane 未检测到 HS 时提示）
    assign O_lane_match_error = ~S_hsrx_clk_detect;

    wire S_lp_ch0_dp, S_lp_ch0_dn;
    assign O_lp_rx_lane0_p = S_lp_ch0_dp;
    assign O_lp_rx_lane0_n = S_lp_ch0_dn;

    // 8bit FIFO 模式；不使用 GPLL 采样时钟，直接用 hsrx_byteclk_to_fabric
    DR1_PHY_MIPIIO #(
        .DPHY_TIMING_MODE("MIPI"),
        .RTERM_CTL_0(4'b1000), .RTERM_CTL_1(4'b1000),
        .RTERM_CTL_2(4'b1000), .RTERM_CTL_3(4'b1000),
        .RTERM_CTL_C(4'b1000),
        .LVCMOS_EN_0("DISABLE"), .LVCMOS_EN_1("DISABLE"),
        .LVCMOS_EN_2("DISABLE"), .LVCMOS_EN_3("DISABLE"),
        .LVCMOS_EN_C("DISABLE"),
        .EQ_CAL_EN_0("DISABLE"), .EQ_CAL_EN_1("DISABLE"),
        .EQ_CAL_EN_2("DISABLE"), .EQ_CAL_EN_3("DISABLE"),
        .EQ_CAL_EN_C("DISABLE")
    ) u_DR1_PHY_MIPIIO (
        .fifo_ch0_read_1al_empty (),  .fifo_ch1_read_1al_empty (),
        .fifo_ch2_read_1al_empty (),  .fifo_ch3_read_1al_empty (),
        .fifo_ch0_read_3al_empty (),  .fifo_ch1_read_3al_empty (),
        .fifo_ch2_read_3al_empty (),  .fifo_ch3_read_3al_empty (),
        .fifo_ch0_read_empty   (S_ch0_empty), .fifo_ch1_read_empty (S_ch1_empty),
        .fifo_ch2_read_empty   (),           .fifo_ch3_read_empty (),
        .fifo_ch0_write_full   (S_ch0_wfull), .fifo_ch1_write_full (S_ch1_wfull),
        .fifo_ch2_write_full   (),           .fifo_ch3_write_full (),
        .fifo_ch0_write_alfull (S_ch0_walfull), .fifo_ch1_write_alfull (S_ch1_walfull),
        .fifo_ch2_write_alfull (),           .fifo_ch3_write_alfull (),
        .lvds_clk_to_gpll       (),
        .hsrx_byteclk_to_fabric (S_hsrx_byteclk),
        .hsrx_clk_detect        (S_hsrx_clk_detect),
        .lprx_ch0_cd_dn (), .lprx_ch1_cd_dn (), .lprx_ch2_cd_dn (),
        .lprx_ch3_cd_dn (), .lprx_chc_cd_dn (),
        .lprx_ch0_cd_dp (), .lprx_ch1_cd_dp (), .lprx_ch2_cd_dp (),
        .lprx_ch3_cd_dp (), .lprx_chc_cd_dp (),
        .lprx_chc_dn   (), .lprx_ch0_dn (S_lp_ch0_dn), .lprx_ch1_dn (),
        .lprx_ch2_dn   (), .lprx_ch3_dn (),
        .lprx_chc_dp   (), .lprx_ch0_dp (S_lp_ch0_dp), .lprx_ch1_dp (),
        .lprx_ch2_dp   (), .lprx_ch3_dp (),
        .sgcc_lvcmos_n_0 (), .sgcc_lvcmos_n_1 (), .sgcc_lvcmos_n_2 (),
        .sgcc_lvcmos_n_3 (), .sgcc_lvcmos_n_c (),
        .sgcc_lvcmos_p_0 (), .sgcc_lvcmos_p_1 (), .sgcc_lvcmos_p_2 (),
        .sgcc_lvcmos_p_3 (), .sgcc_lvcmos_p_c (),
        .fifo_ch1_data_low (S_ch1_data), .fifo_ch3_data_low (),
        .fifo_ch1_data_high(), .fifo_ch2_data_high(),
        .fifo_ch2_data_low (), .fifo_ch0_data_low (S_ch0_data),
        .fifo_ch3_data_high(), .fifo_ch0_data_high(),
        .cn (IO_rx_clk_pad_n), .cp (IO_rx_clk_pad_p),
        .dn_0 (IO_rx_data_pad_n[0]), .dn_1 (IO_rx_data_pad_n[1]),
        .dn_2 (IO_rx_data_pad_n[2]), .dn_3 (IO_rx_data_pad_n[3]),
        .dp_0 (IO_rx_data_pad_p[0]), .dp_1 (IO_rx_data_pad_p[1]),
        .dp_2 (IO_rx_data_pad_p[2]), .dp_3 (IO_rx_data_pad_p[3]),
        .fifo_read_clk          (S_hsrx_byteclk),
        .fifo_bypass            (1'b0),
        .sgcc_byte_16           (1'b0),
        .hsrx_ch0_deskew_enb    (1'b1), .hsrx_ch1_deskew_enb (1'b1),
        .hsrx_ch2_deskew_enb    (1'b1), .hsrx_ch3_deskew_enb (1'b1),
        .hsrx_chc_deskew_enb    (1'b1),
        .hsrx_ch0_en            (1'b1), .hsrx_ch1_en (1'b1),
        .hsrx_ch2_en            (1'b0), .hsrx_ch3_en (1'b0),
        .hsrx_chc_en            (1'b1),
        .hsrx_ch0_rstn          (S_rst_n), .hsrx_ch1_rstn (S_rst_n),
        .hsrx_ch2_rstn          (1'b0),    .hsrx_ch3_rstn (1'b0),
        .hsrx_chc_rstn          (S_rst_n),
        .fifo_read_8bit_enable  (1'b1),
        .fifo_read_16bit_enable (1'b0),
        .fifo_write_enable      (1'b1),
        .lprx_ch0_en            (1'b1), .lprx_ch1_en (1'b1),
        .lprx_ch2_en            (1'b0), .lprx_ch3_en (1'b0),
        .lprx_chc_en            (1'b1),
        .lptx_ch0_dn            (I_lp_tx_lane0_n),
        .lptx_ch0_dp            (I_lp_tx_lane0_p),
        .lptx_ch0_en            (I_lp_tx_en),
        // MIPI 模式(sgcc_lvds_mode=0)下不使用 GPLL 采样时钟；
        // TD 要求该口只能由 ioclk 驱动，接常量会报 PHY-8023，故不连接。
        .lvds_clk_from_gpll     (),
        .sgcc_lvds_mode         (1'b0),
        .mipi_pd                (1'b0),
        .fifo_read_rstn         (S_rst_n),
        .rterm_ch0_en           (1'b1), .rterm_ch1_en (1'b1),
        .rterm_ch2_en           (1'b0), .rterm_ch3_en (1'b0),
        .rterm_chc_en           (1'b1),
        .fifo_write_rstn        (S_rst_n),
        .hsrx_ch3_deskew_ctl    (9'h1FF), .hsrx_ch1_deskew_ctl (9'h1FF),
        .hsrx_ch2_deskew_ctl    (9'h1FF), .hsrx_ch0_deskew_ctl (9'h1FF),
        .hsrx_chc_deskew_ctl    (9'h1FF),
        .hsrx_ch0_eqrn          (2'b11), .hsrx_ch1_eqrn (2'b11),
        .hsrx_ch2_eqrn          (2'b11), .hsrx_ch3_eqrn (2'b11),
        .hsrx_chc_eqrn          (2'b11),
        .hsrx_ch0_eqcn          (1'b1), .hsrx_ch1_eqcn (1'b1),
        .hsrx_ch2_eqcn          (1'b1), .hsrx_ch3_eqcn (1'b1),
        .hsrx_chc_eqcn          (1'b1),
        .hsrx_ch0_att_ctl       (4'hF), .hsrx_ch1_att_ctl (4'hF),
        .hsrx_ch2_att_ctl       (4'hF), .hsrx_ch3_att_ctl (4'hF),
        .hsrx_chc_att_ctl       (4'hF)
    );

endmodule
