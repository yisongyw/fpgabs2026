//-----------------------------------------------------------------------------
// Module     : PLL_eth
// Project    : HX1P35Aimg_process_create1
// Description: dedicated 125 MHz PLL for the RGMII Gigabit Ethernet TX clock.
//
//   The main PLL (ip_source/PLL/PLL.v) is left untouched (create1 keeps the
//   improve4 video/ISP clock plan : 100 / 23.81 / 75 / 375 MHz).  This second
//   PLL reuses the very same generated wrapper ph1p_phy_pll_wrapper_25a56e5ce2f9
//   and the very same loop parameters (REFCLK_DIV=1, FBCLK_DIV=2, VCO=1500 MHz).
//   CLKC0 is kept as the 100 MHz external-feedback anchor (DIV=15) and CLKC2
//   (unused in the video PLL) is enabled with DIV=12 :
//
//       50 MHz * 2 * 15 / 1 = VCO 1500 MHz
//       1500 / 15 = 100 MHz  (CLKC0, feedback only)
//       1500 / 12 = 125 MHz  (CLKC2, RGMII GTX_CLK)
//
//   Reference : 33_ethernet_udp_led/uisrc/03_ip/pll/pll/pll.v (proven 125 MHz
//   config on the HX1P35A family), parameters translated to the VCO=1500 MHz
//   loop used by the video PLL.
//-----------------------------------------------------------------------------
module PLL_eth (
    input  wire refclk,      // 50 MHz board oscillator
    input  wire reset,
    output wire clk125_out,  // 125 MHz, RGMII GTX_CLK
    output wire lock
);

    wire clk0_buf;
    wire clk0_g;
    wire clk2_buf;

    PH1P_LOGIC_BUFG bufg_fb  (.i(clk0_buf), .o(clk0_g));
    PH1P_LOGIC_BUFG bufg_125 (.i(clk2_buf), .o(clk125_out));

    ph1p_phy_pll_wrapper_25a56e5ce2f9 #(
        .FBKCLK                ("CLKC0_EXT"),
        .FBKCLK_INT            ("CLKC0_EXT"),
        .FIN                   ("50.000000000"),
        .REFCLK_DIV            (1),
        .FBCLK_DIV             (2),
        .CLKC0_ENABLE          ("ENABLE"),
        .CLKC0_DIV             (15),
        .CLKC0_CPHASE          (14),
        .CLKC0_FPHASE          (0),
        .CLKC0_FPHASE_RSTSEL   (0),
        .CLKC0_DUTY50          ("ENABLE"),
        .CLKC0_DUTY_INT        (8),
        .CLKC1_ENABLE          ("DISABLE"),
        .CLKC1_DIV             (2),
        .CLKC1_CPHASE          (1),
        .CLKC1_FPHASE          (0),
        .CLKC1_FPHASE_RSTSEL   (0),
        .CLKC1_DUTY50          ("ENABLE"),
        .CLKC1_DUTY_INT        (1),
        .CLKC2_ENABLE          ("ENABLE"),
        .CLKC2_DIV             (12),
        .CLKC2_CPHASE          (11),
        .CLKC2_FPHASE          (0),
        .CLKC2_FPHASE_RSTSEL   (0),
        .CLKC2_DUTY50          ("ENABLE"),
        .CLKC2_DUTY_INT        (6),
        .CLKC3_ENABLE          ("DISABLE"),
        .CLKC3_DIV             (2),
        .CLKC3_CPHASE          (1),
        .CLKC3_FPHASE          (0),
        .CLKC3_FPHASE_RSTSEL   (0),
        .CLKC3_DUTY50          ("ENABLE"),
        .CLKC3_DUTY_INT        (1),
        .CLKC4_ENABLE          ("DISABLE"),
        .CLKC4_DIV             (2),
        .CLKC4_CPHASE          (1),
        .CLKC4_FPHASE          (0),
        .CLKC4_FPHASE_RSTSEL   (0),
        .CLKC4_DUTY50          ("ENABLE"),
        .CLKC4_DUTY_INT        (1),
        .CLKC5_ENABLE          ("DISABLE"),
        .CLKC5_DIV             (2),
        .CLKC5_CPHASE          (1),
        .CLKC5_FPHASE          (0),
        .CLKC5_FPHASE_RSTSEL   (0),
        .CLKC5_DUTY50          ("ENABLE"),
        .CLKC5_DUTY_INT        (1),
        .CLKC6_ENABLE          ("DISABLE"),
        .CLKC6_DIV             (2),
        .CLKC6_CPHASE          (1),
        .CLKC6_FPHASE          (0),
        .CLKC6_FPHASE_RSTSEL   (0),
        .CLKC6_DUTY50          ("ENABLE"),
        .CLKC6_DUTY_INT        (1),
        .PLL_USR_RST           ("ENABLE"),
        .PLL_FEED_TYPE         ("EXTERNAL"),
        .PLL_FASTLOOP          ("ENABLE"),
        .LPF_RES               (0),
        .LPF_CAP               (13),
        .ICP_CUR               (29),
        .PHASE_PATH_SEL        (0),
        .DYN_PHASE_PATH_SEL    ("DISABLE"),
        .DYN_FPHASE_EN         ("DISABLE"),
        .PI_OUT_SEL            ("NORMAL"),
        .PI_FRAC_EN            ("DISABLE"),
        .CLKC0_PI_SHIFT_EN     ("DISABLE"),
        .FEEDBK_MODE           ("NORMAL"),
        .DYN_CPHASE_EN         ("DISABLE")
    ) u_pll_eth (
        .clk1_en        (1'b0),
        .clk1_out       (),
        .clkb1_out      (),
        .clk2_en        (1'b1),
        .clk2_out       (clk2_buf),
        .clkb2_out      (),
        .clk3_en        (1'b0),
        .clk3_out       (),
        .clkb3_out      (),
        .clk4_en        (1'b0),
        .clk4_out       (),
        .clkb4_out      (),
        .clk5_en        (1'b0),
        .clk5_out       (),
        .clkb5_out      (),
        .clk6_en        (1'b0),
        .clk6_out       (),
        .clkb6_out      (),
        .refclk         (refclk),
        .drp_clk        (1'b0),
        .drp_rstn       (1'b1),
        .drp_sel        (1'b0),
        .drp_rd         (1'b0),
        .drp_wr         (1'b0),
        .drp_addr       (8'd0),
        .drp_wdata      (8'd0),
        .drp_err        (),
        .drp_rdy        (),
        .drp_rdata      (),
        .psclk          (1'b0),
        .psclksel       (3'b000),
        .psstep         (1'b0),
        .psdown         (1'b0),
        .cps_step       (1'b0),
        .psdone         (),
        .ssc_reset      (1'b0),
        .clkc_rst       (2'b00),
        .pllpd          (1'b0),
        .fbclk          (clk0_g),
        .wakeup         (1'b1),
        .refclk_rst     (1'b0),
        .clk0_en        (1'b1),
        .clkb0_out      (),
        .clk0_out       (clk0_buf),
        .lock           (lock),
        .reset          (reset)
    );

endmodule
