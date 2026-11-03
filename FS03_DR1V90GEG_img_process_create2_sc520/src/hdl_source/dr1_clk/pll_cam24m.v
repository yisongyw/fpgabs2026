/************************************************************\
**  FS03_DR1V90GEG_mipi_hdmi_SC500_720P  --  camera 24 MHz PLL
**  Target device : DR1V90GEG484 (Anlogic SALDRAGON 1, FS03 board)
**
**  SC500CS 需要 24 MHz 基准时钟(原工程由 PH1P PLL 的 clk1_out 给出)。
**  DR1 的 PLL 反馈采用 CLKC0_EXT 时 clk0_out 必须是 refclk 的整数倍，
**  25 MHz 无法直接整除出 24 MHz，故本 PLL 令 clk0_out = 75 MHz
**  (FBCLK_DIV = 3)，VCO = 25 x 3 x 8 = 600 MHz，再由 clk1_out 分频出
**  600 / 25 = 24 MHz 供摄像头使用。
**
**  输出：clk0_out = 75 MHz(仅反馈用，可悬空)；clk1_out = 24 MHz -> O_cam_24m
\************************************************************/
`timescale 1 ns / 100 fs

module pll_cam24m
(
  input                         refclk,      // 25 MHz, FS03 I_sysclk @ L18
  input                         pllreset,    // 高有效
  output                        clk0_out,    // 75 MHz (反馈用)
  output                        clk1_out,    // 24 MHz (摄像头基准)
  output                        lock
);
  wire                          clk0_buf;

  DR1_LOGIC_BUFG bufg_feedback (
    .i ( clk0_buf  ),
    .o ( clk0_out  )
  );

  DR1_PHY_PLL_WRAPERR_d2d4a8d4bd40
  #(
      .FBKCLK              ("CLKC0_EXT"),
      .REFCLK_DIV          (1),
      .FBCLK_DIV           (3),          // clk0_out / refclk = 75 / 25
      .CLKC0_FPHASE        (0),
      .CLKC0_CPHASE        (7),          // DIV-1
      .CLKC0_DIV           (8),          // VCO = 75 x 8 = 600 MHz
      .CLKC0_DUTY_INT      (4),
      .CLKC0_ENABLE        ("ENABLE"),
      .FIN                 ("25.0"),
      .FEEDBK_MODE         ("NORMAL"),
      .PLL_USR_RST         ("ENABLE"),
      .PLL_FEED_TYPE       ("EXTERNAL"),
      .LPF_RES             (3),
      .LPF_CAP             (2),
      .ICP_CUR             (11),
      .GMC_GAIN            (1),
      .FRAC_ENABLE         ("DISABLE"),
      .DITHER_ENABLE       ("DISABLE"),
      .SDM_FRAC            (0),
      .SSC_AMP             (0.0000),
      .MPHASE_ENABLE       ("DISABLE"),
      .DYN_PHASE_PATH_SEL  ("DISABLE"),
      .DYN_FPHASE_EN       ("DISABLE"),
      .CLKC0_FPHASE_RSTSEL (0),
      .CLKC0_DUTY50        ("ENABLE"),
      .INTPI               (0),
      .SSC_ENABLE          ("DISABLE"),
      .SSC_MODE            ("DOWN"),
      .SSC_FREQ_DIV        (0),
      .SSC_RNGE            (0),
      .HIGH_SPEED_EN       ("DISABLE"),
      .CLKC1_ENABLE        ("ENABLE"),
      .CLKC1_DIV           (25),         // 600 / 25 = 24 MHz
      .CLKC1_CPHASE        (24),
      .CLKC1_FPHASE        (0),
      .CLKC1_FPHASE_RSTSEL (0),
      .CLKC1_DUTY50        ("ENABLE"),
      .CLKC1_DUTY_INT      (13)
  ) DR1_PHY_PLL_WRAPERR_cam_Inst
  (
      .clk2_en           (1'b0),
      .clk2_out          (),
      .clkb2_out         (),
      .clk3_en           (1'b0),
      .clk3_out          (),
      .clkb3_out         (),
      .clk4_en           (1'b0),
      .clk4_out          (),
      .clkb4_out         (),
      .clk5_en           (1'b0),
      .clk5_out          (),
      .clkb5_out         (),
      .clk6_en           (1'b0),
      .clk6_out          (),
      .clkb6_out         (),
      .refclk            (refclk),
      .ssc_en            (1'b0),
      .ext_freq_mod_val  (17'b00000000000000000),
      .ext_freq_mod_en   (1'b0),
      .ext_freq_mod_clk  (1'b0),
      .clkc_rst          (2'b00),
      .fbclk             (clk0_out),
      .drp_rdata         (),
      .drp_rdy           (),
      .drp_err           (),
      .drp_wdata         (8'b00000000),
      .drp_addr          (8'b00000000),
      .drp_wr            (1'b0),
      .drp_rd            (1'b0),
      .drp_sel           (1'b0),
      .drp_rstn          (1'b1),
      .drp_clk           (1'b0),
      .cps_step          (2'b00),
      .psclksel          (3'b000),
      .psdone            (),
      .psstep            (1'b0),
      .psdown            (1'b0),
      .psclk             (1'b0),
      .pllpd             (1'b0),
      .wakeup            (1'b0),
      .refclk_rst        (1'b0),
      .clk0_en           (1'b1),
      .clkb0_out         (),
      .clk0_out          (clk0_buf),
      .clk1_en           (1'b1),
      .clkb1_out         (),
      .clk1_out          (clk1_out),
      .lock              (lock),
      .pllreset          (pllreset)
  );

endmodule
