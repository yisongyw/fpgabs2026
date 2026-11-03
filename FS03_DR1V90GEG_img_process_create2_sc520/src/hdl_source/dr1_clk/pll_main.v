/************************************************************\
**  FS03_DR1V90GEG_mipi_hdmi_SC500_720P  --  main PLL
**  Target device : DR1V90GEG484 (Anlogic SALDRAGON 1, FS03 board)
**  Reference     : FS03_DR1V90GEG_exp_prj/ex_fpga_base/.../mypll/mypll.v
**
**  FS03 板载系统时钟为 25 MHz(HX1P35A 为 50 MHz)，因此 PLL 需重新配置。
**  输出时钟与原工程(PH1P)保持一致：
**      clk0_out = 100 MHz   (MIPI/逻辑辅助时钟，原 S_100m_clk)
**      clk4_out =  75 MHz   (HDMI 720P60 像素时钟 74.25M 取整约定值)
**      clk5_out = 375 MHz   (HDMI 5 倍串行时钟)
**  VCO = 25 MHz x 4 x 15 = 1500 MHz
**  说明：DR1 一颗 PLL 无法同时整除出 24 MHz(摄像头基准) 与 375 MHz，
**        故 24 MHz 由独立的 pll_cam24m.v 产生(合计 2 颗 PLL，原工程 3 颗)。
\************************************************************/
`timescale 1 ns / 100 fs

module pll_main
(
  input                         refclk,      // 25 MHz, FS03 I_sysclk @ L18
  input                         pllreset,    // 高有效
  output                        clk0_out,    // 100 MHz
  output                        clk4_out,    //  75 MHz (HDMI pixel)
  output                        clk5_out,    // 375 MHz (HDMI serial = 5x pixel)
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
      .FBCLK_DIV           (4),          // clk0_out / refclk = 100 / 25
      .CLKC0_FPHASE        (0),
      .CLKC0_CPHASE        (14),         // DIV-1
      .CLKC0_DIV           (15),         // 1500/15 = 100 MHz
      .CLKC0_DUTY_INT      (8),
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
      .CLKC4_ENABLE        ("ENABLE"),
      .CLKC4_DIV           (20),         // 1500/20 = 75 MHz
      .CLKC4_CPHASE        (19),
      .CLKC4_FPHASE        (0),
      .CLKC4_FPHASE_RSTSEL (0),
      .CLKC4_DUTY50        ("ENABLE"),
      .CLKC4_DUTY_INT      (10),
      .CLKC5_ENABLE        ("ENABLE"),
      .CLKC5_DIV           (4),          // 1500/4 = 375 MHz
      .CLKC5_CPHASE        (3),
      .CLKC5_FPHASE        (0),
      .CLKC5_FPHASE_RSTSEL (0),
      .CLKC5_DUTY50        ("ENABLE"),
      .CLKC5_DUTY_INT      (2)
  ) DR1_PHY_PLL_WRAPERR_main_Inst
  (
      .clk1_en           (1'b0),
      .clk1_out          (),
      .clkb1_out         (),
      .clk2_en           (1'b0),
      .clk2_out          (),
      .clkb2_out         (),
      .clk3_en           (1'b0),
      .clk3_out          (),
      .clkb3_out         (),
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
      .clk4_en           (1'b1),
      .clkb4_out         (),
      .clk4_out          (clk4_out),
      .clk5_en           (1'b1),
      .clkb5_out         (),
      .clk5_out          (clk5_out),
      .lock              (lock),
      .pllreset          (pllreset)
  );

endmodule
