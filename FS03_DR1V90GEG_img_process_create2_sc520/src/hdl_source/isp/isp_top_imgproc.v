//-----------------------------------------------------------------------------
// Module     : isp_top_imgproc
// Project    : HX1P35Aimg_process_extend1
// Description: ISP top level with the competition image processing pipeline
//              inserted between AWB and the 96bit -> 128bit packer.
//
//              Data flow (all in the S_csi_rx_clk domain):
//                RAW10 40bit -> demosaic (BGGR -> RGB888 128bit)
//                            -> data128_96 (96bit = 4 x RGB888)
//                            -> awb_lite          (DSP free white balance)
//                            -> img_proc_top   <-- base layer (SW1/2/3)
//                                                   + style layer (KEY3)
//                            -> data_96bit_to_128bit -> DDR
//
//              This file is a copy of the reference isp_top.v with the
//              algorithm stage added, so the official isp_top.v is left
//              untouched and can still be used for A/B comparison.
//
//              base2 : * "awb" is replaced by "awb_lite" (no divider IP, 0 DSP)
//                      * base algorithms are selected by three independent
//                        enables (SW1 SOBEL / SW2 GRAY / SW3 BIN)
//                      * the threshold is 9 bit wide (0 .. 300, 16 steps)
//
//              extend1 : * the style selector I_style[3:0] (KEY3) is passed
//                          down to img_proc_top (stylization / visual
//                          enhancement layer, 16 positions reserved)
//
//              extend2 : * obj_detect / obj_feature_extract are added as a
//                          pure SIDE PATH.  They tap the same RGB stream that
//                          feeds img_proc_top and never modify a pixel, so the
//                          base layer and the style layer behave exactly like
//                          in extend1.  The result is the object list
//                          O_obj_bus (4 objects) + O_obj_num.
//-----------------------------------------------------------------------------
module isp_top_imgproc #(
    parameter CLS_W = 6,            // bit width of a class index (2**CLS_W max)
    parameter NOUT  = 64,           // number of MLP classes (NOUT <= 2**CLS_W)
    parameter REC_W = 50 + 2*CLS_W  // width of one object record (CLS_W=6 -> 62)
)(
    input            axi4s_video_aclk,
    input            I_rst_n,
    input            I_tlast,
    input            I_tuser,
    input  [39:0]    I_tdata,
    input            I_tvalid,
    input  [9:0]     I_tdest,
    input            O_tready,          // downstream ready  (name kept as is)
    input            I_en_sobel,        // SW1
    input            I_en_gray,         // SW2
    input            I_en_bin,          // SW3
    input  [8:0]     I_th,              // algorithm threshold, 0 .. 300
    input  [3:0]     I_style,           // KEY3 : stylization / enhancement

    // ---- create1 : SD card image stream injection (remote SD / CAM cmd) ----
    input            I_sd_mode,         // 1 = SD image stream, 0 = camera stream
    input  [95:0]    I_sd_tdata,        // 4 x RGB888 from sd_img_feed
    input            I_sd_tvalid,
    input            I_sd_tuser,        // frame start marker (first beat)
    input            I_sd_tlast,        // end of line marker  (beat 319)

    // ---- improve2 : image quality auto adjustment (advanced req. 2.5-5) ----
    input  [2:0]     I_qual_mode,       // 0 off / 1 HE / 2 CLAHE / 3 STRETCH
    output [7:0]     O_yavg,            // mean luminance of the last frame
    output [31:0]    O_sum_y,           // sum of all sampled luminance values
    output [17:0]    O_nsamp,           // number of samples of the last frame
    output [17:0]    O_over,            // pixels with Y >= 250 (highlight prot.)
    output [17:0]    O_under,           // pixels with Y <= 8
    output           O_stat_valid,      // 1 clock pulse at every frame start

    output [2*REC_W-1:0] O_obj_bus,    // extend3 : 2 x REC_W bit object records
    output [3:0]     O_obj_num,         // extend2 : number of targets

    // ---- improve1 : hardware motion detection (advanced requirement 2.5-4)
    output [11:0]    O_mot_cnt,         // moving cells of the last frame
    output [10:0]    O_bb_xmin,
    output [10:0]    O_bb_xmax,
    output [9:0]     O_bb_ymin,
    output [9:0]     O_bb_ymax,
    output           O_bb_valid,
    output           O_mot_alarm,
    output           O_mot_frame,       // 1 clock pulse at the frame end
    output           O_flag_we,         // motion flag memory write port
    output [13:0]    O_flag_addr,
    output [7:0]     O_flag_data,
    output [1:0]     O_wr_page_g,

    output [127:0]   O_tdata,
    output           O_tlast,
    output           O_tuser,
    output           O_tvalid,
    output           I_tready           // upstream ready    (name kept as is)
);

    wire         m_aixs_tvalid;  //synthesis keep
    wire [127:0] m_aixs_tdata;   //synthesis keep
    wire         m_aixs_tuser;   //synthesis keep
    wire         m_aixs_tlast;   //synthesis keep
    wire         m_aixs_tready;  //synthesis keep

    wire [95:0]  m_aixs_tdata_96; //synthesis keep

    wire         awb_O_tlast;    //synthesis keep
    wire         awb_O_tuser;    //synthesis keep
    wire [95:0]  awb_O_tdata;    //synthesis keep
    wire         awb_O_tvalid;   //synthesis keep
    wire         awb_O_tready;   //synthesis keep

    // create1 : source select mux -- SD image stream or camera AWB stream.
    //           img_proc_top / obj_detect / frame_diff3 / hist_stat all
    //           consume the selected stream (S_m_*).
    wire [95:0]  S_m_tdata  = I_sd_mode ? I_sd_tdata  : awb_O_tdata;
    wire         S_m_tvalid = I_sd_mode ? I_sd_tvalid : awb_O_tvalid;
    wire         S_m_tuser  = I_sd_mode ? I_sd_tuser  : awb_O_tuser;
    wire         S_m_tlast  = I_sd_mode ? I_sd_tlast  : awb_O_tlast;

    wire         proc_O_tlast;   //synthesis keep
    wire         proc_O_tuser;   //synthesis keep
    wire [95:0]  proc_O_tdata;   //synthesis keep
    wire         proc_O_tvalid;  //synthesis keep
    wire         proc_O_tready;  //synthesis keep

    wire         awb_O_tuser_128;  //synthesis keep
    wire         awb_O_tvalid_128; //synthesis keep
    wire [127:0] awb_O_tdata_128;  //synthesis keep

    // improve2 : the output of the quality enhancement stage (hist_eq).  It is
    // a separate pipeline stage so that the combinational depth of img_proc_top
    // is not increased by the tone mapping.
    wire         q_O_tlast;
    wire         q_O_tuser;
    wire [95:0]  q_O_tdata;
    wire         q_O_tvalid;

    assign proc_O_tready = O_tready;
    assign awb_O_tready  = O_tready;

    assign O_tdata  = awb_O_tdata_128;
    assign O_tlast  = q_O_tlast;
    assign O_tuser  = awb_O_tuser_128;
    assign O_tvalid = awb_O_tvalid_128;

    //-------------------------------------------------------------------------
    // Bayer -> RGB888
    //-------------------------------------------------------------------------
    demosaic #(
        .IMG_HEIGHT          (720 ),
        .IMG_WIDTH           (1280),
        .data_complete_delay (50  ),
        .BAYER_MODE          ("BGGR")
    ) u_demosaic (
        .I_clk             ( axi4s_video_aclk ),
        .I_rst_n           ( I_rst_n          ),
        .axi4s_video_tdata ( I_tdata          ),
        .axi4s_video_tdest ( I_tdest          ),
        .axi4s_video_tlast ( I_tlast          ),
        .axi4s_video_tvalid( I_tvalid         ),
        .axi4s_video_tuser ( I_tuser          ),
        .axi4s_video_tready( I_tready         ),
        .O_tlast           ( m_aixs_tlast     ),
        .O_tuser           ( m_aixs_tuser     ),
        .O_tdata           ( m_aixs_tdata     ),
        .O_tvalid          ( m_aixs_tvalid    ),
        .O_tready          ( m_aixs_tready    )
    );

    data128_96 u_data128_96 (
        .I_tdata ( m_aixs_tdata    ),
        .O_tdata ( m_aixs_tdata_96 )
    );

    //-------------------------------------------------------------------------
    // Auto white balance (DSP free version, 0 divider IP, 0 DSP)
    //-------------------------------------------------------------------------
    awb_lite #(
        .IMG_HEIGHT ( 720  ),
        .IMG_WIDTH  ( 1280 )
    ) u_awb_lite (
        .I_clk   ( axi4s_video_aclk ),
        .I_rst_n ( I_rst_n          ),
        .I_tlast ( m_aixs_tlast     ),
        .I_tuser ( m_aixs_tuser     ),
        .I_tdata ( m_aixs_tdata_96  ),
        .I_tvalid( m_aixs_tvalid    ),
        .I_tready( m_aixs_tready    ),
        .O_tlast ( awb_O_tlast      ),
        .O_tuser ( awb_O_tuser      ),
        .O_tdata ( awb_O_tdata      ),
        .O_tvalid( awb_O_tvalid     ),
        .O_tready( awb_O_tready     )
    );

    //-------------------------------------------------------------------------
    // Competition image processing
    //   layer 1 : GRAY / SOBEL / BINARY, independent (SW1 / SW2 / SW3)
    //   layer 2 : stylization / visual enhancement     (KEY3)
    //-------------------------------------------------------------------------
    img_proc_top #(
        .IMG_WIDTH    ( 1280 ),
        .IMG_HEIGHT   ( 720  ),
        .FILL_GAP_MAX ( 5'd16 )
    ) u_img_proc_top (
        .I_clk      ( axi4s_video_aclk ),
        .I_rst_n    ( I_rst_n          ),
        .I_en_sobel ( I_en_sobel       ),
        .I_en_gray  ( I_en_gray        ),
        .I_en_bin   ( I_en_bin         ),
        .I_th       ( I_th             ),
        .I_style    ( I_style          ),
        .I_tlast    ( S_m_tlast        ),
        .I_tuser    ( S_m_tuser        ),
        .I_tdata    ( S_m_tdata        ),
        .I_tvalid   ( S_m_tvalid       ),
        .I_tready   ( O_tready         ),
        .O_tlast    ( proc_O_tlast     ),
        .O_tuser    ( proc_O_tuser     ),
        .O_tdata    ( proc_O_tdata     ),
        .O_tvalid   ( proc_O_tvalid    ),
        .O_tready   ( proc_O_tready    )
    );

    //-------------------------------------------------------------------------
    // extend2 : target detection + recognition (pure side path, no pixel is
    //           modified).  The detector works on the SAME stream that feeds
    //           img_proc_top (S_m_tdata, SD or camera), it only produces the
    //           object list.
    //-------------------------------------------------------------------------
    wire         det_tile_valid;
    wire         det_tile_last;
    wire [3:0]   det_tile_id;
    wire [16:0]  det_cnt;
    wire [10:0]  det_xmin;
    wire [10:0]  det_xmax;
    wire [9:0]   det_ymin;
    wire [9:0]   det_ymax;
    wire [16:0]  det_peri;
    wire [24:0]  det_sum_cb;
    wire [24:0]  det_sum_cr;
    wire [16:0]  det_skin;
    wire [7:0]   det_ylo;
    wire [7:0]   det_yhi;
    wire         det_tile_next;

    obj_detect #(
        .IMG_WIDTH  ( 1280 ),
        .IMG_HEIGHT ( 720  )
    ) u_obj_detect (
        .I_clk          ( axi4s_video_aclk ),
        .I_rst_n        ( I_rst_n          ),
        .I_en           ( 1'b1             ),
        .I_tvalid       ( S_m_tvalid       ),
        .I_tuser        ( S_m_tuser        ),
        .I_tdata        ( S_m_tdata        ),
        .I_tile_next    ( det_tile_next    ),
        .O_tile_valid   ( det_tile_valid   ),
        .O_tile_last    ( det_tile_last    ),
        .O_tile_id      ( det_tile_id      ),
        .O_cnt          ( det_cnt          ),
        .O_xmin         ( det_xmin         ),
        .O_xmax         ( det_xmax         ),
        .O_ymin         ( det_ymin         ),
        .O_ymax         ( det_ymax         ),
        .O_peri         ( det_peri         ),
        .O_sum_cb       ( det_sum_cb       ),
        .O_sum_cr       ( det_sum_cr       ),
        .O_skin         ( det_skin         ),
        .O_ylo          ( det_ylo          ),
        .O_yhi          ( det_yhi          )
    );

    obj_feature_extract #(
        .MIN_AREA ( 17'd500 ),
        .NOBJ     ( 2       ),          // extend3 : 4 -> 2 objects
        .CLS_W    ( CLS_W   ),
        .NOUT     ( NOUT    ),
        .REC_W    ( REC_W   )
    ) u_obj_feature_extract (
        .I_clk          ( axi4s_video_aclk ),
        .I_rst_n        ( I_rst_n          ),
        .I_tile_valid   ( det_tile_valid   ),
        .I_tile_last    ( det_tile_last    ),
        .I_cnt          ( det_cnt          ),
        .I_xmin         ( det_xmin         ),
        .I_xmax         ( det_xmax         ),
        .I_ymin         ( det_ymin         ),
        .I_ymax         ( det_ymax         ),
        .I_peri         ( det_peri         ),
        .I_sum_cb       ( det_sum_cb       ),
        .I_sum_cr       ( det_sum_cr       ),
        .I_skin         ( det_skin         ),
        .I_ylo          ( det_ylo          ),
        .I_yhi          ( det_yhi          ),
        .O_tile_next    ( det_tile_next    ),
        .O_obj_num      ( O_obj_num        ),
        .O_obj_bus      ( O_obj_bus        )
    );

    //-------------------------------------------------------------------------
    // improve1 : hardware motion detection (three frame difference).
    //
    //   Like obj_detect it is a pure SIDE PATH : it taps the very same stream
    //   that feeds img_proc_top and never modifies a pixel, therefore it can
    //   not add a single clock of end to end latency and it can not disturb
    //   the base layer or the style layer.
    //-------------------------------------------------------------------------
    frame_diff3 #(
        .IMG_WIDTH ( 1280 ),
        .IMG_HEIGHT( 720  ),
        .DS_X      ( 16   ),
        .DS_Y      ( 16   ),
        .MOT_TH    ( 60   )
    ) u_frame_diff3 (
        .I_clk        ( axi4s_video_aclk ),
        .I_rst_n      ( I_rst_n          ),
        .I_en         ( 1'b1             ),
        .I_tvalid     ( S_m_tvalid       ),
        .I_tuser      ( S_m_tuser        ),
        .I_tdata      ( S_m_tdata        ),
        .I_th         ( I_th             ),
        .O_flag_we    ( O_flag_we        ),
        .O_flag_addr  ( O_flag_addr      ),
        .O_flag_data  ( O_flag_data      ),
        .O_wr_page_g  ( O_wr_page_g      ),
        .O_mot_cnt    ( O_mot_cnt        ),
        .O_bb_xmin    ( O_bb_xmin        ),
        .O_bb_xmax    ( O_bb_xmax        ),
        .O_bb_ymin    ( O_bb_ymin        ),
        .O_bb_ymax    ( O_bb_ymax        ),
        .O_bb_valid   ( O_bb_valid       ),
        .O_mot_alarm  ( O_mot_alarm      ),
        .O_frame_tick ( O_mot_frame      )
    );

    //-------------------------------------------------------------------------
    // improve2 : luminance statistics + automatic quality enhancement
    //   hist_stat is a side path (it taps the AWB output like obj_detect),
    //   hist_eq is the only stage that modifies pixels and it is switched by
    //   the new KEY3 positions 4 / 5 / 6.  Both are pure streaming blocks,
    //   hist_eq adds exactly one clock of pipeline (4 pixels) which is
    //   0.07 us of the 16.7 ms frame budget.
    //-------------------------------------------------------------------------
    wire [5:0]  stat_bin_addr;
    wire [19:0] stat_bin_data;

    hist_stat #(
        .NBIN ( 64 )
    ) u_hist_stat (
        .I_clk        ( axi4s_video_aclk ),
        .I_rst_n      ( I_rst_n          ),
        .I_en         ( 1'b1             ),
        .I_tvalid     ( S_m_tvalid       ),
        .I_tuser      ( S_m_tuser        ),
        .I_tdata      ( S_m_tdata        ),
        .O_sum_y      ( O_sum_y          ),
        .O_nsamp      ( O_nsamp          ),
        .O_over       ( O_over           ),
        .O_under      ( O_under          ),
        .O_yavg       ( O_yavg           ),
        .O_valid      ( O_stat_valid     ),
        .I_bin_addr   ( stat_bin_addr    ),
        .O_bin_data   ( stat_bin_data    )
    );

    hist_eq #(
        .NBIN    ( 64   ),
        .SCALE   ( 20'd73 ),
        .USE_DSP ( 1     )
    ) u_hist_eq (
        .I_clk        ( axi4s_video_aclk ),
        .I_rst_n      ( I_rst_n          ),
        .O_bin_addr   ( stat_bin_addr    ),
        .I_bin_data   ( stat_bin_data    ),
        .I_nsamp      ( O_nsamp          ),
        .I_stat_valid ( O_stat_valid     ),
        .I_mode       ( I_qual_mode      ),
        .I_tvalid     ( proc_O_tvalid    ),
        .I_tuser      ( proc_O_tuser     ),
        .I_tlast      ( proc_O_tlast     ),
        .I_tdata      ( proc_O_tdata     ),
        .O_tvalid     ( q_O_tvalid       ),
        .O_tuser      ( q_O_tuser        ),
        .O_tlast      ( q_O_tlast        ),
        .O_tdata      ( q_O_tdata        )
    );

    //-------------------------------------------------------------------------
    // 96 bit (4 x RGB888) -> 128 bit, ready for the DDR frame buffer
    //-------------------------------------------------------------------------
    data_96bit_to_128bit u_data_96bit_to_128bit(
        .I_clk             ( axi4s_video_aclk  ),
        .I_rst_n           ( I_rst_n           ),

        .I_96b_frame_start ( q_O_tuser         ),
        .I_96b_valid       ( q_O_tvalid        ),
        .I_96b_data        ( q_O_tdata         ),

        .O_128b_frame_start( awb_O_tuser_128   ),
        .O_128b_valid      ( awb_O_tvalid_128  ),
        .O_128b_data       ( awb_O_tdata_128   )
    );

endmodule
