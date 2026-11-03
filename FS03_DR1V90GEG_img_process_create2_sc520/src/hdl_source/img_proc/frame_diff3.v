//-----------------------------------------------------------------------------
// Module     : frame_diff3
// Project    : HX1P35Aimg_process_improve1
// Description: Hardware motion detection with the THREE FRAME DIFFERENCE method
//              (advanced requirement 2.5 (4) : "try to implement the frame
//              difference method in hardware logic and overlay a motion area
//              hint or a counter on the picture").
//
//   Algorithm
//   ---------
//        D1 = |f(n)   - f(n-1)|
//        D2 = |f(n-1) - f(n-2)|
//        M  = (D1 > Tm) AND (D2 > Tm)
//
//   The AND of TWO differences is what removes the "ghost" : a single frame
//   difference also flags the background that becomes visible again after an
//   object has moved away.  That background only appears in ONE of the two
//   differences and is therefore suppressed.
//
//   Why 16 x 16 down sampling
//   -------------------------
//   A full resolution reference frame would need 7.4 Mbit of on chip memory,
//   the PH1P35 only has ~2160 Kbit.  Taking one luminance sample out of every
//   16 x 16 pixel block leaves 80 x 45 = 3600 samples = 28.8 Kbit per
//   reference frame - two of them fit into a handful of ERAM blocks, which are
//   the most idle resource of this device (the LUTs are the bottleneck).
//   One samples covers a 16 x 16 pixel block, so the smallest object that can
//   be detected is 16 x 16 pixel, exactly the granularity the detailed
//   specification asks for.
//
//   Storage
//   -------
//        refA / refB : 2 * 3600 * 8 bit = 57.6 Kbit   (ERAM, this module)
//        flag pages  : 3 * 3600 * 8 bit = 86.4 Kbit   (ERAM, inside
//                      osd_motion_overlay - the read port has to run on the
//                      HDMI pixel clock, the write port on the CSI clock)
//
//   Three flag pages
//   ----------------
//   The writer (CSI domain) and the reader (HDMI domain) run on two unrelated
//   clocks.  Three pages are rotated and the reader always takes the page that
//   has been completed one frame ago, so no page is ever written and read at
//   the same time.  The page pointer is transferred as a GRAY code (only one
//   bit changes at a time) so a plain two stage synchroniser is safe.
//
//   Timing
//   ------
//   With 4 pixel per beat the samples arrive one per four clocks, the datapath
//   needs two clocks per sample, therefore the pipeline never stalls and the
//   60 fps pixel stream is not touched at all.  The module is a pure SIDE PATH
//   (it taps the AWB output like obj_detect) - it never modifies a pixel and
//   therefore adds ZERO end to end latency.
//-----------------------------------------------------------------------------
module frame_diff3 #(
    parameter IMG_WIDTH  = 1280,
    parameter IMG_HEIGHT = 720,
    parameter DS_X       = 16,      // horizontal down sampling (power of two)
    parameter DS_Y       = 16,      // vertical   down sampling (power of two)
    parameter MOT_TH     = 60       // alarm threshold : moving cells (of 3600)
)(
    input  wire             I_clk,          // CSI pixel clock (~60 MHz)
    input  wire             I_rst_n,
    input  wire             I_en,

    input  wire             I_tvalid,
    input  wire             I_tuser,        // 1 = first beat of a frame
    input  wire [95:0]      I_tdata,        // 4 x RGB888, pixel 0 = [95:72]

    input  wire [8:0]       I_th,           // algorithm threshold (KEY2), 0..300

    // ---- motion flag memory write port (CSI domain, see osd_motion_overlay)
    output reg              O_flag_we,
    output reg  [13:0]      O_flag_addr,    // { page(2 bit), cell(12 bit) }
    output reg  [7:0]       O_flag_data,    // bit 0 = 1 : this cell moves
    output reg  [1:0]       O_wr_page_g,    // gray coded page currently written

    // ---- result of the last COMPLETED frame (latched, stable for one frame)
    output reg  [11:0]      O_mot_cnt,      // number of moving cells (0..3600)
    output reg  [10:0]      O_bb_xmin,
    output reg  [10:0]      O_bb_xmax,
    output reg  [9:0]       O_bb_ymin,
    output reg  [9:0]       O_bb_ymax,
    output reg              O_bb_valid,
    output reg              O_mot_alarm,
    output reg              O_frame_tick    // 1 clock pulse at the frame end
);

    //-------------------------------------------------------------------------
    // geometry
    //-------------------------------------------------------------------------
    localparam IMG_WIDTH_GRP = IMG_WIDTH  >> 2;              // 320 beats/line
    localparam GW      = IMG_WIDTH  / DS_X;                  // 80 cells per row
    localparam GH      = IMG_HEIGHT / DS_Y;                  // 45 cell rows
    localparam NCELL   = GW * GH;                            // 3600
    localparam CELL_ADW = 12;                                // 3600 -> 12 bit

    //-------------------------------------------------------------------------
    // input stream position (identical scheme to obj_detect)
    //-------------------------------------------------------------------------
    reg [8:0]  r_xgrp;              // 0 .. 319
    reg [9:0]  r_y;                 // 0 .. 719

    wire S_tick = I_tvalid && I_en;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            r_xgrp <= 9'd0;
            r_y    <= 10'd0;
        end
        else if(S_tick) begin
            if(I_tuser) begin
                r_xgrp <= 9'd0;
                r_y    <= 10'd0;
            end
            else if(r_xgrp == IMG_WIDTH_GRP[8:0] - 9'd1) begin
                r_xgrp <= 9'd0;
                if(r_y == IMG_HEIGHT[9:0] - 10'd1) r_y <= 10'd0;
                else                               r_y <= r_y + 10'd1;
            end
            else begin
                r_xgrp <= r_xgrp + 9'd1;
            end
        end
    end

    // last beat of the frame (see obj_detect : the counters are updated AT the
    // beat, so IMG_WIDTH_GRP-1 is never visible)
    wire S_frame_end = S_tick && (r_xgrp == IMG_WIDTH_GRP[8:0] - 9'd2) &&
                                 (r_y    == IMG_HEIGHT[9:0]   - 10'd1);

    //-------------------------------------------------------------------------
    // down sampling : one sample out of every DS_X x DS_Y block
    //      4 pixel per beat  ->  x group index   r_xgrp[1:0] == 0  (every 16 px)
    //      one line per beat ->  line index      r_y[3:0]    == 0  (every 16 th)
    //-------------------------------------------------------------------------
    wire [6:0]  S_gx   = r_xgrp[8:2];               // 0 .. 79
    wire [5:0]  S_gy   = r_y[9:4];                  // 0 .. 44
    wire [11:0] S_addr = ({S_gy, 6'd0} + {S_gy, 4'd0}) + {5'd0, S_gx};

    wire S_samp = S_tick && (r_xgrp[1:0] == 2'd0) && (r_y[3:0] == 4'd0);

    // luminance of pixel 0 : Y = (R>>2) + (G>>1) + (B>>2)   (max 253, 0 DSP)
    wire [7:0] S_y = {2'd0, I_tdata[95:90]} + {1'd0, I_tdata[87:81]}
                   + {2'd0, I_tdata[79:74]};

    // motion threshold : reuse the KEY2 threshold (no extra control needed)
    wire [7:0] S_tm = {3'd0, I_th[7:4]};            // 0 .. 18

    //-------------------------------------------------------------------------
    // pipeline stage 1 : remember the sample, the reference frames are read
    //-------------------------------------------------------------------------
    reg        r_s1;
    reg [7:0]  r_ycur;
    reg [11:0] r_addr;
    reg [6:0]  r_gx;
    reg [5:0]  r_gy;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            r_s1   <= 1'b0;
            r_ycur <= 8'd0;
            r_addr <= 12'd0;
            r_gx   <= 7'd0;
            r_gy   <= 6'd0;
        end
        else if(S_tick) begin
            r_s1   <= S_samp;
            r_ycur <= S_y;
            r_addr <= S_addr;
            r_gx   <= S_gx;
            r_gy   <= S_gy;
        end
    end

    //-------------------------------------------------------------------------
    // reference frames (block RAM, CSI clock)
    //-------------------------------------------------------------------------
    wire [7:0] S_refa;
    wire [7:0] S_refb;

    reg        r_we;
    reg [11:0] r_waddr;
    reg [7:0]  r_wdata_a;
    reg [7:0]  r_wdata_b;

    motion_eram #(
        .DATA_W ( 8    ),
        .ADDR_W ( CELL_ADW ),
        .DEPTH  ( 4096 )
    ) u_ref_a (
        .I_wclk  ( I_clk    ),
        .I_we    ( r_we     ),
        .I_waddr ( r_waddr  ),
        .I_wdata ( r_wdata_a),
        .I_rclk  ( I_clk    ),
        .I_raddr ( S_addr   ),
        .O_rdata ( S_refa   )
    );

    motion_eram #(
        .DATA_W ( 8    ),
        .ADDR_W ( CELL_ADW ),
        .DEPTH  ( 4096 )
    ) u_ref_b (
        .I_wclk  ( I_clk    ),
        .I_we    ( r_we     ),
        .I_waddr ( r_waddr  ),
        .I_wdata ( r_wdata_b),
        .I_rclk  ( I_clk    ),
        .I_raddr ( S_addr   ),
        .O_rdata ( S_refb   )
    );

    //-------------------------------------------------------------------------
    // pipeline stage 2 : three frame difference + statistics
    //-------------------------------------------------------------------------
    wire [7:0] S_d1 = (r_ycur > S_refa) ? (r_ycur - S_refa) : (S_refa - r_ycur);
    wire [7:0] S_d2 = (S_refa > S_refb) ? (S_refa - S_refb) : (S_refb - S_refa);
    wire       S_mot_raw = (S_d1 > S_tm) && (S_d2 > S_tm);

    // the first two frames have no valid reference yet -> suppress them
    reg [1:0] r_warm;
    wire      S_mot = S_mot_raw && (r_warm == 2'd3);

    // cell rectangle in pixel coordinates
    wire [10:0] S_cx0 = {r_gx, 4'd0};
    wire [10:0] S_cx1 = {r_gx, 4'd0} + 11'd15;
    wire [9:0]  S_cy0 = {r_gy, 4'd0};
    wire [9:0]  S_cy1 = {r_gy, 4'd0} + 10'd15;

    reg [11:0] r_mcnt;
    reg [10:0] r_bx0, r_bx1;
    reg [9:0]  r_by0, r_by1;
    reg        r_bbv;

    reg [1:0] r_page_bin;

    // binary page -> gray code (0 -> 00, 1 -> 01, 2 -> 11) : only one bit
    // changes, so a two stage synchroniser in the HDMI domain is safe
    function [1:0] gray3;
        input [1:0] b;
        begin
            case(b)
                2'd0: gray3 = 2'b00;
                2'd1: gray3 = 2'b01;
                default: gray3 = 2'b11;
            endcase
        end
    endfunction

    integer i;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            r_we        <= 1'b0;
            r_waddr     <= 12'd0;
            r_wdata_a   <= 8'd0;
            r_wdata_b   <= 8'd0;
            r_mcnt      <= 12'd0;
            r_bx0       <= 11'd0;
            r_bx1       <= 11'd0;
            r_by0       <= 10'd0;
            r_by1       <= 10'd0;
            r_bbv       <= 1'b0;
            r_warm      <= 2'd0;
            r_page_bin  <= 2'd0;
            O_flag_we   <= 1'b0;
            O_flag_addr <= 14'd0;
            O_flag_data <= 8'd0;
            O_wr_page_g <= 2'b00;
            O_mot_cnt   <= 12'd0;
            O_bb_xmin   <= 11'd0;
            O_bb_xmax   <= 11'd0;
            O_bb_ymin   <= 10'd0;
            O_bb_ymax   <= 10'd0;
            O_bb_valid  <= 1'b0;
            O_mot_alarm <= 1'b0;
            O_frame_tick<= 1'b0;
        end
        else begin
            O_frame_tick <= 1'b0;
            O_flag_we    <= 1'b0;

            if(S_tick) begin
                // ---- shift the reference frames : refB <= refA, refA <= cur
                r_we      <= r_s1;
                r_waddr   <= r_addr;
                r_wdata_a <= r_ycur;
                r_wdata_b <= S_refa;

                // ---- write the motion flag of this cell into the page
                if(r_s1) begin
                    O_flag_we   <= 1'b1;
                    O_flag_addr <= {r_page_bin, r_addr};
                    O_flag_data <= {7'd0, S_mot};
                end

                // ---- statistics of the current frame
                if(S_frame_end) begin
                    O_mot_cnt    <= r_mcnt;
                    O_bb_xmin    <= r_bx0;
                    O_bb_xmax    <= r_bx1;
                    O_bb_ymin    <= r_by0;
                    O_bb_ymax    <= r_by1;
                    O_bb_valid   <= r_bbv;
                    O_mot_alarm  <= (r_mcnt > MOT_TH[11:0]);
                    O_frame_tick <= 1'b1;

                    r_mcnt <= 12'd0;
                    r_bbv  <= 1'b0;
                    r_bx0  <= 11'd0;
                    r_bx1  <= 11'd0;
                    r_by0  <= 10'd0;
                    r_by1  <= 10'd0;

                    if(r_warm != 2'd3) r_warm <= r_warm + 1'b1;
                    r_page_bin <= (r_page_bin == 2'd2) ? 2'd0 : r_page_bin + 1'b1;
                    O_wr_page_g <= gray3((r_page_bin == 2'd2) ? 2'd0
                                                              : r_page_bin + 1'b1);
                end
                else if(I_tuser) begin
                    r_mcnt <= 12'd0;
                    r_bbv  <= 1'b0;
                    r_bx0  <= 11'd0;
                    r_bx1  <= 11'd0;
                    r_by0  <= 10'd0;
                    r_by1  <= 10'd0;
                end
                else if(r_s1 && S_mot) begin
                    r_mcnt <= r_mcnt + 12'd1;
                    if(!r_bbv) begin
                        r_bbv <= 1'b1;
                        r_bx0 <= S_cx0;  r_bx1 <= S_cx1;
                        r_by0 <= S_cy0;  r_by1 <= S_cy1;
                    end
                    else begin
                        if(S_cx0 < r_bx0) r_bx0 <= S_cx0;
                        if(S_cx1 > r_bx1) r_bx1 <= S_cx1;
                        if(S_cy0 < r_by0) r_by0 <= S_cy0;
                        if(S_cy1 > r_by1) r_by1 <= S_cy1;
                    end
                end
            end
        end
    end

endmodule
