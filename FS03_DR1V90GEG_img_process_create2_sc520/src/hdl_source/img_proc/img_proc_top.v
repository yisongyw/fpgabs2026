//-----------------------------------------------------------------------------
// Module     : img_proc_top
// Project    : HX1P35Aimg_process_extend1
// Description: Real-time image processing pipeline for the PH1P35 competition
//              design (topic 2).
//
//   Interface : AXI-Stream like, 96 bit = 4 pixels of RGB888 per beat.
//   Clock     : S_csi_rx_clk (MIPI byte clock domain, single clock domain).
//
//   -------------------------------------------------------------------------
//   LAYER 1  -  base part (identical to HX1P35Aimg_process_base2)
//   -------------------------------------------------------------------------
//   The three base algorithm blocks are switched INDEPENDENTLY by three slide
//   switches (HX1P35A SW1 / SW2 / SW3):
//
//       I_en_gray  (SW2) : Y = R/4 + G/2 + B/4, replicated to R,G,B
//       I_en_sobel (SW1) : Sobel edge, white edge on black background
//       I_en_bin   (SW3) : global threshold binarization, white / black
//
//   Priority:  SOBEL > BIN > GRAY > (bypass RGB)
//
//   -------------------------------------------------------------------------
//   LAYER 2  -  extension part : stylization / visual enhancement (KEY3)
//   -------------------------------------------------------------------------
//   I_style is a 4 bit mutually exclusive selector, 16 positions are reserved.
//   Implemented positions:
//
//       0  OFF   : layer 2 disabled, the base layer output is used unchanged
//       1  EDGE  : edge extraction  - Sobel outline drawn in COLOR_EDGE
//       2  FILL  : region filling   - the brightness region is filled with a
//                                     semi transparent COLOR_FILL block
//       3  BOTH  : edge extraction + region filling (outline on top of fill)
//       4 .. 15  : reserved, behave like OFF (OSD shows "RSVnn")
//
//   Region fill algorithm (scan line / run length fill, see PH1P35实时图像
//   处理扩展详细方案.md 4.11 "游程段填充"):
//
//       mask(x,y) = ( Y(x,y) >= I_th )
//       For every row the mask is scanned left to right.  A "span" opens at
//       the first mask pixel and stays open while the gaps between the mask
//       pixels are shorter than FILL_GAP_MAX pixels, so small holes inside a
//       region are filled as well.  Everything inside a span is painted with
//       COLOR_FILL blended 50/50 with the layer 1 output.
//
//       -> pure streaming, 4 pixels per beat, no extra frame buffer, no ERAM
//       -> the fill can never leak further than FILL_GAP_MAX pixels beyond
//          the mask, so no "whole screen painted" artifacts
//
//   Switching stability
//       I_style is only copied into the working register at the frame start
//       (I_tuser), so a style change can never produce a half / half frame.
//
//   3x3 window implementation
//   -------------------------
//   Full 1280 column resolution, NO horizontal decimation.
//   Four grayscale samples are packed into one 32 bit word, therefore each
//   line only needs IMG_WIDTH/4 = 320 words:
//
//       reg [31:0] line1_mem [0:319];   // row N-1
//       reg [31:0] line2_mem [0:319];   // row N-2
//
//   Total 2 x 320 x 32 = 20480 bit, inferred as distributed RAM / ERAM.
//
//   Timing (output is delayed by exactly one beat = 4 pixels):
//
//       beat k-2 -> *_p2    : 1 sample at column 4w-5
//       beat k-1 -> *_w_d   : 4 samples at columns 4w-4 .. 4w-1
//       beat k   -> nx_*    : 1 sample at column 4w (combinational read)
//
//       WX = {p2, w_d, nx}  covers columns 4w-5 .. 4w  (6 samples, 48 bit)
//       output pixel j has its window centre at column 4w-4+j, i.e. WX[j+1]
//       left = WX[j],  centre = WX[j+1],  right = WX[j+2]
//
//   NOTE: the line memories are intentionally NOT cleared by reset.  A reset
//   loop forces the synthesizer to implement them as slice registers instead
//   of RAM, which explodes the DFF usage.  The first two rows and the first
//   beat of every row are masked by window_valid instead.
//-----------------------------------------------------------------------------
module img_proc_top #(
    parameter IMG_WIDTH     = 1280,
    parameter IMG_HEIGHT    = 720,
    parameter FILL_GAP_MAX  = 5'd16       // max hole width that is filled
)(
    input  wire        I_clk,
    input  wire        I_rst_n,

    input  wire        I_en_sobel,  // SW1 : Sobel edge detection
    input  wire        I_en_gray,   // SW2 : grey conversion
    input  wire        I_en_bin,    // SW3 : global threshold binarization
    input  wire [8:0]  I_th,        // threshold 0 .. 300 (KEY2, 16 steps)

    input  wire [3:0]  I_style,     // KEY3 : stylization / visual enhancement

    input  wire        I_tlast,
    input  wire        I_tuser,
    input  wire [95:0] I_tdata,
    input  wire        I_tvalid,
    input  wire        I_tready,

    output reg         O_tlast,
    output reg         O_tuser,
    output reg  [95:0] O_tdata,
    output reg         O_tvalid,
    output wire        O_tready
);

    //-------------------------------------------------------------------------
    // Style / colour constants
    //-------------------------------------------------------------------------
    localparam [3:0] STYLE_OFF  = 4'd0;
    localparam [3:0] STYLE_EDGE = 4'd1;
    localparam [3:0] STYLE_FILL = 4'd2;
    localparam [3:0] STYLE_BOTH = 4'd3;

    localparam [23:0] COLOR_EDGE = 24'h00ff00;   // bright green outline
    localparam [23:0] COLOR_FILL = 24'hff8000;   // orange region fill

    assign O_tready = I_tready;

    //-------------------------------------------------------------------------
    // Input unpack : 96 bit -> 4 x RGB888 (pix0 is the left most pixel)
    //-------------------------------------------------------------------------
    wire [23:0] pix0 = I_tdata[95:72];
    wire [23:0] pix1 = I_tdata[71:48];
    wire [23:0] pix2 = I_tdata[47:24];
    wire [23:0] pix3 = I_tdata[23:0];

    // Low cost luminance: Y = R/4 + G/2 + B/4  (max 253, no overflow)
    wire [7:0] gray0 = {2'b0,pix0[23:18]} + {1'b0,pix0[15:9]} + {2'b0,pix0[7:2]};
    wire [7:0] gray1 = {2'b0,pix1[23:18]} + {1'b0,pix1[15:9]} + {2'b0,pix1[7:2]};
    wire [7:0] gray2 = {2'b0,pix2[23:18]} + {1'b0,pix2[15:9]} + {2'b0,pix2[7:2]};
    wire [7:0] gray3 = {2'b0,pix3[23:18]} + {1'b0,pix3[15:9]} + {2'b0,pix3[7:2]};

    localparam IMG_WIDTH_GRP = IMG_WIDTH >> 2;          // 320 words / line

    //-------------------------------------------------------------------------
    // Two line buffers (4 grayscale samples per word)
    //-------------------------------------------------------------------------
    reg [31:0] line1_mem [0:IMG_WIDTH_GRP-1] /* fehdl force_ram=1, ram_style="bram" */;
    reg [31:0] line2_mem [0:IMG_WIDTH_GRP-1] /* fehdl force_ram=1, ram_style="bram" */;

    reg [8:0]  x_grp;                                   // 0 .. 319
    reg [9:0]  y_pos;                                   // 0 .. 719

    wire [31:0] bot_word = {gray0, gray1, gray2, gray3};// current row (row N)

    reg  [31:0] top_word_r, mid_word_r;                 // BRAM sync read regs
    wire [31:0] top_word = top_word_r;                  // row N-2, cols 4w..4w+3
    wire [31:0] mid_word = mid_word_r;                  // row N-1, cols 4w..4w+3

    // Pre-fetch address = next x_grp (one beat ahead of the write address, so
    // the BRAM read never collides with the write and returns the old word).
    wire [8:0] S_x_grp_nxt = !I_tvalid ? x_grp :
                             I_tuser   ? 9'd0 :
                             (x_grp == IMG_WIDTH_GRP-1) ? 9'd0 : (x_grp + 9'd1);

    // BRAM synchronous read (unconditional, one pre-fetch beat ahead).
    // S_x_grp_nxt@N-1 == x_grp@N in every case, so this equals the old async read.
    always @(posedge I_clk) begin
        top_word_r <= line2_mem[S_x_grp_nxt];
        mid_word_r <= line1_mem[S_x_grp_nxt];
    end

    // BRAM write port in its own purely synchronous block (no async reset) so the
    // arrays cleanly infer block RAM.  Write address is the CURRENT x_grp, one
    // behind the pre-fetch read address -> never collides with the read.
    always @(posedge I_clk) begin
        if(I_tvalid) begin
            line2_mem[x_grp] <= mid_word;   // shift row N-1 -> row N-2
            line1_mem[x_grp] <= bot_word;   // shift row N   -> row N-1
        end
    end

    // One beat delayed words (columns 4w-4 .. 4w-1) and two beat delayed
    // last sample (column 4w-5)
    reg [31:0] top_w_d, mid_w_d, bot_w_d;
    reg [7:0]  top_p2,  mid_p2,  bot_p2;
    reg [95:0] pix_d;                                   // delayed original RGB
    reg [95:0] pix_d_r;                                 // 1 extra beat : part of the
                                                        // I_tdata -> Sobel -> O_tdata
                                                        // setup pipeline (MIPI path)
    reg        window_valid_r;                          // 1 extra beat : same pipeline
    reg        O_tvalid_d, O_tuser_d, O_tlast_d;        // 1-beat output handshake shift
                                                        // to keep "data lags valid by 1
                                                        // clock" exactly as before

    wire [7:0] nx_top = top_word[31:24];                // column 4w
    wire [7:0] nx_mid = mid_word[31:24];
    wire [7:0] nx_bot = bot_word[31:24];

    wire [47:0] WX_top = {top_p2, top_w_d, nx_top};     // cols 4w-5 .. 4w
    wire [47:0] WX_mid = {mid_p2, mid_w_d, nx_mid};
    wire [47:0] WX_bot = {bot_p2, bot_w_d, nx_bot};

    // The 3x3 window is only meaningful from the 3rd row on, and not on the
    // first beat of a row (where the delayed words still belong to the
    // previous line).
    wire window_valid = (y_pos >= 10'd2) && (x_grp != 9'd0);

    wire [13:0] th14 = {5'd0, I_th};

    //-------------------------------------------------------------------------
    // Style working register - updated at the frame start only, so that a
    // KEY3 switch can never split a frame into two different styles.
    //-------------------------------------------------------------------------
    reg [3:0] r_style;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            r_style <= STYLE_OFF;
        else if(I_tvalid && I_tuser)
            r_style <= I_style;
    end

    //-------------------------------------------------------------------------
    // 50 / 50 blend of a pixel with the region fill colour (no DSP, no overflow)
    //-------------------------------------------------------------------------
    function [7:0] blend_ch;
        input [7:0] v;
        input [7:0] c;
        begin
            blend_ch = {1'b0, v[7:1]} + {1'b0, c[7:1]};   // (v + c) / 2
        end
    endfunction

    //-------------------------------------------------------------------------
    // Run length (scan line) region fill : one step for one pixel
    //   in  : mask   - the pixel belongs to the mask
    //         open   - a span is already open
    //         gap    - number of mask==0 pixels seen since the last mask pixel
    //   out : {fill, next_open, next_gap}
    //-------------------------------------------------------------------------
    function [6:0] span_step;
        input       mask;
        input       open;
        input [4:0] gap;
        input [4:0] gap_max;
        reg         n_open;
        reg  [4:0]  n_gap;
        reg         fill;
        begin
            if(mask) begin
                fill   = 1'b1;          // inside the region
                n_open = 1'b1;
                n_gap  = 5'd0;
            end
            else if(open) begin
                if(gap < gap_max) begin
                    fill   = 1'b1;      // hole narrower than gap_max -> filled
                    n_open = 1'b1;
                    n_gap  = gap + 5'd1;
                end
                else begin
                    fill   = 1'b0;      // gap too wide -> span closed
                    n_open = 1'b0;
                    n_gap  = 5'd0;
                end
            end
            else begin
                fill   = 1'b0;
                n_open = 1'b0;
                n_gap  = 5'd0;
            end
            span_step = {fill, n_open, n_gap};
        end
    endfunction

    reg        r_span_open;
    reg [4:0]  r_gap;

    wire [3:0] S_mask;      // brightness mask of the 4 output pixels, [0] = left

    // four unrolled steps -> the fill flags of the current beat
    wire [6:0] S_st0 = span_step(S_mask[0], r_span_open, r_gap,       FILL_GAP_MAX);
    wire [6:0] S_st1 = span_step(S_mask[1], S_st0[5],    S_st0[4:0],  FILL_GAP_MAX);
    wire [6:0] S_st2 = span_step(S_mask[2], S_st1[5],    S_st1[4:0],  FILL_GAP_MAX);
    wire [6:0] S_st3 = span_step(S_mask[3], S_st2[5],    S_st2[4:0],  FILL_GAP_MAX);

    wire [3:0] S_fill     = {S_st3[6], S_st2[6], S_st1[6], S_st0[6]};
    wire       S_span_end = S_st3[5];
    wire [4:0] S_gap_end  = S_st3[4:0];

    //-------------------------------------------------------------------------
    // Four parallel pixel datapaths
    //-------------------------------------------------------------------------
    wire [95:0] S_out_px;

    genvar j;
    generate
        for (j = 0; j < 4; j = j + 1) begin : g_px

            wire [7:0] t_l = WX_top[47 - 8*(j  ) -: 8];
            wire [7:0] t_m = WX_top[47 - 8*(j+1) -: 8];
            wire [7:0] t_r = WX_top[47 - 8*(j+2) -: 8];

            wire [7:0] m_l = WX_mid[47 - 8*(j  ) -: 8];
            wire [7:0] m_m = WX_mid[47 - 8*(j+1) -: 8];
            wire [7:0] m_r = WX_mid[47 - 8*(j+2) -: 8];

            wire [7:0] b_l = WX_bot[47 - 8*(j  ) -: 8];
            wire [7:0] b_m = WX_bot[47 - 8*(j+1) -: 8];
            wire [7:0] b_r = WX_bot[47 - 8*(j+2) -: 8];

            wire [13:0] mag;
            reg  [13:0] mag_r;       // 1-stage pipeline on the Sobel output : the
                                    // clean cut point that splits the long
                                    // I_tdata -> Sobel -> O_tdata setup chain (the
                                    // MIPI path) into two short halves.
            always @(posedge I_clk) begin
                if(I_tvalid)
                    mag_r <= mag;
            end
            sobel3x3 u_sobel3x3 (
                .a00 ( t_l ),
                .a01 ( t_m ),
                .a02 ( t_r ),
                .a10 ( m_l ),
                .a11 ( m_m ),
                .a12 ( m_r ),
                .a20 ( b_l ),
                .a21 ( b_m ),
                .a22 ( b_r ),
                .O_mag( mag )
            );

            wire [23:0] px_org  = pix_d_r[95 - 24*j -: 24];
            wire [7:0]  y_dir   = {2'b0,px_org[23:18]} + {1'b0,px_org[15:9]}
                                + {2'b0,px_org[7:2]};
            wire        bin_on  = (y_dir >= I_th);
            wire        edge_on = window_valid_r && (mag_r >= th14);

            // brightness mask of this pixel -> feeds the run length filler
            assign S_mask[j] = bin_on;

            //--------------------------------------------------------------
            // layer 1 : base algorithms (base2, unchanged)
            //--------------------------------------------------------------
            wire [23:0] px_base =
                I_en_sobel ? (edge_on ? 24'hffffff : 24'h000000) :
                I_en_bin   ? (bin_on  ? 24'hffffff : 24'h000000) :
                I_en_gray  ? {y_dir, y_dir, y_dir}               :
                             px_org;

            //--------------------------------------------------------------
            // layer 2 : stylization / visual enhancement (KEY3)
            //--------------------------------------------------------------
            wire        fill_hit = S_fill[j];

            wire [23:0] px_fill  = { blend_ch(px_base[23:16], COLOR_FILL[23:16]),
                                     blend_ch(px_base[15: 8], COLOR_FILL[15: 8]),
                                     blend_ch(px_base[ 7: 0], COLOR_FILL[ 7: 0]) };

            wire [23:0] out_px  =
                (r_style == STYLE_EDGE) ? (edge_on ? COLOR_EDGE : px_base) :
                (r_style == STYLE_FILL) ? (fill_hit ? px_fill   : px_base) :
                (r_style == STYLE_BOTH) ? (edge_on  ? COLOR_EDGE :
                                           (fill_hit ? px_fill  : px_base)) :
                                          px_base;          // OFF + reserved

            assign S_out_px[95 - 24*j -: 24] = out_px;
        end
    endgenerate

    //-------------------------------------------------------------------------
    // Sequential part
    //-------------------------------------------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_tlast     <= 1'b0;
            O_tuser     <= 1'b0;
            O_tdata     <= 96'd0;
            O_tvalid    <= 1'b0;
            pix_d_r     <= 96'd0;
            window_valid_r <= 1'b0;
            O_tvalid_d  <= 1'b0;
            O_tuser_d   <= 1'b0;
            O_tlast_d   <= 1'b0;
            x_grp       <= 9'd0;
            y_pos       <= 10'd0;
            top_w_d     <= 32'd0;
            mid_w_d     <= 32'd0;
            bot_w_d     <= 32'd0;
            top_p2      <= 8'd0;
            mid_p2      <= 8'd0;
            bot_p2      <= 8'd0;
            pix_d       <= 96'd0;
            r_span_open <= 1'b0;
            r_gap       <= 5'd0;
        end
        else begin
            // 1-beat handshake shift : the internal Sobel / pixel pipeline is now
            // one beat deeper, so the valid is delayed by the same single beat to
            // keep "data lags valid by 1 clock" exactly as before.  The downstream
            // OSD relies on this fixed relationship.
            O_tlast  <= O_tlast_d;
            O_tuser  <= O_tuser_d;
            O_tvalid <= O_tvalid_d;
            O_tlast_d  <= I_tlast;
            O_tuser_d  <= I_tuser;
            O_tvalid_d <= I_tvalid;

            if(I_tvalid) begin
                O_tdata  <= S_out_px;
                pix_d    <= I_tdata;
                pix_d_r  <= pix_d;             // 1 extra beat (setup pipeline)
                window_valid_r <= window_valid; // 1 extra beat (setup pipeline)

                top_w_d  <= top_word;
                mid_w_d  <= mid_word;
                bot_w_d  <= bot_word;
                top_p2   <= top_w_d[7:0];
                mid_p2   <= mid_w_d[7:0];
                bot_p2   <= bot_w_d[7:0];

                // The pipeline output lags the input by one beat, therefore the
                // beat with x_grp == IMG_WIDTH_GRP-1 (the LAST group of a row)
                // is the one that emits the last four pixels of that row.  The
                // span has to be closed right after that beat, so that the
                // first group of the next row (emitted by the beat with
                // x_grp == 0) starts with a clean state.
                if(I_tuser || (x_grp == IMG_WIDTH_GRP - 1)) begin
                    r_span_open <= 1'b0;
                    r_gap       <= 5'd0;
                end
                else begin
                    r_span_open <= S_span_end;
                    r_gap       <= S_gap_end;
                end

                if(I_tuser) begin
                    x_grp <= 9'd0;
                    y_pos <= 10'd0;
                end
                else if(x_grp == IMG_WIDTH_GRP - 1) begin
                    x_grp <= 9'd0;
                    if(y_pos == IMG_HEIGHT - 1)
                        y_pos <= 10'd0;
                    else
                        y_pos <= y_pos + 10'd1;
                end
                else begin
                    x_grp <= x_grp + 9'd1;
                end
            end
            else begin
                O_tdata <= 96'd0;
            end
        end
    end

endmodule
