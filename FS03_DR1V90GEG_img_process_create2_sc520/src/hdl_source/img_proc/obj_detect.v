//-----------------------------------------------------------------------------
// Module     : obj_detect
// Project    : HX1P35Aimg_process_extend2
// Description: Streaming target detector (competition extension requirement 2,
//              see PH1P35实时图像处理扩展详细方案.md chapter 5 / chapter 5A).
//
//   The module is a pure SIDE PATH : it taps the same RGB stream that feeds
//   img_proc_top, it never modifies a pixel.  Therefore the extend1 base layer
//   (SOBEL / GRAY / BIN) and the style layer (KEY3) behave exactly as before.
//
//   Pipeline (4 pixels per beat, no horizontal decimation, 0 DSP)
//
//     RGB888 -> RGB to YCbCr (BT.601 shift-add approximation)
//            -> chroma cluster classifier      -> 1 bit mask + skin flag
//            -> 3x3 erosion   (mask3x3)
//            -> 3x3 dilation  (mask3x3)        -> opened mask, noise removed
//            -> 4 x 3 = 12 tile grid projection
//            -> per frame : serial readout of the 12 tile records
//
//   Why YCbCr instead of HSV / RGB thresholds
//   -----------------------------------------
//   * Y is separated from the chroma, so a change of the illumination mainly
//     moves Y and leaves Cb / Cr nearly untouched -> the detector keeps working
//     when the room light changes (competition requirement: robust against
//     illumination changes).
//   * HSV would need max/min selection and a real division per pixel, its cost
//     is an order of magnitude higher.
//
//   Why the 3x3 opening (erode -> dilate)
//   -------------------------------------
//   Sensor noise produces isolated single pixels which would open a bounding
//   box in every tile.  One erosion deletes every pixel that is not part of a
//   3x3 solid block, the following dilation restores the original size of the
//   remaining blobs.  Cost : 2 x 2.56 kbit line memory, no more line buffers.
//
//   Why 12 independent tiles (4 columns x 3 rows)
//   --------------------------------------------
//   A single full frame projection can only return ONE bounding box.  12 tiles
//   give up to 12 candidates per frame for the price of 12 accumulator sets
//   (about 1.8 kbit of registers in total) and no extra data path.
//   A side effect that is very welcome during the demonstration : two objects
//   with DIFFERENT colours that touch each other inside one tile are still not
//   merged, because each tile carries the colour of the pixels it collected.
//
//   Output : one tile record per clock at the frame end
//     O_cnt    number of mask pixels                (area)
//     O_xmin / O_xmax / O_ymin / O_ymax  bounding box
//     O_peri   number of boundary pixels            (perimeter, -> roundness)
//     O_sum_cb / O_sum_cr   chroma sums             (-> mean colour)
//     O_skin   number of skin coloured pixels       (-> "MAN" class)
//     O_ylo / O_yhi   luminance range inside the blob (-> texture)
//-----------------------------------------------------------------------------
module obj_detect #(
    parameter IMG_WIDTH  = 1280,
    parameter IMG_HEIGHT = 720
)(
    input  wire        I_clk,
    input  wire        I_rst_n,

    input  wire        I_en,            // 1 : detector running
    input  wire        I_tvalid,
    input  wire        I_tuser,         // start of frame
    input  wire [95:0] I_tdata,         // 4 x RGB888, pixel 0 left most

    input  wire        I_tile_next,     // consumer took the record -> advance

    output reg         O_tile_valid,    // tile record available on this clock
    output reg         O_tile_last,     // last tile of the frame
    output wire [3:0]  O_tile_id,       // 0 .. 11
    output wire [16:0] O_cnt,
    output wire [10:0] O_xmin,
    output wire [10:0] O_xmax,
    output wire [9:0]  O_ymin,
    output wire [9:0]  O_ymax,
    output wire [16:0] O_peri,
    output wire [24:0] O_sum_cb,
    output wire [24:0] O_sum_cr,
    output wire [16:0] O_skin,
    output wire [7:0]  O_ylo,
    output wire [7:0]  O_yhi
);

    localparam IMG_WIDTH_GRP = IMG_WIDTH >> 2;      // 320 beats per line
    localparam TILES         = 12;
    // improve1 note : a parameterised 4 x 2 tile grid (thresholds derived
    //   from IMG_WIDTH / IMG_HEIGHT) was tried here to save accumulator
    //   logic, but it had to be REVERTED.  extend3 uses FIXED thresholds
    //   (320/640/960, 240/480) designed for the 1280 x 720 stream; with
    //   size-relative thresholds a small frame or a small object is split
    //   across tiles and obj_detect reports only the strongest tile region
    //   (tb_obj_detect : cnt 160..192 instead of 384).  Requirement 1 says
    //   the extend3 behaviour must stay unchanged, so the original fixed
    //   12 tile grid stays.

    //-------------------------------------------------------------------------
    // RGB -> YCbCr (BT.601 integer approximation, shift and add only, 0 DSP)
    //   Y  = ( 77R + 150G +  29B) >> 8
    //   Cb = (-43R -  85G + 128B) >> 8 + 128
    //   Cr = (128R - 107G -  21B) >> 8 + 128
    //-------------------------------------------------------------------------
    function [23:0] rgb2ycc;
        input [23:0] rgb;
        reg  [7:0]  r, g, b;
        reg  [16:0] yy, cbv, crv;
        begin
            r = rgb[23:16];
            g = rgb[15:8];
            b = rgb[7:0];

            // 77 = 64 + 8 + 4 + 1    150 = 128 + 16 + 4 + 2   29 = 16 + 8 + 4 + 1
            yy  = ({r,6'b0} + {r,3'b0} + {r,2'b0} + r)
                + ({g,7'b0} + {g,4'b0} + {g,3'b0} + {g,1'b0})
                + ({b,4'b0} + {b,3'b0} + {b,2'b0} + b);

            // 43 = 32 + 8 + 2 + 1    85 = 64 + 16 + 4 + 1     128
            cbv = ({b,7'b0})
                - ({r,5'b0} + {r,3'b0} + {r,1'b0} + r)
                - ({g,6'b0} + {g,4'b0} + {g,2'b0} + g);

            // 128                    107 = 64 + 32 + 8 + 2 + 1  21 = 16 + 4 + 1
            crv = ({r,7'b0})
                - ({g,6'b0} + {g,5'b0} + {g,3'b0} + {g,1'b0} + g)
                - ({b,4'b0} + {b,2'b0} + b);

            rgb2ycc = { yy[15:8], (cbv[15:8] + 8'd128), (crv[15:8] + 8'd128) };
        end
    endfunction

    //-------------------------------------------------------------------------
    // chroma cluster classifier
    //   0 = background         -> no target pixel
    //   1 = skin  (MAN)        -> typical face / hand cluster
    //   2 = red   object
    //   3 = other saturated colour (orange / yellow / blue / green)
    //-------------------------------------------------------------------------
    function [1:0] color_class;
        input [7:0] y;
        input [7:0] cb;
        input [7:0] cr;
        begin
            if((cb >= 8'd77) && (cb <= 8'd135) &&
               (cr >= 8'd133) && (cr <= 8'd178) && (y >= 8'd40))
                color_class = 2'd1;                             // skin
            else if((cr >= 8'd185) && (cb >= 8'd60) && (cb <= 8'd125) &&
                    (y >= 8'd30))
                color_class = 2'd2;                             // red
            else if((cr >= 8'd150) && (cb <= 8'd70) && (y >= 8'd40))
                color_class = 2'd3;                             // orange / yellow
            else if((cb >= 8'd140) && (cr <= 8'd130) && (y >= 8'd30))
                color_class = 2'd3;                             // blue
            else if((cr <= 8'd115) && (cb <= 8'd125) && (y >= 8'd40) &&
                    (cr <= cb))
                color_class = 2'd3;                             // green family
            else
                color_class = 2'd0;                             // background
        end
    endfunction

    //-------------------------------------------------------------------------
    // minimum / maximum of the four hit pixels of a beat
    //   a missing pixel contributes 8'hff (minimum) or 8'h00 (maximum)
    //-------------------------------------------------------------------------
    function [7:0] ymin4;
        input [3:0] hit;
        input [7:0] v0, v1, v2, v3;
        reg   [7:0] m01, m23;
        begin
            if(hit[0] && hit[1]) m01 = (v0 < v1) ? v0 : v1;
            else if(hit[0])      m01 = v0;
            else if(hit[1])      m01 = v1;
            else                 m01 = 8'hff;

            if(hit[2] && hit[3]) m23 = (v2 < v3) ? v2 : v3;
            else if(hit[2])      m23 = v2;
            else if(hit[3])      m23 = v3;
            else                 m23 = 8'hff;

            ymin4 = (m01 < m23) ? m01 : m23;
        end
    endfunction

    function [7:0] ymax4;
        input [3:0] hit;
        input [7:0] v0, v1, v2, v3;
        reg   [7:0] m01, m23;
        begin
            if(hit[0] && hit[1]) m01 = (v0 > v1) ? v0 : v1;
            else if(hit[0])      m01 = v0;
            else if(hit[1])      m01 = v1;
            else                 m01 = 8'h00;

            if(hit[2] && hit[3]) m23 = (v2 > v3) ? v2 : v3;
            else if(hit[2])      m23 = v2;
            else if(hit[3])      m23 = v3;
            else                 m23 = 8'h00;

            ymax4 = (m01 > m23) ? m01 : m23;
        end
    endfunction

    //-------------------------------------------------------------------------
    // input stream position
    //-------------------------------------------------------------------------
    reg [8:0]  r_xgrp;              // 0 .. 319
    reg [9:0]  r_y;                 // 0 .. 719
    reg [95:0] r_pix_d1, r_pix_d2;
    reg [8:0]  r_x_d1,  r_x_d2;
    reg [9:0]  r_y_d1,  r_y_d2;

    // y delayed by exactly 2 lines (the window centre row, see S_win_ok / bbox).
    // r_y is constant inside a line, so a 2-deep line shift register is enough.
    reg [9:0]  r_y_sr0, r_y_sr1;
    wire S_line_start = S_tick &&
                        (I_tuser || (r_xgrp == (IMG_WIDTH_GRP[8:0] - 9'd1)));

    wire S_tick = I_tvalid && I_en;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            r_xgrp    <= 9'd0;
            r_y       <= 10'd0;
            r_pix_d1  <= 96'd0;
            r_pix_d2  <= 96'd0;
            r_x_d1    <= 9'd0;
            r_x_d2    <= 9'd0;
            r_y_d1    <= 10'd0;
            r_y_d2    <= 10'd0;
            r_y_sr0   <= 10'd0;
            r_y_sr1   <= 10'd0;
        end
        else if(S_tick) begin
            r_pix_d1 <= I_tdata;
            r_pix_d2 <= r_pix_d1;
            r_x_d1   <= r_xgrp;
            r_x_d2   <= r_x_d1;
            r_y_d1   <= r_y;
            r_y_d2   <= r_y_d1;

            if(S_line_start) begin
                r_y_sr1 <= r_y_sr0;
                r_y_sr0 <= r_y;
            end

            if(I_tuser) begin
                r_xgrp <= 9'd0;
                r_y    <= 10'd0;
            end
            else if(r_xgrp == IMG_WIDTH_GRP[8:0] - 9'd1) begin
                r_xgrp <= 9'd0;
                if(r_y == IMG_HEIGHT[9:0] - 10'd1)
                    r_y <= 10'd0;
                else
                    r_y <= r_y + 10'd1;
            end
            else begin
                r_xgrp <= r_xgrp + 9'd1;
            end
        end
    end

    // last beat of the frame (input side)
    //
    //   The position counters are updated AT the beat, therefore during the
    //   last beat of a line the value seen here is still IMG_WIDTH_GRP-2 :
    //   the value IMG_WIDTH_GRP-1 only shows up on the first beat of the
    //   following line (that is where the line wrap resets it to 0).
    //   Comparing against -1 would need one more beat after the last pixel,
    //   which never arrives -> the readout would never start.
    wire S_frame_end = S_tick && (r_xgrp == IMG_WIDTH_GRP[8:0] - 9'd2) &&
                                 (r_y    == IMG_HEIGHT[9:0] - 10'd1);

    //-------------------------------------------------------------------------
    // per pixel : Y / Cb / Cr / class / mask   (all combinational)
    //-------------------------------------------------------------------------
    wire [23:0] S_ycc0 = rgb2ycc(I_tdata[95:72]);
    wire [23:0] S_ycc1 = rgb2ycc(I_tdata[71:48]);
    wire [23:0] S_ycc2 = rgb2ycc(I_tdata[47:24]);
    wire [23:0] S_ycc3 = rgb2ycc(I_tdata[23: 0]);

    wire [1:0] S_cls0 = color_class(S_ycc0[23:16], S_ycc0[15:8], S_ycc0[7:0]);
    wire [1:0] S_cls1 = color_class(S_ycc1[23:16], S_ycc1[15:8], S_ycc1[7:0]);
    wire [1:0] S_cls2 = color_class(S_ycc2[23:16], S_ycc2[15:8], S_ycc2[7:0]);
    wire [1:0] S_cls3 = color_class(S_ycc3[23:16], S_ycc3[15:8], S_ycc3[7:0]);

    wire S_m0 = (S_cls0 != 2'd0);
    wire S_m1 = (S_cls1 != 2'd0);
    wire S_m2 = (S_cls2 != 2'd0);
    wire S_m3 = (S_cls3 != 2'd0);

    //-------------------------------------------------------------------------
    // morphological opening : erode -> dilate
    //-------------------------------------------------------------------------
    wire [8:0] S_win_e0, S_win_e1, S_win_e2, S_win_e3;

    mask3x3 #(
        .IMG_WIDTH_GRP ( IMG_WIDTH_GRP )
    ) u_mask_erode (
        .I_clk   ( I_clk                        ),
        .I_en    ( S_tick                       ),
        .I_xgrp  ( r_xgrp                       ),
        .I_mask4 ( {S_m0, S_m1, S_m2, S_m3}     ),
        .O_w0    ( S_win_e0                     ),
        .O_w1    ( S_win_e1                     ),
        .O_w2    ( S_win_e2                     ),
        .O_w3    ( S_win_e3                     )
    );

    wire S_e0 = &S_win_e0;
    wire S_e1 = &S_win_e1;
    wire S_e2 = &S_win_e2;
    wire S_e3 = &S_win_e3;

    wire [8:0] S_win_d0, S_win_d1, S_win_d2, S_win_d3;

    mask3x3 #(
        .IMG_WIDTH_GRP ( IMG_WIDTH_GRP )
    ) u_mask_dilate (
        .I_clk   ( I_clk                        ),
        .I_en    ( S_tick                       ),
        .I_xgrp  ( r_xgrp                       ),
        .I_mask4 ( {S_e0, S_e1, S_e2, S_e3}     ),
        .O_w0    ( S_win_d0                     ),
        .O_w1    ( S_win_d1                     ),
        .O_w2    ( S_win_d2                     ),
        .O_w3    ( S_win_d3                     )
    );

    wire S_o0 = |S_win_d0;
    wire S_o1 = |S_win_d1;
    wire S_o2 = |S_win_d2;
    wire S_o3 = |S_win_d3;

    // the raw neighbourhood has to travel one more beat so that it lines up
    // with the opened mask (used for the perimeter count)
    reg [8:0] r_rw0, r_rw1, r_rw2, r_rw3;
    always @(posedge I_clk) begin
        if(S_tick) begin
            r_rw0 <= S_win_e0;
            r_rw1 <= S_win_e1;
            r_rw2 <= S_win_e2;
            r_rw3 <= S_win_e3;
        end
    end

    //-------------------------------------------------------------------------
    // chroma / luminance of the DELAYED pixels (the ones the opened mask
    // belongs to).
    // extend3 performance optimisation : the YCbCr of the LIVE pixels is
    // delayed by two registers instead of being recomputed from r_pix_d2.
    // This removes 4 x rgb2ycc (shift/add) combinational datapaths per pixel
    // (lower LUT usage and lower dynamic power) while staying bit identical to
    // the previous recomputation (both evaluate the YCbCr of the pixel that is
    // exactly two beats old).
    //-------------------------------------------------------------------------
    reg [95:0] r_ycc_d1, r_ycc_d2;
    always @(posedge I_clk) begin
        if(S_tick) begin
            r_ycc_d1 <= {S_ycc0, S_ycc1, S_ycc2, S_ycc3};
            r_ycc_d2 <= r_ycc_d1;
        end
    end

    wire [23:0] S_dycc0 = r_ycc_d2[95:72];
    wire [23:0] S_dycc1 = r_ycc_d2[71:48];
    wire [23:0] S_dycc2 = r_ycc_d2[47:24];
    wire [23:0] S_dycc3 = r_ycc_d2[23: 0];

    wire [1:0] S_dcls0 = color_class(S_dycc0[23:16], S_dycc0[15:8], S_dycc0[7:0]);
    wire [1:0] S_dcls1 = color_class(S_dycc1[23:16], S_dycc1[15:8], S_dycc1[7:0]);
    wire [1:0] S_dcls2 = color_class(S_dycc2[23:16], S_dycc2[15:8], S_dycc2[7:0]);
    wire [1:0] S_dcls3 = color_class(S_dycc3[23:16], S_dycc3[15:8], S_dycc3[7:0]);

    //-------------------------------------------------------------------------
    // hit evaluation for the current output beat
    //-------------------------------------------------------------------------
    wire        S_win_ok = (r_y_sr1 >= 10'd2) && (r_x_d1 != 9'd0);
    wire [3:0]  S_hit     = { S_win_ok & S_o3, S_win_ok & S_o2,
                              S_win_ok & S_o1, S_win_ok & S_o0 };   // [0] = pix0
    wire        S_any     = |S_hit;

    wire [10:0] S_col_base = {r_x_d1, 2'b00};

    wire [1:0] S_first = S_hit[0] ? 2'd0 :
                         S_hit[1] ? 2'd1 :
                         S_hit[2] ? 2'd2 : 2'd3;
    wire [1:0] S_last  = S_hit[3] ? 2'd3 :
                         S_hit[2] ? 2'd2 :
                         S_hit[1] ? 2'd1 : 2'd0;

    wire [10:0] S_xmin_n = S_col_base + {9'd0, S_first};
    wire [10:0] S_xmax_n = S_col_base + {9'd0, S_last};

    wire [2:0] S_nhit = {2'd0, S_hit[0]} + {2'd0, S_hit[1]}
                      + {2'd0, S_hit[2]} + {2'd0, S_hit[3]};

    // boundary pixels inside this beat (a pixel belongs to the boundary when at
    // least one of its 8 neighbours is not part of the raw mask)
    wire S_p0 = S_hit[0] && (r_rw0[8:0] != 9'h1ff);
    wire S_p1 = S_hit[1] && (r_rw1[8:0] != 9'h1ff);
    wire S_p2 = S_hit[2] && (r_rw2[8:0] != 9'h1ff);
    wire S_p3 = S_hit[3] && (r_rw3[8:0] != 9'h1ff);

    wire [2:0] S_nperi = {2'd0, S_p0} + {2'd0, S_p1}
                       + {2'd0, S_p2} + {2'd0, S_p3};

    wire [1:0] S_skin0 = (S_hit[0] && (S_dcls0 == 2'd1)) ? 2'd1 : 2'd0;
    wire [1:0] S_skin1 = (S_hit[1] && (S_dcls1 == 2'd1)) ? 2'd1 : 2'd0;
    wire [1:0] S_skin2 = (S_hit[2] && (S_dcls2 == 2'd1)) ? 2'd1 : 2'd0;
    wire [1:0] S_skin3 = (S_hit[3] && (S_dcls3 == 2'd1)) ? 2'd1 : 2'd0;

    wire [2:0] S_nskin = {1'd0, S_skin0} + {1'd0, S_skin1}
                       + {1'd0, S_skin2} + {1'd0, S_skin3};

    wire [9:0] S_cb_sum = (S_hit[0] ? {2'd0, S_dycc0[15:8]} : 10'd0)
                        + (S_hit[1] ? {2'd0, S_dycc1[15:8]} : 10'd0)
                        + (S_hit[2] ? {2'd0, S_dycc2[15:8]} : 10'd0)
                        + (S_hit[3] ? {2'd0, S_dycc3[15:8]} : 10'd0);

    wire [9:0] S_cr_sum = (S_hit[0] ? {2'd0, S_dycc0[7:0]} : 10'd0)
                        + (S_hit[1] ? {2'd0, S_dycc1[7:0]} : 10'd0)
                        + (S_hit[2] ? {2'd0, S_dycc2[7:0]} : 10'd0)
                        + (S_hit[3] ? {2'd0, S_dycc3[7:0]} : 10'd0);

    // luminance range inside the blob (cheap texture measure)
    wire [7:0] S_y_of0 = S_dycc0[23:16];
    wire [7:0] S_y_of1 = S_dycc1[23:16];
    wire [7:0] S_y_of2 = S_dycc2[23:16];
    wire [7:0] S_y_of3 = S_dycc3[23:16];

    wire [7:0] S_y_min_beat = ymin4(S_hit, S_y_of0, S_y_of1, S_y_of2, S_y_of3);
    wire [7:0] S_y_max_beat = ymax4(S_hit, S_y_of0, S_y_of1, S_y_of2, S_y_of3);

    // tile index
    wire [1:0] S_tcol = (S_col_base >= 11'd960) ? 2'd3 :
                        (S_col_base >= 11'd640) ? 2'd2 :
                        (S_col_base >= 11'd320) ? 2'd1 : 2'd0;
    wire [1:0] S_trow = (r_y_sr1 >= 10'd480) ? 2'd2 :
                        (r_y_sr1 >= 10'd240) ? 2'd1 : 2'd0;
    wire [3:0] S_tile = {S_trow, S_tcol};

    //-------------------------------------------------------------------------
    // tile accumulators
    //-------------------------------------------------------------------------
    reg [16:0] t_cnt   [0:TILES-1];
    reg [10:0] t_xmin  [0:TILES-1];
    reg [10:0] t_xmax  [0:TILES-1];
    reg [9:0]  t_ymin  [0:TILES-1];
    reg [9:0]  t_ymax  [0:TILES-1];
    reg [16:0] t_peri  [0:TILES-1];
    reg [24:0] t_cb    [0:TILES-1];
    reg [24:0] t_cr    [0:TILES-1];
    reg [16:0] t_skin  [0:TILES-1];
    reg [7:0]  t_ylo   [0:TILES-1];
    reg [7:0]  t_yhi   [0:TILES-1];

    reg        r_read;
    reg [3:0]  r_rd_idx;

    integer k;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            for(k = 0; k < TILES; k = k + 1) begin
                t_cnt [k] <= 17'd0;
                t_xmin[k] <= 11'd0;
                t_xmax[k] <= 11'd0;
                t_ymin[k] <= 10'd0;
                t_ymax[k] <= 10'd0;
                t_peri[k] <= 17'd0;
                t_cb  [k] <= 25'd0;
                t_cr  [k] <= 25'd0;
                t_skin[k] <= 17'd0;
                t_ylo [k] <= 8'hff;
                t_yhi [k] <= 8'd0;
            end
            r_read    <= 1'b0;
            r_rd_idx  <= 4'd0;
            O_tile_valid <= 1'b0;
            O_tile_last  <= 1'b0;
        end
        else begin
            //--------------------------------------------------------------
            // tile accumulation
            //
            //   This is an INDEPENDENT if and NOT part of the readout / frame
            //   end chain below : the frame end branch must not swallow the
            //   accumulation of the very last beat of the frame.
            //--------------------------------------------------------------
            if(S_tick && !r_read && S_any) begin
                for(k = 0; k < TILES; k = k + 1) begin
                    if(k[3:0] == S_tile) begin
                        if(t_cnt[k] == 17'd0) begin
                            t_xmin[k] <= S_xmin_n;
                            t_xmax[k] <= S_xmax_n;
                            t_ymin[k] <= r_y_sr1;
                            t_ymax[k] <= r_y_sr1;
                        end
                        else begin
                            if(S_xmin_n < t_xmin[k]) t_xmin[k] <= S_xmin_n;
                            if(S_xmax_n > t_xmax[k]) t_xmax[k] <= S_xmax_n;
                            if(r_y_sr1   < t_ymin[k]) t_ymin[k] <= r_y_sr1;
                            if(r_y_sr1   > t_ymax[k]) t_ymax[k] <= r_y_sr1;
                        end
                        t_cnt [k] <= t_cnt[k]  + {14'd0, S_nhit};
                        t_peri[k] <= t_peri[k] + {14'd0, S_nperi};
                        t_skin[k] <= t_skin[k] + {14'd0, S_nskin};
                        t_cb  [k] <= t_cb[k]   + {15'd0, S_cb_sum};
                        t_cr  [k] <= t_cr[k]   + {15'd0, S_cr_sum};
                        if(S_y_min_beat < t_ylo[k]) t_ylo[k] <= S_y_min_beat;
                        if(S_y_max_beat > t_yhi[k]) t_yhi[k] <= S_y_max_beat;
                    end
                end
            end
            //--------------------------------------------------------------
            // frame end readout : one tile record per clock
            //--------------------------------------------------------------
            if(r_read && I_tile_next) begin
                // the record is handed over -> the tile is free again
                t_cnt [r_rd_idx] <= 17'd0;
                t_xmin[r_rd_idx] <= 11'd0;
                t_xmax[r_rd_idx] <= 11'd0;
                t_ymin[r_rd_idx] <= 10'd0;
                t_ymax[r_rd_idx] <= 10'd0;
                t_peri[r_rd_idx] <= 17'd0;
                t_cb  [r_rd_idx] <= 25'd0;
                t_cr  [r_rd_idx] <= 25'd0;
                t_skin[r_rd_idx] <= 17'd0;
                t_ylo [r_rd_idx] <= 8'hff;
                t_yhi [r_rd_idx] <= 8'd0;

                if(r_rd_idx == TILES[3:0] - 4'd1) begin
                    r_read       <= 1'b0;
                    O_tile_valid <= 1'b0;
                    O_tile_last  <= 1'b0;
                end
                else begin
                    r_rd_idx     <= r_rd_idx + 4'd1;
                    O_tile_last  <= (r_rd_idx == TILES[3:0] - 4'd2);
                end
            end
            //--------------------------------------------------------------
            else if(S_frame_end) begin
                r_read       <= 1'b1;
                r_rd_idx     <= 4'd0;
                O_tile_valid <= 1'b1;
                O_tile_last  <= 1'b0;
            end
            //--------------------------------------------------------------
            // safety : a new frame always starts with empty tiles
            // (also covers a truncated frame).  Crucially we also release a
            // stuck readout (r_read) here : if the previous frame's consumer
            // stopped early, r_read would otherwise stay 1 and block the tile
            // accumulation AND the next readout for ever -> the simulation
            // handshake deadlocks (while(!done) never returns).
            else if(S_tick && I_tuser) begin
                for(k = 0; k < TILES; k = k + 1) begin
                    t_cnt [k] <= 17'd0;
                    t_xmin[k] <= 11'd0;
                    t_xmax[k] <= 11'd0;
                    t_ymin[k] <= 10'd0;
                    t_ymax[k] <= 10'd0;
                    t_peri[k] <= 17'd0;
                    t_cb  [k] <= 25'd0;
                    t_cr  [k] <= 25'd0;
                    t_skin[k] <= 17'd0;
                    t_ylo [k] <= 8'hff;
                    t_yhi [k] <= 8'd0;
                end
                r_read       <= 1'b0;
                O_tile_valid <= 1'b0;
                O_tile_last  <= 1'b0;
            end
        end
    end

    //-------------------------------------------------------------------------
    // readout ports (combinational view of the tile arrays)
    //-------------------------------------------------------------------------
    assign O_tile_id = r_rd_idx;
    assign O_cnt     = t_cnt [r_rd_idx];
    assign O_xmin    = t_xmin[r_rd_idx];
    assign O_xmax    = t_xmax[r_rd_idx];
    assign O_ymin    = t_ymin[r_rd_idx];
    assign O_ymax    = t_ymax[r_rd_idx];
    assign O_peri    = t_peri[r_rd_idx];
    assign O_sum_cb  = t_cb  [r_rd_idx];
    assign O_sum_cr  = t_cr  [r_rd_idx];
    assign O_skin    = t_skin[r_rd_idx];
    assign O_ylo     = t_ylo [r_rd_idx];
    assign O_yhi     = t_yhi [r_rd_idx];

endmodule
