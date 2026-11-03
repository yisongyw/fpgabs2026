//-----------------------------------------------------------------------------
// Module     : hist_eq
// Project    : HX1P35Aimg_process_improve2
// Description: Automatic image quality enhancement (advanced requirement 2.5-5).
//
//   Three tone mapping algorithms share ONE histogram, ONE CDF accumulator and
//   ONE ping-pong mapping memory, so the second and the third algorithm are
//   almost free :
//
//        mode 1  HE       classic 64 bin histogram equalisation
//        mode 2  CLAHE    contrast LIMITED adaptive histogram equalisation :
//                         every bin is clipped to 4 x the average bin count and
//                         the clipped excess is redistributed over all 64 bins.
//                         Plain HE amplifies the noise of flat areas (a white
//                         wall becomes grainy), CLAHE does not - this is the
//                         variant that real cameras use.
//        mode 3  STRETCH  automatic contrast stretch : the black level and the
//                         white level are the 1.6 % and the 98.4 % percentile
//                         of the histogram, everything between them is mapped
//                         linearly onto 0 .. 255.  This is what "auto levels"
//                         does to a low contrast scene.
//
//   Frame end processing (about 130 clocks = 2 us of a 16.7 ms frame)
//   ----------------------------------------------------------------
//        pass A : excess = SUM max(h[i] - clip)        (CLAHE)
//                 percentile scan for lo / hi          (STRETCH)
//        pass B : cdf[i] , map[i] = curve(cdf[i]) , gain[i] = 64 * map[i] / Yc
//
//   The mapping memory is a ping-pong pair : while frame N is displayed with
//   bank A the curve of frame N is written into bank B and the banks swap at
//   the end of pass B, so no frame is ever displayed with a half updated curve.
//
//   Per pixel application (1 clock pipeline stage, 4 pixels per beat)
//   ----------------------------------------------------------------
//        Y    = R/4 + G/2 + B/4                       (shift - add, 0 DSP)
//        gain = gain_ram[Y >> 2]                      (UQ2.6, 64 == 1.0)
//        C'   = sat( (C * gain) >> 6 )    for R, G and B
//
//   All three channels are multiplied by the SAME factor, therefore the ratio
//   R:G:B - and with it the hue and the saturation - is preserved.  A per
//   channel equalisation would shift the colours.
//
//   Resource balancing
//   ------------------
//   This design is short on LUTs (91 % used) while 33 of the 40 DSP blocks are
//   idle.  The 12 per pixel multiplications (4 pixels x 3 channels) are
//   therefore mapped onto DSP blocks (USE_DSP = 1).  A shift-and-add
//   implementation of the same 12 multipliers would cost about 600 LUTs, i.e.
//   more than the whole rest of this module.  USE_DSP = 0 selects the LUT
//   implementation.
//
//   Two constant tables (recip / ramp_inc) replace 64 run time divisions per
//   frame, so the module needs no divider at all.
//-----------------------------------------------------------------------------
module hist_eq #(
    parameter NBIN    = 64,      // histogram bins (power of two, <= 64)
    parameter SCALE   = 20'd73,  // round(255 * 2^16 / 230400)
    parameter USE_DSP = 1        // 1 : per pixel multiplies inside DSP blocks
)(
    input  wire        I_clk,
    input  wire        I_rst_n,

    // ---- histogram port (driven by hist_stat) ----
    output wire [5:0]  O_bin_addr,
    input  wire [19:0] I_bin_data,
    input  wire [17:0] I_nsamp,
    input  wire        I_stat_valid,

    // ---- mode : 0 off / 1 HE / 2 CLAHE / 3 STRETCH ----
    input  wire [2:0]  I_mode,

    // ---- video stream, 96 bit = 4 x RGB888 ----
    input  wire        I_tvalid,
    input  wire        I_tuser,
    input  wire        I_tlast,
    input  wire [95:0] I_tdata,

    output reg         O_tvalid,
    output reg         O_tuser,
    output reg         O_tlast,
    output reg  [95:0] O_tdata
);

    localparam [2:0] M_OFF     = 3'd0;
    localparam [2:0] M_HE      = 3'd1;
    localparam [2:0] M_CLAHE   = 3'd2;
    localparam [2:0] M_STRETCH = 3'd3;

    localparam [7:0] GAIN_ONE = 8'd64;      // UQ2.6 : 64 == a gain of 1.0

    //-------------------------------------------------------------------------
    // constant tables (generated with python, see the project documentation)
    //   recip[i]    = round(16384 / (4*i+2))   -> gain = (map * recip) >> 8
    //   ramp_inc[d] = round(65280 / d)         -> contrast stretch slope
    //-------------------------------------------------------------------------
    reg [15:0] recip    [0:63];
    reg [15:0] ramp_inc [0:63];

    initial begin
        recip[ 0] = 16'd8192;  recip[ 1] = 16'd2731;  recip[ 2] = 16'd1638;
        recip[ 3] = 16'd1170;  recip[ 4] = 16'd910;   recip[ 5] = 16'd745;
        recip[ 6] = 16'd630;   recip[ 7] = 16'd546;   recip[ 8] = 16'd482;
        recip[ 9] = 16'd431;   recip[10] = 16'd390;   recip[11] = 16'd356;
        recip[12] = 16'd328;   recip[13] = 16'd303;   recip[14] = 16'd282;
        recip[15] = 16'd264;   recip[16] = 16'd248;   recip[17] = 16'd234;
        recip[18] = 16'd221;   recip[19] = 16'd210;   recip[20] = 16'd200;
        recip[21] = 16'd191;   recip[22] = 16'd182;   recip[23] = 16'd174;
        recip[24] = 16'd167;   recip[25] = 16'd161;   recip[26] = 16'd155;
        recip[27] = 16'd149;   recip[28] = 16'd144;   recip[29] = 16'd139;
        recip[30] = 16'd134;   recip[31] = 16'd130;   recip[32] = 16'd126;
        recip[33] = 16'd122;   recip[34] = 16'd119;   recip[35] = 16'd115;
        recip[36] = 16'd112;   recip[37] = 16'd109;   recip[38] = 16'd106;
        recip[39] = 16'd104;   recip[40] = 16'd101;   recip[41] = 16'd99;
        recip[42] = 16'd96;    recip[43] = 16'd94;    recip[44] = 16'd92;
        recip[45] = 16'd90;    recip[46] = 16'd88;    recip[47] = 16'd86;
        recip[48] = 16'd84;    recip[49] = 16'd83;    recip[50] = 16'd81;
        recip[51] = 16'd80;    recip[52] = 16'd78;    recip[53] = 16'd77;
        recip[54] = 16'd75;    recip[55] = 16'd74;    recip[56] = 16'd72;
        recip[57] = 16'd71;    recip[58] = 16'd70;    recip[59] = 16'd69;
        recip[60] = 16'd68;    recip[61] = 16'd67;    recip[62] = 16'd66;
        recip[63] = 16'd65;

        ramp_inc[ 0] = 16'd0;     ramp_inc[ 1] = 16'd65280;
        ramp_inc[ 2] = 16'd32640; ramp_inc[ 3] = 16'd21760;
        ramp_inc[ 4] = 16'd16320; ramp_inc[ 5] = 16'd13056;
        ramp_inc[ 6] = 16'd10880; ramp_inc[ 7] = 16'd9326;
        ramp_inc[ 8] = 16'd8160;  ramp_inc[ 9] = 16'd7253;
        ramp_inc[10] = 16'd6528;  ramp_inc[11] = 16'd5935;
        ramp_inc[12] = 16'd5440;  ramp_inc[13] = 16'd5022;
        ramp_inc[14] = 16'd4663;  ramp_inc[15] = 16'd4352;
        ramp_inc[16] = 16'd4080;  ramp_inc[17] = 16'd3840;
        ramp_inc[18] = 16'd3627;  ramp_inc[19] = 16'd3436;
        ramp_inc[20] = 16'd3264;  ramp_inc[21] = 16'd3109;
        ramp_inc[22] = 16'd2967;  ramp_inc[23] = 16'd2838;
        ramp_inc[24] = 16'd2720;  ramp_inc[25] = 16'd2611;
        ramp_inc[26] = 16'd2511;  ramp_inc[27] = 16'd2418;
        ramp_inc[28] = 16'd2331;  ramp_inc[29] = 16'd2251;
        ramp_inc[30] = 16'd2176;  ramp_inc[31] = 16'd2106;
        ramp_inc[32] = 16'd2040;  ramp_inc[33] = 16'd1978;
        ramp_inc[34] = 16'd1920;  ramp_inc[35] = 16'd1865;
        ramp_inc[36] = 16'd1813;  ramp_inc[37] = 16'd1764;
        ramp_inc[38] = 16'd1718;  ramp_inc[39] = 16'd1674;
        ramp_inc[40] = 16'd1632;  ramp_inc[41] = 16'd1592;
        ramp_inc[42] = 16'd1554;  ramp_inc[43] = 16'd1518;
        ramp_inc[44] = 16'd1484;  ramp_inc[45] = 16'd1451;
        ramp_inc[46] = 16'd1419;  ramp_inc[47] = 16'd1389;
        ramp_inc[48] = 16'd1360;  ramp_inc[49] = 16'd1332;
        ramp_inc[50] = 16'd1306;  ramp_inc[51] = 16'd1280;
        ramp_inc[52] = 16'd1255;  ramp_inc[53] = 16'd1232;
        ramp_inc[54] = 16'd1209;  ramp_inc[55] = 16'd1187;
        ramp_inc[56] = 16'd1166;  ramp_inc[57] = 16'd1145;
        ramp_inc[58] = 16'd1126;  ramp_inc[59] = 16'd1106;
        ramp_inc[60] = 16'd1088;  ramp_inc[61] = 16'd1070;
        ramp_inc[62] = 16'd1053;  ramp_inc[63] = 16'd1036;
    end

    //-------------------------------------------------------------------------
    // mapping memories : two banks of 64 x 8 bit for the curve and two banks of
    // 64 x 8 bit for the gain.  One bank is read by the pixel path while the
    // other one is written by the frame end processor.
    //-------------------------------------------------------------------------
    reg [7:0] map0  [0:NBIN-1];
    reg [7:0] map1  [0:NBIN-1];
    reg [7:0] gain0 [0:NBIN-1];
    reg [7:0] gain1 [0:NBIN-1];

    //-------------------------------------------------------------------------
    // frame end processor
    //-------------------------------------------------------------------------
    localparam [1:0] ST_INIT = 2'd0;
    localparam [1:0] ST_IDLE = 2'd1;
    localparam [1:0] ST_A    = 2'd2;
    localparam [1:0] ST_B    = 2'd3;

    reg [1:0]  r_state;
    reg [5:0]  r_i;
    reg [17:0] r_nsamp;
    reg [2:0]  r_mode;
    reg [19:0] r_cdf;
    reg [19:0] r_excess;
    reg [19:0] r_clip;
    reg [19:0] r_redist;
    reg [5:0]  r_lo;
    reg [5:0]  r_hi;
    reg        r_got_lo;
    reg        r_got_hi;
    reg        r_rb;            // bank that the pixel path reads

    assign O_bin_addr = r_i;

    //-------------------------------------------------------------------------
    // pass A
    //-------------------------------------------------------------------------
    wire [19:0] S_h      = I_bin_data;
    wire [19:0] S_exc    = (S_h > r_clip) ? (S_h - r_clip) : 20'd0;
    wire [19:0] S_cdf_a  = r_cdf + S_h;
    wire [19:0] S_thr_lo = {8'd0, r_nsamp[17:6]};               // nsamp / 64
    wire [19:0] S_thr_hi = {2'd0, r_nsamp} - S_thr_lo;

    //-------------------------------------------------------------------------
    // pass B
    //-------------------------------------------------------------------------
    wire [19:0] S_hc    = (r_mode == M_CLAHE)
                          ? ((S_h > r_clip ? r_clip : S_h) + r_redist)
                          : S_h;
    wire [19:0] S_cdf_n = r_cdf + S_hc;

    // HE / CLAHE : map = clamp( (cdf * SCALE) >> 16 ),  SCALE = 64 + 8 + 1
    // improve2 (place fix) : cdf <= 2^18, so cdf * 73 < 2^25 and the two
    // adders only need 25 bit instead of 32 (bit identical result).
    wire [24:0] S_m32    = ({7'd0, S_cdf_n} << 6)
                         + ({7'd0, S_cdf_n} << 3)
                         + {7'd0, S_cdf_n};
    wire [15:0] S_mv     = {7'd0, S_m32[24:16]};
    wire [7:0]  S_map_he = (S_mv > 16'd255) ? 8'd255 : S_mv[7:0];

    // STRETCH : linear ramp between the two percentile bins
    wire [5:0]  S_d    = (r_hi > r_lo) ? (r_hi - r_lo) : 6'd0;
    wire [5:0]  S_di   = (r_i  > r_lo) ? (r_i  - r_lo) : 6'd0;

    wire [23:0] S_rm;
    generate
        if (USE_DSP != 0) begin : g_rm_dsp
            assign S_rm = ramp_inc[S_d] * {10'd0, S_di};
        end
        else begin : g_rm_lut
            assign S_rm = mul16_lut(ramp_inc[S_d], {2'd0, S_di});
        end
    endgenerate

    wire [7:0]  S_map_st = (r_i >= r_hi) ? 8'd255
                         : (r_i <= r_lo) ? 8'd0
                         : ((|S_rm[23:16]) ? 8'd255 : S_rm[15:8]);

    wire [7:0]  S_map    = (r_mode == M_STRETCH) ? S_map_st : S_map_he;

    // gain = clamp( (map * recip[i]) >> 8 ) == round(64 * map / bin centre)
    wire [23:0] S_gm;
    generate
        if (USE_DSP != 0) begin : g_gm_dsp
            assign S_gm = recip[r_i] * {8'd0, S_map};
        end
        else begin : g_gm_lut
            assign S_gm = mul16_lut(recip[r_i], S_map);
        end
    endgenerate

    wire [7:0]  S_gain = (|S_gm[23:16]) ? 8'd255 : S_gm[15:8];

    //-------------------------------------------------------------------------
    // shift-and-add helpers (only used when USE_DSP == 0)
    //-------------------------------------------------------------------------
    function [23:0] mul16_lut;
        input [15:0] a;
        input [7:0]  b;
        reg [23:0] s;
        integer j;
        begin
            s = 24'd0;
            for (j = 0; j < 8; j = j + 1)
                if (b[j]) s = s + ({8'd0, a} << j);
            mul16_lut = s;
        end
    endfunction

    function [15:0] mul8_lut;
        input [7:0] a;
        input [7:0] b;
        reg [15:0] s;
        integer j;
        begin
            s = 16'd0;
            for (j = 0; j < 8; j = j + 1)
                if (b[j]) s = s + ({8'd0, a} << j);
            mul8_lut = s;
        end
    endfunction

    //-------------------------------------------------------------------------
    // per pixel tone mapping : 4 parallel datapaths, 1 clock of pipeline
    //-------------------------------------------------------------------------
    wire [95:0] S_out;
    reg  [2:0]  r_mode_act;     // mode latched at the frame start

    genvar gi;
    generate
        for (gi = 0; gi < 4; gi = gi + 1) begin : g_px
            wire [23:0] S_ip = I_tdata[95 - 24*gi -: 24];
            wire [7:0]  S_y8 = {2'd0, S_ip[23:18]} + {1'd0, S_ip[15:9]}
                             + {2'd0, S_ip[7:2]};
            wire [5:0]  S_bi = S_y8[7:2];
            wire [7:0]  S_gn = r_rb ? gain1[S_bi] : gain0[S_bi];

            wire [15:0] S_pr, S_pg, S_pb;

            if (USE_DSP != 0) begin : g_dsp
                assign S_pr = {2'd0, S_ip[23:16]} * {2'd0, S_gn};
                assign S_pg = {2'd0, S_ip[15: 8]} * {2'd0, S_gn};
                assign S_pb = {2'd0, S_ip[ 7: 0]} * {2'd0, S_gn};
            end
            else begin : g_lut
                assign S_pr = mul8_lut(S_ip[23:16], S_gn);
                assign S_pg = mul8_lut(S_ip[15: 8], S_gn);
                assign S_pb = mul8_lut(S_ip[ 7: 0], S_gn);
            end

            assign S_out[95 - 24*gi -: 24] = {
                (|S_pr[15:14]) ? 8'd255 : S_pr[13:6],
                (|S_pg[15:14]) ? 8'd255 : S_pg[13:6],
                (|S_pb[15:14]) ? 8'd255 : S_pb[13:6] };
        end
    endgenerate

    //-------------------------------------------------------------------------
    // state machine
    //-------------------------------------------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            r_state    <= ST_INIT;
            r_i        <= 6'd0;
            r_nsamp    <= 18'd0;
            r_mode     <= M_OFF;
            r_cdf      <= 20'd0;
            r_excess   <= 20'd0;
            r_clip     <= 20'd0;
            r_redist   <= 20'd0;
            r_lo       <= 6'd0;
            r_hi       <= 6'd63;
            r_got_lo   <= 1'b0;
            r_got_hi   <= 1'b0;
            r_rb       <= 1'b0;
            r_mode_act <= M_OFF;
        end
        else begin
            case(r_state)
                //-------------------------------------------------------------
                // write the identity curve into both banks so that the very
                // first frames are never black
                //-------------------------------------------------------------
                ST_INIT: begin
                    map0 [r_i] <= (r_i << 2) + 8'd2;
                    map1 [r_i] <= (r_i << 2) + 8'd2;
                    gain0[r_i] <= GAIN_ONE;
                    gain1[r_i] <= GAIN_ONE;
                    if(r_i == NBIN[5:0] - 6'd1) begin
                        r_i     <= 6'd0;
                        r_state <= ST_IDLE;
                    end
                    else
                        r_i <= r_i + 6'd1;
                end

                //-------------------------------------------------------------
                ST_IDLE: begin
                    if(I_stat_valid && (I_nsamp > 18'd0)) begin
                        r_nsamp  <= I_nsamp;
                        r_mode   <= I_mode;
                        r_i      <= 6'd0;
                        r_cdf    <= 20'd0;
                        r_excess <= 20'd0;
                        r_clip   <= {6'd0, I_nsamp[17:4]};   // 4 x average
                        r_lo     <= 6'd0;
                        r_hi     <= 6'd63;
                        r_got_lo <= 1'b0;
                        r_got_hi <= 1'b0;
                        r_state  <= ST_A;
                    end
                end

                //-------------------------------------------------------------
                // pass A : one clock per bin
                //-------------------------------------------------------------
                ST_A: begin
                    if(r_mode == M_CLAHE)
                        r_excess <= r_excess + S_exc;

                    if(r_mode == M_STRETCH) begin
                        r_cdf <= S_cdf_a;
                        if(!r_got_lo && (S_cdf_a >= S_thr_lo)) begin
                            r_lo     <= r_i;
                            r_got_lo <= 1'b1;
                        end
                        if(!r_got_hi && (S_cdf_a >= S_thr_hi)) begin
                            r_hi     <= r_i;
                            r_got_hi <= 1'b1;
                        end
                    end

                    if(r_i == NBIN[5:0] - 6'd1) begin
                        r_i      <= 6'd0;
                        r_cdf    <= 20'd0;
                        r_redist <= (r_excess + S_exc) >> 6;
                        r_state  <= ST_B;
                    end
                    else
                        r_i <= r_i + 6'd1;
                end

                //-------------------------------------------------------------
                // pass B : cdf -> curve -> gain, written into the idle bank
                //-------------------------------------------------------------
                ST_B: begin
                    r_cdf <= S_cdf_n;

                    // write the bank that the pixel path is NOT reading :
                    //   r_rb = 1 -> bank1 is displayed, bank0 is updated
                    //   r_rb = 0 -> bank0 is displayed, bank1 is updated
                    if(r_rb) begin
                        map0 [r_i] <= S_map;
                        gain0[r_i] <= S_gain;
                    end
                    else begin
                        map1 [r_i] <= S_map;
                        gain1[r_i] <= S_gain;
                    end

                    if(r_i == NBIN[5:0] - 6'd1) begin
                        r_rb    <= ~r_rb;
                        r_i     <= 6'd0;
                        r_state <= ST_IDLE;
                    end
                    else
                        r_i <= r_i + 6'd1;
                end

                default: r_state <= ST_IDLE;
            endcase

            // the displayed mode only changes at the frame boundary, so a
            // switch can never produce a half processed frame
            if(I_tvalid && I_tuser)
                r_mode_act <= I_mode;
        end
    end

    //-------------------------------------------------------------------------
    // output register : one clock of pipeline, transparent while mode == 0
    //-------------------------------------------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_tvalid <= 1'b0;
            O_tuser  <= 1'b0;
            O_tlast  <= 1'b0;
            O_tdata  <= 96'd0;
        end
        else begin
            O_tvalid <= I_tvalid;
            O_tuser  <= I_tuser;
            O_tlast  <= I_tlast;
            O_tdata  <= (r_mode_act == M_OFF) ? I_tdata : S_out;
        end
    end

endmodule
