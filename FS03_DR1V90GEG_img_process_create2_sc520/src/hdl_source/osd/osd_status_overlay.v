//-----------------------------------------------------------------------------
// Module     : osd_status_overlay
// Project    : HX1P35Aimg_process_extend2
// Description: Status text block in the BOTTOM LEFT corner of the 720p picture.
//
//   extend1 already drew two lines there :
//
//       row 0 : "ALG:xxxx"   base algorithm (SW1 / SW2 / SW3)
//       row 1 : "STY:xxxx"   stylization / visual enhancement (KEY3)
//
//   extend2 adds three more lines for the target detection :
//
//       row 2 : "RULE:xxxx"        result of the pure rule classifier
//       row 3 : "MLP:xxxx nn%"     result of the INT8 MLP + confidence
//       row 4 : "FPS:nn"           measured frame rate
//
//   Why bottom left :
//       * the official Anlogic logo occupies (0,0) - (159,159)
//       * the official hdmi_mixer text and the base2 osd_mode_overlay text
//         live around x = 375, y = 120 ... 208
//       * the bottom left corner is free, so the status block never overlaps
//         the logo or the existing text
//
//   Character size : 8 x 8 (osd_text8_lib), i.e. half the size of the 16 x 16
//   glyphs used by the other layers - small enough not to disturb the picture.
//   Every line has its own colour, so the five blocks can be told apart at a
//   glance even on a busy picture:
//
//       ALG   bright green 24'h00ff00
//       STY   orange       24'hffa500
//       RULE  light blue   24'h40a0ff
//       MLP   magenta      24'hff40ff
//       FPS   yellow       24'hffff00
//       panel the video pixel darkened to 1/8
//
//   I_osd_en (SW4) switches the whole block on (1) and off (0).  When it is 0
//   the module is completely transparent.
//
//   Window (default) : x = 24, y = 656, 120 x 60
//       row glyph rows :  2..9 / 14..21 / 26..33 / 38..45 / 50..57
//       text columns   :  4..107  (13 characters, 8 pixel each)
//-----------------------------------------------------------------------------
module osd_status_overlay #(
    parameter OSD_X = 12'd24,
    parameter OSD_Y = 12'd568,     // improve2 : 12 rows -> 144 pixel window
    parameter OSD_W = 12'd120,
    parameter OSD_H = 12'd144,
    parameter CLS_W = 6,           // bit width of a class index (2**CLS_W max)
    parameter NOUT  = 64           // number of classes of the model
)(
    input  wire        I_clk,
    input  wire        I_rst_n,

    input  wire        I_video_vsync,
    input  wire        I_video_hsync,
    input  wire        I_video_de,
    input  wire        I_video_user,
    input  wire        I_video_last,

    input  wire        I_osd_en,        // SW4 : 1 = show the status block
    input  wire [1:0]  I_alg_mode,      // 0 RGB / 1 GRAY / 2 SOBEL / 3 BIN
    input  wire [3:0]  I_style,         // 0 OFF / 1 EDGE / 2 FILL / 3 EDGE+FILL
                                        // 4..15 reserved -> "RSVnn"

    input  wire        I_obj_valid,     // 1 : a target has been recognised
    input  wire [CLS_W-1:0] I_rule_cls, // rule classifier result
    input  wire [CLS_W-1:0] I_mlp_cls,  // MLP classifier result
    input  wire [6:0]  I_mlp_conf,      // MLP confidence 0 .. 99
    input  wire [6:0]  I_fps,           // measured frames per second 0 .. 99
    input  wire [3:0]  I_trk_num,       // extend3 : number of active tracks
    input  wire        I_alarm,         // extend3 : ROI alarm
    input  wire [9:0]  I_latency_tenth_ms, // improve3 : latency in 0.1 ms
    input  wire        I_latency_valid, // improve3 : at least one valid sample
    input  wire [11:0] I_mot_cnt,        // improve1 : moving cells (motion det.)

    // improve2 : image quality auto adjustment (advanced requirement 2.5-5)
    input  wire [7:0]  I_yavg,           // mean luminance of the last frame
    input  wire [15:0] I_exp,            // exposure the AE loop asks for
    input  wire [6:0]  I_gain,           // sensor gain the AE loop asks for
    input  wire        I_ae_on,          // 1 : the AE loop is running

    input  wire [23:0] I_video_data,

    output reg         O_video_vsync,
    output reg         O_video_hsync,
    output reg         O_video_de,
    output reg  [23:0] O_video_data
);

    localparam COLOR_ALG_TEXT  = 24'h00ff00;   // bright green
    localparam COLOR_STY_TEXT  = 24'hffa500;   // orange
    localparam COLOR_RULE_TEXT = 24'h40a0ff;   // light blue
    localparam COLOR_MLP_TEXT  = 24'hff40ff;   // magenta
    localparam COLOR_FPS_TEXT  = 24'hffff00;   // yellow
    localparam COLOR_TRK_TEXT  = 24'h00ffff;   // cyan     (extend3)
    localparam COLOR_DLY_TEXT  = 24'hff8000;   // orange   (extend3)
    localparam COLOR_ALM_TEXT  = 24'hff0000;   // red      (extend3)
    localparam COLOR_MOT_TEXT  = 24'hff4000;   // orange   (improve1 - motion)
    localparam COLOR_CLS_TEXT  = 24'hc0c0c0;   // light grey (improve1 - classes)
    localparam COLOR_LUM_TEXT  = 24'h40ffd0;   // aqua     (improve2 - luminance)
    localparam COLOR_AE_TEXT   = 24'hffd040;   // gold     (improve2 - auto expo.)
    localparam COLOR_BAR_GREEN = 24'h00e060;   // latency < 25.0 ms
    localparam COLOR_BAR_YELLOW= 24'hffe000;   // latency 25.0 .. 40.0 ms
    localparam COLOR_BAR_RED   = 24'hff3030;   // latency > 40.0 ms
    localparam COLOR_BAR_BG    = 24'h202020;

    // improve3 latency bar: 240 px represents 50.0 ms at the bottom edge.
    localparam [11:0] BAR_X = 12'd24;
    localparam [11:0] BAR_Y = 12'd712;
    localparam [11:0] BAR_W = 12'd240;
    localparam [11:0] BAR_H = 12'd8;

    //-------------------------------------------------------------------------
    // character constants (ASCII, same encoding as osd_char_lib)
    //-------------------------------------------------------------------------
    localparam CHAR_SPACE = 8'h20;
    localparam CHAR_PLUS  = 8'h2b;
    localparam CHAR_DOT   = 8'h2e;
    localparam CHAR_COLON = 8'h3a;
    localparam CHAR_PCT   = 8'h25;
    localparam CHAR_A     = 8'h41;
    localparam CHAR_B     = 8'h42;
    localparam CHAR_C     = 8'h43;
    localparam CHAR_D     = 8'h44;
    localparam CHAR_E     = 8'h45;
    localparam CHAR_F     = 8'h46;
    localparam CHAR_G     = 8'h47;
    localparam CHAR_H     = 8'h48;   // improve2 : CLAHE / STRETCH labels
    localparam CHAR_I     = 8'h49;
    localparam CHAR_J     = 8'h4a;
    localparam CHAR_K     = 8'h4b;
    localparam CHAR_L     = 8'h4c;
    localparam CHAR_M     = 8'h4d;
    localparam CHAR_N     = 8'h4e;
    localparam CHAR_O     = 8'h4f;
    localparam CHAR_P     = 8'h50;
    localparam CHAR_R     = 8'h52;
    localparam CHAR_S     = 8'h53;
    localparam CHAR_T     = 8'h54;
    localparam CHAR_U     = 8'h55;
    localparam CHAR_V     = 8'h56;
    localparam CHAR_X     = 8'h58;
    localparam CHAR_Y     = 8'h59;
    localparam CHAR_0     = 8'h30;

    localparam [11:0] TEXT_X0   = 12'd4;    // left margin inside the window
    localparam [11:0] TEXT_LEN  = 12'd13;   // max characters per line
    localparam [11:0] TEXT_END  = TEXT_X0 + (TEXT_LEN << 3);   // 108

    // reserved style -> two decimal digits
    wire [3:0] S_rsv_ten = (I_style >= 4'd10) ? 4'd1 : 4'd0;
    wire [3:0] S_rsv_one = (I_style >= 4'd10) ? (I_style - 4'd10) : I_style;

    function [7:0] digit_char;
        input [3:0] d;
        begin
            digit_char = CHAR_0 + {4'd0, d};
        end
    endfunction

    // decimal helpers without a divider (the value never exceeds 99)
    function [3:0] div10;
        input [6:0] v;
        begin
            div10 = (v >= 7'd90) ? 4'd9 : (v >= 7'd80) ? 4'd8 :
                    (v >= 7'd70) ? 4'd7 : (v >= 7'd60) ? 4'd6 :
                    (v >= 7'd50) ? 4'd5 : (v >= 7'd40) ? 4'd4 :
                    (v >= 7'd30) ? 4'd3 : (v >= 7'd20) ? 4'd2 :
                    (v >= 7'd10) ? 4'd1 : 4'd0;
        end
    endfunction

    function [3:0] mod10;
        input [6:0] v;
        input [3:0] t;
        begin
            mod10 = v[3:0] - ({t, 3'd0} + {t, 1'd0});
        end
    endfunction

    //-------------------------------------------------------------------------
    // improve1 : the 32 class names come from the shared lookup table
    // (osd_cls_lut) instead of a 32 branch case statement - the PH1P35 is
    // short on LUTs but has plenty of block / distributed RAM.
    //-------------------------------------------------------------------------
    wire [31:0] S_rule_name;
    wire [31:0] S_mlp_name;

    osd_cls_lut u_lut_rule (
        .I_cls   ( I_rule_cls  ),
        .O_name  ( S_rule_name ),
        .O_color (             )
    );

    osd_cls_lut u_lut_mlp (
        .I_cls   ( I_mlp_cls   ),
        .O_name  ( S_mlp_name  ),
        .O_color (             )
    );

    // one character of a 4 character class name (index 0 = left most)
    function [7:0] name_char;
        input [31:0] nm;
        input [1:0]  idx;
        begin
            name_char = nm[(3 - idx)*8 +: 8];
        end
    endfunction

    // "NONE" when nothing has been detected
    function [7:0] obj_char;
        input        valid;
        input [31:0] nm;
        input [1:0]  idx;
        begin
            if(valid) begin
                obj_char = name_char(nm, idx);
            end
            else begin
                case(idx)
                    2'd0: obj_char = CHAR_N;
                    2'd1: obj_char = CHAR_O;
                    2'd2: obj_char = CHAR_N;
                    default: obj_char = CHAR_E;
                endcase
            end
        end
    endfunction

    //-------------------------------------------------------------------------
    // improve1 : decimal conversion of the moving cell counter (0 .. 3600)
    //-------------------------------------------------------------------------
    function [15:0] bcd12;              // 12 bit binary -> 4 BCD digits
        input [11:0] bin;
        reg [27:0] t;
        integer ib;
        begin
            t = 28'd0;
            t[11:0] = bin;
            for (ib = 0; ib < 12; ib = ib + 1) begin
                if (t[12 +: 4] > 4) t[12 +: 4] = t[12 +: 4] + 4'd3;
                if (t[16 +: 4] > 4) t[16 +: 4] = t[16 +: 4] + 4'd3;
                if (t[20 +: 4] > 4) t[20 +: 4] = t[20 +: 4] + 4'd3;
                if (t[24 +: 4] > 4) t[24 +: 4] = t[24 +: 4] + 4'd3;
                t = t << 1;
            end
            bcd12 = t[27:12];
        end
    endfunction

    wire [15:0] S_mot_dec = bcd12(I_mot_cnt);
    wire [3:0]  S_cls_ten = div10(NOUT[6:0]);
    wire [3:0]  S_cls_one = mod10(NOUT[6:0], S_cls_ten);

    // improve2 : luminance (0..255 -> 3 digits) and exposure (0..1125 -> 4)
    wire [15:0] S_lum_dec = bcd12({4'd0, I_yavg});
    wire [15:0] S_exp_dec = bcd12(I_exp);
    wire [3:0]  S_gain_ten = div10(I_gain);
    wire [3:0]  S_gain_one = mod10(I_gain, S_gain_ten);

    wire [3:0] S_conf_ten = div10(I_mlp_conf);
    wire [3:0] S_conf_one = mod10(I_mlp_conf, S_conf_ten);
    wire [3:0] S_fps_ten  = div10(I_fps);
    wire [3:0] S_fps_one  = mod10(I_fps, S_fps_ten);

    // improve3 : frame-stable 0.1 ms latency display and graphical bar.
    // 123/256 ~= 240/500, therefore 50.0 ms maps to the full 240 px width.
    reg  [9:0] S_latency_frame;
    reg        S_latency_valid_frame;
    reg  [7:0] S_bar_len;
    wire [16:0] S_bar_scaled = I_latency_tenth_ms * 7'd123;
    wire [7:0]  S_bar_len_next = !I_latency_valid ? 8'd0 :
                                     (I_latency_tenth_ms >= 10'd500) ? 8'd240 :
                                                                       S_bar_scaled[15:8];
    wire [15:0] S_lat_dec = bcd12({2'd0, S_latency_frame});
    wire [23:0] S_bar_color = (S_latency_frame < 10'd250) ? COLOR_BAR_GREEN :
                              (S_latency_frame <= 10'd400) ? COLOR_BAR_YELLOW :
                                                             COLOR_BAR_RED;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_latency_frame       <= 10'd0;
            S_latency_valid_frame <= 1'b0;
            S_bar_len             <= 8'd0;
        end
        else if(I_video_user) begin
            S_latency_frame       <= I_latency_tenth_ms;
            S_latency_valid_frame <= I_latency_valid;
            S_bar_len             <= S_bar_len_next;
        end
    end

    reg [11:0] S_x;
    reg [11:0] S_y;
    reg [11:0] S_x_1d, S_x_2d;
    reg [11:0] S_y_1d, S_y_2d;
    reg        S_de_1d,  S_de_2d;
    reg        S_vs_1d,  S_vs_2d;
    reg        S_hs_1d,  S_hs_2d;
    reg [23:0] S_data_1d, S_data_2d;

    reg [11:0] S_lx;
    reg [11:0] S_ly;
    reg [11:0] S_lxm;
    reg [11:0] S_ly_off;
    reg [3:0]  S_rowsel;      // improve1 : 10 rows -> 4 bit
    reg [3:0]  S_idx;
    reg [3:0]  S_n;
    reg [7:0]  S_char;
    reg [2:0]  S_col;
    reg [2:0]  S_row;
    reg        S_valid;
    reg        S_on;
    reg        S_hit;
    reg        S_in_win;
    reg        S_bar_area;
    reg        S_bar_fill;

    // Pipeline registers (HDMI_PIXEL_CLK setup fix) : every OSD "decision"
    // signal is registered by exactly one stage and the video/sync stream is
    // delayed by one extra stage (3 in total).  The relative 2-clock offset
    // between the overlay decision and the video pixel is therefore preserved
    // exactly, so the whole output stream is merely shifted by one pixel-clock
    // (bit-exact, visually identical).  The long combinatorial chain
    //   I_mot_cnt -> bcd12 -> char/row select -> font ROM -> S_on -> S_hit
    // is split at the font-ROM output (S_row_bits), cutting the worst path
    // from ~12 logic levels to ~8 (to S_row_bits_r) + ~3 (to O_video_data).
    reg  [7:0]  S_row_bits_r;
    reg  [2:0]  S_col_r;
    reg         S_valid_r;
    reg  [3:0]  S_rowsel_r;
    reg         S_bar_fill_r;
    reg         S_bar_area_r;
    reg         S_in_win_r;
    reg  [11:0] S_x_3d, S_y_3d;
    reg         S_vs_3d, S_hs_3d, S_de_3d;
    reg  [23:0] S_data_3d;

    wire [7:0] S_row_bits;

    osd_text8_lib u_osd_text8_lib(
        .I_char     ( S_char      ),
        .I_row      ( S_row       ),
        .O_row_bits ( S_row_bits  )
    );

    //-------------------------------------------------------------------------
    // Pixel coordinate counter (identical scheme to hdmi_mixer /
    // osd_mode_overlay)
    //-------------------------------------------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
        end
        else if(I_video_user) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
        end
        else if(I_video_de) begin
            if(I_video_last) begin
                S_x <= 12'd0;
                S_y <= S_y + 12'd1;
            end
            else begin
                S_x <= S_x + 12'd1;
            end
        end
    end

    //-------------------------------------------------------------------------
    // Character selection + rendering (combinational)
    //-------------------------------------------------------------------------
    always @(*) begin
        S_lx       = 12'd0;
        S_ly       = 12'd0;
        S_lxm      = 12'd0;
        S_ly_off   = 12'd0;
        S_rowsel   = 3'd0;
        S_idx      = 4'd0;
        S_n        = 4'd0;
        S_char     = CHAR_SPACE;
        S_col      = 3'd0;
        S_row      = 3'd0;
        S_valid    = 1'b0;
        S_on       = 1'b0;
        S_hit      = 1'b0;
        S_in_win   = 1'b0;
        S_bar_area = 1'b0;
        S_bar_fill = 1'b0;

        if(S_de_2d && I_osd_en) begin
            if(S_latency_valid_frame &&
               (S_x_2d >= BAR_X) && (S_x_2d < (BAR_X + BAR_W)) &&
               (S_y_2d >= BAR_Y) && (S_y_2d < (BAR_Y + BAR_H))) begin
                S_bar_area = 1'b1;
                if((S_x_2d - BAR_X) < {4'd0, S_bar_len})
                    S_bar_fill = 1'b1;
            end

            if((S_x_2d >= OSD_X) && (S_x_2d < (OSD_X + OSD_W)) &&
               (S_y_2d >= OSD_Y) && (S_y_2d < (OSD_Y + OSD_H))) begin

                S_in_win = 1'b1;
                S_lx     = S_x_2d - OSD_X;
                S_ly     = S_y_2d - OSD_Y;

                // twelve text rows (extend3 adds TRK / DLY / ALM,
                //                  improve1 adds MOT / CLS,
                //                  improve2 adds LUM / AE)
                S_rowsel = (S_ly >= 12'd134) ? 4'd11 :
                           (S_ly >= 12'd122) ? 4'd10 :
                           (S_ly >= 12'd110) ? 4'd9 :
                           (S_ly >= 12'd98)  ? 4'd8 :
                           (S_ly >= 12'd86)  ? 4'd7 :
                           (S_ly >= 12'd74)  ? 4'd6 :
                           (S_ly >= 12'd62)  ? 4'd5 :
                           (S_ly >= 12'd50)  ? 4'd4 :
                           (S_ly >= 12'd38)  ? 4'd3 :
                           (S_ly >= 12'd26)  ? 4'd2 :
                           (S_ly >= 12'd14)  ? 4'd1 : 4'd0;

                case(S_rowsel)
                    4'd0: S_ly_off = S_ly - 12'd2;
                    4'd1: S_ly_off = S_ly - 12'd14;
                    4'd2: S_ly_off = S_ly - 12'd26;
                    4'd3: S_ly_off = S_ly - 12'd38;
                    4'd4: S_ly_off = S_ly - 12'd50;
                    4'd5: S_ly_off = S_ly - 12'd62;
                    4'd6: S_ly_off = S_ly - 12'd74;
                    4'd7: S_ly_off = S_ly - 12'd86;
                    4'd8:  S_ly_off = S_ly - 12'd98;
                    4'd9:  S_ly_off = S_ly - 12'd110;
                    4'd10: S_ly_off = S_ly - 12'd122;
                    default: S_ly_off = S_ly - 12'd134;
                endcase

                S_valid  = (S_ly_off < 12'd8);
                S_row    = S_ly_off[2:0];

                if((S_lx >= TEXT_X0) && (S_lx < TEXT_END)) begin
                    S_lxm = S_lx - TEXT_X0;
                    S_idx = S_lxm[6:3];        // character index 0 .. 12
                    S_col = S_lxm[2:0];        // column inside the glyph

                    //------------------------------------------------------
                    // fixed labels : "ALG:" / "STY:" / "RULE:" / "MLP:" / "FPS:"
                    //------------------------------------------------------
                    if(S_idx < 4'd4) begin
                        case(S_rowsel)
                            3'd0: begin
                                case(S_idx)
                                    4'd0:    S_char = CHAR_A;
                                    4'd1:    S_char = CHAR_L;
                                    4'd2:    S_char = CHAR_G;
                                    default: S_char = CHAR_COLON;
                                endcase
                            end
                            3'd1: begin
                                case(S_idx)
                                    4'd0:    S_char = CHAR_S;
                                    4'd1:    S_char = CHAR_T;
                                    4'd2:    S_char = CHAR_Y;
                                    default: S_char = CHAR_COLON;
                                endcase
                            end
                            3'd2: begin
                                case(S_idx)
                                    4'd0:    S_char = CHAR_R;
                                    4'd1:    S_char = CHAR_U;
                                    4'd2:    S_char = CHAR_L;
                                    4'd3:    S_char = CHAR_E;
                                    default: S_char = CHAR_COLON;
                                endcase
                            end
                            3'd3: begin
                                case(S_idx)
                                    4'd0:    S_char = CHAR_M;
                                    4'd1:    S_char = CHAR_L;
                                    4'd2:    S_char = CHAR_P;
                                    default: S_char = CHAR_COLON;
                                endcase
                            end
                            3'd4: begin
                                case(S_idx)
                                    4'd0:    S_char = CHAR_F;
                                    4'd1:    S_char = CHAR_P;
                                    4'd2:    S_char = CHAR_S;
                                    default: S_char = CHAR_COLON;
                                endcase
                            end
                            3'd5: begin
                                case(S_idx)
                                    4'd0:    S_char = CHAR_T;
                                    4'd1:    S_char = CHAR_R;
                                    4'd2:    S_char = CHAR_K;
                                    default: S_char = CHAR_COLON;
                                endcase
                            end
                            3'd6: begin
                                case(S_idx)
                                    4'd0:    S_char = CHAR_D;
                                    4'd1:    S_char = CHAR_L;
                                    4'd2:    S_char = CHAR_Y;
                                    default: S_char = CHAR_COLON;
                                endcase
                            end
                            3'd7: begin
                                case(S_idx)
                                    4'd0:    S_char = CHAR_A;
                                    4'd1:    S_char = CHAR_L;
                                    4'd2:    S_char = CHAR_M;
                                    default: S_char = CHAR_COLON;
                                endcase
                            end
                            4'd8: begin                  // MOT (improve1)
                                case(S_idx)
                                    4'd0:    S_char = CHAR_M;
                                    4'd1:    S_char = CHAR_O;
                                    4'd2:    S_char = CHAR_T;
                                    default: S_char = CHAR_COLON;
                                endcase
                            end
                            4'd9: begin                  // CLS (improve1)
                                case(S_idx)
                                    4'd0:    S_char = CHAR_C;
                                    4'd1:    S_char = CHAR_L;
                                    4'd2:    S_char = CHAR_S;
                                    default: S_char = CHAR_COLON;
                                endcase
                            end
                            4'd10: begin                 // LUM (improve2)
                                case(S_idx)
                                    4'd0:    S_char = CHAR_L;
                                    4'd1:    S_char = CHAR_U;
                                    4'd2:    S_char = CHAR_M;
                                    default: S_char = CHAR_COLON;
                                endcase
                            end
                            4'd11: begin                 // AE  (improve2)
                                case(S_idx)
                                    4'd0:    S_char = CHAR_A;
                                    4'd1:    S_char = CHAR_E;
                                    4'd2:    S_char = CHAR_COLON;
                                    default: S_char = CHAR_SPACE;
                                endcase
                            end
                        endcase
                    end
                    else begin
                        S_n = S_idx - 4'd4;    // 0 .. 8
                        case(S_rowsel)
                            //------------------------------------------------
                            // base algorithm name (unchanged from extend1)
                            //------------------------------------------------
                            3'd0: begin
                                case(I_alg_mode)
                                    2'd0: begin                     // RGB
                                        case(S_n)
                                            4'd0:    S_char = CHAR_R;
                                            4'd1:    S_char = CHAR_G;
                                            4'd2:    S_char = CHAR_B;
                                            default: S_char = CHAR_SPACE;
                                        endcase
                                    end
                                    2'd1: begin                     // GRAY
                                        case(S_n)
                                            4'd0:    S_char = CHAR_G;
                                            4'd1:    S_char = CHAR_R;
                                            4'd2:    S_char = CHAR_A;
                                            4'd3:    S_char = CHAR_Y;
                                            default: S_char = CHAR_SPACE;
                                        endcase
                                    end
                                    2'd2: begin                     // SOBEL
                                        case(S_n)
                                            4'd0:    S_char = CHAR_S;
                                            4'd1:    S_char = CHAR_O;
                                            4'd2:    S_char = CHAR_B;
                                            4'd3:    S_char = CHAR_E;
                                            4'd4:    S_char = CHAR_L;
                                            default: S_char = CHAR_SPACE;
                                        endcase
                                    end
                                    default: begin                  // BIN
                                        case(S_n)
                                            4'd0:    S_char = CHAR_B;
                                            4'd1:    S_char = CHAR_I;
                                            4'd2:    S_char = CHAR_N;
                                            default: S_char = CHAR_SPACE;
                                        endcase
                                    end
                                endcase
                            end

                            //------------------------------------------------
                            // style name (unchanged from extend1)
                            //------------------------------------------------
                            3'd1: begin
                                case(I_style)
                                    4'd0: begin                     // OFF
                                        case(S_n)
                                            4'd0:    S_char = CHAR_O;
                                            4'd1:    S_char = CHAR_F;
                                            4'd2:    S_char = CHAR_F;
                                            default: S_char = CHAR_SPACE;
                                        endcase
                                    end
                                    4'd1: begin                     // EDGE
                                        case(S_n)
                                            4'd0:    S_char = CHAR_E;
                                            4'd1:    S_char = CHAR_D;
                                            4'd2:    S_char = CHAR_G;
                                            4'd3:    S_char = CHAR_E;
                                            default: S_char = CHAR_SPACE;
                                        endcase
                                    end
                                    4'd2: begin                     // FILL
                                        case(S_n)
                                            4'd0:    S_char = CHAR_F;
                                            4'd1:    S_char = CHAR_I;
                                            4'd2:    S_char = CHAR_L;
                                            4'd3:    S_char = CHAR_L;
                                            default: S_char = CHAR_SPACE;
                                        endcase
                                    end
                                    4'd3: begin                     // EDGE+FILL
                                        case(S_n)
                                            4'd0:    S_char = CHAR_E;
                                            4'd1:    S_char = CHAR_D;
                                            4'd2:    S_char = CHAR_G;
                                            4'd3:    S_char = CHAR_E;
                                            4'd4:    S_char = CHAR_PLUS;
                                            4'd5:    S_char = CHAR_F;
                                            4'd6:    S_char = CHAR_I;
                                            4'd7:    S_char = CHAR_L;
                                            4'd8:    S_char = CHAR_L;
                                            default: S_char = CHAR_SPACE;
                                            endcase
                                            end
                                            // improve2 : image quality enhancement modes
                                            4'd4: begin                     // HE
                                            case(S_n)
                                                4'd0:    S_char = CHAR_H;
                                                4'd1:    S_char = CHAR_E;
                                                default: S_char = CHAR_SPACE;
                                            endcase
                                            end
                                            4'd5: begin                     // CLAHE
                                            case(S_n)
                                                4'd0:    S_char = CHAR_C;
                                                4'd1:    S_char = CHAR_L;
                                                4'd2:    S_char = CHAR_A;
                                                4'd3:    S_char = CHAR_H;
                                                4'd4:    S_char = CHAR_E;
                                                default: S_char = CHAR_SPACE;
                                            endcase
                                            end
                                            4'd6: begin                     // STRETCH
                                            case(S_n)
                                                4'd0:    S_char = CHAR_S;
                                                4'd1:    S_char = CHAR_T;
                                                4'd2:    S_char = CHAR_R;
                                                4'd3:    S_char = CHAR_E;
                                                4'd4:    S_char = CHAR_T;
                                                4'd5:    S_char = CHAR_C;
                                                4'd6:    S_char = CHAR_H;
                                                default: S_char = CHAR_SPACE;
                                            endcase
                                            end
                                            default: begin                  // reserved
                                        case(S_n)
                                            4'd0:    S_char = CHAR_R;
                                            4'd1:    S_char = CHAR_S;
                                            4'd2:    S_char = CHAR_V;
                                            4'd3:    S_char = digit_char(S_rsv_ten);
                                            4'd4:    S_char = digit_char(S_rsv_one);
                                            default: S_char = CHAR_SPACE;
                                        endcase
                                    end
                                endcase
                            end

                            //------------------------------------------------
                            // RULE:<class>
                            //------------------------------------------------
                            3'd2: begin
                                if(S_n < 4'd4)
                                    S_char = obj_char(I_obj_valid, S_rule_name,
                                                      S_n[1:0]);
                                else
                                    S_char = CHAR_SPACE;
                            end

                            //------------------------------------------------
                            // MLP:<class> nn%
                            //------------------------------------------------
                            3'd3: begin
                                case(S_n)
                                    4'd0, 4'd1, 4'd2, 4'd3:
                                        S_char = obj_char(I_obj_valid, S_mlp_name,
                                                          S_n[1:0]);
                                    4'd4: S_char = CHAR_SPACE;
                                    4'd5: S_char = digit_char(S_conf_ten);
                                    4'd6: S_char = digit_char(S_conf_one);
                                    4'd7: S_char = CHAR_PCT;
                                    default: S_char = CHAR_SPACE;
                                endcase
                            end

                            //------------------------------------------------
                            // FPS:nn
                            //------------------------------------------------
                            3'd4: begin
                                case(S_n)
                                    4'd0: S_char = digit_char(S_fps_ten);
                                    4'd1: S_char = digit_char(S_fps_one);
                                    default: S_char = CHAR_SPACE;
                                endcase
                            end

                            //------------------------------------------------
                            // TRK:n  (extend3 : number of active tracks)
                            //------------------------------------------------
                            3'd5: begin
                                if(S_n < 4'd1)
                                    S_char = digit_char(I_trk_num);
                                else
                                    S_char = CHAR_SPACE;
                            end

                            //------------------------------------------------
                            // DLY:nn.nMS  (improve3 : 0.1 ms resolution)
                            //------------------------------------------------
                            3'd6: begin
                                case(S_n)
                                    4'd0:    S_char = digit_char(S_lat_dec[11:8]);
                                    4'd1:    S_char = digit_char(S_lat_dec[7:4]);
                                    4'd2:    S_char = CHAR_DOT;
                                    4'd3:    S_char = digit_char(S_lat_dec[3:0]);
                                    4'd4:    S_char = CHAR_M;
                                    4'd5:    S_char = CHAR_S;
                                    default: S_char = CHAR_SPACE;
                                endcase
                            end

                            //------------------------------------------------
                            // ALM:ON / ALM:OFF  (extend3 : ROI alarm)
                            //------------------------------------------------
                            3'd7: begin
                                if(I_alarm) begin
                                    case(S_n)
                                        4'd0: S_char = CHAR_O;
                                        4'd1: S_char = CHAR_N;
                                        default: S_char = CHAR_SPACE;
                                    endcase
                                end else begin
                                    case(S_n)
                                        4'd0: S_char = CHAR_O;
                                        4'd1: S_char = CHAR_F;
                                        4'd2: S_char = CHAR_F;
                                        default: S_char = CHAR_SPACE;
                                    endcase
                                end
                            end

                            //------------------------------------------------
                            // MOT:nnnn  (improve1 : moving cells, 0 .. 3600)
                            //------------------------------------------------
                            4'd8: begin
                                case(S_n)
                                    4'd0: S_char = CHAR_0 + {1'd0, S_mot_dec[15:12]};
                                    4'd1: S_char = CHAR_0 + S_mot_dec[11:8];
                                    4'd2: S_char = CHAR_0 + S_mot_dec[7:4];
                                    4'd3: S_char = CHAR_0 + S_mot_dec[3:0];
                                    default: S_char = CHAR_SPACE;
                                endcase
                            end

                            //------------------------------------------------
                            // CLS:nn    (improve1 : number of model classes)
                            //------------------------------------------------
                            4'd9: begin
                                case(S_n)
                                    4'd0: S_char = digit_char(S_cls_ten);
                                    4'd1: S_char = digit_char(S_cls_one);
                                    default: S_char = CHAR_SPACE;
                                endcase
                            end

                            //------------------------------------------------
                            // LUM:nnn  (improve2 : mean luminance 0 .. 255)
                            //------------------------------------------------
                            4'd10: begin
                                case(S_n)
                                    4'd0: S_char = CHAR_0 + S_lum_dec[11:8];
                                    4'd1: S_char = CHAR_0 + S_lum_dec[7:4];
                                    4'd2: S_char = CHAR_0 + S_lum_dec[3:0];
                                    default: S_char = CHAR_SPACE;
                                endcase
                            end

                            //------------------------------------------------
                            // AE:nnnn Gnn   (improve2 : exposure + gain)
                            //         OFF   when the loop is disabled
                            //------------------------------------------------
                            4'd11: begin
                                if(I_ae_on) begin
                                    case(S_n)
                                        4'd0: S_char = CHAR_0 + S_exp_dec[15:12];
                                        4'd1: S_char = CHAR_0 + S_exp_dec[11:8];
                                        4'd2: S_char = CHAR_0 + S_exp_dec[7:4];
                                        4'd3: S_char = CHAR_0 + S_exp_dec[3:0];
                                        4'd4: S_char = CHAR_SPACE;
                                        4'd5: S_char = CHAR_G;
                                        4'd6: S_char = digit_char(S_gain_ten);
                                        4'd7: S_char = digit_char(S_gain_one);
                                        default: S_char = CHAR_SPACE;
                                    endcase
                                end
                                else begin
                                    case(S_n)
                                        4'd0: S_char = CHAR_O;
                                        4'd1: S_char = CHAR_F;
                                        4'd2: S_char = CHAR_F;
                                        default: S_char = CHAR_SPACE;
                                    endcase
                                end
                            end
                        endcase
                    end

                    // use the registered font bits / column so the overlay
                    // decision is delayed by one stage (matches the video
                    // pipeline below)
                    if(S_valid_r)
                        S_on = S_row_bits_r[7 - S_col_r];

                    if(S_on)
                        S_hit = 1'b1;
                end
            end
        end
    end

    //-------------------------------------------------------------------------
    // Output alignment : coordinates, syncs and data are all delayed by 2
    //-------------------------------------------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x_1d     <= 12'd0;
            S_x_2d     <= 12'd0;
            S_y_1d     <= 12'd0;
            S_y_2d     <= 12'd0;
            S_de_1d    <= 1'b0;
            S_de_2d    <= 1'b0;
            S_vs_1d    <= 1'b0;
            S_vs_2d    <= 1'b0;
            S_hs_1d    <= 1'b0;
            S_hs_2d    <= 1'b0;
            S_data_1d  <= 24'd0;
            S_data_2d  <= 24'd0;
            S_x_3d     <= 12'd0;
            S_y_3d     <= 12'd0;
            S_vs_3d    <= 1'b0;
            S_hs_3d    <= 1'b0;
            S_de_3d    <= 1'b0;
            S_data_3d  <= 24'd0;
            S_row_bits_r  <= 8'd0;
            S_col_r       <= 3'd0;
            S_valid_r     <= 1'b0;
            S_rowsel_r    <= 4'd0;
            S_bar_fill_r  <= 1'b0;
            S_bar_area_r  <= 1'b0;
            S_in_win_r    <= 1'b0;
            O_video_vsync <= 1'b0;
            O_video_hsync <= 1'b0;
            O_video_de    <= 1'b0;
            O_video_data  <= 24'd0;
        end
        else begin
            S_x_1d    <= S_x;
            S_x_2d    <= S_x_1d;
            S_y_1d    <= S_y;
            S_y_2d    <= S_y_1d;

            S_vs_1d   <= I_video_vsync;
            S_vs_2d   <= S_vs_1d;
            S_hs_1d   <= I_video_hsync;
            S_hs_2d   <= S_hs_1d;
            S_de_1d   <= I_video_de;
            S_de_2d   <= S_de_1d;
            S_data_1d <= I_video_data;
            S_data_2d <= S_data_1d;

            // 3rd delay stage (video/sync stream) for the OSD timing pipeline
            S_x_3d    <= S_x_2d;
            S_y_3d    <= S_y_2d;
            S_vs_3d   <= S_vs_2d;
            S_hs_3d   <= S_hs_2d;
            S_de_3d   <= S_de_2d;
            S_data_3d <= S_data_2d;

            // register the OSD decision signals by one stage; the long
            // combinatorial chain (bcd -> char/row select -> font ROM ->
            // S_on -> S_hit) is now split at S_row_bits -> S_row_bits_r
            S_row_bits_r  <= S_row_bits;
            S_col_r       <= S_col;
            S_valid_r     <= S_valid;
            S_rowsel_r    <= S_rowsel;
            S_bar_fill_r  <= S_bar_fill;
            S_bar_area_r  <= S_bar_area;
            S_in_win_r    <= S_in_win;

            O_video_vsync <= S_vs_3d;
            O_video_hsync <= S_hs_3d;
            O_video_de    <= S_de_3d;

            if(S_hit) begin
                case(S_rowsel_r)
                    4'd0: O_video_data <= COLOR_ALG_TEXT;
                    4'd1: O_video_data <= COLOR_STY_TEXT;
                    4'd2: O_video_data <= COLOR_RULE_TEXT;
                    4'd3: O_video_data <= COLOR_MLP_TEXT;
                    4'd4: O_video_data <= COLOR_FPS_TEXT;
                    4'd5: O_video_data <= COLOR_TRK_TEXT;
                    4'd6: O_video_data <= COLOR_DLY_TEXT;
                    4'd7: O_video_data <= COLOR_ALM_TEXT;
                    4'd8:  O_video_data <= COLOR_MOT_TEXT;
                    4'd9:  O_video_data <= COLOR_CLS_TEXT;
                    4'd10: O_video_data <= COLOR_LUM_TEXT;
                    4'd11: O_video_data <= COLOR_AE_TEXT;
                    default: O_video_data <= COLOR_FPS_TEXT;
                endcase
            end
            else if(S_bar_fill_r)
                O_video_data <= S_bar_color;
            else if(S_bar_area_r)
                O_video_data <= COLOR_BAR_BG;
            else if(S_in_win_r)
                // soft dark panel : the video pixel darkened to 1/8
                O_video_data <= {3'b0, S_data_3d[23:3]};
            else
                O_video_data <= S_data_3d;
        end
    end

endmodule
