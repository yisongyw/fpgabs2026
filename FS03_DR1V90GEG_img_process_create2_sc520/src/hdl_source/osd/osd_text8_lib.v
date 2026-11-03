//-----------------------------------------------------------------------------
// Module     : osd_text8_lib
// Project    : HX1P35Aimg_process_extend2
// Description: Compact 8 x 8 character generator used by the small status OSD
//              and by the detection overlay.
//
//   The official 16 x 16 font (osd_char_lib.v) is far too big for a status
//   line.  Instead of writing a second font by hand this module re-uses the
//   verified glyphs and down scales every 16 x 16 glyph to 8 x 8:
//   each output pixel is the OR of the corresponding 2 x 2 block, so thin
//   strokes (2 pixel wide in the 16 x 16 font) survive the down scaling.
//
//   The official font only contains A-Z, 0-9 and ':'.  Seven extra glyphs are
//   generated locally, every other character is taken from the official font,
//   therefore the official osd_char_lib.v stays untouched:
//
//        '+'  (0x2B)  "EDGE+FILL"                (extend1)
//        '%'  (0x25)  "MLP:BALL 92%"             (extend2)
//        '.'  (0x2E)  decimal points             (extend2)
//        '('  (0x28)  "(640,360)"                (improve1 - coordinates)
//        ')'  (0x29)  "(640,360)"                (improve1 - coordinates)
//        ','  (0x2C)  "(640,360)"                (improve1 - coordinates)
//        '-'  (0x2D)  "V(+012,-034)"             (improve1 - signed velocity)
//
//   Bit order : O_row_bits[7] = left most pixel (same convention as
//               osd_char_lib, whose bit 15 is the left most pixel).
//-----------------------------------------------------------------------------
module osd_text8_lib (
    input  wire [7:0] I_char,
    input  wire [2:0] I_row,          // 0 .. 7
    output wire [7:0] O_row_bits
);

    localparam CHAR_PLUS  = 8'h2b;
    localparam CHAR_PCT   = 8'h25;
    localparam CHAR_DOT   = 8'h2e;
    localparam CHAR_LBR   = 8'h28;      // '('
    localparam CHAR_RBR   = 8'h29;      // ')'
    localparam CHAR_COMMA = 8'h2c;      // ','
    localparam CHAR_MINUS = 8'h2d;      // '-'

    wire        S_is_plus  = (I_char == CHAR_PLUS);
    wire        S_is_pct   = (I_char == CHAR_PCT);
    wire        S_is_dot   = (I_char == CHAR_DOT);
    wire        S_is_lbr   = (I_char == CHAR_LBR);
    wire        S_is_rbr   = (I_char == CHAR_RBR);
    wire        S_is_comma = (I_char == CHAR_COMMA);
    wire        S_is_minus = (I_char == CHAR_MINUS);

    wire [3:0]  S_row_a = {I_row, 1'b0};      // even source row
    wire [3:0]  S_row_b = {I_row, 1'b1};      // odd  source row

    wire [15:0] S_lib_a;
    wire [15:0] S_lib_b;

    osd_char_lib u_lib_a (
        .I_char     ( I_char  ),
        .I_row      ( S_row_a ),
        .O_row_bits ( S_lib_a )
    );

    osd_char_lib u_lib_b (
        .I_char     ( I_char  ),
        .I_row      ( S_row_b ),
        .O_row_bits ( S_lib_b )
    );

    //-------------------------------------------------------------------------
    // local glyphs are defined on the 8 x 8 grid and then expanded to the
    // 16 x 16 source grid (every pixel is doubled), so that the down scaler
    // below reproduces exactly the intended 8 x 8 pattern.
    //-------------------------------------------------------------------------
    function [15:0] expand8;
        input [7:0] t;
        begin
            expand8 = {{t[7],t[7]}, {t[6],t[6]}, {t[5],t[5]}, {t[4],t[4]},
                       {t[3],t[3]}, {t[2],t[2]}, {t[1],t[1]}, {t[0],t[0]}};
        end
    endfunction

    function [15:0] plus_row;
        input [3:0] r;
        begin
            case (r >> 1)
                3'd3, 3'd4 : plus_row = expand8(8'h3c);
                default    : plus_row = expand8(8'h18);
            endcase
        end
    endfunction

    function [15:0] pct_row;
        input [3:0] r;
        begin
            case (r >> 1)
                3'd0: pct_row = expand8(8'h66);
                3'd1: pct_row = expand8(8'h6c);
                3'd2: pct_row = expand8(8'h0c);
                3'd3: pct_row = expand8(8'h18);
                3'd4: pct_row = expand8(8'h30);
                3'd5: pct_row = expand8(8'h36);
                3'd6: pct_row = expand8(8'h66);
                default: pct_row = 16'd0;
            endcase
        end
    endfunction

    function [15:0] dot_row;
        input [3:0] r;
        begin
            dot_row = ((r >> 1) == 3'd6) ? expand8(8'h03) : 16'd0;
        end
    endfunction

    // '(' : the vertical stroke sits at the left, the ends curl to the right
    function [15:0] lbr_row;
        input [3:0] r;
        begin
            case (r >> 1)
                3'd0:    lbr_row = expand8(8'h18);
                3'd1:    lbr_row = expand8(8'h30);
                3'd2, 3'd3, 3'd4, 3'd5: lbr_row = expand8(8'h60);
                3'd6:    lbr_row = expand8(8'h30);
                default: lbr_row = expand8(8'h18);
            endcase
        end
    endfunction

    // ')' : mirrored
    function [15:0] rbr_row;
        input [3:0] r;
        begin
            case (r >> 1)
                3'd0:    rbr_row = expand8(8'h18);
                3'd1:    rbr_row = expand8(8'h0c);
                3'd2, 3'd3, 3'd4, 3'd5: rbr_row = expand8(8'h06);
                3'd6:    rbr_row = expand8(8'h0c);
                default: rbr_row = expand8(8'h18);
            endcase
        end
    endfunction

    // ',' : bottom right dot with a short tail
    function [15:0] comma_row;
        input [3:0] r;
        begin
            case (r >> 1)
                3'd6:    comma_row = expand8(8'h30);
                default: comma_row = expand8(8'h60);
            endcase
        end
    endfunction

    // '-' : horizontal bar in the middle of the cell
    function [15:0] minus_row;
        input [3:0] r;
        begin
            case (r >> 1)
                3'd3, 3'd4: minus_row = expand8(8'h3c);
                default:    minus_row = 16'd0;
            endcase
        end
    endfunction

    wire [15:0] S_a = S_is_pct   ? pct_row  (S_row_a) :
                      S_is_dot   ? dot_row  (S_row_a) :
                      S_is_plus  ? plus_row (S_row_a) :
                      S_is_lbr   ? lbr_row  (S_row_a) :
                      S_is_rbr   ? rbr_row  (S_row_a) :
                      S_is_comma ? comma_row(S_row_a) :
                      S_is_minus ? minus_row(S_row_a) : S_lib_a;

    wire [15:0] S_b = S_is_pct   ? pct_row  (S_row_b) :
                      S_is_dot   ? dot_row  (S_row_b) :
                      S_is_plus  ? plus_row (S_row_b) :
                      S_is_lbr   ? lbr_row  (S_row_b) :
                      S_is_rbr   ? rbr_row  (S_row_b) :
                      S_is_comma ? comma_row(S_row_b) :
                      S_is_minus ? minus_row(S_row_b) : S_lib_b;

    genvar k;
    generate
        for (k = 0; k < 8; k = k + 1) begin : g_downscale
            // OR of the 2 x 2 block ( even/odd column, even/odd row )
            assign O_row_bits[7 - k] = S_a[15 - 2*k]     | S_a[15 - (2*k + 1)] |
                                       S_b[15 - 2*k]     | S_b[15 - (2*k + 1)];
        end
    endgenerate

endmodule
