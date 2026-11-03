//-----------------------------------------------------------------------------
// Module     : osd_mode_overlay
// Description: Second OSD layer.  Draws the current image processing algorithm
//              name and the current threshold on top of the video produced by
//              hdmi_mixer, so that the official hdmi_mixer.v (Anlogic logo +
//              "an lu CNT:") does NOT need to be modified.
//
//   Layout of the OSD window (default at x=375, y=160, 176 x 48):
//
//       row 0 (y 4 .. 19)  : "MODE:" + algorithm name, 16x16 glyphs
//       row 1 (y 28 .. 43) : "TH:"   + threshold value (decimal, 3 digits)
//
//   Algorithm name mapping (I_proc_mode):
//       2'd0 -> RGB      2'd1 -> GRAY     2'd2 -> SOBEL    2'd3 -> BIN
//
//   Threshold (I_th, 0 .. 300) is converted to BCD with bin2bcd10 and printed
//   with leading zeros blanked, e.g. "0", "20", "180", "300".
//
//   Transparency: when the character bitmap bit is 0 the original video pixel
//   is passed through unchanged.
//-----------------------------------------------------------------------------
module osd_mode_overlay #(
    parameter OSD_X = 12'd375,
    parameter OSD_Y = 12'd160,
    parameter OSD_W = 12'd176,
    parameter OSD_H = 12'd48
)(
    input  wire        I_clk,
    input  wire        I_rst_n,

    input  wire        I_video_vsync,
    input  wire        I_video_hsync,
    input  wire        I_video_de,
    input  wire        I_video_user,
    input  wire        I_video_last,

    input  wire [1:0]  I_proc_mode,
    input  wire [8:0]  I_th,
    input  wire [23:0] I_video_data,

    output reg         O_video_vsync,
    output reg         O_video_hsync,
    output reg         O_video_de,
    output reg  [23:0] O_video_data
);

    localparam COLOR_MODE_TEXT = 24'h00ff00;   // bright green
    localparam COLOR_TH_TEXT   = 24'h00ffff;   // cyan

    localparam CHAR_SPACE  = 8'h20;
    localparam CHAR_COLON  = 8'h3a;
    localparam CHAR_M      = 8'h4d;
    localparam CHAR_O      = 8'h4f;
    localparam CHAR_D      = 8'h44;
    localparam CHAR_E      = 8'h45;
    localparam CHAR_R      = 8'h52;
    localparam CHAR_G      = 8'h47;
    localparam CHAR_B      = 8'h42;
    localparam CHAR_A      = 8'h41;
    localparam CHAR_Y      = 8'h59;
    localparam CHAR_S      = 8'h53;
    localparam CHAR_L      = 8'h4c;
    localparam CHAR_I      = 8'h49;
    localparam CHAR_N      = 8'h4e;
    localparam CHAR_T      = 8'h54;
    localparam CHAR_H      = 8'h48;

    //-------------------------------------------------------------------------
    // threshold -> BCD
    //-------------------------------------------------------------------------
    wire [3:0] S_th_hun;
    wire [3:0] S_th_ten;
    wire [3:0] S_th_one;

    bin2bcd10 u_bin2bcd10 (
        .I_bin      ( {1'b0, I_th} ),
        .O_hundreds ( S_th_hun     ),
        .O_tens     ( S_th_ten     ),
        .O_units    ( S_th_one     )
    );

    wire S_show_hun = (S_th_hun != 4'd0);
    wire S_show_ten = (S_th_hun != 4'd0) || (S_th_ten != 4'd0);

    function [7:0] digit_char;
        input [3:0] d;
        begin
            digit_char = 8'h30 + {4'd0, d};
        end
    endfunction

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
    reg [11:0] S_ly_off;
    reg [11:0] S_nx;
    reg [3:0]  S_idx;
    reg [7:0]  S_char;
    reg [3:0]  S_col;
    reg [3:0]  S_row;
    reg [4:0]  S_row5;
    reg        S_valid;
    reg        S_on;
    reg        S_osd_hit;
    reg        S_row1;

    wire [15:0] S_row_bits;

    osd_char_lib u_osd_char_lib(
        .I_char     ( S_char      ),
        .I_row      ( S_row       ),
        .O_row_bits ( S_row_bits  )
    );

    //-------------------------------------------------------------------------
    // Pixel coordinate counter (identical scheme to hdmi_mixer)
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
    // Character rendering (combinational)
    //-------------------------------------------------------------------------
    always @(*) begin
        S_lx       = 12'd0;
        S_ly       = 12'd0;
        S_ly_off   = 12'd0;
        S_nx       = 12'd0;
        S_idx      = 4'd0;
        S_char     = CHAR_SPACE;
        S_col      = 4'd0;
        S_row      = 4'd0;
        S_row5     = 5'd0;
        S_valid    = 1'b0;
        S_on       = 1'b0;
        S_osd_hit  = 1'b0;
        S_row1     = 1'b0;

        if(S_de_2d) begin
            if((S_x_2d >= OSD_X) && (S_x_2d < (OSD_X + OSD_W)) &&
               (S_y_2d >= OSD_Y) && (S_y_2d < (OSD_Y + OSD_H))) begin

                S_lx    = S_x_2d - OSD_X;
                S_ly    = S_y_2d - OSD_Y;

                // two text rows : y 4..19 and y 28..43
                S_row1  = (S_ly >= 12'd24);
                S_ly_off = S_row1 ? (S_ly - 12'd28) : (S_ly - 12'd4);
                S_valid = ((S_ly >= 12'd4)  && (S_ly < 12'd20)) ||
                          ((S_ly >= 12'd28) && (S_ly < 12'd44));
                S_row   = S_ly_off[3:0];

                if(!S_row1) begin
                    //------------------------------------------------------
                    // row 0 : "MODE:" + algorithm name
                    //------------------------------------------------------
                    if(S_lx < 12'd80) begin
                        S_idx  = S_lx[6:4];
                        S_col  = S_lx[3:0];
                        case(S_idx)
                            4'd0:    S_char = CHAR_M;
                            4'd1:    S_char = CHAR_O;
                            4'd2:    S_char = CHAR_D;
                            4'd3:    S_char = CHAR_E;
                            default: S_char = CHAR_COLON;
                        endcase
                    end
                    else if((S_lx >= 12'd88) && (S_lx < 12'd168)) begin
                        S_nx   = S_lx - 12'd88;
                        S_idx  = S_nx[6:4];
                        S_col  = S_nx[3:0];
                        case(I_proc_mode)
                            2'd0: begin                          // RGB
                                case(S_idx)
                                    4'd0:    S_char = CHAR_R;
                                    4'd1:    S_char = CHAR_G;
                                    4'd2:    S_char = CHAR_B;
                                    default: S_char = CHAR_SPACE;
                                endcase
                            end
                            2'd1: begin                          // GRAY
                                case(S_idx)
                                    4'd0:    S_char = CHAR_G;
                                    4'd1:    S_char = CHAR_R;
                                    4'd2:    S_char = CHAR_A;
                                    4'd3:    S_char = CHAR_Y;
                                    default: S_char = CHAR_SPACE;
                                endcase
                            end
                            2'd2: begin                          // SOBEL
                                case(S_idx)
                                    4'd0:    S_char = CHAR_S;
                                    4'd1:    S_char = CHAR_O;
                                    4'd2:    S_char = CHAR_B;
                                    4'd3:    S_char = CHAR_E;
                                    4'd4:    S_char = CHAR_L;
                                    default: S_char = CHAR_SPACE;
                                endcase
                            end
                            default: begin                       // BIN
                                case(S_idx)
                                    4'd0:    S_char = CHAR_B;
                                    4'd1:    S_char = CHAR_I;
                                    4'd2:    S_char = CHAR_N;
                                    default: S_char = CHAR_SPACE;
                                endcase
                            end
                        endcase
                    end
                end
                else begin
                    //------------------------------------------------------
                    // row 1 : "TH:" + decimal threshold
                    //------------------------------------------------------
                    if(S_lx < 12'd48) begin
                        S_idx  = S_lx[6:4];
                        S_col  = S_lx[3:0];
                        case(S_idx)
                            4'd0:    S_char = CHAR_T;
                            4'd1:    S_char = CHAR_H;
                            default: S_char = CHAR_COLON;
                        endcase
                    end
                    else if((S_lx >= 12'd56) && (S_lx < 12'd104)) begin
                        S_nx   = S_lx - 12'd56;
                        S_idx  = S_nx[6:4];
                        S_col  = S_nx[3:0];
                        case(S_idx)
                            4'd0:    S_char = S_show_hun ? digit_char(S_th_hun) : CHAR_SPACE;
                            4'd1:    S_char = S_show_ten ? digit_char(S_th_ten) : CHAR_SPACE;
                            default: S_char = digit_char(S_th_one);
                        endcase
                    end
                end

                if(S_valid)
                    S_on = S_row_bits[15 - S_col];

                if(S_on)
                    S_osd_hit = 1'b1;
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

            O_video_vsync <= S_vs_2d;
            O_video_hsync <= S_hs_2d;
            O_video_de    <= S_de_2d;

            if(S_osd_hit)
                O_video_data <= S_row1 ? COLOR_TH_TEXT : COLOR_MODE_TEXT;
            else
                O_video_data <= S_data_2d;
        end
    end

endmodule
