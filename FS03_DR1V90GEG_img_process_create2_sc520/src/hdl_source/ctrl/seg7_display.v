//-----------------------------------------------------------------------------
// Module     : seg7_display
// Description: 8 digit, common anode 7 segment display driver for the
//              HX1P35A board.
//
//   Board wiring (新HX1P35A开发板手册-HDL版2604, chapter 8/11):
//       DIG0 .. DIG7 = a, b, c, d, e, f, g, h(DP)
//           C9 / C11 / B11 / E7 / E6 / B6 / B7 / E5
//       SEL0 .. SEL7 = digit 1(left most) .. digit 8(right most)
//           G15 / G14 / A17 / A18 / B16 / A15 / B15 / A14
//
//   The display is a COMMON ANODE type:
//       segment pin = 0 -> segment lit      (output is active low)
//       select  pin = 1 -> digit powered    (output is active high)
//
//   Value layout : the number is right aligned, i.e. the least significant
//   digit is the right most tube (SEL7).  Leading zeros are blanked.
//
//       SEL0 SEL1 SEL2 SEL3 SEL4   SEL5     SEL6   SEL7
//        -    -    -    -    -    hundreds  tens   units
//
//   Dynamic scan: one digit per 2^SCAN_SHIFT clock cycles.
//   With a 100 MHz clock and SCAN_SHIFT = 14 the frame rate is
//   100e6 / (2^14 * 8) = 763 Hz, well above the 50 Hz flicker limit.
//
//   NOTE: the tube pins are shared with the 8 dual colour LEDs.  SW6 on the
//   board must be switched UP ("1") to connect the tubes.
//-----------------------------------------------------------------------------
module seg7_display #(
    parameter SCAN_SHIFT = 14
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire [9:0]  I_value,      // 0 .. 999

    // create1 addition : peripheral status view on the dual colour LEDs.
    // The 8 dual colour LED groups share the SEL/DIG bus (SW6 selects the
    // LEDs instead of the tubes).  With I_status_en = 1 the bus statically
    // encodes the UART / Ethernet / SD status (see innov/periph_status.v):
    //     DIG[i] (low active) = green lamp of group i
    //     SEL[i] (high active) = red lamp of group i
    input  wire        I_status_en,
    input  wire [7:0]  I_status_g,
    input  wire [7:0]  I_status_r,

    output reg  [7:0]  O_seg_dig,    // DIG0..DIG7 = a,b,c,d,e,f,g,dp  (low active)
    output reg  [7:0]  O_seg_sel     // SEL0..SEL7                     (high active)
);

    localparam BLANK = 4'd15;

    //-------------------------------------------------------------------------
    // binary -> BCD
    //-------------------------------------------------------------------------
    wire [3:0] S_hun;
    wire [3:0] S_ten;
    wire [3:0] S_one;

    bin2bcd10 u_bin2bcd10 (
        .I_bin      ( I_value ),
        .O_hundreds ( S_hun   ),
        .O_tens     ( S_ten   ),
        .O_units    ( S_one   )
    );

    // leading zero blanking
    wire       S_show_hun = (S_hun != 4'd0);
    wire       S_show_ten = (S_hun != 4'd0) || (S_ten != 4'd0);

    //-------------------------------------------------------------------------
    // scan counter
    //-------------------------------------------------------------------------
    reg [19:0] r_cnt;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            r_cnt <= 20'd0;
        else
            r_cnt <= r_cnt + 20'd1;
    end

    wire [2:0] S_digit_idx = r_cnt[SCAN_SHIFT +: 3];

    //-------------------------------------------------------------------------
    // digit mux (SEL0 = left most, SEL7 = right most)
    //-------------------------------------------------------------------------
    reg [3:0] S_digit;

    always @(*) begin
        case(S_digit_idx)
            3'd5:    S_digit = S_show_hun ? S_hun : BLANK;
            3'd6:    S_digit = S_show_ten ? S_ten : BLANK;
            3'd7:    S_digit = S_one;
            default: S_digit = BLANK;
        endcase
    end

    //-------------------------------------------------------------------------
    // segment decoder, common anode (active low)
    //        bit0 = a, bit1 = b, ... bit6 = g, bit7 = dp
    //-------------------------------------------------------------------------
    function [7:0] seg7_enc;
        input [3:0] d;
        begin
            case(d)
                4'd0:    seg7_enc = 8'hC0;
                4'd1:    seg7_enc = 8'hF9;
                4'd2:    seg7_enc = 8'hA4;
                4'd3:    seg7_enc = 8'hB0;
                4'd4:    seg7_enc = 8'h99;
                4'd5:    seg7_enc = 8'h92;
                4'd6:    seg7_enc = 8'h82;
                4'd7:    seg7_enc = 8'hF8;
                4'd8:    seg7_enc = 8'h80;
                4'd9:    seg7_enc = 8'h90;
                default: seg7_enc = 8'hFF;   // blank
            endcase
        end
    endfunction

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_seg_dig <= 8'hFF;
            O_seg_sel <= 8'h00;
        end
        else if(I_status_en) begin
            // dual colour LED status view (no scanning needed)
            O_seg_dig <= ~I_status_g;   // green lamp, low active
            O_seg_sel <=  I_status_r;   // red lamp, high active
        end
        else begin
            O_seg_dig <= seg7_enc(S_digit);
            O_seg_sel <= (8'd1 << S_digit_idx);
        end
    end

endmodule
