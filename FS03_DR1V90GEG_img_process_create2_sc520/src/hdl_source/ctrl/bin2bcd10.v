//-----------------------------------------------------------------------------
// Module     : bin2bcd10
// Description: 10 bit binary (0 .. 999) -> three BCD digits.
//
//   Pure combinational comparator chain: no divider, no multiplier, no DSP.
//   Used by the 7 segment display and by the OSD threshold text.
//-----------------------------------------------------------------------------
module bin2bcd10 (
    input  wire [9:0] I_bin,
    output wire [3:0] O_hundreds,
    output wire [3:0] O_tens,
    output wire [3:0] O_units
);

    wire [9:0] S_v = (I_bin > 10'd999) ? 10'd999 : I_bin;

    assign O_hundreds =
        (S_v >= 10'd900) ? 4'd9 :
        (S_v >= 10'd800) ? 4'd8 :
        (S_v >= 10'd700) ? 4'd7 :
        (S_v >= 10'd600) ? 4'd6 :
        (S_v >= 10'd500) ? 4'd5 :
        (S_v >= 10'd400) ? 4'd4 :
        (S_v >= 10'd300) ? 4'd3 :
        (S_v >= 10'd200) ? 4'd2 :
        (S_v >= 10'd100) ? 4'd1 :
                           4'd0 ;

    wire [9:0] S_r1 = S_v - (O_hundreds * 10'd100);

    assign O_tens =
        (S_r1 >= 10'd90) ? 4'd9 :
        (S_r1 >= 10'd80) ? 4'd8 :
        (S_r1 >= 10'd70) ? 4'd7 :
        (S_r1 >= 10'd60) ? 4'd6 :
        (S_r1 >= 10'd50) ? 4'd5 :
        (S_r1 >= 10'd40) ? 4'd4 :
        (S_r1 >= 10'd30) ? 4'd3 :
        (S_r1 >= 10'd20) ? 4'd2 :
        (S_r1 >= 10'd10) ? 4'd1 :
                           4'd0 ;

    wire [9:0] S_r2 = S_r1 - (O_tens * 10'd10);

    assign O_units = S_r2[3:0];

endmodule
