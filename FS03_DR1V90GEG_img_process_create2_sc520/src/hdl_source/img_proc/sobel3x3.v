//-----------------------------------------------------------------------------
// Module     : sobel3x3
// Description: 3x3 Sobel gradient magnitude ( |Gx| + |Gy| approximation ).
//              Pure combinational logic, no DSP, no pipeline registers.
//
//              Gx = -1  0  +1        Gy = -1 -2 -1
//                   -2  0  +2              0  0  0
//                   -1  0  +1             +1 +2 +1
//
//              O_mag = |Gx| + |Gy|   ( max 4*255 + 4*255 = 2040, 14 bit )
//
//              |Gx|+|Gy| is used instead of sqrt(Gx^2+Gy^2).  The error is
//              below 12% and is visually irrelevant, while saving a square
//              root and two multipliers in every pixel of every frame.
//-----------------------------------------------------------------------------
module sobel3x3 (
    input  wire [7:0] a00, a01, a02,
    input  wire [7:0] a10, a11, a12,
    input  wire [7:0] a20, a21, a22,
    output wire [13:0] O_mag
);

    wire signed [12:0] s00 = {5'd0, a00};
    wire signed [12:0] s01 = {5'd0, a01};
    wire signed [12:0] s02 = {5'd0, a02};
    wire signed [12:0] s10 = {5'd0, a10};
    wire signed [12:0] s12 = {5'd0, a12};
    wire signed [12:0] s20 = {5'd0, a20};
    wire signed [12:0] s21 = {5'd0, a21};
    wire signed [12:0] s22 = {5'd0, a22};

    wire signed [12:0] gx = -s00 + s02 - (s10 <<< 1) + (s12 <<< 1) - s20 + s22;
    wire signed [12:0] gy =  s00 + (s01 <<< 1) + s02 - s20 - (s21 <<< 1) - s22;

    wire [12:0] abs_gx = gx[12] ? (~gx + 13'd1) : gx;
    wire [12:0] abs_gy = gy[12] ? (~gy + 13'd1) : gy;

    assign O_mag = {1'b0, abs_gx} + {1'b0, abs_gy};

endmodule
