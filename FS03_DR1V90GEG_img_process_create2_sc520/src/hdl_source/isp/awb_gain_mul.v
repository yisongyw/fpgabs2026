//-----------------------------------------------------------------------------
// Module     : awb_gain_mul
// Description: White balance gain multiplication   O_pix = pix * gain >> 16
//
//   gain is a 20 bit unsigned 4.16 fixed point number (1.0 = 20'h10000).
//
//   The eight low order gain bits (weight 2^-9 .. 2^-16) are dropped, i.e.
//   only gain[19:8] is used.  The resulting error is
//
//        255 * (2^-8)  <  1 LSB of the 8 bit pixel
//
//   which is invisible.  What is left is an 8 bit x 12 bit product:
//
//        product = I_pix * gain[19:8]                (20 bit, never overflows)
//        result  = product >> 8      -> product[15:8]
//        if product[19:16] != 0 the result is saturated to 255
//
//   improve2 (place fix, bit exact) : the product used to be built from a
//   shift-and-add tree ("NO DSP"), which was the right choice when this
//   module was written - at that time the device was short on DSP blocks.
//   The resource balance of improve2 is the opposite : LUTs are the scarce
//   resource (the eight instances cost 8 x 163 = 1304 logic LUT) while
//   plenty of DSP blocks are idle.  The tree is therefore replaced by the
//   arithmetically IDENTICAL '*' operator, which TD maps into one DSP block
//   per instance.  Same numbers in, same numbers out - zero functional
//   change - but 1304 LUT move out of the slices into blocks that were
//   empty anyway.
//-----------------------------------------------------------------------------
module awb_gain_mul (
    input  wire [7:0]  I_pix,
    input  wire [19:0] I_gain,      // 4.16 unsigned fixed point
    output wire [7:0]  O_pix
);

    wire [11:0] S_g = I_gain[19:8];

    // bit exact same product the shift-and-add tree computed
    wire [19:0] S_prod = I_pix * S_g;

    assign O_pix = (S_prod[19:16] != 4'd0) ? 8'hFF : S_prod[15:8];

endmodule
