//-----------------------------------------------------------------------------
// Module     : mask3x3   (helper of obj_detect)
// Project    : HX1P35Aimg_process_extend2
// Description: Streaming 3x3 neighbourhood generator for a 1 bit (binary) mask,
//              4 pixels per beat.
//
//   Only ONE bit per pixel has to be stored, therefore the two line buffers
//   are extremely small:
//
//        2 x 320 words x 4 bit = 2.56 kbit per instance
//
//   (compare with 20 kbit for the 8 bit grayscale window of img_proc_top).
//
//   The output lags the input by exactly one beat, exactly like the grayscale
//   window of img_proc_top:
//
//        beat k-2 -> p2    : column 4w-5
//        beat k-1 -> *_w_d : columns 4w-4 .. 4w-1
//        beat k   -> nx_*  : column 4w   (combinational read of the line RAM)
//
//   O_w0 .. O_w3 are packed as {a00,a01,a02, a10,a11,a12, a20,a21,a22}
//   where a00 is the TOP LEFT neighbour and a22 the BOTTOM RIGHT one.
//-----------------------------------------------------------------------------
module mask3x3 #(
    parameter IMG_WIDTH_GRP = 320
)(
    input  wire             I_clk,
    input  wire             I_en,
    input  wire [8:0]       I_xgrp,
    input  wire [3:0]       I_mask4,        // [0] = left most pixel
    output wire [8:0]       O_w0,
    output wire [8:0]       O_w1,
    output wire [8:0]       O_w2,
    output wire [8:0]       O_w3
);

    reg [3:0] l1_mem [0:IMG_WIDTH_GRP-1];   // row N-1
    reg [3:0] l2_mem [0:IMG_WIDTH_GRP-1];   // row N-2

    wire [3:0] top_word = l2_mem[I_xgrp];   // row N-2, columns 4w .. 4w+3
    wire [3:0] mid_word = l1_mem[I_xgrp];   // row N-1
    wire [3:0] bot_word = I_mask4;          // row N

    reg [3:0] top_w_d, mid_w_d, bot_w_d;
    reg       top_p2,  mid_p2,  bot_p2;

    wire nx_top = top_word[3];              // column 4w of row N-2
    wire nx_mid = mid_word[3];
    wire nx_bot = bot_word[3];

    wire [5:0] wx_top = {top_p2, top_w_d, nx_top};
    wire [5:0] wx_mid = {mid_p2, mid_w_d, nx_mid};
    wire [5:0] wx_bot = {bot_p2, bot_w_d, nx_bot};

    // hand written instead of a generate loop so that the bit order stays
    // obvious and readable:  centre = wx[j+1], left = wx[j+2], right = wx[j]
    assign O_w0 = { wx_top[5], wx_top[4], wx_top[3],
                    wx_mid[5], wx_mid[4], wx_mid[3],
                    wx_bot[5], wx_bot[4], wx_bot[3] };

    assign O_w1 = { wx_top[4], wx_top[3], wx_top[2],
                    wx_mid[4], wx_mid[3], wx_mid[2],
                    wx_bot[4], wx_bot[3], wx_bot[2] };

    assign O_w2 = { wx_top[3], wx_top[2], wx_top[1],
                    wx_mid[3], wx_mid[2], wx_mid[1],
                    wx_bot[3], wx_bot[2], wx_bot[1] };

    assign O_w3 = { wx_top[2], wx_top[1], wx_top[0],
                    wx_mid[2], wx_mid[1], wx_mid[0],
                    wx_bot[2], wx_bot[1], wx_bot[0] };

    always @(posedge I_clk) begin
        if(I_en) begin
            top_w_d <= top_word;
            mid_w_d <= mid_word;
            bot_w_d <= bot_word;

            top_p2  <= top_w_d[0];      // column 4w-5 = last pixel of BO_t_d
            mid_p2  <= mid_w_d[0];
            bot_p2  <= bot_w_d[0];

            l2_mem[I_xgrp] <= mid_word;
            l1_mem[I_xgrp] <= bot_word;
        end
    end

endmodule
