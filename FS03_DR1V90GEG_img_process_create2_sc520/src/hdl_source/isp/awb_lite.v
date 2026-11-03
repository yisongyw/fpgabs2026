//-----------------------------------------------------------------------------
// Module     : awb_lite
// Description: Grey world auto white balance - resource optimised rewrite of
//              the reference "awb" module.
//
//   Differences to the reference awb.v (which is left untouched):
//
//   1) the gain is no longer produced by two "divider" IP cores (each about
//      640 LUT + 2800 registers, started on EVERY pixel beat).  One shared
//      bit serial divider (awb_gain_calc, ~60 LUT + 140 registers) computes
//      both channel gains once per frame.
//
//   2) the eight  20 bit x 8 bit  gain multipliers (8 DSP blocks, i.e. 8 of
//      the 9 DSP blocks the whole base1 design used) are replaced by
//      awb_gain_mul, a shift-and-add constant coefficient datapath.
//      -> the image processing pipeline now uses 0 DSP blocks.
//
//   The algorithm itself is unchanged:
//       sum_r/g/b accumulate the (non saturated) pixels of the frame,
//       gain_r = sum_g / sum_r , gain_b = sum_g / sum_b   (4.16 fixed point)
//       R and B channels are multiplied by the gain, G is passed through,
//       the result is clipped to 255.
//
//   Timing note: the gains are updated once per frame (about 100 clock cycles
//   after the frame start pulse), which is exactly how a real AWB loop works
//   and is invisible to the eye.
//-----------------------------------------------------------------------------
module awb_lite #(
    parameter IMG_HEIGHT = 720,
    parameter IMG_WIDTH  = 1280
)(
    input             I_clk,
    input             I_rst_n,

    input             I_tlast,
    input             I_tuser,
    input  [95:0]     I_tdata,
    input             I_tvalid,
    output            I_tready,

    output            O_tlast,
    output            O_tuser,
    output [95:0]     O_tdata,
    output            O_tvalid,
    input             O_tready
);

    //-------------------------------------------------------------------------
    // input pipeline
    //-------------------------------------------------------------------------
    reg         I_tvalid_r0, I_tvalid_r1;
    reg  [95:0] I_tdata_d0,  I_tdata_d1;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            I_tdata_d0  <= 96'd0;
            I_tdata_d1  <= 96'd0;
            I_tvalid_r0 <= 1'b0;
            I_tvalid_r1 <= 1'b0;
        end
        else begin
            I_tdata_d0  <= I_tdata;
            I_tdata_d1  <= I_tdata_d0;
            I_tvalid_r0 <= I_tvalid;
            I_tvalid_r1 <= I_tvalid_r0;
        end
    end

    //-------------------------------------------------------------------------
    // channel sums (pixel 0 of every beat, same decimation as the reference)
    //-------------------------------------------------------------------------
    wire [7:0] S_r_in = I_tdata[16 +: 8];
    wire [7:0] S_g_in = I_tdata[ 8 +: 8];
    wire [7:0] S_b_in = I_tdata[ 0 +: 8];

    reg [9:0]  rgb_sum;                 // R+G+B of the current pixel 0

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            rgb_sum <= 10'd0;
        else if(I_tvalid)
            rgb_sum <= S_r_in + S_g_in + S_b_in;
    end

    wire [7:0] S_r_d0 = I_tdata_d0[16 +: 8];
    wire [7:0] S_g_d0 = I_tdata_d0[ 8 +: 8];
    wire [7:0] S_b_d0 = I_tdata_d0[ 0 +: 8];

    reg [31:0] sum_r, sum_g, sum_b;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n || I_tuser) begin
            sum_r <= 32'd1;
            sum_g <= 32'd1;
            sum_b <= 32'd1;
        end
        else if (I_tvalid_r0 && (rgb_sum < 10'd720)) begin   // suppress white
            sum_r <= sum_r + S_r_d0;
            sum_g <= sum_g + S_g_d0;
            sum_b <= sum_b + S_b_d0;
        end
    end

    //-------------------------------------------------------------------------
    // gain estimation : one division per channel and per frame
    //-------------------------------------------------------------------------
    reg [19:0] gain_r, gain_b;          // 4.16 fixed point
    reg [31:0] snap_r, snap_g, snap_b;
    reg        calc_ch;                 // 0 = R channel, 1 = B channel
    reg        calc_start;
    wire       calc_done;
    wire       calc_busy;
    wire [19:0] calc_gain;

    localparam ST_IDLE = 2'd0;
    localparam ST_R    = 2'd1;
    localparam ST_B    = 2'd2;

    reg [1:0]  state;

    awb_gain_calc u_awb_gain_calc (
        .I_clk   ( I_clk                            ),
        .I_rst_n ( I_rst_n                          ),
        .I_start ( calc_start                       ),
        .I_num   ( snap_g                           ),
        .I_den   ( calc_ch ? snap_b : snap_r        ),
        .O_gain  ( calc_gain                        ),
        .O_done  ( calc_done                        ),
        .O_busy  ( calc_busy                        )
    );

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            state   <= ST_IDLE;
            calc_ch <= 1'b0;
            snap_r  <= 32'd1;
            snap_g  <= 32'd1;
            snap_b  <= 32'd1;
            gain_r  <= 20'h10000;
            gain_b  <= 20'h10000;
        end
        else begin
            case(state)
                ST_IDLE : if(I_tuser) begin
                    // snapshot of the finished frame, the accumulators are
                    // cleared by I_tuser in the block above
                    snap_r  <= sum_r;
                    snap_g  <= sum_g;
                    snap_b  <= sum_b;
                    calc_ch <= 1'b0;
                    state   <= ST_R;
                end
                ST_R    : if(calc_done) begin
                    gain_r  <= calc_gain;
                    calc_ch <= 1'b1;
                    state   <= ST_B;
                end
                ST_B    : if(calc_done) begin
                    gain_b  <= calc_gain;
                    state   <= ST_IDLE;
                end
            endcase
        end
    end

    // registered start pulse, aligned with the state machine
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            calc_start <= 1'b0;
        else
            calc_start <= ((state == ST_IDLE) && I_tuser) ||
                          ((state == ST_R)    && calc_done);
    end

    //-------------------------------------------------------------------------
    // pixel delay : the gain of the running frame must not be applied to the
    // pixels that produced it (50 beats, identical to the reference design)
    //-------------------------------------------------------------------------
    wire         valid_d0;
    wire         I_tuser_r0;
    wire [95:0]  data_d0;

    signal_delay #(
        .IMG_HEIGHT   ( IMG_HEIGHT ),
        .IMG_WIDTH    ( IMG_WIDTH  ),
        .DATA_WIDTH   ( 96         ),
        .DELAY_CYCLE  ( 50         )
    ) signal_delay_d (
        .I_clk   ( I_clk        ),
        .I_rst_n ( I_rst_n      ),
        .I_tuser ( I_tuser      ),
        .I_valid ( I_tvalid_r1  ),
        .I_data  ( I_tdata_d1   ),
        .O_valid ( valid_d0     ),
        .O_data  ( data_d0      ),
        .O_tuser ( I_tuser_r0   )
    );

    reg         valid_d1, I_tuser_r1;
    reg [95:0]  data_d1;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            valid_d1   <= 1'b0;
            data_d1    <= 96'd0;
            I_tuser_r1 <= 1'b0;
        end
        else begin
            valid_d1   <= valid_d0;
            data_d1    <= data_d0;
            I_tuser_r1 <= I_tuser_r0;
        end
    end

    //-------------------------------------------------------------------------
    // unpack the 4 pixels (RGB888) of the delayed beat
    //-------------------------------------------------------------------------
    wire [7:0] S_r_d [0:3];
    wire [7:0] S_g_d [0:3];
    wire [7:0] S_b_d [0:3];

    assign S_r_d[0] = data_d1[16 +: 8];
    assign S_g_d[0] = data_d1[ 8 +: 8];
    assign S_b_d[0] = data_d1[ 0 +: 8];

    assign S_r_d[1] = data_d1[40 +: 8];
    assign S_g_d[1] = data_d1[32 +: 8];
    assign S_b_d[1] = data_d1[24 +: 8];

    assign S_r_d[2] = data_d1[64 +: 8];
    assign S_g_d[2] = data_d1[56 +: 8];
    assign S_b_d[2] = data_d1[48 +: 8];

    assign S_r_d[3] = data_d1[88 +: 8];
    assign S_g_d[3] = data_d1[80 +: 8];
    assign S_b_d[3] = data_d1[72 +: 8];

    // DSP free gain multiplication, 8 instances
    wire [7:0] S_r_out [0:3];
    wire [7:0] S_b_out [0:3];

    genvar k;
    generate
        for (k = 0; k < 4; k = k + 1) begin : g_gain_mul
            awb_gain_mul u_mul_r (
                .I_pix  ( S_r_d[k]  ),
                .I_gain ( gain_r    ),
                .O_pix  ( S_r_out[k])
            );
            awb_gain_mul u_mul_b (
                .I_pix  ( S_b_d[k]  ),
                .I_gain ( gain_b    ),
                .O_pix  ( S_b_out[k])
            );
        end
    endgenerate

    //-------------------------------------------------------------------------
    // output registers : R/B gain corrected, G passed through
    //-------------------------------------------------------------------------
    reg [7:0] R_new_x1, G_new_x1, B_new_x1;
    reg [7:0] R_new_x2, G_new_x2, B_new_x2;
    reg [7:0] R_new_x3, G_new_x3, B_new_x3;
    reg [7:0] R_new_x4, G_new_x4, B_new_x4;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            R_new_x1 <= 8'd0;  G_new_x1 <= 8'd0;  B_new_x1 <= 8'd0;
            R_new_x2 <= 8'd0;  G_new_x2 <= 8'd0;  B_new_x2 <= 8'd0;
            R_new_x3 <= 8'd0;  G_new_x3 <= 8'd0;  B_new_x3 <= 8'd0;
            R_new_x4 <= 8'd0;  G_new_x4 <= 8'd0;  B_new_x4 <= 8'd0;
        end
        else if(valid_d1) begin
            R_new_x1 <= S_r_out[0];  G_new_x1 <= S_g_d[0];  B_new_x1 <= S_b_out[0];
            R_new_x2 <= S_r_out[1];  G_new_x2 <= S_g_d[1];  B_new_x2 <= S_b_out[1];
            R_new_x3 <= S_r_out[2];  G_new_x3 <= S_g_d[2];  B_new_x3 <= S_b_out[2];
            R_new_x4 <= S_r_out[3];  G_new_x4 <= S_g_d[3];  B_new_x4 <= S_b_out[3];
        end
    end

    //-------------------------------------------------------------------------
    // output handshake
    //-------------------------------------------------------------------------
    reg valid_d2, I_tuser_r2;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            valid_d2   <= 1'b0;
            I_tuser_r2 <= 1'b0;
        end
        else begin
            valid_d2   <= valid_d1;
            I_tuser_r2 <= I_tuser_r1;
        end
    end

    assign O_tlast  = (valid_d1 == 1'b0) && (valid_d2 == 1'b1);
    assign O_tuser  = I_tuser;
    assign O_tvalid = valid_d2;
    assign O_tdata  = O_tvalid ? {R_new_x4, G_new_x4, B_new_x4,
                                  R_new_x3, G_new_x3, B_new_x3,
                                  R_new_x2, G_new_x2, B_new_x2,
                                  R_new_x1, G_new_x1, B_new_x1} : 96'd0;
    assign I_tready = O_tready;

endmodule
