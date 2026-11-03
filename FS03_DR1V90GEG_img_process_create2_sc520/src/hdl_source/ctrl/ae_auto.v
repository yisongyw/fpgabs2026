//-----------------------------------------------------------------------------
// Module     : ae_auto
// Project    : HX1P35Aimg_process_improve2
// Description: Closed loop automatic exposure for the SC520CS sensor
//              (advanced requirement 2.5-5 : auto exposure / white balance /
//              histogram equalisation).
//
//   Why the sensor and not the FPGA ?
//   ---------------------------------
//   Exposure happens INSIDE the sensor.  By the time the FPGA sees a pixel the
//   exposure has already happened, a too bright picture can only be darkened
//   again with a loss of the highlight information.  The loop therefore writes
//   the exposure and the gain registers of the SC520CS over the SCCB (I2C)
//   interface that the reference design already uses (ae_set -> uicfgcs520).
//
//   Control law (see PH1P35实时图像处理进阶详细方案.md chapter 8.3)
//   ---------------------------------------------------------------
//        e_k    = Yref - Yavg                      Yref = 118
//        dExp   = clamp(e_k / 4, -dMax, +dMax)     dMax = (EXP_MAX-EXP_MIN)/16
//        Exp    = clamp(Exp + dExp, EXP_MIN, EXP_MAX)
//
//   Division free : ae_auto never computes Yavg.  hist_stat delivers the exact
//   SUM of all sampled luminance values together with the number of samples, so
//   the error is formed as
//
//        err_sum = Yref * nsamp - sum_y      (Yref * nsamp is a shift-add)
//
//   and the step is taken as (err_sum >> 20) which is err/4.55 for the nominal
//   nsamp = 230400 - close enough for a feedback loop that is re-evaluated
//   every 8 frames anyway.  Not a single divider is instantiated.
//
//   Three anti oscillation measures
//   -------------------------------
//        dead zone   : |err| <= 6 LSB      -> no action
//        step limit  : one step <= 1/16 of the exposure range
//        rate limit  : one update every 8 frames (133 ms)
//
//   Highlight protection (the feature that makes AE actually usable)
//   ---------------------------------------------------------------
//        over  = number of pixels with Y >= 250
//        if over > nsamp/32 (3.1 %) the exposure is forced DOWN regardless of
//        the mean value : a bright window in the picture must not be burnt out
//        just because the rest of the scene is dark.
//        Symmetrically, under > nsamp/32 forces the exposure UP.
//
//   Exposure / gain split
//   ---------------------
//        The exposure is the preferred actuator because it does not add noise.
//        Only when the exposure hits its ceiling (EXP_MAX = 1125 row periods
//        = 25 ms, which keeps the end to end latency inside 40 ms) the gain is
//        increased by one step per update, and vice versa at the floor.
//-----------------------------------------------------------------------------
module ae_auto #(
    parameter EXP_MIN   = 16'd64,     // row periods
    parameter EXP_MAX   = 16'd1125,   // 1125 x 22.2 us = 25 ms -> latency limit
    parameter GAIN_MIN  = 16'd16,
    parameter GAIN_MAX  = 16'd89,
    parameter YREF      = 8'd118,
    parameter DEAD      = 8'd6,
    parameter FRAMES    = 4'd8,       // one update every 8 frames (133 ms)
    parameter EXP_INIT  = 16'd1125,
    parameter GAIN_INIT = 16'd32
)(
    input  wire        I_clk,
    input  wire        I_rst_n,

    input  wire        I_en,          // 1 : the loop runs (0 : hold, AE frozen)
    input  wire [31:0] I_sum_y,       // sum of all sampled Y of the last frame
    input  wire [17:0] I_nsamp,       // number of samples (0 = no frame)
    input  wire [17:0] I_over,        // pixels with Y >= 250
    input  wire [17:0] I_under,       // pixels with Y <= 8
    input  wire        I_valid,       // 1 clock pulse at every frame start

    output reg  [15:0] O_exp,
    output reg  [15:0] O_gain
);

    //-------------------------------------------------------------------------
    // error formation : ref_sum = YREF * nsamp = nsamp*(64+32+16+4+2)
    //-------------------------------------------------------------------------
    wire [31:0] S_ref = ({14'd0, I_nsamp} << 6) + ({14'd0, I_nsamp} << 5)
                      + ({14'd0, I_nsamp} << 4) + ({14'd0, I_nsamp} << 2)
                      + ({14'd0, I_nsamp} << 1);

    wire signed [32:0] S_err = $signed({1'b0, S_ref}) - $signed({1'b0, I_sum_y});

    wire [31:0] S_abs   = S_err[32] ? (~S_err[31:0] + 32'd1) : S_err[31:0];
    wire [31:0] S_dead  = ({14'd0, I_nsamp} << 2) + ({14'd0, I_nsamp} << 1);

    // step : err_sum >> 20  ~=  err / 4.55
    wire [31:0] S_step_raw = S_abs >> 20;

    localparam [15:0] DMAX = (EXP_MAX - EXP_MIN) >> 4;   // 1/16 of the range

    wire [15:0] S_step = (S_step_raw > {16'd0, DMAX}) ? DMAX : S_step_raw[15:0];

    wire S_too_dark  = (S_err[32] == 1'b0) && (S_abs > S_dead);   // Yavg < Yref
    wire S_too_brigh = (S_err[32] == 1'b1) && (S_abs > S_dead);   // Yavg > Yref

    // highlight / shadow protection : 3.1 % of all samples
    wire [17:0] S_tail   = I_nsamp >> 5;
    wire        S_over   = (I_nsamp > 18'd0) && (I_over  > S_tail);
    wire        S_under  = (I_nsamp > 18'd0) && (I_under > S_tail);

    reg [3:0] r_fcnt;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_exp   <= EXP_INIT;
            O_gain  <= GAIN_INIT;
            r_fcnt  <= 4'd0;
        end
        else if(I_valid) begin
            if(r_fcnt != 4'd0)
                r_fcnt <= r_fcnt - 4'd1;
            else begin
                r_fcnt <= FRAMES - 4'd1;

                if(!I_en) begin
                    // AE disabled : the exposure is frozen, which is exactly
                    // what the "one key fallback" of the demo needs
                    O_exp  <= O_exp;
                    O_gain <= O_gain;
                end
                else if(S_over) begin
                    // highlight protection : force the exposure down
                    if(O_exp > (EXP_MIN + DMAX)) O_exp <= O_exp - DMAX;
                    else begin
                        O_exp <= EXP_MIN;
                        if(O_gain > GAIN_MIN) O_gain <= O_gain - 16'd1;
                    end
                end
                else if(S_under) begin
                    if(O_exp < (EXP_MAX - DMAX)) O_exp <= O_exp + DMAX;
                    else begin
                        O_exp <= EXP_MAX;
                        if(O_gain < GAIN_MAX) O_gain <= O_gain + 16'd1;
                    end
                end
                else if(S_too_dark) begin
                    if(O_exp < (EXP_MAX - S_step)) O_exp <= O_exp + S_step;
                    else begin
                        O_exp <= EXP_MAX;
                        if(O_gain < GAIN_MAX) O_gain <= O_gain + 16'd1;
                    end
                end
                else if(S_too_brigh) begin
                    if(O_exp > (EXP_MIN + S_step)) O_exp <= O_exp - S_step;
                    else begin
                        O_exp <= EXP_MIN;
                        if(O_gain > GAIN_MIN) O_gain <= O_gain - 16'd1;
                    end
                end
            end
        end
    end

endmodule
