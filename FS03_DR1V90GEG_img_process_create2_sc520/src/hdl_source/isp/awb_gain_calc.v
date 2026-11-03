//-----------------------------------------------------------------------------
// Module     : awb_gain_calc
// Description: Unsigned division  gain = (I_num << 16) / I_den
//              implemented as a bit serial restoring divider.
//
//   This module replaces the "divider" IP core that the reference AWB used
//   twice (once per channel).  The IP is a fully parallel 48 bit divider and
//   costs about 640 LUT + 2800 registers per instance; this implementation
//   costs roughly 60 LUT + 140 registers and is shared by both channels.
//
//   Result format : 4.16 unsigned fixed point (1.0 = 20'h10000),
//                   saturated at 20'hFFFFF (15.9999).
//
//   Latency : 48 clock cycles (one bit of the 48 bit numerator per cycle).
//   The gain only changes once per frame, so the latency is irrelevant.
//-----------------------------------------------------------------------------
module awb_gain_calc (
    input  wire        I_clk,
    input  wire        I_rst_n,

    input  wire        I_start,        // 1 clock pulse
    input  wire [31:0] I_num,          // numerator   (sum_g)
    input  wire [31:0] I_den,          // denominator (sum_r / sum_b)

    output reg  [19:0] O_gain,         // 4.16, saturates at 20'hFFFFF
    output reg         O_done,
    output wire        O_busy
);

    reg        r_busy;
    reg [5:0]  r_cnt;
    reg [47:0] r_num;                  // {I_num, 16'd0}, shifted left each step
    reg [47:0] r_quot;
    reg [32:0] r_rem;                  // always < I_den, so 33 bit is enough

    assign O_busy = r_busy;

    wire [32:0] S_den    = {1'b0, I_den};
    wire [32:0] S_rem_sh = {r_rem[31:0], r_num[47]};
    wire        S_ge     = (S_rem_sh >= S_den);
    wire [32:0] S_rem_nx = S_ge ? (S_rem_sh - S_den) : S_rem_sh;
    wire [47:0] S_quot_nx = {r_quot[46:0], S_ge};

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            r_busy  <= 1'b0;
            r_cnt   <= 6'd0;
            r_num   <= 48'd0;
            r_quot  <= 48'd0;
            r_rem   <= 33'd0;
            O_done  <= 1'b0;
            O_gain  <= 20'h10000;      // neutral gain 1.0
        end
        else begin
            O_done <= 1'b0;

            if(!r_busy) begin
                if(I_start) begin
                    r_busy <= 1'b1;
                    r_cnt  <= 6'd0;
                    r_num  <= {I_num, 16'd0};
                    r_quot <= 48'd0;
                    r_rem  <= 33'd0;
                end
            end
            else begin
                r_num  <= {r_num[46:0], 1'b0};
                r_quot <= S_quot_nx;
                r_rem  <= S_rem_nx;
                r_cnt  <= r_cnt + 6'd1;

                if(r_cnt == 6'd47) begin
                    r_busy <= 1'b0;
                    O_done <= 1'b1;
                    // saturate anything that does not fit into 4.16
                    O_gain <= (S_quot_nx[47:20] != 28'd0) ? 20'hFFFFF
                                                          : S_quot_nx[19:0];
                end
            end
        end
    end

endmodule
