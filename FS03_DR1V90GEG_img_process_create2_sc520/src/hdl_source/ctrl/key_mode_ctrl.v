//-----------------------------------------------------------------------------
// Module     : key_mode_ctrl
// Description: Debounced key input that increments a value on every key
//              press.  Used for the threshold step selection (16 steps).
//
//   I_key is active low (board pull-up).
//   CNT_MAX is the number of clock cycles the key state has to be stable.
//   With a 100 MHz clock, CNT_MAX = 20'd999999 gives about 10 ms.
//
//   WIDTH : width of the counter
//   MOD   : the counter wraps back to 0 after MOD-1
//
//   Behaviour : one increment per press, no auto repeat while held down,
//               no false trigger caused by bouncing.
//-----------------------------------------------------------------------------
module key_mode_ctrl #(
    parameter CNT_MAX = 20'd999999,
    parameter WIDTH   = 4,
    parameter MOD     = 16
)(
    input  wire             I_clk,
    input  wire             I_rst_n,
    input  wire             I_key,          // active low
    output reg [WIDTH-1:0]  O_mode
);

    localparam [WIDTH-1:0] WRAP = MOD - 1;

    reg [1:0]  r_key_sync;
    reg [19:0] r_cnt;
    reg        r_pressed;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            r_key_sync <= 2'b11;
            r_cnt      <= 20'd0;
            r_pressed  <= 1'b1;
            O_mode     <= {WIDTH{1'b0}};
        end
        else begin
            r_key_sync <= {r_key_sync[0], I_key};

            if(r_key_sync[1] != r_pressed) begin
                if(r_cnt == CNT_MAX) begin
                    r_cnt     <= 20'd0;
                    r_pressed <= r_key_sync[1];
                    if(!r_key_sync[1])
                        O_mode <= (O_mode == WRAP) ? {WIDTH{1'b0}} : O_mode + 1'b1;
                end
                else begin
                    r_cnt <= r_cnt + 20'd1;
                end
            end
            else begin
                r_cnt <= 20'd0;
            end
        end
    end

endmodule
