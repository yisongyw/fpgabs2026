//-----------------------------------------------------------------------------
// Module     : key_mode_delay_ctrl
// Project    : HX1P35Aimg_process_extend1
// Description: Debounced key input that increments a *pending* value on every
//              key RELEASE and copies it into O_mode DELAY_MAX clock cycles
//              later.
//
//   Competition requirement (extend1):
//       "the algorithm / effect may only change 1 s AFTER the key has been
//        pressed and released"
//   so a switch never happens while the operator still touches the board and
//   the picture has time to settle before the new algorithm becomes active.
//
//   I_key    : active low (board pull-up)
//   CNT_MAX  : debounce time in clock cycles
//              20'd999999 @ 100 MHz -> ~10 ms
//   DELAY_MAX: commit delay in clock cycles, counted from the release edge
//              27'd99999999 @ 100 MHz -> 1.000 s
//   WIDTH    : width of the counter
//   MOD      : the counter wraps back to 0 after MOD-1
//
//   Behaviour
//     * one increment per press / release cycle
//     * no auto repeat while the key is held down
//     * no false trigger caused by bouncing
//     * O_mode is updated DELAY_MAX cycles after the release edge
//     * pressing again while a commit is pending restarts the 1 s timer and
//       increments the pending value once more (the last press always wins)
//
//   O_waiting can be used to drive a "switch pending" indication.
//-----------------------------------------------------------------------------
module key_mode_delay_ctrl #(
    parameter CNT_MAX   = 20'd999999,
    parameter DELAY_MAX = 27'd99999999,
    parameter WIDTH     = 4,
    parameter MOD       = 16
)(
    input  wire             I_clk,
    input  wire             I_rst_n,
    input  wire             I_key,          // active low
    // create1 addition : remote virtual press (UART / Ethernet short string
    // command).  One clock pulse equals one physical press + release : it
    // increments the pending value and restarts the 1 s commit timer, so the
    // switching function is identical to the push button.
    input  wire             I_ext_evt,

    output reg [WIDTH-1:0]  O_mode,         // committed (active) value
    output reg [WIDTH-1:0]  O_pending,      // value waiting for the commit
    output reg              O_waiting       // 1 : a commit is scheduled
);

    localparam [WIDTH-1:0] WRAP = MOD - 1;

    reg [1:0]  r_key_sync;
    reg [19:0] r_cnt;
    reg        r_pressed;       // debounced key state, 0 = pressed
    reg [26:0] r_timer;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            r_key_sync <= 2'b11;
            r_cnt      <= 20'd0;
            r_pressed  <= 1'b1;
            r_timer    <= 27'd0;
            O_mode     <= {WIDTH{1'b0}};
            O_pending  <= {WIDTH{1'b0}};
            O_waiting  <= 1'b0;
        end
        else begin
            r_key_sync <= {r_key_sync[0], I_key};

            //--------------------------------------------------------------
            // debounce (10 ms) - increment on the RELEASE edge
            //--------------------------------------------------------------
            if(r_key_sync[1] != r_pressed) begin
                if(r_cnt == CNT_MAX) begin
                    r_cnt     <= 20'd0;
                    r_pressed <= r_key_sync[1];
                    if(r_key_sync[1]) begin          // 0 -> 1 : key released
                        O_pending <= (O_pending == WRAP) ? {WIDTH{1'b0}}
                                                         : O_pending + 1'b1;
                        r_timer   <= DELAY_MAX;
                        O_waiting <= 1'b1;
                    end
                end
                else begin
                    r_cnt <= r_cnt + 20'd1;
                end
            end
            else begin
                r_cnt <= 20'd0;
            end

            //--------------------------------------------------------------
            // remote virtual press (UART / Ethernet command) : same effect
            // as one debounced physical press + release
            //--------------------------------------------------------------
            if(I_ext_evt) begin
                O_pending <= (O_pending == WRAP) ? {WIDTH{1'b0}}
                                                 : O_pending + 1'b1;
                r_timer   <= DELAY_MAX;
                O_waiting <= 1'b1;
            end

            //--------------------------------------------------------------
            // delayed commit : apply the pending value 1 s after release
            //--------------------------------------------------------------
            if(O_waiting) begin
                if(r_timer == 27'd0) begin
                    O_mode    <= O_pending;
                    O_waiting <= 1'b0;
                end
                else begin
                    r_timer <= r_timer - 27'd1;
                end
            end
        end
    end

endmodule
