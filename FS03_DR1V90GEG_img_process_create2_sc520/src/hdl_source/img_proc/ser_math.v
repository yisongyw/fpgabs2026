//-----------------------------------------------------------------------------
// Module     : ser_math
// Project    : HX1P35Aimg_process_extend2
// Description: Tiny bit serial arithmetic unit shared by every slow (frame end)
//              computation of the target detector.
//
//   Rationale (extension requirement 4 - minimise resources, area and power)
//   -----------------------------------------------------------------------
//   The detector needs a handful of multiplications and divisions per frame
//   (bounding box area, fill ratio, aspect ratio, mean colour, compactness ...).
//   All of them are needed ONCE per detected object, at the frame end, i.e.
//   they have a whole frame period (about 230000 pixel beats) of slack.
//
//   Parallel dividers would cost several thousand LUTs, therefore ONE iterative
//   unit that is reused for every division is used:
//
//        0 DSP, roughly 130 LUT / 130 DFF, one operation at a time
//
//      op = 0 : unsigned multiply     16 x 16 -> 32 bit    (16 clocks)
//      op = 1 : restoring division    32 / 24 -> 32 bit    (32 clocks)
//
//   Usage:
//        assert I_start together with I_op / I_a / I_b for one clock
//        wait for O_done = 1 and read O_result
//
//   Division by zero is trapped : O_result = 0 and O_done is issued after one
//   clock, so a caller can never hang.
//-----------------------------------------------------------------------------
module ser_math (
    input  wire        I_clk,
    input  wire        I_rst_n,

    input  wire        I_start,
    input  wire        I_op,            // 0 = multiply, 1 = divide
    input  wire [31:0] I_a,
    input  wire [23:0] I_b,

    output reg  [31:0] O_result,
    output reg         O_done
);

    localparam [1:0] S_IDLE = 2'd0;
    localparam [1:0] S_MUL  = 2'd1;
    localparam [1:0] S_DIV  = 2'd2;

    localparam [5:0] MUL_STEPS = 6'd16;
    localparam [5:0] DIV_STEPS = 6'd32;

    reg [1:0]  r_state;
    reg [5:0]  r_cnt;

    reg [31:0] r_acc;       // product accumulator
    reg [31:0] r_ash;       // shifted multiplicand / dividend
    reg [23:0] r_b;         // multiplier (low bits) / divisor
    reg [31:0] r_rem;       // division remainder
    reg [31:0] r_quo;       // division quotient

    //-------------------------------------------------------------------------
    // combinational next step
    //-------------------------------------------------------------------------
    wire [31:0] S_sh    = {r_rem[30:0], r_ash[31]};
    wire [31:0] S_den   = {8'd0, r_b};
    wire        S_ge    = (r_b != 24'd0) && (S_sh >= S_den);
    wire [31:0] S_rem_n = S_ge ? (S_sh - S_den) : S_sh;

    wire [31:0] S_acc_n = r_b[0] ? (r_acc + {16'd0, r_ash[15:0]}) : r_acc;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            r_state  <= S_IDLE;
            r_cnt    <= 6'd0;
            r_acc    <= 32'd0;
            r_ash    <= 32'd0;
            r_b      <= 24'd0;
            r_rem    <= 32'd0;
            r_quo    <= 32'd0;
            O_result <= 32'd0;
            O_done   <= 1'b0;
        end
        else begin
            O_done <= 1'b0;

            case(r_state)
                //-------------------------------------------------------------
                S_IDLE: begin
                    if(I_start) begin
                        if(I_op == 1'b1) begin
                            if(I_b == 24'd0) begin
                                // divide by zero -> 0, report immediately
                                O_result <= 32'd0;
                                O_done   <= 1'b1;
                            end
                            else begin
                                r_cnt   <= DIV_STEPS;
                                r_ash   <= I_a;
                                r_b     <= I_b;
                                r_rem   <= 32'd0;
                                r_quo   <= 32'd0;
                                r_state <= S_DIV;
                            end
                        end
                        else begin
                            r_cnt   <= MUL_STEPS;
                            r_acc   <= 32'd0;
                            r_ash   <= {16'd0, I_a[15:0]};
                            r_b     <= I_b[15:0];
                            r_state <= S_MUL;
                        end
                    end
                end

                //-------------------------------------------------------------
                // shift and add multiply : 16 iterations
                //-------------------------------------------------------------
                S_MUL: begin
                    r_acc <= S_acc_n;
                    r_ash <= {16'd0, r_ash[14:0], 1'b0};
                    r_b   <= {1'b0, r_b[23:1]};
                    if(r_cnt == 6'd1) begin
                        O_result <= S_acc_n;
                        O_done   <= 1'b1;
                        r_state  <= S_IDLE;
                    end
                    else begin
                        r_cnt <= r_cnt - 6'd1;
                    end
                end

                //-------------------------------------------------------------
                // restoring division : 32 iterations, MSB first
                //-------------------------------------------------------------
                S_DIV: begin
                    r_rem <= S_rem_n;
                    r_ash <= {r_ash[30:0], 1'b0};
                    r_quo <= {r_quo[30:0], S_ge};
                    if(r_cnt == 6'd1) begin
                        O_result <= {r_quo[30:0], S_ge};
                        O_done   <= 1'b1;
                        r_state  <= S_IDLE;
                    end
                    else begin
                        r_cnt <= r_cnt - 6'd1;
                    end
                end

                default: r_state <= S_IDLE;
            endcase
        end
    end

endmodule
