//-----------------------------------------------------------------------------
// Module     : uart_cmd_rx
// Project    : HX1P35Aimg_process_create1
// Description: UART command front end (115200 8N1) + shared short-string
//              decoder.  Remote button events (KEY1..KEY4) and the SD / CAM /
//              LD / R? commands are produced as one shot pulses in the
//              S_100m_clk domain and merged into the same control paths the
//              physical push buttons use.
//-----------------------------------------------------------------------------
module uart_cmd_rx #(
    parameter CLK_FREQ = 100_000_000,
    parameter BAUD     = 115_200
)(
    input  wire       I_clk,
    input  wire       I_rst_n,
    input  wire       I_uart_rx,

    output wire [3:0] O_key_evt,     // KEY1..KEY4 virtual presses
    output wire       O_sd_evt,
    output wire       O_cam_evt,
    output wire       O_ld_evt,
    output wire O_rep_evt,
    output wire O_demo_evt,
    output wire       O_cmd_valid,
    output reg        O_rx_act       // byte received (status blink)
);

    localparam integer BIT_CLKS = CLK_FREQ / BAUD;   // 868 @ 100 MHz / 115200

    // input synchroniser
    reg [2:0] r_rx_sync;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            r_rx_sync <= 3'b111;
        else
            r_rx_sync <= {r_rx_sync[1:0], I_uart_rx};
    end
    wire S_rx = r_rx_sync[2];

    //-------------------------------------------------------------------------
    // 8N1 receiver : sample in the middle of every bit
    //-------------------------------------------------------------------------
    reg [15:0] r_baud;
    reg [3:0]  r_bit;
    reg [7:0]  r_sr;
    reg        r_busy;
    reg        r_valid;
    reg [7:0]  r_data;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            r_baud  <= 16'd0;
            r_bit   <= 4'd0;
            r_sr    <= 8'd0;
            r_busy  <= 1'b0;
            r_valid <= 1'b0;
            r_data  <= 8'd0;
            O_rx_act<= 1'b0;
        end
        else begin
            r_valid  <= 1'b0;
            O_rx_act <= 1'b0;
            if(!r_busy) begin
                if(S_rx == 1'b0) begin
                    // falling edge of start bit
                    r_busy <= 1'b1;
                    r_baud <= BIT_CLKS / 2;
                    r_bit  <= 4'd0;
                end
            end
            else begin
                if(r_baud != 16'd0) begin
                    r_baud <= r_baud - 16'd1;
                end
                else begin
                    if(r_bit == 4'd0) begin
                        // middle of the start bit : still low?
                        if(S_rx != 1'b0) begin
                            r_busy <= 1'b0;        // false start
                        end
                        else begin
                            r_bit  <= 4'd1;
                            r_baud <= BIT_CLKS - 1;
                        end
                    end
                    else if(r_bit <= 4'd8) begin
                        r_sr   <= {S_rx, r_sr[7:1]};   // LSB first
                        r_bit  <= r_bit + 4'd1;
                        r_baud <= BIT_CLKS - 1;
                    end
                    else begin
                        // stop bit
                        r_busy  <= 1'b0;
                        r_valid <= 1'b1;
                        r_data  <= r_sr;
                        O_rx_act<= 1'b1;
                    end
                end
            end
        end
    end

    //-------------------------------------------------------------------------
    // shared command decoder
    //-------------------------------------------------------------------------
    cmd_parser u_cmd_parser (
        .I_clk      ( I_clk    ),
        .I_rst_n    ( I_rst_n  ),
        .I_rx_valid ( r_valid  ),
        .I_rx_data  ( r_data   ),
        .O_key_evt  ( O_key_evt ),
        .O_sd_evt   ( O_sd_evt  ),
        .O_cam_evt  ( O_cam_evt ),
        .O_ld_evt   ( O_ld_evt  ),
        .O_rep_evt   ( O_rep_evt  ),
        .O_demo_evt ( O_demo_evt ),
        .O_cmd_valid( O_cmd_valid )
    );

endmodule
