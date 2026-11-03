//-----------------------------------------------------------------------------
// Module     : periph_status
// Project    : HX1P35Aimg_process_create1
// Description: UART / Ethernet / SD card working status for the "remaining"
//              LED resource : the 8 dual colour LED groups that share the
//              SEL/DIG bus with the 7 segment display (SW6 selects the LED
//              side).  When the LED view is active ("LD" command) the two
//              buses directly encode the status :
//                  DIG[i] (active low) = green status of group i
//                  SEL[i] (active high) = red activity / error of group i
//
//   group 0 : UART RX   (green = ready, red = byte activity)
//   group 1 : UART TX   (green = ready, red = byte activity)
//   group 2 : ETH LINK  (green = link up, red = RX activity)
//   group 3 : ETH TX    (green = link up, red = TX activity)
//   group 4 : ETH ERR   (red = MAC/IP error activity)
//   group 5 : SD INIT   (green = initialised, red = not initialised)
//   group 6 : SD READ   (green = initialised, red = sector activity)
//   group 7 : CMD       (green = idle, red = command received)
//-----------------------------------------------------------------------------
module periph_status (
    input  wire       I_clk,        // S_100m_clk
    input  wire       I_rst_n,

    input  wire       I_uart_rx_act,
    input  wire       I_uart_tx_act,
    input  wire       I_eth_link,
    input  wire       I_eth_tx_act,
    input  wire       I_eth_rx_act,
    input  wire       I_eth_err,
    input  wire       I_sd_init,
    input  wire       I_sd_act,
    input  wire       I_sd_err,
    input  wire       I_cmd_act,

    output wire [7:0] O_st_g,       // green level per group
    output wire [7:0] O_st_r        // red activity per group
);

    // stretch every activity pulse so it is visible on the LEDs
    reg [21:0] r_uart_rx;
    reg [21:0] r_uart_tx;
    reg [21:0] r_cmd;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            r_uart_rx <= 22'd0;
            r_uart_tx <= 22'd0;
            r_cmd     <= 22'd0;
        end
        else begin
            if(I_uart_rx_act) r_uart_rx <= 22'h3FFFFF;
            else if(r_uart_rx != 22'd0) r_uart_rx <= r_uart_rx - 22'd1;

            if(I_uart_tx_act) r_uart_tx <= 22'h3FFFFF;
            else if(r_uart_tx != 22'd0) r_uart_tx <= r_uart_tx - 22'd1;

            if(I_cmd_act) r_cmd <= 22'h3FFFFF;
            else if(r_cmd != 22'd0) r_cmd <= r_cmd - 22'd1;
        end
    end

    assign O_st_g = {1'b1, I_sd_init, I_sd_init & ~I_sd_err,
                     1'b0, I_eth_link, I_eth_link, 1'b1, 1'b1};

    assign O_st_r = {|(r_cmd), I_sd_act, ~I_sd_init | I_sd_err,
                     I_eth_err, I_eth_tx_act, I_eth_rx_act,
                     |(r_uart_tx), |(r_uart_rx)};

endmodule
