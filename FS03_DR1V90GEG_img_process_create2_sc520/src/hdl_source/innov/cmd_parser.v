//-----------------------------------------------------------------------------
// Module     : cmd_parser
// Project    : HX1P35Aimg_process_create1
// Description: short-string command decoder, shared by the UART and the
//              Ethernet command channels (requirement: both channels must
//              behave IDENTICALLY).
//
//   The module consumes a plain byte stream (any clock) and matches the most
//   recent ASCII word against the command table on every terminator byte
//   (CR / LF / space / ';' / 0 / any non alphanumeric).  Matching is case
//   insensitive and accepts both short and long button aliases.
//
//   Command table (identical on UART and Ethernet/UDP):
//       "K1" / "KEY1" : system reset        (same as KEY1)
//       "K2" / "KEY2" : threshold level +1  (same as KEY2)
//       "K3" / "KEY3" : style     level +1  (same as KEY3)
//       "K4" / "KEY4" : classifier switch   (same as KEY4)
//       "SD"          : load next image from the SD card (interaction)
//       "CAM"         : back to the live camera stream
//       "LD"          : toggle the dual colour LED status view
//       "R?"          : request one status / result report on this channel
//
//   Every accepted command also raises O_cmd_valid for status indication.
//-----------------------------------------------------------------------------
module cmd_parser (
    input  wire       I_clk,
    input  wire       I_rst_n,
    input  wire       I_rx_valid,     // 1 clock byte strobe
    input  wire [7:0] I_rx_data,

    output reg  [3:0] O_key_evt,      // KEY1..KEY4 one shot pulses
    output reg        O_sd_evt,       // SD image request
    output reg        O_cam_evt,      // back to camera
    output reg        O_ld_evt,       // LED view toggle
    output reg O_rep_evt,
    output reg O_demo_evt,      // report request
    output reg        O_cmd_valid     // any accepted command (status blink)
);

    // 4 character shift register + population counter
    reg [31:0] r_shift;
    reg [2:0]  r_len;

    function is_alnum;
        input [7:0] c;
        begin
            is_alnum = ((c >= "A") && (c <= "Z")) ||
                       ((c >= "a") && (c <= "z")) ||
                       ((c >= "0") && (c <= "9"));
        end
    endfunction

    function [7:0] upcase;
        input [7:0] c;
        begin
            upcase = ((c >= "a") && (c <= "z")) ? (c - 8'h20) : c;
        end
    endfunction

    // word valid at terminator time = r_shift with r_len chars, right aligned
    wire [7:0]  c3 = upcase(r_shift[31:24]);
    wire [7:0]  c2 = upcase(r_shift[23:16]);
    wire [7:0]  c1 = upcase(r_shift[15: 8]);
    wire [7:0]  c0 = upcase(r_shift[ 7: 0]);   // most recent character

    // name of the decoded word : the last r_len characters
    wire        w1  = (r_len == 3'd1);
    wire        w2  = (r_len == 3'd2);
    wire        w3  = (r_len == 3'd3);
    wire        w4  = (r_len == 3'd4);

    wire        m_k1 = (w2 && (c1 == "K") && (c0 == "1")) ||
                       (w2 && (c1 == "R") && (c0 == "1")) ||
                       (w4 && (c3 == "K") && (c2 == "E") && (c1 == "Y") && (c0 == "1")) ||
                       (w3 && (c2 == "R") && (c1 == "S") && (c0 == "T"));
    wire        m_k2 = (w2 && (c1 == "K") && (c0 == "2")) ||
                       (w4 && (c3 == "K") && (c2 == "E") && (c1 == "Y") && (c0 == "2"));
    wire        m_k3 = (w2 && (c1 == "K") && (c0 == "3")) ||
                       (w4 && (c3 == "K") && (c2 == "E") && (c1 == "Y") && (c0 == "3"));
    wire        m_k4 = (w2 && (c1 == "K") && (c0 == "4")) ||
                       (w4 && (c3 == "K") && (c2 == "E") && (c1 == "Y") && (c0 == "4"));
    wire        m_sd = (w2 && (c1 == "S") && (c0 == "D"));
    wire        m_cam= (w3 && (c2 == "C") && (c1 == "A") && (c0 == "M"));
    wire        m_ld = (w2 && (c1 == "L") && (c0 == "D"));
    wire m_rp = (w2 && (c1 == "R") && (c0 == "?"));
    wire m_demo = (w4 && (c3 == "D") && (c2 == "E") && (c1 == "M") && (c0 == "O"));

    wire        S_any = m_k1 | m_k2 | m_k3 | m_k4 | m_sd | m_cam | m_ld | m_rp | m_demo;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            r_shift    <= 32'd0;
            r_len      <= 3'd0;
            O_key_evt  <= 4'd0;
            O_sd_evt   <= 1'b0;
            O_cam_evt  <= 1'b0;
            O_ld_evt   <= 1'b0;
            O_rep_evt   <= 1'b0;
            O_demo_evt  <= 1'b0;
            O_cmd_valid<= 1'b0;
        end
        else begin
            O_key_evt   <= 4'd0;
            O_sd_evt    <= 1'b0;
            O_cam_evt   <= 1'b0;
            O_ld_evt    <= 1'b0;
            O_rep_evt   <= 1'b0;
            O_demo_evt  <= 1'b0;
            O_cmd_valid <= 1'b0;

            if(I_rx_valid) begin
                if(is_alnum(I_rx_data)) begin
                    r_shift <= {r_shift[23:0], upcase(I_rx_data)};
                    if(r_len != 3'd4)
                        r_len <= r_len + 3'd1;
                end
                else if(r_len != 3'd0) begin
                    // terminator : decode the buffered word
                    O_key_evt   <= {m_k4, m_k3, m_k2, m_k1};
                    O_sd_evt    <= m_sd;
                    O_cam_evt   <= m_cam;
                    O_ld_evt    <= m_ld;
                    O_rep_evt   <= m_rp;
                    O_demo_evt  <= m_demo;
                    O_cmd_valid <= S_any;
                    r_shift     <= 32'd0;
                    r_len       <= 3'd0;
                end
            end
        end
    end

endmodule
