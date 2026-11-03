//-----------------------------------------------------------------------------
// Module     : tm1638_led_key
// Project    : FS03_DR1V90GEG_img_process_base2
// Description: Driver for the external "8 digit tube + 8 LED + 8 key" module
//              (TM1638 LED&KEY).  It replaces the parallel common anode tube
//              driver (ctrl/seg7_display.v) used on the HX1P35A board, because
//              the FS03 board has no on board tube / key / LED and the module
//              is connected through a 3 wire serial port (STB / CLK / DIO).
//
//   Wiring (see 在FS03_DR1V90GEG连接外设gpio最终引脚说明.md)
//        CEP2(J12), bottom row, normal orientation, start column 2 :
//        VCC = W12   GND = AB12   STB = Y8   CLK = AB11   DIO = Y10
//        VCC / GND are supplied by two FPGA pins driven 1 / 0.
//
//   Display mapping (identical to the HX1P35A version):
//        digit 0 .. 7 = left .. right,  value is right aligned, i.e.
//        hundreds -> digit 5, tens -> digit 6, units -> digit 7 (right most).
//        Leading zeros are blanked, "0" is shown as "0".
//
//   LED mapping : bit0 of the odd display register byte -> LED of that digit,
//        so I_led[0..7] = LED1..LED8 (left .. right).
//
//   Key mapping : TM1638 key scan returns 4 bytes (BYTE1..BYTE4), LSB first:
//        BYTE1 = KS1 (bit2:0 = K3,K2,K1) + KS2 (bit6:4)
//        BYTE2 = KS3 / KS4      BYTE3 = KS5 / KS6      BYTE4 = KS7 / KS8
//        The module wires S1..S8 as KS1,KS3,KS5,KS7,KS2,KS4,KS6,KS8.
//        All three K lines are OR-ed so the mapping works whatever K line the
//        module actually uses.
//
//   Timing : serial clock ~1 MHz (TM1638 limit), the whole transaction
//            (write 16 display registers + read 4 key bytes) takes about
//            180 us and is repeated at REFRESH_HZ (default 200 Hz, 5 ms),
//            so the tube never flickers and the key latency is < 5 ms.
//-----------------------------------------------------------------------------
`timescale 1ns / 100ps

module tm1638_led_key #(
    parameter CLK_HZ     = 100000000,   // system clock of the I_clk input
    parameter SCK_HZ     = 1000000,     // TM1638 serial clock (~1 MHz max)
    parameter REFRESH_HZ = 200          // display refresh / key scan rate
)(
    input  wire        I_clk,
    input  wire        I_rst_n,

    input  wire [9:0]  I_value,     // 0 .. 999, decimal, right aligned
    input  wire [7:0]  I_led,       // LED1..LED8 (bit0 = LED1)

    output reg  [7:0]  O_key,       // S1..S8 (bit0 = S1), 1 = pressed
    output reg         O_valid,     // 1 after the first complete key scan

    output reg         O_stb,       // STB, active low
    output reg         O_sck,       // CLK
    inout  wire        IO_dio       // DIO, open drain with pull up
);

    // ------------------------------------------------------------------
    // commands
    // ------------------------------------------------------------------
    localparam [7:0] CMD_DATA_AUTO = 8'h40;   // write display register, addr++
    localparam [7:0] CMD_ADDR0     = 8'hC0;   // address 0 (GRID1)
    localparam [7:0] CMD_DISP_ON   = 8'h8F;   // display on, brightness 7
    localparam [7:0] CMD_READ_KEY  = 8'h42;   // read key scan data

    // half period of the serial clock, in I_clk cycles
    localparam integer C_HALF = (CLK_HZ + SCK_HZ) / (2 * SCK_HZ);
    localparam integer C_GAP  = CLK_HZ / REFRESH_HZ;
    localparam [3:0]   C_STB_GAP = 4'd4;      // STB high, in half periods

    // ------------------------------------------------------------------
    // states
    // ------------------------------------------------------------------
    localparam [3:0] S_IDLE  = 4'd0;
    localparam [3:0] S_W40   = 4'd1;
    localparam [3:0] S_P1    = 4'd2;
    localparam [3:0] S_WC0   = 4'd3;
    localparam [3:0] S_WDAT  = 4'd4;
    localparam [3:0] S_P2    = 4'd5;
    localparam [3:0] S_W8F   = 4'd6;
    localparam [3:0] S_P3    = 4'd7;
    localparam [3:0] S_W42   = 4'd8;
    localparam [3:0] S_RWAIT = 4'd9;
    localparam [3:0] S_RKEY  = 4'd10;
    localparam [3:0] S_P4    = 4'd11;

    // ------------------------------------------------------------------
    // binary -> BCD and digit selection
    // ------------------------------------------------------------------
    wire [3:0] S_hun;
    wire [3:0] S_ten;
    wire [3:0] S_one;

    bin2bcd10 u_bin2bcd10 (
        .I_bin      ( I_value ),
        .O_hundreds ( S_hun   ),
        .O_tens     ( S_ten   ),
        .O_units    ( S_one   )
    );

    wire S_show_hun = (S_hun != 4'd0);
    wire S_show_ten = (S_hun != 4'd0) || (S_ten != 4'd0);

    function [3:0] dig_val;
        input [2:0] d;
        begin
            case (d)
                3'd5:    dig_val = S_show_hun ? S_hun : 4'd10;
                3'd6:    dig_val = S_show_ten ? S_ten : 4'd10;
                3'd7:    dig_val = S_one;
                default: dig_val = 4'd10;          // 10 = blank
            endcase
        end
    endfunction

    // segment order: bit0 = a, bit1 = b, ... bit6 = g, bit7 = dp, 1 = lit
    function [7:0] seg7_enc;
        input [3:0] d;
        begin
            case (d)
                4'd0:    seg7_enc = 8'h3F;
                4'd1:    seg7_enc = 8'h06;
                4'd2:    seg7_enc = 8'h5B;
                4'd3:    seg7_enc = 8'h4F;
                4'd4:    seg7_enc = 8'h66;
                4'd5:    seg7_enc = 8'h6D;
                4'd6:    seg7_enc = 8'h7D;
                4'd7:    seg7_enc = 8'h07;
                4'd8:    seg7_enc = 8'h7F;
                4'd9:    seg7_enc = 8'h6F;
                default: seg7_enc = 8'h00;         // blank
            endcase
        end
    endfunction

    // display register byte : even index = digit segments, odd index = LED
    function [7:0] frame_byte;
        input [3:0] i;
        reg   [2:0] d;
        begin
            d = i[3:1];
            if (i[0] == 1'b0)
                frame_byte = seg7_enc(dig_val(d));
            else
                frame_byte = {7'b0000000, I_led[d]};
        end
    endfunction

    // ------------------------------------------------------------------
    // key decode
    // ------------------------------------------------------------------
    reg [7:0] r_b1;
    reg [7:0] r_b2;
    reg [7:0] r_b3;
    reg [7:0] r_b4;

    wire [2:0] S_ks1 = r_b1[2:0];
    wire [2:0] S_ks2 = r_b1[6:4];
    wire [2:0] S_ks3 = r_b2[2:0];
    wire [2:0] S_ks4 = r_b2[6:4];
    wire [2:0] S_ks5 = r_b3[2:0];
    wire [2:0] S_ks6 = r_b3[6:4];
    wire [2:0] S_ks7 = r_b4[2:0];
    wire [2:0] S_ks8 = r_b4[6:4];

    // S1..S4 = KS1,KS3,KS5,KS7 ;  S5..S8 = KS2,KS4,KS6,KS8
    wire [7:0] S_key_dec = { |S_ks8, |S_ks6, |S_ks4, |S_ks2,
                             |S_ks7, |S_ks5, |S_ks3, |S_ks1 };

    // ------------------------------------------------------------------
    // three wire interface
    // ------------------------------------------------------------------
    reg         r_dio_o;
    reg         r_dio_oe;

    assign IO_dio = r_dio_oe ? r_dio_o : 1'bz;
    wire   S_dio  = IO_dio;

    reg [31:0] r_half;
    reg [31:0] r_gap;
    reg [3:0]  r_state;
    reg [3:0]  r_bit;      // bit index inside the current byte
    reg [3:0]  r_idx;      // byte index (display frame / key bytes)
    reg        r_hp;       // half period phase : 0 = SCK low, 1 = SCK high
    reg [7:0]  r_shift;

    wire S_send = (r_state == S_W40)  || (r_state == S_WC0) ||
                  (r_state == S_WDAT) || (r_state == S_W8F)  ||
                  (r_state == S_W42);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            r_half   <= 32'd0;
            r_gap    <= 32'd0;
            r_state  <= S_IDLE;
            r_bit    <= 4'd0;
            r_idx    <= 4'd0;
            r_hp     <= 1'b0;
            r_shift  <= 8'd0;
            r_dio_o  <= 1'b1;
            r_dio_oe <= 1'b1;
            r_b1     <= 8'd0;
            r_b2     <= 8'd0;
            r_b3     <= 8'd0;
            r_b4     <= 8'd0;
            O_stb    <= 1'b1;
            O_sck    <= 1'b1;
            O_key    <= 8'd0;
            O_valid  <= 1'b0;
        end
        else begin
            // free running half period counter
            if (r_half == C_HALF - 1)
                r_half <= 32'd0;
            else
                r_half <= r_half + 32'd1;

            if (r_half == C_HALF - 1) begin
                case (r_state)

                    // --------------------------------------------------
                    // idle : STB high, wait for the next refresh slot
                    // --------------------------------------------------
                    S_IDLE: begin
                        O_stb    <= 1'b1;
                        O_sck    <= 1'b1;
                        r_dio_oe <= 1'b1;
                        r_dio_o  <= 1'b1;
                        if (r_gap == C_GAP - 1) begin
                            r_gap   <= 32'd0;
                            r_state <= S_W40;
                            r_shift <= CMD_DATA_AUTO;
                            r_bit   <= 4'd0;
                            r_hp    <= 1'b0;
                            O_stb   <= 1'b0;
                        end
                        else begin
                            r_gap <= r_gap + 32'd1;
                        end
                    end

                    // --------------------------------------------------
                    // send one byte, LSB first, latched on the SCK rise
                    // --------------------------------------------------
                    S_W40, S_WC0, S_WDAT, S_W8F, S_W42: begin
                        r_dio_oe <= 1'b1;
                        if (r_hp == 1'b0) begin
                            O_sck   <= 1'b0;
                            r_dio_o <= r_shift[0];
                            r_hp    <= 1'b1;
                        end
                        else begin
                            O_sck   <= 1'b1;
                            r_shift <= {1'b0, r_shift[7:1]};
                            r_hp    <= 1'b0;
                            if (r_bit == 4'd7) begin
                                r_bit <= 4'd0;
                                case (r_state)
                                    S_W40:  begin r_state <= S_P1; O_stb <= 1'b1; end
                                    S_WC0:  begin
                                                r_state <= S_WDAT;
                                                r_idx   <= 4'd0;
                                                r_shift <= frame_byte(4'd0);
                                            end
                                    S_WDAT: begin
                                                if (r_idx == 4'd15) begin
                                                    r_state <= S_P2;
                                                    O_stb   <= 1'b1;
                                                end
                                                else begin
                                                    r_idx   <= r_idx + 4'd1;
                                                    r_shift <= frame_byte(r_idx + 4'd1);
                                                end
                                            end
                                    S_W8F:  begin r_state <= S_P3; O_stb <= 1'b1; end
                                    default: begin                      // S_W42
                                                r_state <= S_RWAIT;
                                                r_bit   <= 4'd0;
                                                r_dio_oe<= 1'b0;   // release DIO
                                            end
                                endcase
                            end
                            else begin
                                r_bit <= r_bit + 4'd1;
                            end
                        end
                    end

                    // --------------------------------------------------
                    // STB high for a few half periods between the commands
                    // --------------------------------------------------
                    S_P1: begin
                        O_sck    <= 1'b1;
                        r_dio_oe <= 1'b1;
                        if (r_bit == C_STB_GAP) begin
                            r_state <= S_WC0;
                            r_shift <= CMD_ADDR0;
                            r_bit   <= 4'd0;
                            r_hp    <= 1'b0;
                            O_stb   <= 1'b0;
                        end
                        else begin
                            r_bit <= r_bit + 4'd1;
                        end
                    end

                    S_P2: begin
                        O_sck    <= 1'b1;
                        r_dio_oe <= 1'b1;
                        if (r_bit == C_STB_GAP) begin
                            r_state <= S_W8F;
                            r_shift <= CMD_DISP_ON;
                            r_bit   <= 4'd0;
                            r_hp    <= 1'b0;
                            O_stb   <= 1'b0;
                        end
                        else begin
                            r_bit <= r_bit + 4'd1;
                        end
                    end

                    S_P3: begin
                        O_sck    <= 1'b1;
                        r_dio_oe <= 1'b1;
                        if (r_bit == C_STB_GAP) begin
                            r_state <= S_W42;
                            r_shift <= CMD_READ_KEY;
                            r_bit   <= 4'd0;
                            r_hp    <= 1'b0;
                            O_stb   <= 1'b0;
                        end
                        else begin
                            r_bit <= r_bit + 4'd1;
                        end
                    end

                    // --------------------------------------------------
                    // wait (STB stays low) before the key bytes are read
                    // --------------------------------------------------
                    S_RWAIT: begin
                        O_sck    <= 1'b1;
                        r_dio_oe <= 1'b0;
                        if (r_bit == C_STB_GAP) begin
                            r_state <= S_RKEY;
                            r_idx   <= 4'd0;
                            r_bit   <= 4'd0;
                            r_hp    <= 1'b0;
                        end
                        else begin
                            r_bit <= r_bit + 4'd1;
                        end
                    end

                    // --------------------------------------------------
                    // read 4 key bytes, LSB first, sampled while SCK is high
                    // --------------------------------------------------
                    S_RKEY: begin
                        r_dio_oe <= 1'b0;
                        if (r_hp == 1'b0) begin
                            O_sck <= 1'b0;
                            r_hp  <= 1'b1;
                        end
                        else begin
                            O_sck <= 1'b1;
                            r_hp  <= 1'b0;
                            if (r_bit == 4'd7) begin
                                r_bit <= 4'd0;
                                case (r_idx)
                                    4'd0: r_b1 <= {S_dio, r_shift[7:1]};
                                    4'd1: r_b2 <= {S_dio, r_shift[7:1]};
                                    4'd2: r_b3 <= {S_dio, r_shift[7:1]};
                                    default: r_b4 <= {S_dio, r_shift[7:1]};
                                endcase
                                if (r_idx == 4'd3) begin
                                    r_state <= S_P4;
                                    O_stb   <= 1'b1;
                                end
                                else begin
                                    r_idx <= r_idx + 4'd1;
                                end
                            end
                            else begin
                                r_shift <= {S_dio, r_shift[7:1]};
                                r_bit   <= r_bit + 4'd1;
                            end
                        end
                    end

                    // --------------------------------------------------
                    // end of transaction : latch the decoded key vector
                    // --------------------------------------------------
                    default: begin                       // S_P4
                        O_sck    <= 1'b1;
                        r_dio_oe <= 1'b1;
                        r_dio_o  <= 1'b1;
                        if (r_bit == C_STB_GAP) begin
                            O_key   <= S_key_dec;
                            O_valid <= 1'b1;
                            r_state <= S_IDLE;
                            r_bit   <= 4'd0;
                        end
                        else begin
                            r_bit <= r_bit + 4'd1;
                        end
                    end
                endcase
            end
        end
    end

endmodule
