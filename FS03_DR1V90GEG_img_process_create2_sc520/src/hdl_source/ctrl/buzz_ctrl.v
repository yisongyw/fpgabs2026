//==========================================================================
// buzz_ctrl.v  (HX1P35Aimg_process_extend3)
//--------------------------------------------------------------------------
// Drives the active-LOW buzzer on the HX1P35A expansion header.
//
//   * I_alarm (already synced to this clock) asserts while an object sits
//     inside the ROI
//   * while alarm is asserted the module produces a 1 kHz square wave that
//     is pulsed ON for 300 ms / OFF for 300 ms (a recognizable "beep -
//     beep" pattern) so it is easy to hear and does not run continuously
//   * O_buzz is 1 (silent) at all other times
//
// The 100 MHz system clock is assumed (CLK_HZ parameter).
//==========================================================================
`timescale 1ns / 1ps

module buzz_ctrl
#(
    parameter CLK_HZ = 100_000_000,
    parameter TONE_HZ = 1000,
    parameter BLINK_MS = 300
)
(
    input  wire I_clk,
    input  wire I_rst,
    input  wire I_alarm,
    output reg  O_buzz            // active LOW (1 = silent)
);

    localparam integer TONE_DIV = CLK_HZ / (2 * TONE_HZ);   // half period
    localparam integer BLINK_CNT = CLK_HZ * BLINK_MS / 1000;

    reg [31:0] r_tone_cnt;
    reg        r_tone;
    reg [31:0] r_blink_cnt;
    reg        r_blink_on;
    reg        r_alarm_d;

    always @(posedge I_clk) begin
        if (I_rst) begin
            r_tone_cnt <= 32'd0;
            r_tone     <= 1'b0;
            r_blink_cnt<= 32'd0;
            r_blink_on <= 1'b0;
            r_alarm_d  <= 1'b0;
            O_buzz     <= 1'b1;
        end else begin
            r_alarm_d <= I_alarm;

            // 1 kHz square-wave generator
            if (r_tone_cnt >= TONE_DIV - 1) begin
                r_tone_cnt <= 32'd0;
                r_tone     <= ~r_tone;
            end else begin
                r_tone_cnt <= r_tone_cnt + 1'b1;
            end

            // 300 ms on / 300 ms off blink pattern, only while alarm
            if (I_alarm) begin
                if (r_blink_cnt >= BLINK_CNT - 1) begin
                    r_blink_cnt <= 32'd0;
                    r_blink_on  <= ~r_blink_on;
                end else begin
                    r_blink_cnt <= r_blink_cnt + 1'b1;
                end
            end else begin
                r_blink_cnt <= 32'd0;
                r_blink_on  <= 1'b0;
            end

            O_buzz <= (I_alarm && r_blink_on) ? ~r_tone : 1'b1;
        end
    end

endmodule
