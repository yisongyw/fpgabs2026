// ===========================================================================
// FS03_DR1V90GEG_img_process_create2_sc520  --  auto_demo_ctrl.v
//
// Pure-PL innovation module: Auto-Demo Patrol
//   * Toggled by UART command "DEMO" (same 115200 8N1 channel as K1..K4).
//   * When enabled it periodically produces KEY3(style+1) / KEY2(threshold+1)
//     single-clock pulses, auto-iterating the 16 style levels and sweeping the
//     16 threshold levels hands-free, ideal for live demo / unattended showcase.
//   * Shares the same control path as the physical/UART keys
//     (S_key_press -> key_mode_delay_ctrl): no new CDC, no change to the pixel
//     main link, so end-to-end latency is unchanged.
//   * Also outputs O_led_chase[3:0] for a TM1638 LED "chase" status indicator.
// ===========================================================================
`timescale 1ns / 100ps

module auto_demo_ctrl #(
    parameter CLK_FREQ = 100_000_000
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_en,          // 1 = auto-demo on
    output reg         O_key3_pulse,  // style +1 pulse (1 clk)
    output reg         O_key2_pulse,  // threshold +1 pulse (1 clk)
    output reg  [3:0]  O_led_chase    // chase (LED1..LED4)
);

    // style pulse every 2.0 s ; threshold pulse every 1.0 s ; chase every 0.5 s
    localparam integer DIV_STYLE = CLK_FREQ * 2;
    localparam integer DIV_TH    = CLK_FREQ * 1;

    reg [31:0] r_cnt_style;
    reg [31:0] r_cnt_th;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            r_cnt_style <= 32'd0;
            r_cnt_th    <= 32'd0;
            O_key3_pulse<= 1'b0;
            O_key2_pulse<= 1'b0;
            O_led_chase <= 4'b0001;
        end
        else begin
            O_key3_pulse <= 1'b0;
            O_key2_pulse <= 1'b0;

            if (!I_en) begin
                r_cnt_style <= 32'd0;
                r_cnt_th    <= 32'd0;
                O_led_chase <= 4'b0001;
            end
            else begin
                // style pulse (every 2 s)
                if (r_cnt_style == (DIV_STYLE - 1)) begin
                    r_cnt_style <= 32'd0;
                    O_key3_pulse <= 1'b1;
                end
                else
                    r_cnt_style <= r_cnt_style + 32'd1;

                // threshold pulse (every 1 s)
                if (r_cnt_th == (DIV_TH - 1)) begin
                    r_cnt_th <= 32'd0;
                    O_key2_pulse <= 1'b1;
                end
                else
                    r_cnt_th <= r_cnt_th + 32'd1;

                // chase (every 0.5 s)
                if (r_cnt_th == (DIV_TH/2 - 1))
                    O_led_chase <= {O_led_chase[2:0], O_led_chase[3]};
            end
        end
    end

endmodule