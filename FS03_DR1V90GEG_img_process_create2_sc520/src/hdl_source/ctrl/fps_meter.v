//-----------------------------------------------------------------------------
// Module     : fps_meter
// Project    : HX1P35Aimg_process_extend2
// Description: Measured frame rate (frames per second) of the camera input.
//
//   The measured value is the number of camera frames that arrived during the
//   last full second, so a real "system is running at xx fps" number is shown
//   instead of the nominal 60.  It is used for the "FPS:nn" line in the bottom
//   left corner of the HDMI picture (extension requirement 5 / optimisation
//   evidence of requirement 4).
//
//   * I_frame  is a one clock pulse in the camera pixel clock domain, it is
//     synchronised into the meter clock domain here
//   * I_clk    is the 100 MHz system clock, so the 1 s time base is exact
//   * O_fps    is updated once per second and saturated at 99
//
//   Cost : ~50 LUT / ~45 DFF, 0 DSP, and it never touches the video path.
//-----------------------------------------------------------------------------
module fps_meter #(
    parameter CLK_FREQ = 32'd100000000     // meter clock frequency in Hz
)(
    input  wire        I_clk,
    input  wire        I_rst_n,

    input  wire        I_frame,            // one pulse per frame (async)

    output reg  [6:0]  O_fps               // 0 .. 99
);

    reg [2:0]  r_sync;
    reg [26:0] r_timer;
    reg [6:0]  r_cnt;

    wire S_tick = r_sync[1] & ~r_sync[2];   // rising edge of the frame pulse

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            r_sync  <= 3'd0;
            r_timer <= CLK_FREQ[26:0] - 27'd1;
            r_cnt   <= 7'd0;
            O_fps   <= 7'd0;
        end
        else begin
            r_sync <= {r_sync[1:0], I_frame};

            if(S_tick)
                r_cnt <= (r_cnt == 7'd99) ? 7'd99 : (r_cnt + 7'd1);

            if(r_timer == 27'd0) begin
                O_fps   <= r_cnt;
                r_cnt   <= 7'd0;
                r_timer <= CLK_FREQ[26:0] - 27'd1;
            end
            else begin
                r_timer <= r_timer - 27'd1;
            end
        end
    end

endmodule
