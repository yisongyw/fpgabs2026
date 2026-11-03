//==========================================================================
// latency_meter.v  (HX1P35Aimg_process_create1)
//--------------------------------------------------------------------------
// Camera-to-display latency visualisation for advanced requirement 2.5 (3).
//
// The camera and display frame counters live in their native clock domains.
// Their Gray-coded values cross into the 100 MHz measurement domain through
// two flip-flop synchronisers, so a frame transition can never expose a
// multi-bit binary intermediate value.  The measured value is
//
//   latency = (frame_delta - 1) * 16.7 ms + sub-frame phase
//
// The phase counter has 0.1 ms resolution (10,000 clocks at 100 MHz).  This is
// precise enough for the OSD while avoiding a wide divider or per-pixel logic.
// A 100-sample min/max window also reports peak-to-peak latency jitter.
//==========================================================================
`timescale 1ns / 1ps

module latency_meter #(
    parameter integer TENTH_MS_TICKS = 10000,
    parameter integer FRAME_TENTH_MS = 167,
    parameter integer JITTER_SAMPLES = 100
)(
    input  wire       I_clk,                  // 100 MHz measurement clock
    input  wire       I_rst,                  // active high
    input  wire       I_cam_clk,              // camera / CSI pixel clock
    input  wire       I_cam_frame_pulse,
    input  wire       I_disp_clk,             // HDMI pixel clock
    input  wire       I_disp_frame_pulse,
    output reg  [7:0] O_latency_ms,           // compatibility: integer ms
    output reg  [7:0] O_latency_frames,
    output reg  [9:0] O_latency_tenth_ms,     // 0.1 ms, saturated at 99.9 ms
    output reg  [9:0] O_jitter_tenth_ms,      // peak-to-peak over 100 samples
    output reg        O_valid
);

    // Native-domain binary counters and Gray-code launch registers.
    reg [7:0] r_cam_bin,  r_cam_gray;
    reg [7:0] r_disp_bin, r_disp_gray;

    wire [7:0] w_cam_bin_next  = r_cam_bin  + 8'd1;
    wire [7:0] w_disp_bin_next = r_disp_bin + 8'd1;
    wire [7:0] w_cam_gray_next = w_cam_bin_next ^ (w_cam_bin_next >> 1);
    wire [7:0] w_disp_gray_next = w_disp_bin_next ^ (w_disp_bin_next >> 1);

    always @(posedge I_cam_clk) begin
        if(I_rst) begin
            r_cam_bin  <= 8'd0;
            r_cam_gray <= 8'd0;
        end
        else if(I_cam_frame_pulse) begin
            r_cam_bin  <= w_cam_bin_next;
            r_cam_gray <= w_cam_gray_next;
        end
    end

    always @(posedge I_disp_clk) begin
        if(I_rst) begin
            r_disp_bin  <= 8'd0;
            r_disp_gray <= 8'd0;
        end
        else if(I_disp_frame_pulse) begin
            r_disp_bin  <= w_disp_bin_next;
            r_disp_gray <= w_disp_gray_next;
        end
    end

    function [7:0] gray_to_bin;
        input [7:0] gray;
        integer i;
        begin
            gray_to_bin[7] = gray[7];
            for(i = 6; i >= 0; i = i - 1)
                gray_to_bin[i] = gray_to_bin[i + 1] ^ gray[i];
        end
    endfunction

    // Measurement-domain synchronisers.  ASYNC_REG attributes are understood
    // by TD and keep the two stages together during placement.
    (* ASYNC_REG = "TRUE" *) reg [7:0] r_cam_gray_s1,  r_cam_gray_s2;
    (* ASYNC_REG = "TRUE" *) reg [7:0] r_disp_gray_s1, r_disp_gray_s2;
    reg [7:0] r_cam_gray_prev, r_disp_gray_prev;

    wire [7:0] w_cam_bin_sync  = gray_to_bin(r_cam_gray_s2);
    wire [7:0] w_disp_bin_sync = gray_to_bin(r_disp_gray_s2);
    wire       w_cam_tick      = (r_cam_gray_s2  != r_cam_gray_prev);
    wire       w_disp_tick     = (r_disp_gray_s2 != r_disp_gray_prev);
    wire [7:0] w_frame_delta   = w_cam_bin_sync - w_disp_bin_sync;

    // Sub-frame phase in tenths of a millisecond.  14 bits cover 0..9999.
    reg [13:0] r_tenth_div;
    reg [7:0]  r_phase_tenth;

    // Constant multiplication is implemented as a small shift/add network (or
    // may be packed into a spare DSP by TD); it is evaluated only at disp_tick.
    wire [15:0] w_frame_base = (w_frame_delta == 8'd0) ? 16'd0 :
                               (w_frame_delta - 8'd1) * FRAME_TENTH_MS;
    wire [16:0] w_total_raw = {1'b0, w_frame_base} + {9'd0, r_phase_tenth};
    wire [9:0]  w_total_cap = (w_total_raw > 17'd999) ? 10'd999 :
                                                               w_total_raw[9:0];

    reg [6:0] r_jitter_count;
    reg [9:0] r_jitter_min, r_jitter_max;
    wire [9:0] w_jitter_min_next = (w_total_cap < r_jitter_min) ?
                                    w_total_cap : r_jitter_min;
    wire [9:0] w_jitter_max_next = (w_total_cap > r_jitter_max) ?
                                    w_total_cap : r_jitter_max;

    always @(posedge I_clk) begin
        if(I_rst) begin
            r_cam_gray_s1   <= 8'd0;
            r_cam_gray_s2   <= 8'd0;
            r_disp_gray_s1  <= 8'd0;
            r_disp_gray_s2  <= 8'd0;
            r_cam_gray_prev <= 8'd0;
            r_disp_gray_prev<= 8'd0;
            r_tenth_div     <= 14'd0;
            r_phase_tenth   <= 8'd0;
            r_jitter_count  <= 7'd0;
            r_jitter_min    <= 10'd999;
            r_jitter_max    <= 10'd0;
            O_latency_ms    <= 8'd0;
            O_latency_frames <= 8'd0;
            O_latency_tenth_ms <= 10'd0;
            O_jitter_tenth_ms  <= 10'd0;
            O_valid         <= 1'b0;
        end
        else begin
            r_cam_gray_s1    <= r_cam_gray;
            r_cam_gray_s2    <= r_cam_gray_s1;
            r_disp_gray_s1   <= r_disp_gray;
            r_disp_gray_s2   <= r_disp_gray_s1;
            r_cam_gray_prev  <= r_cam_gray_s2;
            r_disp_gray_prev <= r_disp_gray_s2;

            if(w_cam_tick) begin
                r_tenth_div   <= 14'd0;
                r_phase_tenth <= 8'd0;
            end
            else if(r_tenth_div >= TENTH_MS_TICKS - 1) begin
                r_tenth_div <= 14'd0;
                if(r_phase_tenth != 8'hff)
                    r_phase_tenth <= r_phase_tenth + 8'd1;
            end
            else begin
                r_tenth_div <= r_tenth_div + 14'd1;
            end

            // A zero delta occurs during startup before both streams are live;
            // ignore it so the OSD never advertises a false valid measurement.
            if(w_disp_tick && (w_frame_delta != 8'd0)) begin
                O_latency_frames   <= w_frame_delta;
                O_latency_tenth_ms <= w_total_cap;
                O_latency_ms       <= w_total_cap / 10;
                O_valid            <= 1'b1;

                if(r_jitter_count == 7'd0) begin
                    r_jitter_min   <= w_total_cap;
                    r_jitter_max   <= w_total_cap;
                    r_jitter_count <= 7'd1;
                end
                else if(r_jitter_count >= JITTER_SAMPLES - 1) begin
                    O_jitter_tenth_ms <= w_jitter_max_next - w_jitter_min_next;
                    r_jitter_min      <= 10'd999;
                    r_jitter_max      <= 10'd0;
                    r_jitter_count    <= 7'd0;
                end
                else begin
                    r_jitter_min   <= w_jitter_min_next;
                    r_jitter_max   <= w_jitter_max_next;
                    r_jitter_count <= r_jitter_count + 7'd1;
                end
            end
        end
    end

endmodule
