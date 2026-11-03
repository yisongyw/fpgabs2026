//-----------------------------------------------------------------------------
// Module     : hist_stat
// Project    : HX1P35Aimg_process_improve2
// Description: Frame luminance statistics for the automatic image quality
//              control (advanced requirement 2.5-5 : auto exposure / white
//              balance / histogram equalisation).
//
//   The module is a pure SIDE PATH like obj_detect and frame_diff3 : it taps
//   the AWB output stream, it never modifies a pixel, therefore it can not add
//   a single clock of end to end latency and it can not disturb the base layer
//   or the style layer.
//
//   What is measured (one frame, 1280 x 720, 4 pixels per beat)
//   -----------------------------------------------------------
//     * 1 of the 4 pixels of every beat is sampled, the sampling phase rotates
//       (0,1,2,3,0,...) so that the sample set is a uniform 1/4 grid instead of
//       a fixed column stripe.
//         -> 230 400 samples / frame, 320 x 720 beats / frame
//     * Y = R/4 + G/2 + B/4            (same low cost formula as img_proc_top)
//     * hist[0..63]  : 64 bin luminance histogram   (Y >> 2)
//     * sum_y        : the sum of all sampled Y      (exact, for ae_auto)
//     * nsamp        : the number of samples         (exact)
//     * over / under : pixels with Y >= 250 / Y <= 8  (highlight protection)
//     * yavg         : mean luminance (shift approximation, OSD display only)
//
//   Storage
//   -------
//     Two histogram banks are used : while bank A is filled for frame N the
//     finished histogram of frame N-1 (bank B) is read by hist_eq.  At the
//     frame start the banks swap and the new bank is zeroed by a 64 cycle
//     sweep (0.03 % of the frame, those samples are simply skipped).
//     Each bank is a 64 x 20 distributed RAM (1280 bit) - the PH1P35 has
//     plenty of block / distributed RAM but almost no free LUT left, so the
//     histogram must NOT be built from a 64 branch case statement.
//
//   Read-modify-write
//   -----------------
//     One beat can increment the same bin as the beat before it.  Because the
//     RAM write happens one clock after the read, the pending (not yet
//     written) value of the previous beat is bypassed :
//
//         base = (bin == bin_prev) ? pending : ram[bin]
//
//     so no increment is ever lost and only ONE adder is needed.
//
//   Division free
//   -------------
//     No divider is instantiated.  ae_auto compares sum_y with YREF * nsamp
//     (a shift-add by a constant), therefore the control law is EXACT even
//     though the mean value itself is never computed.  The 8 bit O_yavg that
//     is shown on the OSD uses the approximation
//
//         yavg ~= (sum_y >> 18) + (sum_y >> 21) + (sum_y >> 24) + (sum_y >> 26)
//
//     which reproduces 1/230400 with an error below 0.6 % (< 1 LSB at 118).
//-----------------------------------------------------------------------------
module hist_stat #(
    parameter NBIN = 64                 // must be a power of two, <= 64
)(
    input  wire        I_clk,
    input  wire        I_rst_n,

    input  wire        I_en,
    input  wire        I_tvalid,
    input  wire        I_tuser,
    input  wire [95:0] I_tdata,         // 4 x RGB888 (side path, not modified)

    // frame result, stable for the whole following frame
    output reg  [31:0] O_sum_y,
    output reg  [17:0] O_nsamp,
    output reg  [17:0] O_over,
    output reg  [17:0] O_under,
    output reg  [7:0]  O_yavg,
    output reg         O_valid,         // 1 clock pulse at every frame start

    // histogram read port, read by hist_eq during the next frame
    input  wire [5:0]  I_bin_addr,
    output wire [19:0] O_bin_data
);

    //-------------------------------------------------------------------------
    // sampling : rotating phase, 1 of 4 pixels per beat
    //-------------------------------------------------------------------------
    reg [1:0]  r_sel;
    reg [23:0] S_px;

    always @(*) begin
        case(r_sel)
            2'd0:    S_px = I_tdata[95:72];
            2'd1:    S_px = I_tdata[71:48];
            2'd2:    S_px = I_tdata[47:24];
            default: S_px = I_tdata[23:0];
        endcase
    end

    wire [7:0] S_y   = {2'd0, S_px[23:18]} + {1'd0, S_px[15:9]}
                     + {2'd0, S_px[7:2]};
    wire [5:0] S_bin = S_y[7:2];
    wire       S_ovr = (S_y >= 8'd250);
    wire       S_und = (S_y <= 8'd8);

    //-------------------------------------------------------------------------
    // counters
    //-------------------------------------------------------------------------
    reg [31:0] r_sum;
    reg [17:0] r_nsamp;
    reg [17:0] r_over;
    reg [17:0] r_under;

    //-------------------------------------------------------------------------
    // two histogram banks + the RMW pipeline
    //-------------------------------------------------------------------------
    reg [19:0] h0 [0:NBIN-1];
    reg [19:0] h1 [0:NBIN-1];

    reg        r_wb;        // bank that is filled by the running frame
    reg        r_clr;       // zero sweep of the new bank right after the swap
    reg [5:0]  r_clr_i;
    reg [5:0]  r_bin_d;     // bin of the previous beat (write address)
    reg [19:0] r_wdata;     // value that is written into the bank this clock

    wire       S_acc    = I_en & I_tvalid & ~r_clr;
    wire       S_start  = I_en & I_tvalid & I_tuser;

    // read of the bank that is being filled (the previous beat is bypassed)
    wire [19:0] S_hcur = r_wb ? h1[S_bin] : h0[S_bin];
    wire [19:0] S_base = (S_bin == r_bin_d) ? r_wdata : S_hcur;
    wire [19:0] S_new  = S_base + 20'd1;

    // mean luminance, shift only (see the header comment)
    // improve2 (place fix) : r_sum can never exceed 230400 x 255 < 2^26, so
    // r_sum[31:18] is at most 224 and the whole expression fits into 10 bit.
    // The value is bit identical to the previous 32 bit version, but the two
    // wide 32 bit adders (about 45 LUT) collapse into a few narrow ones.
    wire [8:0]  S_ya   = {1'd0, r_sum[25:18]} + {1'd0, r_sum[28:21]};  // <= 252
    wire [9:0]  S_yavg = {2'd0, S_ya} + {2'd0, r_sum[31:24]} + {8'd0, r_sum[26]};

    assign O_bin_data = r_wb ? h0[I_bin_addr] : h1[I_bin_addr];

    //-------------------------------------------------------------------------
    // improve2 (place fix) : the histogram banks must NOT be touched by the
    // async reset.  A storage array that is cleared by the reset loop of an
    // always block can not be mapped into distributed RAM - TD then builds it
    // from plain flip flops plus 64:1 read multiplexer trees, which cost
    // ~1600 LUT + ~2800 FF + 640 f7mux + 320 f8mux and pushed the design over
    // the slice limit (118 %).  Without the reset both banks become 64 x 20
    // distributed RAM (2 x 20 LUT6, on a device that is short on LUTs but has
    // free RAM) and the whole array disappears from the LUT budget.
    //   * initial block  : simulation only, keeps iverilog X-free
    //   * real hardware  : distributed RAM initialises to 0 from the bitstream
    //   * belt & braces  : the zero sweep at every frame start (r_clr) also
    //                      clears the bank that is about to be filled
    //-------------------------------------------------------------------------
    initial begin
        for(i = 0; i < NBIN; i = i + 1) begin
            h0[i] = 20'd0;
            h1[i] = 20'd0;
        end
    end

    integer i;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            r_sel    <= 2'd0;
            r_sum    <= 32'd0;
            r_nsamp  <= 18'd0;
            r_over   <= 18'd0;
            r_under  <= 18'd0;
            r_wb     <= 1'b0;
            r_clr    <= 1'b0;
            r_clr_i  <= 6'd0;
            r_bin_d  <= 6'd0;
            r_wdata  <= 20'd0;
            O_sum_y  <= 32'd0;
            O_nsamp  <= 18'd0;
            O_over   <= 18'd0;
            O_under  <= 18'd0;
            O_yavg   <= 8'd0;
            O_valid  <= 1'b0;
            // h0 / h1 are NOT reset here : see the comment above the initial
            // block (reset storage can not become distributed RAM)
        end
        else begin
            O_valid <= 1'b0;

            if(S_start) begin
                // ---------------------------------------------------------
                // frame start : publish the result of the frame that just
                // ended, swap the banks and zero the new one
                // ---------------------------------------------------------
                // the increment of the very last beat is still pending in
                // r_wdata, it has to be written into the OLD bank before the
                // swap, otherwise one sample per frame would be lost
                if(r_wb) h1[r_bin_d] <= r_wdata;
                else     h0[r_bin_d] <= r_wdata;

                O_sum_y  <= r_sum;
                O_nsamp  <= r_nsamp;
                O_over   <= r_over;
                O_under  <= r_under;
                O_yavg   <= (|S_yavg[9:8]) ? 8'd255 : S_yavg[7:0];
                O_valid  <= 1'b1;

                r_wb     <= ~r_wb;
                r_clr    <= 1'b1;
                r_clr_i  <= 6'd0;
                r_sum    <= 32'd0;
                r_nsamp  <= 18'd0;
                r_over   <= 18'd0;
                r_under  <= 18'd0;
                r_sel    <= 2'd0;
                r_bin_d  <= 6'd0;
                r_wdata  <= 20'd0;
            end
            else if(r_clr) begin
                // 64 cycle zero sweep of the bank that starts now
                if(r_wb) h1[r_clr_i] <= 20'd0;
                else     h0[r_clr_i] <= 20'd0;
                if(r_clr_i == NBIN[5:0] - 6'd1)
                    r_clr <= 1'b0;
                else
                    r_clr_i <= r_clr_i + 6'd1;
            end
            else if(S_acc) begin
                // ---------------------------------------------------------
                // one sample per beat
                // ---------------------------------------------------------
                r_sum   <= r_sum + {24'd0, S_y};
                r_nsamp <= r_nsamp + 18'd1;
                if(S_ovr) r_over  <= r_over  + 18'd1;
                if(S_und) r_under <= r_under + 18'd1;

                r_sel <= r_sel + 2'd1;

                if(r_wb) h1[r_bin_d] <= r_wdata;
                else     h0[r_bin_d] <= r_wdata;

                r_bin_d <= S_bin;
                r_wdata <= S_new;
            end
        end
    end

endmodule
