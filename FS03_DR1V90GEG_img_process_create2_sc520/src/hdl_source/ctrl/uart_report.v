//==========================================================================
// uart_report.v  (HX1P35Aimg_process_extend3)
//--------------------------------------------------------------------------
// Periodically (or on I_report) sends a short ASCII / binary status packet
// over the UART (115200 bps) describing the currently tracked objects.
//
// Packet (12 bytes, LSB first, 8N1):
//   [0] 0xA5              preamble
//   [1] {valid, mlp[2:0], rule[2:0], alarm}   track 0   (low 3 class bits)
//   [2] {mlp[5:3], rule[5:3], 2'd0}          track 0   (high 3 class bits)
//   [3] cx0[9:2]
//   [4] cy0[7:0]
//   [5] {valid, mlp[2:0], rule[2:0], alarm}  track 1   (low 3 class bits)
//   [6] {mlp[5:3], rule[5:3], 2'd0}          track 1   (high 3 class bits)
//   [7] cx1[9:2]
//   [8] cy1[7:0]
//   [9] 0x5A              tail
//   [10] 0x0D  CR
//   [11] 0x0A  LF
//
// improve4 : 64 classes need 6 bit per class index.  Instead of dropping the
//   upper bits (a 32 / 64 class model would then be indistinguishable on the
//   wire) one extra byte per track carries the upper three bits of both class
//   indices.  Preamble, tail, CR/LF, baud rate and the auto report period are
//   unchanged, so an existing receiver still sees a well formed frame.
//
// The packet is built from the extended 64-bit track records produced by
// obj_tracker.  A 100 MHz system clock is assumed (CLK_HZ parameter).
//==========================================================================
`timescale 1ns / 1ps

module uart_report
#(
    parameter CLK_HZ  = 100_000_000,
    parameter BAUD    = 115200,
    parameter NBYTES  = 12,          // improve4 : 10 -> 12 (6 bit class indices)
    parameter PERIOD  = 250_000_000,  // auto-report period in clocks (250 ms)
    parameter CLS_W   = 6,            // bit width of a class index (2**CLS_W max)
    parameter TRK_W   = 58 + 2*CLS_W  // width of one extended track record (6 -> 70)
)
(
    input  wire          I_clk,
    input  wire          I_rst,
    input  wire          I_report,        // 1-clock pulse forces a report
    input  wire [2*TRK_W-1:0] I_trk_bus, // 2 extended track records
    input  wire          I_alarm,         // alarm of track 0 (ROI)
    output reg           O_uart_tx
);

    localparam BAUD_DIV = CLK_HZ / BAUD;

    //----------------------------------------------------------------------
    // extended track record layout - DERIVED from CLS_W (same as obj_tracker)
    //   CLS_W = 3 -> the historical 64 bit record
    //----------------------------------------------------------------------
    localparam T_MISS = 0;
    localparam T_ZERO = T_MISS + 4;
    localparam T_CONF = T_ZERO + 1;
    localparam T_MLP  = T_CONF + 7;
    localparam T_RULE = T_MLP  + CLS_W;
    localparam T_ID   = T_RULE + CLS_W;
    localparam T_YBR  = T_ID   + 3;
    localparam T_YTL  = T_YBR  + 10;
    localparam T_XBR  = T_YTL  + 10;
    localparam T_XTL  = T_XBR  + 11;
    localparam T_VAL  = T_XTL  + 11;        // == TRK_W-1

    // ---- unpack track 0 ----
    wire              v0  = I_trk_bus[0*TRK_W + T_VAL];
    wire [CLS_W-1:0]  r0  = I_trk_bus[(0*TRK_W + T_RULE + CLS_W - 1) -: CLS_W];
    wire [CLS_W-1:0]  m0  = I_trk_bus[(0*TRK_W + T_MLP  + CLS_W - 1) -: CLS_W];
    wire              a0  = I_alarm;
    wire [10:0]       cx0 = I_trk_bus[(0*TRK_W + T_XTL + 10) -: 11];
    wire [9:0]        cy0 = I_trk_bus[(0*TRK_W + T_YTL +  9) -: 10];

    // ---- unpack track 1 ----
    wire              v1  = I_trk_bus[1*TRK_W + T_VAL];
    wire [CLS_W-1:0]  r1  = I_trk_bus[(1*TRK_W + T_RULE + CLS_W - 1) -: CLS_W];
    wire [CLS_W-1:0]  m1  = I_trk_bus[(1*TRK_W + T_MLP  + CLS_W - 1) -: CLS_W];
    wire              a1  = 1'b0;
    wire [10:0]       cx1 = I_trk_bus[(1*TRK_W + T_XTL + 10) -: 11];
    wire [9:0]        cy1 = I_trk_bus[(1*TRK_W + T_YTL +  9) -: 10];

    reg [7:0] r_pkt [0:NBYTES-1];
    reg [7:0] r_byte_idx;
    reg [3:0] r_bit;
    reg [15:0] r_baud;
    reg [9:0] r_shift;
    reg r_busy;
    reg [31:0] r_timer;

    // build the packet bytes (combinational on the records)
    always @(*) begin
        r_pkt[0]  = 8'hA5;
        // low three bits of both class indices (identical to the old layout)
        r_pkt[1]  = { v0, m0[2:0], r0[2:0], a0 };
        // improve4 : upper three bits of both class indices of track 0.
        //   With CLS_W < 6 the missing bits read as 0, so a 8 / 32 class model
        //   produces exactly the same bytes as before.
        r_pkt[2]  = { m0[5:3], r0[5:3], 2'd0 };
        r_pkt[3]  = cx0[9:2];
        r_pkt[4]  = cy0[7:0];
        r_pkt[5]  = { v1, m1[2:0], r1[2:0], a1 };
        r_pkt[6]  = { m1[5:3], r1[5:3], 2'd0 };
        r_pkt[7]  = cx1[9:2];
        r_pkt[8]  = cy1[7:0];
        r_pkt[9]  = 8'h5A;
        r_pkt[10] = 8'h0D;
        r_pkt[11] = 8'h0A;
    end

    always @(posedge I_clk) begin
        if (I_rst) begin
            r_baud     <= 16'd0;
            r_bit      <= 4'd0;
            r_byte_idx <= 8'd0;
            r_shift    <= 10'h3FF;   // idle = all 1
            r_busy     <= 1'b0;
            r_timer    <= 32'd0;
            O_uart_tx  <= 1'b1;
        end else begin
            // auto-report timer
            if (r_timer >= PERIOD - 1) r_timer <= 32'd0;
            else r_timer <= r_timer + 1'b1;

            if (!r_busy && (I_report || (r_timer == PERIOD - 1))) begin
                r_byte_idx <= 8'd0;
                r_bit      <= 4'd0;
                r_baud     <= 16'd0;
                r_shift    <= { 1'b1, r_pkt[0], 1'b0 };
                r_busy     <= 1'b1;
            end else if (r_busy) begin
                if (r_baud >= BAUD_DIV - 1) begin
                    r_baud <= 16'd0;
                    if (r_bit < 4'd9) begin
                        r_bit   <= r_bit + 1'b1;
                        r_shift <= { 1'b1, r_shift[9:1] };
                    end else begin
                        if (r_byte_idx < NBYTES - 1) begin
                            r_byte_idx <= r_byte_idx + 1'b1;
                            r_bit      <= 4'd0;
                            r_shift    <= { 1'b1, r_pkt[r_byte_idx + 1'b1], 1'b0 };
                        end else begin
                            r_busy  <= 1'b0;
                            r_shift <= 10'h3FF;
                        end
                    end
                end else begin
                    r_baud <= r_baud + 1'b1;
                end
            end
            O_uart_tx <= r_shift[0];
        end
    end

endmodule
