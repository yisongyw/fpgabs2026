//-----------------------------------------------------------------------------
// Module     : motion_eram
// Project    : HX1P35Aimg_process_improve1
// Description: Simple dual port block memory used by the motion detection
//              (advanced requirement 2.5 (4)) and by the motion flag pages.
//
//   The PH1P35 has plenty of block RAM (ERAM) but is short on LUTs, therefore
//   every bulk storage of the new motion detection path is deliberately moved
//   into ERAM (this is one of the "balanced resource usage" measures of
//   improve1).  The memory is described with the very same coding style that
//   the Anlogic IP generator produces (see
//   ip_source/w128_d512_fifo/soft_fifo_al_*.v -> ram_infer_*), so the TD
//   synthesiser maps it to ERAM while Icarus Verilog still simulates it.
//
//   Port A : write, clocked by I_wclk
//   Port B : read,  clocked by I_rclk  (may be a different, unrelated clock)
//
//   The read is synchronous (one clock latency).  When both ports use the same
//   clock and the same address the read still delivers the OLD content
//   (read first behaviour) - the reference frame pipeline of frame_diff3
//   relies on it.
//-----------------------------------------------------------------------------
module motion_eram #(
    parameter DATA_W = 8,       // data width in bit
    parameter ADDR_W = 12,      // address width in bit
    parameter DEPTH  = 4096     // number of words (<= 2**ADDR_W)
)(
    // write port
    input  wire                 I_wclk,
    input  wire                 I_we,
    input  wire [ADDR_W-1:0]    I_waddr,
    input  wire [DATA_W-1:0]    I_wdata,

    // read port
    input  wire                 I_rclk,
    input  wire [ADDR_W-1:0]    I_raddr,
    output reg  [DATA_W-1:0]    O_rdata
);

    // synthesis attribute comment : force the inference of a block RAM
    reg [DATA_W-1:0] mem [0:DEPTH-1];   /* fehdl force_ram=1, ram_style="bram_fast" */

    always @(posedge I_wclk) begin
        if (I_we) mem[I_waddr] <= I_wdata;
    end

    always @(posedge I_rclk) begin
        O_rdata <= mem[I_raddr];
    end

endmodule
