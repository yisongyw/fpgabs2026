//-----------------------------------------------------------------------------
// Module     : async_fifo
// Project    : HX1P35Aimg_process_create1
// Description: small generic dual clock FIFO (gray coded pointers).
//              Used to carry the SD image beats from the S_100m_clk domain
//              into the CSI pixel clock domain.
//
//              Resource notes:
//              * full/empty flags are REGISTERED (Cummings style-2) so the
//                next-pointer -> flag -> next-pointer path crosses a register
//                and cannot form a combinational loop (SYN-0413 / SYN-5033).
//              * the array uses a SYNCHRONOUS pre-fetch read so it infers a
//                block RAM instead of distributed LUTRAM -> this keeps the
//                FIFO out of the slice count (PRY-9052 / PRY-7022).
//              * O_rdata keeps the original show-ahead (FWFT) semantics:
//                valid data is already present when O_rempty is low.
//-----------------------------------------------------------------------------
module async_fifo #(
    parameter DATA_W = 98,
    parameter ADDR_W = 8                 // depth = 256 (CDC slack, not a line buffer)
)(
    input  wire              I_wclk,
    input  wire              I_wrst_n,
    input  wire              I_wen,
    input  wire [DATA_W-1:0] I_wdata,
    output wire              O_wfull,

    input  wire              I_rclk,
    input  wire              I_rrst_n,
    input  wire              I_ren,
    output wire [DATA_W-1:0] O_rdata,
    output wire              O_rempty
);

    localparam DEPTH = (1 << ADDR_W);

    reg [DATA_W-1:0] r_mem [0:DEPTH-1] /* fehdl force_ram=1, ram_style="bram" */;

    reg [ADDR_W:0] r_wbin, r_wgray;
    reg [ADDR_W:0] r_rbin, r_rgray;
    reg [ADDR_W:0] r_wgray_rclk, r_wgray_rclk2;
    reg [ADDR_W:0] r_rgray_wclk, r_rgray_wclk2;
    reg            r_wfull,  r_rempty;
    reg [DATA_W-1:0] r_dout;              // BRAM synchronous read register

    function [ADDR_W:0] bin2gray;
        input [ADDR_W:0] b;
        begin
            bin2gray = b ^ (b >> 1);
        end
    endfunction

    //-------------------------------------------------------------------------
    // write side
    //-------------------------------------------------------------------------
    // next pointer is gated by the REGISTERED full flag -> no comb feedback
    wire [ADDR_W:0] S_wbin_next  = r_wbin + {{ADDR_W{1'b0}}, (I_wen & ~r_wfull)};
    wire [ADDR_W:0] S_wgray_next = bin2gray(S_wbin_next);

    // combinational "would-be-full" value, only used to update r_wfull
    wire            S_wfull_val  = (S_wgray_next ==
                      {~r_rgray_wclk2[ADDR_W:ADDR_W-1], r_rgray_wclk2[ADDR_W-2:0]});

    assign O_wfull = r_wfull;

    // memory write port kept in its own purely synchronous block (no async
    // reset) so the array cleanly infers a dual-clock block RAM.
    always @(posedge I_wclk) begin
        if(I_wen && !r_wfull)
            r_mem[r_wbin[ADDR_W-1:0]] <= I_wdata;
    end

    always @(posedge I_wclk or negedge I_wrst_n) begin
        if(!I_wrst_n) begin
            r_wbin  <= 0;
            r_wgray <= 0;
            r_wfull <= 1'b0;
        end
        else begin
            r_wbin  <= S_wbin_next;
            r_wgray <= S_wgray_next;
            r_wfull <= S_wfull_val;
        end
    end

    always @(posedge I_wclk or negedge I_wrst_n) begin
        if(!I_wrst_n) begin
            r_rgray_wclk  <= 0;
            r_rgray_wclk2 <= 0;
        end
        else begin
            r_rgray_wclk  <= r_rgray;
            r_rgray_wclk2 <= r_rgray_wclk;
        end
    end

    //-------------------------------------------------------------------------
    // read side
    //-------------------------------------------------------------------------
    wire [ADDR_W:0] S_rbin_next  = r_rbin + {{ADDR_W{1'b0}}, (I_ren & ~r_rempty)};
    wire [ADDR_W:0] S_rgray_next = bin2gray(S_rbin_next);

    wire            S_rempty_val = (S_rgray_next == r_wgray_rclk2);

    assign O_rempty = r_rempty;

    // Synchronous pre-fetch: r_dout always holds the word at the CURRENT read
    // pointer (it is fetched one cycle early via S_rbin_next). When the FIFO is
    // not empty, r_dout is already valid -> show-ahead behaviour is preserved.
    always @(posedge I_rclk) begin
        r_dout <= r_mem[S_rbin_next[ADDR_W-1:0]];
    end

    assign O_rdata = r_dout;

    always @(posedge I_rclk or negedge I_rrst_n) begin
        if(!I_rrst_n) begin
            r_rbin  <= 0;
            r_rgray <= 0;
            r_rempty <= 1'b1;
        end
        else begin
            r_rbin  <= S_rbin_next;
            r_rgray <= S_rgray_next;
            r_rempty <= S_rempty_val;
        end
    end

    always @(posedge I_rclk or negedge I_rrst_n) begin
        if(!I_rrst_n) begin
            r_wgray_rclk  <= 0;
            r_wgray_rclk2 <= 0;
        end
        else begin
            r_wgray_rclk  <= r_wgray;
            r_wgray_rclk2 <= r_wgray_rclk;
        end
    end

endmodule
