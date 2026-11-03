//-----------------------------------------------------------------------------
// Module     : sd_img_feed
// Project    : HX1P35Aimg_process_create1
// Description: SD card image source for the detection / recognition
//              interaction (requirement 2b).
//
//   A test image is stored on the SD card as raw RGB565 pixels (little endian
//   per pixel : byte0 = {G[2:0],B[4:0]}, byte1 = {R[4:0],G[5:3]}) at a fixed
//   sector area.  python tool pc_tools/gen_sd_images.py produces the images
//   (SD card "素材") exactly in this format.
//
//   Geometry : 1280 x 720 pixels = 1 843 200 bytes = 3600 sectors, stored as
//   4 consecutive image slots (0 .. 3) starting at I_base_sector.
//
//   The sector reader works in the S_100m_clk domain ; pixels are expanded to
//   RGB888, packed 4 pixels / beat and pushed through an asynchronous FIFO
//   into the CSI pixel clock domain, where the stream is presented with the
//   same protocol the ISP RGB stage produces :
//       O_tdata  = 4 x RGB888, first pixel in [23:0]
//       O_tuser  = 1 on the first beat of the frame
//       O_tlast  = 1 on the last beat of every line (320 beats / line)
//   Valid is only asserted when a beat is available, so the write side of the
//   downstream DDR frame buffer simply idles while the image is loading.
//-----------------------------------------------------------------------------
module sd_img_feed #(
    parameter IMG_W       = 1280,
    parameter IMG_H       = 720,
    parameter SECTOR_SIZE = 512,
    parameter IMG_SECTORS = (IMG_W * IMG_H * 2) / SECTOR_SIZE,   // 3600
    parameter IMG_SLOTS   = 4
)(
    input  wire        I_clk,          // S_100m_clk
    input  wire        I_rst_n,
    input  wire        I_csi_clk,      // CSI pixel clock
    input  wire        I_csi_rst_n,

    // control (S_100m_clk)
    input  wire        I_start,        // load the next image slot
    input  wire [31:0] I_base_sector,
    output reg  [1:0]  O_img_idx,      // slot being / last loaded
    output reg         O_busy,
    output reg         O_img_done,     // one shot : frame fully pushed
    output wire        O_sd_init_done,
    output wire        O_sd_act,       // sector activity (status blink)
    output reg         O_sd_err,

    // SD card SPI pins
    output wire        O_sd_cs_n,
    output wire        O_sd_clk,
    output wire        O_sd_mosi,
    input  wire        I_sd_miso,

    // image stream, CSI pixel clock domain
    output wire [95:0] O_tdata,
    output wire        O_tvalid,
    output wire        O_tuser,
    output wire        O_tlast
);

    //-------------------------------------------------------------------------
    // SD SPI sector reader (S_100m_clk domain)
    //-------------------------------------------------------------------------
    wire        sd_init_done;
    reg         r_sd_sec_read;
    reg  [31:0] r_sd_sec_addr;
    wire [7:0]  sd_sec_data;
    wire        sd_sec_valid;
    wire        sd_sec_end;

    sd_card_top u_sd_card_top (
        .clk                   ( I_clk          ),
        .rst                   ( ~I_rst_n       ),
        .SD_nCS                ( O_sd_cs_n      ),
        .SD_DCLK               ( O_sd_clk       ),
        .SD_MOSI               ( O_sd_mosi      ),
        .SD_MISO               ( I_sd_miso      ),
        .sd_init_done          ( sd_init_done   ),
        .sd_sec_read           ( r_sd_sec_read  ),
        .sd_sec_read_addr      ( r_sd_sec_addr  ),
        .sd_sec_read_data      ( sd_sec_data    ),
        .sd_sec_read_data_valid( sd_sec_valid   ),
        .sd_sec_read_end       ( sd_sec_end     ),
        .sd_sec_write          ( 1'b0           ),
        .sd_sec_write_addr     ( 32'd0          ),
        .sd_sec_write_data     ( 8'd0           ),
        .sd_sec_write_data_req (                ),
        .sd_sec_write_end      (                )
    );

    assign O_sd_init_done = sd_init_done;

    // read FSM
    localparam S_IDLE  = 2'd0;
    localparam S_ISSUE = 2'd1;
    localparam S_READ  = 2'd2;

    reg [1:0]  r_state;
    reg [11:0] r_sec_idx;          // 0 .. 3599
    reg [8:0]  r_byte_cnt;         // 0 .. 511
    reg        r_busy_sec;

    assign O_sd_act       = r_busy_sec | (|r_byte_cnt);

    // image slot start sector = I_base_sector + idx * 3600 (constant folded)
    function [31:0] slot_base;
        input [1:0]  idx;
        input [31:0] base;
        reg [31:0] off;
        begin
            case(idx)
                2'd0: off = 32'd0;
                2'd1: off = IMG_SECTORS;
                2'd2: off = IMG_SECTORS * 2;
                default: off = IMG_SECTORS * 3;
            endcase
            slot_base = base + off;
        end
    endfunction

    //-------------------------------------------------------------------------
    // pixel packing : 2 bytes RGB565 -> RGB888, 4 pixels per beat
    //-------------------------------------------------------------------------
    reg        r_hi;                // byte phase inside one pixel
    reg [7:0]  r_pix_lo;
    reg [23:0] r_pix;
    reg [1:0]  r_pix_cnt;
    reg [95:0] r_beat;
    reg [8:0]  r_beat_cnt;
    reg [9:0]  r_line_cnt;
    reg        r_sof_pending;

    reg        r_fifo_wen;
    reg [97:0] r_fifo_wdata;

    // RGB565 -> RGB888 (bit replication), pixel byte order : lo first
    wire [4:0] S_r5 = sd_sec_data[7:3];
    wire [5:0] S_g6 = {sd_sec_data[2:0], r_pix_lo[7:5]};
    wire [4:0] S_b5 = r_pix_lo[4:0];

    wire [23:0] S_rgb888 = {{S_r5, S_r5[4:2]}, {S_g6, S_g6[5:4]}, {S_b5, S_b5[4:2]}};

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            r_state       <= S_IDLE;
            r_sec_idx     <= 12'd0;
            r_byte_cnt    <= 9'd0;
            r_busy_sec    <= 1'b0;
            r_sd_sec_read <= 1'b0;
            r_sd_sec_addr <= 32'd0;
            r_hi          <= 1'b0;
            r_pix_lo      <= 8'd0;
            r_pix         <= 24'd0;
            r_pix_cnt     <= 2'd0;
            r_beat        <= 96'd0;
            r_beat_cnt    <= 9'd0;
            r_line_cnt    <= 10'd0;
            r_sof_pending <= 1'b0;
            r_fifo_wen    <= 1'b0;
            r_fifo_wdata  <= 98'd0;
            O_img_idx     <= 2'd0;
            O_busy        <= 1'b0;
            O_img_done    <= 1'b0;
            O_sd_err      <= 1'b0;
        end
        else begin
            r_fifo_wen <= 1'b0;
            O_img_done <= 1'b0;

            case(r_state)
                S_IDLE: begin
                    r_busy_sec <= 1'b0;
                    if(I_start && sd_init_done) begin
                        O_img_idx     <= O_img_idx + 2'd1;
                        r_sec_idx     <= 12'd0;
                        r_beat_cnt    <= 9'd0;
                        r_line_cnt    <= 10'd0;
                        r_pix_cnt     <= 2'd0;
                        r_hi          <= 1'b0;
                        r_sof_pending <= 1'b1;
                        O_busy        <= 1'b1;
                        r_state       <= S_ISSUE;
                    end
                    else if(I_start && !sd_init_done) begin
                        O_sd_err <= 1'b1;   // no card / not initialised
                    end
                end

                S_ISSUE: begin
                    r_sd_sec_addr <= slot_base(O_img_idx, I_base_sector) +
                                     {20'd0, r_sec_idx};
                    r_sd_sec_read <= 1'b1;
                    r_byte_cnt    <= 9'd0;
                    r_busy_sec    <= 1'b1;
                    r_state       <= S_READ;
                end

                S_READ: begin
                    r_sd_sec_read <= 1'b0;
                    if(sd_sec_valid) begin
                        r_byte_cnt <= r_byte_cnt + 9'd1;
                        if(!r_hi) begin
                            r_pix_lo <= sd_sec_data;
                            r_hi     <= 1'b1;
                        end
                        else begin
                            r_hi  <= 1'b0;
                            // one full pixel collected
                            case(r_pix_cnt)
                                2'd0: r_beat[23: 0] <= S_rgb888;
                                2'd1: r_beat[47:24] <= S_rgb888;
                                2'd2: r_beat[71:48] <= S_rgb888;
                                2'd3: r_beat[95:72] <= S_rgb888;
                            endcase
                            if(r_pix_cnt == 2'd3) begin
                                // 4 pixels -> push one beat
                                r_fifo_wen   <= 1'b1;
                                r_fifo_wdata <= {(r_beat_cnt == 9'd319),
                                                 (r_beat_cnt == 9'd0) && r_sof_pending,
                                                 S_rgb888, r_beat[71:0]};
                                r_sof_pending <= 1'b0;
                                if(r_beat_cnt == 9'd319) begin
                                    r_beat_cnt <= 9'd0;
                                    r_line_cnt <= r_line_cnt + 10'd1;
                                end
                                else begin
                                    r_beat_cnt <= r_beat_cnt + 9'd1;
                                end
                            end
                            r_pix_cnt <= r_pix_cnt + 2'd1;
                        end
                    end

                    if(sd_sec_end) begin
                        if(r_sec_idx == IMG_SECTORS[11:0] - 12'd1) begin
                            // frame fully pushed
                            O_busy     <= 1'b0;
                            O_img_done <= 1'b1;
                            r_state    <= S_IDLE;
                        end
                        else begin
                            r_sec_idx <= r_sec_idx + 12'd1;
                            r_state   <= S_ISSUE;
                        end
                    end
                end

                default: r_state <= S_IDLE;
            endcase
        end
    end

    //-------------------------------------------------------------------------
    // CSI domain output
    //-------------------------------------------------------------------------
    wire [97:0] S_fifo_rdata;
    wire        S_fifo_empty;
    reg         S_fifo_ren;

    async_fifo #(
        .DATA_W ( 98 ),
        .ADDR_W ( 9  )
    ) u_async_fifo (
        .I_wclk   ( I_clk        ),
        .I_wrst_n ( I_rst_n      ),
        .I_wen    ( r_fifo_wen   ),
        .I_wdata  ( r_fifo_wdata ),
        .O_wfull  (              ),
        .I_rclk   ( I_csi_clk    ),
        .I_rrst_n ( I_csi_rst_n  ),
        .I_ren    ( S_fifo_ren   ),
        .O_rdata  ( S_fifo_rdata ),
        .O_rempty ( S_fifo_empty )
    );

    assign O_tdata  = S_fifo_rdata[95:0];
    assign O_tuser  = S_fifo_rdata[96];
    assign O_tlast  = S_fifo_rdata[97];
    assign O_tvalid = !S_fifo_empty;

    always @(*) begin
        S_fifo_ren = !S_fifo_empty;
    end

endmodule
