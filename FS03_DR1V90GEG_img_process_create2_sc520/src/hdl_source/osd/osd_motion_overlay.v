//-----------------------------------------------------------------------------
// Module     : osd_motion_overlay
// Project    : HX1P35Aimg_process_improve1
// Description: Motion overlay layer (advanced requirement 2.5 (4)).
//
//   Draws the result of the hardware frame difference detector into the HDMI
//   picture :
//
//     * every 16 x 16 pixel cell in which the three frame difference fired is
//       tinted with a semi transparent red block (alpha = 1/4, two shifts and
//       an addition - no multiplier) -> the "motion heat map"
//     * the bounding box of the whole moving region is drawn as a 2 pixel
//       white rectangle (red when the motion alarm is active)
//
//   The 3 page flag memory lives inside this module :
//
//        write port : CSI pixel clock   (driven by frame_diff3)
//        read  port : HDMI pixel clock  (this module)
//
//   so the two clock domains never touch the same page (three pages are
//   rotated, the reader always takes the page completed one frame ago).
//
//   I_osd_en (SW4) hides the whole layer - exactly like every other overlay of
//   this project the motion indication is switched by SW4.
//-----------------------------------------------------------------------------
module osd_motion_overlay #(
    parameter IMG_WIDTH = 1280,
    parameter IMG_HEIGHT = 720,
    parameter CELL_W    = 16,       // cell size in pixel (power of two)
    parameter CELL_SHIFT = 4        // log2(CELL_W)
)(
    // ---- video (HDMI pixel clock domain) ----
    input  wire             I_clk,
    input  wire             I_rst_n,

    input  wire             I_video_vsync,
    input  wire             I_video_hsync,
    input  wire             I_video_de,
    input  wire             I_video_user,
    input  wire             I_video_last,
    input  wire             I_osd_en,          // SW4
    input  wire [23:0]      I_video_data,

    // ---- flag memory write port (CSI clock domain, from frame_diff3) ----
    input  wire             I_wclk,
    input  wire             I_flag_we,
    input  wire [13:0]      I_flag_addr,       // { page(2), cell(12) }
    input  wire [7:0]       I_flag_data,
    input  wire [1:0]       I_wr_page_g,       // gray coded write page

    // ---- statistics of the last completed frame ----
    input  wire [10:0]      I_bb_xmin,
    input  wire [10:0]      I_bb_xmax,
    input  wire [9:0]       I_bb_ymin,
    input  wire [9:0]       I_bb_ymax,
    input  wire             I_bb_valid,
    input  wire             I_mot_alarm,

    output reg              O_video_vsync,
    output reg              O_video_hsync,
    output reg              O_video_de,
    output reg  [23:0]      O_video_data
);

    localparam GW    = IMG_WIDTH  >> CELL_SHIFT;      // 80
    localparam GH    = IMG_HEIGHT >> CELL_SHIFT;      // 45
    // 3 pages * 3600 cells : address = { page(2 bit), cell(12 bit) }
    localparam FLAG_ADW = 14;
    localparam FLAG_DEPTH = 16384;

    //-------------------------------------------------------------------------
    // motion flag memory : ERAM, dual clock
    //-------------------------------------------------------------------------
    wire [7:0] S_flag_byte;
    wire [13:0] S_raddr;          // declared before use (VBR3-7215)

    motion_eram #(
        .DATA_W ( 8          ),
        .ADDR_W ( FLAG_ADW   ),
        .DEPTH  ( FLAG_DEPTH )
    ) u_flag (
        .I_wclk  ( I_wclk      ),
        .I_we    ( I_flag_we   ),
        .I_waddr ( I_flag_addr ),
        .I_wdata ( I_flag_data ),
        .I_rclk  ( I_clk       ),
        .I_raddr ( S_raddr     ),
        .O_rdata ( S_flag_byte )
    );

    //-------------------------------------------------------------------------
    // page pointer : gray code -> binary, two stage synchroniser
    //-------------------------------------------------------------------------
    function [1:0] gray2bin3;
        input [1:0] g;
        begin
            case(g)
                2'b00: gray2bin3 = 2'd0;
                2'b01: gray2bin3 = 2'd1;
                default: gray2bin3 = 2'd2;
            endcase
        end
    endfunction

    reg [1:0] r_pg_s0, r_pg_s1;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            r_pg_s0 <= 2'b00;
            r_pg_s1 <= 2'b00;
        end
        else begin
            r_pg_s0 <= I_wr_page_g;
            r_pg_s1 <= r_pg_s0;
        end
    end

    wire [1:0] S_wr_bin = gray2bin3(r_pg_s1);
    // read the page that has been completed one frame ago
    wire [1:0] S_rd_bin = (S_wr_bin == 2'd0) ? 2'd2 : (S_wr_bin - 2'd1);

    //-------------------------------------------------------------------------
    // pixel coordinates (same scheme as every other OSD layer)
    //-------------------------------------------------------------------------
    reg [11:0] S_x;
    reg [11:0] S_y;
    reg [11:0] S_x_1d, S_x_2d;
    reg [11:0] S_y_1d, S_y_2d;
    reg        S_de_1d,  S_de_2d;
    reg        S_vs_1d,  S_vs_2d;
    reg        S_hs_1d,  S_hs_2d;
    reg [23:0] S_data_1d, S_data_2d;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
        end
        else if(I_video_user) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
        end
        else if(I_video_de) begin
            if(I_video_last) begin
                S_x <= 12'd0;
                S_y <= S_y + 12'd1;
            end
            else begin
                S_x <= S_x + 12'd1;
            end
        end
    end

    //-------------------------------------------------------------------------
    // cell address of the CURRENT pixel -> the flag byte is available one
    // clock later (synchronous block RAM read)
    //-------------------------------------------------------------------------
    wire [6:0] S_cx = S_x[11:4];               // 0 .. 79
    wire [5:0] S_cy = S_y[9:4];                // 0 .. 44
    wire [11:0] S_cidx = ({S_cy, 6'd0} + {S_cy, 4'd0}) + {5'd0, S_cx};

    assign S_raddr = {S_rd_bin, S_cidx};

    // the byte read during this clock belongs to the pixel S_x_1d / S_y_1d
    reg r_mot_hit;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) r_mot_hit <= 1'b0;
        else         r_mot_hit <= S_flag_byte[0];
    end
    // r_mot_hit is now aligned with S_x_2d / S_y_2d / S_de_2d

    //-------------------------------------------------------------------------
    // hit tests
    //-------------------------------------------------------------------------
    reg        S_box_hit;
    reg [23:0] S_box_col;

    always @(*) begin
        S_box_hit = 1'b0;
        S_box_col = I_mot_alarm ? 24'hff3020 : 24'he0e0e0;

        if(S_de_2d && I_osd_en && I_bb_valid &&
           (S_x_2d >= {1'd0, I_bb_xmin}) && (S_x_2d <= {1'd0, I_bb_xmax}) &&
           (S_y_2d >= {2'd0, I_bb_ymin}) && (S_y_2d <= {2'd0, I_bb_ymax})) begin
            if (((S_x_2d - {1'd0, I_bb_xmin}) < 12'd2) ||
                (({1'd0, I_bb_xmax} - S_x_2d) < 12'd2) ||
                ((S_y_2d - {2'd0, I_bb_ymin}) < 12'd2) ||
                (({2'd0, I_bb_ymax} - S_y_2d) < 12'd2))
                S_box_hit = 1'b1;
        end
    end

    // semi transparent red : 3/4 * video + 1/4 * red   (shifts only, 0 DSP)
    wire [7:0] S_mr = {1'd0, S_data_2d[23:17]} + {2'd0, S_data_2d[23:18]} + 8'd63;
    wire [7:0] S_mg = {1'd0, S_data_2d[15: 9]} + {2'd0, S_data_2d[15:10]};
    wire [7:0] S_mb = {1'd0, S_data_2d[ 7: 1]} + {2'd0, S_data_2d[ 7: 2]};

    //-------------------------------------------------------------------------
    // output : everything delayed by 2 clocks, like every other OSD layer
    //-------------------------------------------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x_1d     <= 12'd0;
            S_x_2d     <= 12'd0;
            S_y_1d     <= 12'd0;
            S_y_2d     <= 12'd0;
            S_de_1d    <= 1'b0;
            S_de_2d    <= 1'b0;
            S_vs_1d    <= 1'b0;
            S_vs_2d    <= 1'b0;
            S_hs_1d    <= 1'b0;
            S_hs_2d    <= 1'b0;
            S_data_1d  <= 24'd0;
            S_data_2d  <= 24'd0;
            O_video_vsync <= 1'b0;
            O_video_hsync <= 1'b0;
            O_video_de    <= 1'b0;
            O_video_data  <= 24'd0;
        end
        else begin
            S_x_1d    <= S_x;
            S_x_2d    <= S_x_1d;
            S_y_1d    <= S_y;
            S_y_2d    <= S_y_1d;

            S_vs_1d   <= I_video_vsync;
            S_vs_2d   <= S_vs_1d;
            S_hs_1d   <= I_video_hsync;
            S_hs_2d   <= S_hs_1d;
            S_de_1d   <= I_video_de;
            S_de_2d   <= S_de_1d;
            S_data_1d <= I_video_data;
            S_data_2d <= S_data_1d;

            O_video_vsync <= S_vs_2d;
            O_video_hsync <= S_hs_2d;
            O_video_de    <= S_de_2d;

            if (S_box_hit)
                O_video_data <= S_box_col;
            else if (S_de_2d && I_osd_en && r_mot_hit)
                O_video_data <= {S_mr, S_mg, S_mb};
            else
                O_video_data <= S_data_2d;
        end
    end

endmodule
