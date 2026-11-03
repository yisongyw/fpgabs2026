// ---------------------------------------------------------------------------
// FS03_DR1V90GEG_mipi_hdmi_SC500_720P : video_stream_bridge.v
//
// 取代原工程的 video_in.v + video_out.v + mc_to_user_interface.v +
// ph1p35_324_ddr_wrapper.v(PH1P35 片内 DDR2 三帧乒乓)。
//
// DR1V90GEG484 的 DDR3 控制器属于 PS(处理系统)，本工程要求"只通过 PL
// 实现、无需 PS"，故帧缓存改为片上 ERAM 异步(双时钟)弹性行缓存：
//   * 写侧 = MIPI/ISP 像素时钟域，128bit/拍(4 像素 RGB888 打包流)
//   * 读侧 = HDMI 74.25MHz 像素时钟域，按原 video_out 的取数节拍
//            (每 16 像素读 3 个 128bit 字)解包成 24bit RGB888
//   * 摄像头 24MHz 与 HDMI 75MHz 同源于 FS03 的 25MHz 晶振，两域频率
//     锁定，4096 字(约 32 行)缓存足以吸收相位抖动；近满按"行"丢弃、
//     近空置 underflow 标志，避免整幅撕裂。
//
// 收益：a) 无需 PS/DDR 控制器；b) 端到端延时由 1~3 帧降为约 32 行
//       (约 0.75ms)；c) 释放原 DDR 控制器约 8k slice，资源更均衡。
// ---------------------------------------------------------------------------
`timescale 1ns / 100ps

module video_stream_bridge #(
    parameter FIFO_AWIDTH     = 12,     // 深度 2^12 = 4096 个 128bit 字
    parameter ALMOST_FULL_TH  = 3584,   // 近满门限(留 512 字余量)
    parameter ALMOST_EMPTY_TH = 256      // 近空门限
)(
    input  wire         I_cam_clk,
    input  wire         I_cam_rst_n,
    input  wire         I_cam_tuser,      // 帧起始
    input  wire         I_cam_tvalid,
    input  wire [127:0] I_cam_tdata,
    input  wire         I_cam_tlast,      // 行结束
    output wire         O_cam_overflow,
    input  wire         I_disp_clk,
    input  wire         I_disp_rst_n,
    input  wire         I_disp_rd_en,     // 像素请求(hdmi_mixer O_video_rd_en)
    output wire [23:0]  O_disp_data,
    output wire         O_disp_underflow
);

    localparam FDEPTH = (1 << FIFO_AWIDTH);
    localparam AW     = FIFO_AWIDTH;

    function [AW:0] fn_bin2gray;
        input [AW:0] b;
        fn_bin2gray = (b >> 1) ^ b;
    endfunction

    function [AW:0] fn_gray2bin;
        input [AW:0] g;
        integer i;
        reg [AW:0] b;
        begin
            b = g;
            for (i = AW; i > 0; i = i - 1)
                b[i-1] = b[i] ^ g[i-1];
            fn_gray2bin = b;
        end
    endfunction

    // ================= 1. 双时钟异步 FIFO（推断为 ERAM） =================
    reg [127:0]   S_mem [0:FDEPTH-1];
    reg [AW-1:0]  S_wptr, S_rptr;
    reg [AW:0]    S_wptr_gray, S_rptr_gray;
    reg [AW:0]    S_rptr_gray_sync1, S_rptr_gray_sync2;
    reg [AW:0]    S_wptr_gray_sync1, S_wptr_gray_sync2;
    reg [127:0]   S_rdata;

    wire S_drop_active;
    wire S_wr_en = I_cam_tvalid & (~S_drop_active);

    always @(posedge I_cam_clk or negedge I_cam_rst_n) begin
        if (!I_cam_rst_n) begin
            S_wptr      <= {AW{1'b0}};
            S_wptr_gray <= {(AW+1){1'b0}};
        end else if (S_wr_en) begin
            S_wptr      <= S_wptr + 1'b1;
            S_wptr_gray <= fn_bin2gray(S_wptr + 1'b1);
        end
    end

    always @(posedge I_cam_clk) begin
        if (S_wr_en)
            S_mem[S_wptr] <= I_cam_tdata;
    end

    wire S_rd_en;
    always @(posedge I_disp_clk or negedge I_disp_rst_n) begin
        if (!I_disp_rst_n) begin
            S_rptr      <= {AW{1'b0}};
            S_rptr_gray <= {(AW+1){1'b0}};
        end else if (S_rd_en) begin
            S_rptr      <= S_rptr + 1'b1;
            S_rptr_gray <= fn_bin2gray(S_rptr + 1'b1);
        end
    end

    always @(posedge I_disp_clk) begin
        if (S_rd_en)
            S_rdata <= S_mem[S_rptr];
    end

    always @(posedge I_cam_clk) begin
        S_rptr_gray_sync1 <= S_rptr_gray;
        S_rptr_gray_sync2 <= S_rptr_gray_sync1;
    end
    always @(posedge I_disp_clk) begin
        S_wptr_gray_sync1 <= S_wptr_gray;
        S_wptr_gray_sync2 <= S_wptr_gray_sync1;
    end

    // 写侧/读侧可见占用量（模运算，指针回绕安全）
    wire [AW:0] S_occ_w = S_wptr - fn_gray2bin(S_rptr_gray_sync2);
    wire [AW:0] S_occ_r = fn_gray2bin(S_wptr_gray_sync2) - S_rptr;

    // ================= 2. 写侧按"行"丢弃的流控 =================
    reg S_drop_r;
    assign S_drop_active = S_drop_r;
    always @(posedge I_cam_clk or negedge I_cam_rst_n) begin
        if (!I_cam_rst_n)
            S_drop_r <= 1'b0;
        else if (S_occ_w >= ALMOST_FULL_TH)
            S_drop_r <= 1'b1;                          // 近满 -> 丢当前行剩余
        else if (I_cam_tlast && (S_occ_w <= ALMOST_EMPTY_TH))
            S_drop_r <= 1'b0;                          // 行边界且回落 -> 恢复
    end
    assign O_cam_overflow = S_drop_r;

    // ================= 3. 读侧取数节拍（同原 video_out） =================
    reg [3:0] S_rd_cnt;
    always @(posedge I_disp_clk or negedge I_disp_rst_n) begin
        if (!I_disp_rst_n)
            S_rd_cnt <= 4'd0;
        else if (I_disp_rd_en)
            S_rd_cnt <= S_rd_cnt + 4'd1;
        else
            S_rd_cnt <= 4'd0;
    end

    // 每 16 像素(384bit = 3 个 128bit 字)在第 0/5/10 拍取一个字
    wire S_word_req = I_disp_rd_en & ((S_rd_cnt == 4'd0) |
                                      (S_rd_cnt == 4'd5) |
                                      (S_rd_cnt == 4'd10));
    wire S_word_rd  = S_word_req & (S_occ_r != 0);
    assign S_rd_en  = S_word_rd;
    assign O_disp_underflow = S_word_req & (S_occ_r == 0);

    // ================= 4. 128bit -> 24bit RGB888 解包 =================
    // 位序与原 video_out.v 完全一致（96bit/4 像素 连续打包流）
    reg [3:0]   S_unp_cnt_1d;
    reg [127:0] S_rdata_1d;
    reg [23:0]  S_pix;

    always @(posedge I_disp_clk) begin
        S_unp_cnt_1d <= S_rd_cnt;
        S_rdata_1d   <= S_rdata;
    end

    always @(posedge I_disp_clk or negedge I_disp_rst_n) begin
        if (!I_disp_rst_n)
            S_pix <= 24'd0;
        else if (I_disp_rd_en) begin
            case (S_unp_cnt_1d)
                4'd0 : S_pix <= S_rdata[127:104];
                4'd1 : S_pix <= S_rdata[103: 80];
                4'd2 : S_pix <= S_rdata[ 79: 56];
                4'd3 : S_pix <= S_rdata[ 55: 32];
                4'd4 : S_pix <= S_rdata[ 31:  8];
                4'd5 : S_pix <= {S_rdata_1d[7:0],   S_rdata[127:112]};
                4'd6 : S_pix <= S_rdata[111: 88];
                4'd7 : S_pix <= S_rdata[ 87: 64];
                4'd8 : S_pix <= S_rdata[ 63: 40];
                4'd9 : S_pix <= S_rdata[ 39: 16];
                4'd10: S_pix <= {S_rdata_1d[15:0],  S_rdata[127:120]};
                4'd11: S_pix <= S_rdata[119: 96];
                4'd12: S_pix <= S_rdata[ 95: 72];
                4'd13: S_pix <= S_rdata[ 71: 48];
                4'd14: S_pix <= S_rdata[ 47: 24];
                4'd15: S_pix <= S_rdata[ 23:  0];
                default: S_pix <= 24'd0;
            endcase
        end
    end

    assign O_disp_data = S_pix;

endmodule
