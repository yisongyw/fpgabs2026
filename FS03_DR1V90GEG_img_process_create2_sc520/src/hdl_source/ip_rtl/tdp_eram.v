// ---------------------------------------------------------------------------
// FS03_DR1V90GEG_mipi_hdmi_SC500_720P : tdp_eram.v
//
// 可推断(Inferable)的真双口 RAM，用来取代原工程里由 TD IpTool 生成的
// blk_mem_gen_* IP。
//
// 原 IP 生成的 RTL 直接例化了 PH1P 器件专属原语 PH1P_LOGIC_ERAM，
// 在 DR1 上 TD 报 "PH1P_LOGIC_ERAM is a black box"，无法使用；
// 而这些 IP 本身只是最普通的真双口 RAM，因此改为行为级描述，
// 由 TD 自动推断成 DR1 的 ERAM 资源，功能与时序完全等价。
//
// 端口行为与原始 IP 参数一致：
//   A 口 WRITEMODE = "NORMAL"           -> 写优先(写周期输出跟随输入)
//   B 口 WRITEMODE = "READBEFOREWRITE"  -> 先读后写(写周期输出为旧值)
//   两口 REGMODE = "NOREG"(输出不额外打拍)，CLKMODE = "ASYNC"(双时钟)
// ---------------------------------------------------------------------------
`timescale 1ns / 1ps

module tdp_eram #(
    parameter DATA_WIDTH_A = 32,
    parameter ADDR_WIDTH_A = 11,
    parameter DATA_DEPTH_A = 2048,
    parameter DATA_WIDTH_B = 32,
    parameter ADDR_WIDTH_B = 11,
    parameter DATA_DEPTH_B = 2048
)(
    input  wire [DATA_WIDTH_A-1:0] dia,
    input  wire [ADDR_WIDTH_A-1:0] addra,
    input  wire                    wea,
    input  wire                    clka,
    output reg  [DATA_WIDTH_A-1:0] doa,
    input  wire [DATA_WIDTH_B-1:0] dib,
    input  wire [ADDR_WIDTH_B-1:0] addrb,
    input  wire                    web,
    input  wire                    clkb,
    output reg  [DATA_WIDTH_B-1:0] dob
);

    reg [DATA_WIDTH_A-1:0] S_mem [0:DATA_DEPTH_A-1];

    // A 口：写优先
    always @(posedge clka) begin
        if (wea) begin
            S_mem[addra] <= dia;
            doa          <= dia;
        end else begin
            doa          <= S_mem[addra];
        end
    end

    // B 口：先读后写
    always @(posedge clkb) begin
        if (web)
            S_mem[addrb] <= dib;
        dob              <= S_mem[addrb];
    end

endmodule
