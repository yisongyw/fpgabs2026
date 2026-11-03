// 原 TD IpTool 生成的 blk_mem_gen_demosaic 例化了 PH1P 专属原语
// PH1P_LOGIC_ERAM，在 DR1 上无法解析；改为例化可推断的 tdp_eram。
// 接口(端口名/位宽)与原 IP 完全一致，上层 line_buffer_demosaic.v 无需改动。
`timescale 1ns / 1ps

module blk_mem_gen_demosaic
(
    input  wire [32:0] dia,
    input  wire [10:0] addra,
    input  wire        wea,
    input  wire        clka,
    input  wire [10:0] addrb,
    input  wire        clkb,
    input  wire [32:0] dib,
    input  wire        web,
    output wire [32:0] doa,
    output wire [32:0] dob
);

    tdp_eram #(
        .DATA_WIDTH_A ( 33    ),
        .ADDR_WIDTH_A ( 11    ),
        .DATA_DEPTH_A ( 2048  ),
        .DATA_WIDTH_B ( 33    ),
        .ADDR_WIDTH_B ( 11    ),
        .DATA_DEPTH_B ( 2048  )
    ) u_tdp_eram (
        .dia   ( dia   ),
        .addra ( addra ),
        .wea   ( wea   ),
        .clka  ( clka  ),
        .dib   ( dib   ),
        .addrb ( addrb ),
        .web   ( web   ),
        .clkb  ( clkb  ),
        .doa   ( doa   ),
        .dob   ( dob   )
    );

endmodule
