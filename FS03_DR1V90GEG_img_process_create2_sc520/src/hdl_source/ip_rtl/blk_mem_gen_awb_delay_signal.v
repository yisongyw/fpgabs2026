// 原 TD IpTool 生成的 blk_mem_gen_awb_delay_signal 例化了 PH1P 专属原语
// PH1P_LOGIC_ERAM，在 DR1 上无法解析；改为例化可推断的 tdp_eram。
// 接口(端口名/位宽)与原 IP 完全一致，上层 signal_delay.v 无需改动。
`timescale 1ns / 1ps

module blk_mem_gen_awb_delay_signal
(
    output wire [95:0] doa,
    input  wire [95:0] dia,
    input  wire [10:0] addra,
    input  wire        wea,
    input  wire        clka,
    output wire [95:0] dob,
    input  wire [10:0] addrb,
    input  wire        clkb,
    input  wire [95:0] dib,
    input  wire        web
);

    tdp_eram #(
        .DATA_WIDTH_A ( 96    ),
        .ADDR_WIDTH_A ( 11    ),
        .DATA_DEPTH_A ( 2048  ),
        .DATA_WIDTH_B ( 96    ),
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
