`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// ICMP ping 应答控制模块(面积优化裁剪)
// 本工程固定 IP 的 UDP 收发不需要 ICMP(ping) 应答功能。按裁剪方案 A，将本模块
// 置空(输出全部绑 0)，保留原端口列表，父模块 uiip_rx 无需改动。综合时本模块内部
// 逻辑被剪除，uiip_rx 中由 O_icmp_req_en 等驱动的 ICMP 状态分支因常量 0 一并剪掉，
// UDP 数据通路完全不受影响。
//-----------------------------------------------------------------------------
module uiicmp_pkg_ctrl(
input       			I_reset,
input       			I_clk,
input       			I_icmp_pkg_valid,
input 		[7:0]	   	I_icmp_pkg_data,
output wire      		O_icmp_req_en,
output wire [15:0]  	O_icmp_req_id,
output wire [15:0] 		O_icmp_req_sq_num,
output wire [15:0]		O_icmp_req_checksum,
output wire        		O_icmp_ping_echo_data_valid,
output wire [7 :0] 		O_icmp_ping_echo_data,
output wire[9 :0] 		O_icmp_ping_echo_data_len
);

assign O_icmp_req_en                = 1'b0;
assign O_icmp_req_id                = 16'd0;
assign O_icmp_req_sq_num            = 16'd0;
assign O_icmp_req_checksum          = 16'd0;
assign O_icmp_ping_echo_data_valid  = 1'b0;
assign O_icmp_ping_echo_data        = 8'd0;
assign O_icmp_ping_echo_data_len    = 10'd0;

endmodule
