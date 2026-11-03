`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// ICMP ping 应答发送模块(面积优化裁剪)
// 本工程固定 IP 的 UDP 收发不需要 ICMP(ping) 应答功能。按裁剪方案 A，将本模块
// 置空(输出全部绑 0)，保留原端口列表，父模块 uiip_tx 无需改动。O_icmp_pkg_req
// 恒为 0 使 uiip_tx 的 SEND_ICMP_PACKET 状态不可达，综合时一并剪除，UDP 通路不变。
//-----------------------------------------------------------------------------
module uiicmp_pkg_tx(
input 	wire		I_clk,
input	wire		I_reset,
input 	wire		I_icmp_req_en,
input 	wire [15:0] I_icmp_req_id,
input 	wire [15:0] I_icmp_req_sq_num,
input   wire [15:0] I_icmp_req_checksum,
input 	wire [31:0] I_icmp_req_ip_addr,

input  	wire [7 :0] I_icmp_ping_echo_data,
input  	wire [9 :0] I_icmp_ping_echo_data_len,
output 	wire       	O_icmp_ping_echo_ren,

input				I_icmp_pkg_busy,
output 	wire		O_icmp_pkg_req,
output 	wire     	O_icmp_pkg_valid,
output 	wire [7 :0] O_icmp_pkg_data,
output 	wire [9 :0] O_icmp_pkg_data_len,
output 	wire [31:0]	O_icmp_pkg_ip_addr
);

assign O_icmp_ping_echo_ren = 1'b0;
assign O_icmp_pkg_req       = 1'b0;
assign O_icmp_pkg_valid     = 1'b0;
assign O_icmp_pkg_data      = 8'd0;
assign O_icmp_pkg_data_len  = 10'd0;
assign O_icmp_pkg_ip_addr   = 32'd0;

endmodule
