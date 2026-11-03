/*******************************MILIANKE*******************************
*Company : MiLianKe Electronic Technology Co., Ltd.
*WebSite:https://www.milianke.com
*TechWeb:https://www.uisrc.com
*tmall-shop:https://milianke.tmall.com
*jd-shop:https://www.jd.com
*taobao-shop1: https://milianke.taobao.com
*Create Date: 2024-06-24
*Module Name:
*File Name:
*Description: 
*The reference demo provided by Milianke is only used for learning. 
*We cannot ensure that the demo itself is free of bugs, so users 
*should be responsible for the technical problems and consequences
*caused by the use of their own products.
*Copyright: Copyright (c) MiLianKe
*All rights reserved.
*Revision: 1.1
*Signal description
*1) I_ input
*2) O_ output
*3) IO_ input output
*4) S_ system internal signal
*5) _n activ low
*6) _dg debug signal 
*7) _r delay or register
*8) _s state mechine
*********************************************************************/
/*******************************mac_cache模块*********************
-- IP -> MAC 地址缓存。
-- 面积优化：固定 IP 的 UDP 收发远端只有一台主机，原 4 表项 CAM 精简为
-- 单表项寄存器，接口与时序保持不变（I_ren 后一拍 O_rmac_done 有效）。
-- 省 3 组 32bit 比较器 + 3 组表项寄存器，减少 slice 占用。
*********************************************************************/
`timescale 1ns/1ps
module	mac_cache
(
	input	wire				I_wclk,
	input	wire				I_reset,
	input	wire				I_wen,
	input	wire	[31:0]		I_wip_addr,
	input	wire	[47:0]		I_wmac_addr,

	input	wire				I_rclk,
	input	wire				I_ren,
	input	wire	[31:0]		I_rip_addr,
	output	reg		[47:0]		O_rmac_addr,
	output	reg					O_rmac_done
);

// 单表项缓存：只保存远端一台主机的 IP->MAC
reg				cache_valid;
reg		[31:0]	cache_ip;
reg		[47:0]	cache_mac;

always@(posedge I_wclk or posedge I_reset) begin
	if(I_reset) begin
		cache_valid	<=	1'b0;
		cache_ip	<=	32'd0;
		cache_mac	<=	48'd0;
	end
	else if(I_wen) begin
		cache_valid	<=	1'b1;
		cache_ip	<=	I_wip_addr;
		cache_mac	<=	I_wmac_addr;
	end
end

always@(posedge I_rclk or posedge I_reset) begin
	if(I_reset) begin
		O_rmac_addr		<=	48'd0;
		O_rmac_done		<=	1'b0;
	end
	else begin
		if(I_ren) begin
			O_rmac_done	<=	1'b1;
			if(cache_valid && I_rip_addr == cache_ip)
				O_rmac_addr	<=	cache_mac;
			else
				O_rmac_addr	<=	48'd0;
		end
		else begin
			O_rmac_done	<=	1'b0;
		end
	end
end

endmodule
