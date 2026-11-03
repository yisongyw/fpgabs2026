`timescale 1ns / 1ps
/*****************************************************************
Company : MiLianKe Electronic Technology Co., Ltd.
WebSite:https://www.milianke.com
TechWeb:https://www.uisrc.com
tmall-shop:https://milianke.tmall.com
jd-shop:https://milianke.jd.com
taobao-shop: https://milianke.taobao.com
Description: 
The reference demo provided by Milianke is only used for learning. 
We cannot ensure that the demo itself is free of bugs, so users 
should be responsible for the technical problems and consequences
caused by the use of their own products.
@Author      :   XiaoQingquan 
@Time        :   2024/10 
version:     :   
@Description :   
*****************************************************************/

module line_buffer_demosaic #(
    parameter IMG_HEIGHT = 1080,   // ͼ��߶Ȳ�����Ĭ��ֵΪ1080
    parameter IMG_WIDTH  = 1920    // ͼ���Ȳ�����Ĭ��ֵΪ1920
) (
    input              I_clk,      // ����ʱ���ź�
    input              I_rst_n,    // ���븴λ�źţ��͵�ƽ��Ч
    input              I_tuser,    // ����tuser�źţ�֡��ʼ��־ 
    input  wire        I_valid,    // ����������Ч�ź�          
    input  wire [32:0] I_data,     // �����������ߣ����Ϊ33λ  
    output wire        O_valid,    // ���������Ч�ź�          
    output wire [32:0] O_data      // ����������ߣ����Ϊ33λ  
);

    // �����ʹ���ź�
    wire        rd_en;  // synthesis keep 

    // �����ַ�������Ĵ��������ݼĴ���
    reg  [10:0] addra;     // д��ַ������               
    reg  [10:0] addrb;     // ����ַ������               
    reg  [32:0] data_d0;   // ��ʱһ��ʱ�ӵ�����         
    reg         valid_d0;  // ��ʱһ��ʱ�ӵ�������Ч�ź�  

    // ����ֲ�����������ͼ���ȵ��ķ�֮һ�����ڵ�ַ����
    localparam IMG_WIDTH_4x = (IMG_WIDTH >> 2);

    // ���ݺ���Ч�źŵļĴ棬��ʱһ��ʱ��
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            data_d0  <= 0;            // ��λʱ��data_d0����
            valid_d0 <= 0;            // ��λʱ��valid_d0����
        end else begin
            data_d0  <= I_data;       // ��������ʱ��data_d0����I_data
            valid_d0 <= I_valid;      // ��������ʱ��valid_d0����I_valid
        end
    end

    // д��ַ������addra�ĸ���
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n || I_tuser) begin
            addra <= 0;    // ��λ����I_tuser��Чʱ����addra����
        end else if (valid_d0) begin
            // ��valid_d0��Чʱ������addra
            addra <= (addra == IMG_WIDTH_4x - 1) ? 0 : addra + 1'b1;
            // ���addra�ﵽ���ֵ��IMG_WIDTH_4x - 1���������㣬�����1
        end
    end

    // �н�����־tlast_lock_d0�ĸ���
    reg tlast_lock_d0;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n || I_tuser) begin
            tlast_lock_d0 <= 1'b0;  // ��λ����I_tuser��Чʱ����tlast_lock_d0����
        end else if ((tlast_lock_d0 == 1'b0) & (addra == IMG_WIDTH_4x - 1)) begin
            tlast_lock_d0 <= 1'b1;  // ����һ��addra�ﵽ���ֵʱ����λtlast_lock_d0
        end
    end

    // ��ʹ���ź�rd_en������
    assign rd_en = (I_valid && tlast_lock_d0) ? 1'b1 : 1'b0;
    // ֻ�е�I_valid��Ч��tlast_lock_d0��λ�󣬲Ų�����ʹ���ź�

    // ����ַ������addrb�ĸ���
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n || I_tuser) begin
            addrb <= 0;    // ��λ����I_tuser��Чʱ����addrb����
        end else begin
            // ��rd_en��Чʱ������addrb
            addrb <= rd_en ? ((addrb == IMG_WIDTH_4x - 1) ? 0 : addrb + 1'b1) : addrb;
            // ���addrb�ﵽ���ֵ��IMG_WIDTH_4x - 1���������㣬�����1
            // ���rd_en��Ч��addrb���ֲ���
        end
    end

    // �����Ч�ź�O_valid������
    assign O_valid = (valid_d0 && tlast_lock_d0) ? 1'b1 : 1'b0;
    // ֻ�е�valid_d0��Ч��tlast_lock_d0��λ�󣬲������Ч����

    // ʵ������洢�����������ݵĴ洢�Ͷ�ȡ
    blk_mem_gen_demosaic blk_mem_gen_demosaic_r0 (
        .clka (I_clk  ),   // д�˿�ʱ���ź�
        .wea  (valid_d0),  // дʹ���ź�
        .addra(addra  ),   // д��ַ
        .dia (data_d0),   // д����
        .clkb (I_clk  ),   // ���˿�ʱ���ź�
        .addrb(addrb  ),   // ����ַ
        .dob(O_data )    // ���������
    );

endmodule

