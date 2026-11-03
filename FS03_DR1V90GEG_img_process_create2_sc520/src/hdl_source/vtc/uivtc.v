/*******************************MILIANKE*******************************
*Company : MiLianKe Electronic Technology Co., Ltd.
*WebSite:https://www.milianke.com
*TechWeb:https://www.uisrc.com
*tmall-shop:https://milianke.tmall.com
*jd-shop:https://milianke.jd.com
*taobao-shop1: https://milianke.taobao.com
*Create Date: 2023/03/23
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

/*************uivtc(video timing controller)��Ƶʱ�������*************
--�汾��1.1
--��������������Ƶ�uivtc(video timing controller)��Ƶʱ�������
--1.�����࣬ռ�ü����߼���Դ������ṹ�������߼�����Ͻ�
--2.ʹ�÷��㣬ֻ��Ҫ����6�������ȿ���ʵ�ֶԲ�ͬ��Ƶ�ֱ���ʱ��Ŀ���
--3.����Ƶʱ����ƣ�һ��ʱ�Ӷ�Ӧһ������
--4.ͨ������˵�����أ�����1080P������1920*1080��ָ��Ƶ����Ч��ʾ����ʵ�ʵ���Ƶ������������ʾ�����򣬱�����ͬ������ͬ��ʱ��
--5.ͨ������˵������Ƶ�źţ�Ҳ��֮Ϊ��Ƶ��ˮƽ�����źţ�����Ƶ�źţ�Ҳ��֮Ϊ��Ƶ�Ĵ�ֱ�����źţ�
*********************************************************************/

`timescale 1ns / 1ns //����ʱ��̶�/����

module uivtc#
(
parameter H_ActiveSize  =   1980,               //��Ƶʱ�����,����Ƶ�źţ�һ����Ч(��Ҫ��ʾ�Ĳ���)������ռ��ʱ������һ��ʱ�Ӷ�Ӧһ����Ч����
parameter H_FrameSize   =   1920+88+44+148,     //��Ƶʱ�����,����Ƶ�źţ�һ����Ƶ�ź��ܼ�ռ�õ�ʱ����
parameter H_SyncStart   =   1920+88,            //��Ƶʱ�����,��ͬ����ʼ��������ʱ������ʼ������ͬ���ź� 
parameter H_SyncEnd     =   1920+88+44,         //��Ƶʱ�����,��ͬ��������������ʱ������ֹͣ������ͬ���źţ�֮���������Ч���ݲ���

parameter V_ActiveSize  =   1080,               //��Ƶʱ�����,����Ƶ�źţ�һ֡ͼ����ռ�õ���Ч(��Ҫ��ʾ�Ĳ���)��������ͨ��˵����Ƶ�ֱ��ʼ�H_ActiveSize*V_ActiveSize
parameter V_FrameSize   =   1080+4+5+36,        //��Ƶʱ�����,����Ƶ�źţ�һ֡��Ƶ�ź��ܼ�ռ�õ�������
parameter V_SyncStart   =   1080+4,             //��Ƶʱ�����,��ͬ����ʼ��������������ʼ������ͬ���ź� 
parameter V_SyncEnd     =   1080+4+5            //��Ƶʱ�����,��ͬ�������������ٳ�����ֹͣ������ͬ���źţ�֮����ǳ���Ч���ݲ���
)
(
input           I_vtc_rstn,     //ϵͳ��λ
input			I_vtc_clk,      //ϵͳʱ��
output			O_vtc_vs,       //��ͬ�����
output          O_vtc_hs,       //��ͬ�����
output          O_vtc_de_valid, //��Ƶ������Ч	
output          O_vtc_user,     //����streamʱ����� user �ź�,����֡ͬ��
output          O_vtc_last      //����streamʱ����� later �ź�,����ÿ�н���
);




reg [11:0] hcnt = 12'd0;    //��Ƶˮƽ�����м��������Ĵ���
reg [11:0] vcnt = 12'd0;    //��Ƶ��ֱ�����м��������Ĵ���   
reg [2 :0] rst_cnt = 3'd0;  //��λ���������Ĵ���
wire rst_sync = rst_cnt[2]; //ͬ����λ

always @(posedge I_vtc_clk or negedge I_vtc_rstn)begin //ͨ������������ͬ����λ
    if(I_vtc_rstn == 1'b0)
        rst_cnt <= 3'd0;
    else if(rst_cnt[2] == 1'b0)
        rst_cnt <= rst_cnt + 1'b1;
end    


//��Ƶˮƽ�����м�����
always @(posedge I_vtc_clk)begin
    if(rst_sync == 1'b0) //��λ
        hcnt <= 12'd0;
    else if(hcnt < (H_FrameSize - 1'b1))//������Χ��0 ~ H_FrameSize-1
        hcnt <= hcnt + 1'b1;
    else 
        hcnt <= 12'd0;
end         

//��Ƶ��ֱ�����м����������ڼ����Ѿ���ɵ�����Ƶ�ź�
always @(posedge I_vtc_clk)begin
    if(rst_sync == 1'b0)
        vcnt <= 12'd0;
    else if(hcnt == (H_ActiveSize  - 1'b1)) begin//��Ƶˮƽ�����Ƿ�һ�н���
           vcnt <= (vcnt == (V_FrameSize - 1'b1)) ? 12'd0 : vcnt + 1'b1;//��Ƶ��ֱ�����м�������1��������Χ0~V_FrameSize - 1
    end
end 

wire hs_valid  =  hcnt < H_ActiveSize; //���ź���Ч���ز���
wire vs_valid  =  vcnt < V_ActiveSize; //���ź���Ч���ز���
wire vtc_hs    =  (hcnt >= H_SyncStart && hcnt < H_SyncEnd);//����hs����ͬ���ź�
wire vtc_vs	   =  (vcnt > V_SyncStart && vcnt <= V_SyncEnd);//����vs����ͬ���ź�      
wire vtc_de    =  hs_valid && vs_valid;//ֻ�е���Ƶˮƽ��������Ч����Ƶ��ֱ������ͬʱ��Ч����Ƶ���ݲ��ֲ�����Ч

//**********************  video stream video rgb  ***************************
//���������RGBʱ����ôתΪstreamʱ��
reg   vtc_vs_r1  ;
reg   vtc_hs_r1  ;
reg   vtc_de_r1  ;
reg   vtc_user_r1 ,vtc_user_r2;
reg   vtc_valid_r1,vtc_valid_r2;
reg   vtc_last_r2;
reg   vs_start;
 
always @(posedge I_vtc_clk )begin
    if(rst_sync == 1'b0) //��λ
        vs_start <= 1'b0;
    else if(vtc_user_r1)//���VS֡ͬ��
        vs_start <= 1'b0;
    else if(vtc_vs && vtc_vs_r1==1'b0)//��vtc_vs�����������������һ֡��ʼ
        vs_start <= 1'b1;
end  
      
always @(posedge I_vtc_clk  )begin
    vtc_vs_r1    <= vtc_vs;
    vtc_hs_r1    <= vtc_hs;
    vtc_user_r1  <= ~vtc_user_r1 & vs_start & vtc_de;//vtc_user�ӳ�1��
    vtc_last_r2  <= ~vtc_de & vtc_valid_r1; //����stream video last �ӳ�����������2��
    vtc_valid_r1 <= vtc_de;//vtc_valid�ӳ�1��
    vtc_valid_r2 <= vtc_valid_r1;//vtc_valid�������ź��ӳ�2�ģ��Ժ�vtc_last_r2�ź�����ͬ��
    vtc_user_r2  <= vtc_user_r1; //vtc_user �������ź��ӳ�2�ģ��Ժ�vtc_last_r2�ź�����ͬ��    
end    

assign O_vtc_vs       =  vtc_vs_r1;
assign O_vtc_hs       =  vtc_hs_r1;
assign O_vtc_de_valid =  vtc_valid_r2;
assign O_vtc_user     =  vtc_user_r2;
assign O_vtc_last     =  vtc_last_r2;


endmodule


