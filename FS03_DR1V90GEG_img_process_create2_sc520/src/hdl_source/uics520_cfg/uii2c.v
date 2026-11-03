
/*******************************MILIANKE*******************************
*Company : MiLianKe Electronic Technology Co., Ltd.
*WebSite:https://www.milianke.com
*TechWeb:https://www.uisrc.com
*tmall-shop:https://milianke.tmall.com
*jd-shop:https://milianke.jd.com
*taobao-shop1: https://milianke.taobao.com
*Create Date: 2022/12/23
*Module Name:uii2c
*File Name:uii2c.v
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

/*******************************I2C������ MASTER*********************
--��������������Ƶ�I2C����MASTER������
--1.�����࣬ռ�ü����߼���Դ������ṹ�������߼�����Ͻ�
--2.ʹ�÷��㣬ֻ��Ҫ�ѷ��͵����ݷ���Ĵ������Ϳ����Զ�֧�����ⳤ�ȵ�д����
--3.ʹ�÷��㣬ֻ��Ҫ�Ѷ������ݸ�֪���������Ϳ����Զ�֧�����ⳤ�ȵĶ�����
--4.1.1�汾��������O_iic_bus_error��־�źţ�������д����û��ACK���أ�����O_iic_bus_error������ͨ���۲�O_iic_bus_error�鿴�����Ƿ��д���
*********************************************************************/

`timescale 1ns / 1ns //����̶�/����

module uii2c#
(
parameter integer WMEN_LEN = 8'd0,//д���ȣ����ֽ�Ϊ��λ������������ַ
parameter integer RMEN_LEN = 8'd0,//�����ȣ����ֽ�Ϊ��λ��������������ַ
parameter integer CLK_DIV  = 16'd499// I2Cʱ�ӷ�Ƶϵ��
)
(
input  wire I_clk,//ϵͳʱ������
input  wire I_rstn,//ϵͳ��λ���͵�ƽ��Ч
output reg  O_iic_scl = 1'b0,//I2Cʱ��SCL
inout  wire IO_iic_sda,//I2C ��������
input  wire [WMEN_LEN*8-1'b1:0]I_wr_data,//д���ݼĴ���������WMEN_LEN���������֧�ֵ������ֽ�����Խ��ռ�õ�FPGA��ԴԽ��
input  wire [7:0]I_wr_cnt,//д���ݼ�����������д�˶��ٸ��ֽ�
output reg  [RMEN_LEN*8-1'b1:0]O_rd_data = 0,//�����ݼĴ���������RMEN_LEN���������֧�ֵ������ֽ�����Խ��ռ�õ�FPGA��ԴԽ��
input  wire [7:0]I_rd_cnt,//�����ݼ�����
input  wire I_iic_req,//I_iic_req == 1 ʹ��I2C����
input  wire I_iic_mode,//I_iic_mode = 1 �����   I_iic_mode = 0 ����ǰ�Ĵ�������ҳ��
output reg  O_iic_busy = 1'b0,//I2C������æ
output reg  O_iic_bus_error, //I2C���ߣ��޷�������ȷACK����
output reg  IO_iic_sda_dg
);

localparam IDLE    = 4'd0;//I2C ���߿���״̬
localparam START   = 4'd1;//I2C �������
localparam W_WAIT  = 4'd2;//I2C ���ߵȴ�д���
localparam W_ACK   = 4'd3;//I2C ���ߵȴ�дWACK
localparam R_WAIT  = 4'd4;//I2C ���ߵȴ������ 
localparam R_ACK   = 4'd5;//I2C ���ߵȴ���RACK 
localparam STOP1   = 4'd6;//I2C ���߲���ֹͣλ
localparam STOP2   = 4'd7;//I2C ���߲���ֹͣλ   

localparam SCL_DIV = CLK_DIV/2;

localparam OFFSET = SCL_DIV - SCL_DIV/4;//����I2C���ߵ�SCLʱ�ӵ�ƫ�ƣ�������SCL��SDA��ʱ��Ҫ���ⲿ��SCL�ӳ��ڲ��İ����ڵ��ķ�֮��

reg [2:0] IIC_S = 4'd0; //I2C ״̬��
//generate  scl
reg [15:0] clkdiv = 16'd0;   //I2C ʱ�ӷ�Ƶ�Ĵ���
reg scl_r 	   = 1'b1;       //I2C��������SCL�ڲ�ʱ��
reg sda_o 	   = 1'b0;       //I2C��������SDA
reg scl_clk    = 1'b0;       //I2C�������ڲ�SCLʱ�ӣ����ⲿʱ�Ӵ���OFFSET�������õ���λƫ��
reg scl_r_toggle = 1'b0;      //data-net mirror of scl_clk (no clock fanout)
reg [7:0] sda_r = 8'd0;      //���ͼĴ���
reg [7:0] sda_i_r = 8'd0;    //���ռĴ���
reg [7:0] wcnt = 8'd0;       //�������ݼ���������byteΪ��λ
reg [7:0] rcnt = 8'd0;       //�������ݼ���������byteΪ��λ
reg [2:0] bcnt = 3'd0;       //bit������
reg  rd_req = 1'b0;          //�����󣬵��жϵ���Ҫ�����ݣ��ڲ�״̬��������1
wire sda_i;                  //sda ����
wire scl_offset;             //scl ʱ��ƫ�ƿ���

assign  sda_i   = IO_iic_sda ;  //������ 
assign  IO_iic_sda = (sda_o == 1'b0) ?  1'b0 : 1'bz;    //д���ߣ�1'bz������裬I2C�ⲿͨ���������裬ʵ�����ߵĸߵ�ƽ

//PULLUP PULLUP_inst (.O(iic_sda));

//XILINX I2C ��IOBUFʵ�ֿ���
//IOBUF #(
//.DRIVE(12), // Specify the output drive strength
//.IBUF_LOW_PWR("TRUE"),  // Low Power - "TRUE", High Performance = "FALSE" 
//.IOSTANDARD("DEFAULT"), // Specify the I/O standard
//.SLEW("SLOW") // Specify the output slew rate
//) IOBUF_inst (
//.O(sda_i),    // Buffer output
//.IO(IO_iic_sda), // Buffer inout port (connect directly to top-level port)
//.I(sda_o),    // Buffer input
//.T(sda_o)     // 3-state enable input, high=input, low=output
//);

//scl ʱ�ӷ�Ƶ��
always@(posedge I_clk)
    if(clkdiv < SCL_DIV)    
        clkdiv <= clkdiv + 1'b1;
    else begin
        clkdiv <= 16'd0; 
        scl_clk <= !scl_clk;
    end


assign  scl_offset  = (clkdiv == OFFSET);//����scl_offset��ʱ�����
always @(posedge I_clk) O_iic_scl <=  scl_offset ?  scl_r : O_iic_scl; //O_iic_scl�ӳ�scl_offsetʱ���scl_r

//�ɼ�I2C ��������sda
always @(posedge I_clk) IO_iic_sda_dg <= sda_i;  

// scl_r mirrors scl_clk as a DATA signal (I_clk domain, same toggle edge as
// scl_clk) so the clock net scl_clk only feeds clock pins (fixes PHY-5060 mixed
// fanout).  scl_r_toggle init 0 == scl_clk init 0, both toggle at clkdiv==SCL_DIV,
// so the SCL waveform is bit-identical to the original tap.
always @(posedge I_clk) begin
    if(clkdiv == SCL_DIV)
        scl_r_toggle <= !scl_r_toggle;
end
always @(*) 
    if(IIC_S == IDLE || IIC_S == STOP1 || IIC_S == STOP2)
        scl_r <= 1'b1;
    else 
        scl_r <= scl_r_toggle;
  

//������IIC_S״̬Ϊ�����ֹͣ����sda=0�����scl������ʼλ������(IIC_S == R_ACK && (rcnt != I_rd_cnt) sda=0�����ڲ�����������ACK
always @(*) 
    if(IIC_S == START || IIC_S == STOP1 || (IIC_S == R_ACK && (rcnt != I_rd_cnt)))
        sda_o <= 1'b0;
    else if(IIC_S == W_WAIT)
        sda_o <= sda_r[7]; 
    else  sda_o <= 1'b1; //��������״̬��Ϊ1����(IIC_S == R_ACK && (rcnt == I_rd_cnt) ����һ��NACK 

//I2C���ݷ���ģ�飬���е�д���ݶ�ͨ����ģ�鷢��
always @(posedge scl_clk) 
    if(IIC_S == W_ACK || IIC_S == START)begin//IIC_S=START��W_ACK������Ҫ���͵����ݣ��Ĵ浽sda_r
        sda_r <= I_wr_data[(wcnt*8) +: 8];//�Ĵ���Ҫ�����͵����ݵ�sda_r
        if( rd_req ) sda_r <= {I_wr_data[7:1],1'b1};//���ڶ�������rd_req���ڲ������������д���һ������(������ַ)����ͨ���ж�I_rd_cnt��ȷ���Ƿ�������Ҫ��
    end
    else if(IIC_S == W_WAIT)//��W_WAT״̬��ͨ����λ�����������ݷ��͵���������
        sda_r <= {sda_r[6:0],1'b1};//��λ����
    else 
        sda_r <= sda_r;

//sda data bus read and hold data to O_rd_data register when IIC_S=R_ACK
//I2C���ݽ���ģ�飬I2C���ڼ䣬������ͨ����λ����������O_rd_data
always @(negedge scl_clk)begin
    if(IIC_S == R_WAIT ) //��IIC_S == R_WAIT ||IIC_S == W_ACK(�������������1��BIT��W_ACK���״̬��)�����λ����
        sda_i_r <= {sda_i_r[6:0],sda_i};
    else if(IIC_S == R_ACK)//��IIC_S == R_ACK,���һ��BYTE���������ݱ��浽O_rd_data
        O_rd_data[((rcnt-1'b1)*8) +: 8] <= sda_i_r[7:0];
    else if(IIC_S == IDLE)//����״̬������sda_i_r
        sda_i_r <= 8'd0;
end

//����æ״̬
always @(posedge scl_clk or negedge I_rstn )begin
	if(I_rstn == 1'b0)
    	O_iic_busy <= 1'b0; 
	else begin
    	if((I_iic_req == 1'b1 || rd_req == 1'b1 || O_iic_bus_error))//I_iic_req == 1'b1 || rd_req == 1'b1���߽���æ״̬
    	    O_iic_busy <= 1'b1; 
        else if(IIC_S == IDLE)
        	O_iic_busy <= 1'b0;
    end 		
end

//����æ״̬
always @(negedge scl_clk or negedge I_rstn )begin
	if(I_rstn == 1'b0)
    	O_iic_bus_error <= 1'b0;  	
	else begin
    	if(IIC_S  == W_ACK && sda_i == 1'b1)//I_iic_req == 1'b1 || rd_req == 1'b1���߽���æ״̬
    	    O_iic_bus_error <= 1'b1; 
        else if(I_iic_req == 0)
        	O_iic_bus_error <= 1'b0;
    end 		
end

//I2C Master������״̬��
always @(posedge scl_clk or negedge I_rstn )begin
		if(I_rstn == 1'b0)begin //�첽��λ����λ��ؼĴ���
           wcnt 	<= 8'd0;
           rcnt 	<= 8'd0;
           rd_req 	<= 1'b0;   
           IIC_S    <= IDLE;
        end
        else begin
        case(IIC_S) //sda = 1 scl =1
        IDLE:begin//�ڿ���״̬��sda=1 scl=1 
           if(I_iic_req == 1'b1 || rd_req == 1'b1) //��I_iic_req == 1'b1����������� �� rd_req == 1'b1 �����������Ҫ����repeated start �ظ����  
              IIC_S  <= START; //����START״̬
           else begin
              wcnt <= 8'd0; //��λ������
              rcnt <= 8'd0; //��λ������
           end
        end
        START:begin //���״̬��ǰ��Ĵ��룬������sda = 0��scl_offset����������scl_clkʱ�ӵ�ƫ�ƣ�֮�� scl_clk =0 ��scl =0 ������ʼλ�����ظ���ʼλ
           bcnt <= 3'd7; //����bcnt�ĳ�ֵ         
           IIC_S  <= W_WAIT;//���뷢�͵ȴ�
        end           
        W_WAIT://�ȴ�������ɣ����﷢��8bits ���ݣ�д������ַ��д�Ĵ�����ַ��д���ݣ��������״̬���
        begin
           if(bcnt > 3'd0)//���8bitsû�����ֱ꣬��������
               bcnt  <= bcnt - 1'b1; //bcnt��������ÿ����1bit��1
           else begin //8bits�������
               wcnt <= wcnt + 1'b1; //wcnt�����������ڼ�¼�Ѿ�д�˶����ֽ�
               IIC_S  <= W_ACK;//����W_ACK״̬
           end
        end 
        W_ACK://�ȴ�WACK���˽׶Σ�Ҳ�ж��Ƿ��ж�����
        begin 
           if(wcnt < I_wr_cnt)begin //�ж��Ƿ��������ݷ���(д)���
              bcnt <= 3'd7; //���û��д�꣬����bcnt
              IIC_S <= W_WAIT;//�����ص�W_WAIT�ȴ����ݷ���(д)���
           end
           else if(I_rd_cnt > 3'd0)begin//I_rd_cnt > 0������������Ҫ����I_rd_cnt�������ж���������Ҫ��
              if(rd_req == 1'b0 && I_iic_mode == 1'b1)begin //���ڵ�һ��д��������ַ�����I_iic_mode==1����֧�������
                  rd_req <= 1'b1;//����rd_req=1�����������
                  IIC_S <= IDLE; //����״̬����IDLE������rd_req��ֵ�����²���һ��Ϊ���������е�repeated�ظ�start
              end
              else //���֮ǰ�Ѿ������repeated�ظ�start����ô��������������ݽ׶�
                  IIC_S <= R_WAIT;//������ȴ�
                  bcnt <= 3'd7;//����bcnt�ĳ�ֵ  
           end
           else //������еķ�����ɣ�Ҳû������Ҫ��������ֹͣ״̬
              IIC_S <= STOP1; 
        end  
        R_WAIT://�ȴ����������
        begin
           rd_req <= 1'b0;//���ö�����rd_req=0
           bcnt  <= bcnt - 1'b1; //bit ������
           if(bcnt == 3'd0)begin //��8bits���ݶ���
              rcnt <= (rcnt < I_rd_cnt) ? (rcnt + 1'b1) : rcnt;//�ж��Ƿ���������Ҫ��
              IIC_S  <= R_ACK;//����R_ACK
           end
        end
        R_ACK://R_ACK״̬����NACK
        begin
           bcnt <= 3'd7;//���ö�����bcnt������
           IIC_S <= (rcnt < I_rd_cnt) ? R_WAIT : STOP1; //����������ݶ��꣬����ֹͣ״̬
        end  
        STOP1:begin//����ֹͣλ sda = 0 scl = 1
            rd_req 	<= 1'b0;              
            IIC_S <= STOP2;
        end
        STOP2://����ֹͣλ  sda = 1 scl = 1
            IIC_S <= IDLE;          
        default:
            IIC_S <= IDLE;
    	endcase
    end
end

endmodule
