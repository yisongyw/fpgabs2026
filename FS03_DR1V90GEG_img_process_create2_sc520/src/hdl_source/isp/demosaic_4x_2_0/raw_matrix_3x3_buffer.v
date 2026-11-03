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
@Time        :   2025/01 
version:     :   1.1
@Description :   ��ģ����Ҫ������3x3������γɣ��Լ���ͼ��߽����صĴ���
                 ��ԭ�������Ż���ʱ�򣬰�ԭ����һЩδ��ȫ������źŽ��ж��롣
*****************************************************************/


module raw_matrix_3x3_buffer #(
    parameter IMG_HEIGHT = 1080,  // ͼ��߶�
    parameter IMG_WIDTH  = 1920,   // ͼ����
    parameter data_complete_delay = 50
)
(
    input           I_clk   ,   // ʱ���ź�
    input           I_rst_n ,   // ��λ�źţ�����Ч

    input [39:0]                         axi4s_video_tdata ,  // AXI4-Stream��Ƶ����
    input [9:0]                          axi4s_video_tdest ,
    input                                axi4s_video_tlast ,  // �н����ź�
    input                                axi4s_video_tvalid,  // ������Ч�ź�
    input                                axi4s_video_tuser ,  // ֡��ʼ�ź�
    output                               axi4s_video_tready,  // ��ģ��׼���ý�������

    output                               O_tlast         ,   // �н����ź����
    output                               O_tuser         ,   // ֡��ʼ�ź����
    output                               O_tvalid        ,   // ������Ч�ź����
    input                                O_tready        ,   // ��һ��ģ��׼���ý�������

    output                               bayer_ypos      ,   // Bayer�˲�����λ��
    output  [95:0]                       matrix_last_line,   // ��һ��3x3��������
    output  [95:0]                       matrix_cur_line ,   // ��ǰ��3x3��������
    output  [95:0]                       matrix_next_line    // ��һ��3x3��������
);


/*********************************************************
******************1.��������*******************************
*********************************************************/
// �������ݵ�λ��
function integer clog2b(
    input integer depth
);
    begin
        for(clog2b=0; depth>0; clog2b=clog2b+1)begin
            depth = depth >> 1;
        end
    end    
endfunction

/*********************************************************
******************2.��������*******************************
*********************************************************/
localparam N = 3;  // �����СΪ3x3

localparam DATA_WIDTH   = 32;  // ����λ��

localparam IMG_WIDTH_4x = (IMG_WIDTH >> 2);  // ͼ���ȳ���4���õ�ÿ�����ݿ������

/*********************************************************
******************3.�ź�����*******************************
*********************************************************/
// ���������ź�
wire                        zhenghe_tlast ;    // �ϲ�����н����ź�
wire                        zhenghe_tuser ;    // �ϲ����֡��ʼ�ź�
wire  [31:0]                zhenghe_tdata ;    // �ϲ��������
wire                        zhenghe_tvalid;    // �ϲ����������Ч�ź�

wire  [DATA_WIDTH-1:0]      raw_data        ;  // ԭʼ����
wire                        raw_data_valid  ;  // ԭʼ������Ч�ź�

wire                valid   [0:N-1] ;          // ��Ч�ź�����
wire [DATA_WIDTH:0] data    [0:N-1] ;          // �������飨����tuserλ��

wire                O_raw_tlast_d0;            // �н����źţ��ӳ�0����
wire                O_raw_tuser_d0;            // ֡��ʼ�źţ��ӳ�0����
wire                I_raw_tready;              // ģ��׼���ý���ԭʼ����
wire [32:0]         next_line_data;            // ��һ������
wire [32:0]         data_d2;                   // �����ӳ�2������
wire                next_line_start;           // ��һ�еĿ�ʼ�ź�
wire                bayer_ypos_d0;             // Bayerλ�ã��ӳ�0����

/*********************************************************
******************4.�Ĵ����ź�����*************************
*********************************************************/
// ����Ĵ����ź�
reg [95:0]      matrix_cur_line_buffer       ; // ��ǰ�е�3x3���󻺳�
reg [95:0]      matrix_last_line_buffer      ; // ��һ�е�3x3���󻺳�
reg [95:0]      matrix_next_line_buffer      ; // ��һ�е�3x3���󻺳�
reg             tlast_d0,tlast_d1,tlast_d2;     // �н����ź��ӳټĴ���
reg             tuser_d0,tuser_d1,tuser_d2;     // ֡��ʼ�ź��ӳټĴ���
reg             ch_valid;                       // ����ͨ����Ч�ź�
reg [13:0]      h_cnt;                          // ˮƽ���ؼ�����
reg [13:0]      v_cnt,v_cnt_d;                  // ��ֱ���ؼ����������ӳ�
reg             next_line_data_valid;           // ��һ��������Ч�ź�
reg             valid_d0,valid_d1;              // ��Ч�ź��ӳټĴ���
reg             bayer_ypos_d;                   // Bayerλ���ӳټĴ���
reg  [32:0]     data_d0_d,data_d0;              // �����ӳټĴ���
reg  [32:0]     data_d1;

/*********************************************************
******************5.����߼�******************************
*********************************************************/
// �����ݺ���Ч�źŽ�������
assign data[0]  = {zhenghe_tuser, zhenghe_tdata};  // ��tuser������ƴ��
assign valid[0] = zhenghe_tvalid;                  // ��һ����Ч�ź�

// ��׼���ź����ӵ��ϼ�ģ��
assign axi4s_video_tready = O_tready;

/*********************************************************/
// �����󻺳����
assign matrix_last_line = matrix_last_line_buffer;
assign matrix_cur_line  = matrix_cur_line_buffer ;
assign matrix_next_line = matrix_next_line_buffer;

// ����Bayer�˲�������λ��
assign bayer_ypos_d0 = !v_cnt_d[0] & ch_valid;

/*********************************************************
******************6.ģ��ʵ����*****************************
*********************************************************/
// ʵ����mipi_to_raw_converterģ�飬��MIPIЭ�������ת��ΪRAW����
mipi_to_raw_converter mipi_to_raw_converter_r0(
    .I_clk       (I_clk  ),              // ʱ���ź�
    .I_rst_n     (I_rst_n),              // ��λ�źţ�����Ч
    .I_raw_tdata (axi4s_video_tdata ),   // ����ԭʼ����
    .I_raw_tdest (axi4s_video_tdest ),   // ����Ŀ�ĵص�ַ
    .I_raw_tlast (axi4s_video_tlast ),   // �����н����ź�
    .I_raw_tvalid(axi4s_video_tvalid),   // ����������Ч�ź�
    .I_raw_tuser (axi4s_video_tuser),    // ����֡��ʼ�ź�
    .I_raw_tready(I_raw_tready  ),       // ���׼�����ź�
    .O_raw_tdata (raw_data      ),       // ���ԭʼ����
    .O_raw_tlast (O_raw_tlast_d0),       // ����н����ź�
    .O_raw_tvalid(raw_data_valid),       // ���������Ч�ź�
    .O_raw_tuser (O_raw_tuser_d0),       // ���֡��ʼ�ź�
    .O_raw_tready(1)                     // ģ��ʼ��׼���ý�������
);

// ʵ������zhenghe��ģ�飬�����ݽ��кϲ�����
zhenghe#(
    .IMG_WIDTH ( IMG_WIDTH ),
    .IMG_HEIGHT( IMG_HEIGHT),
    .DATA_WIDTH( 32),
    .data_complete_delay (data_complete_delay)
) 
zhenghe_d(
    .I_clk    (I_clk   ),               // ʱ���ź�
    .I_rst_n  (I_rst_n ),               // ��λ�źţ�����Ч

    .I_tlast  (O_raw_tlast_d0 ),        // �����н����ź�
    .I_tuser  (O_raw_tuser_d0 ),        // ����֡��ʼ�ź�
    .I_tdata  (raw_data ),              // ����ԭʼ����
    .I_tvalid (raw_data_valid),         // ����������Ч�ź�

    .O_tlast  (zhenghe_tlast ),         // ����ϲ�����н����ź�
    .O_tuser  (zhenghe_tuser ),         // ����ϲ����֡��ʼ�ź�
    .O_tdata  (zhenghe_tdata ),         // ����ϲ��������
    .O_tvalid (zhenghe_tvalid)          // ����ϲ����������Ч�ź�
);

// �������л�������ʹ�����Ի�����ʵ��3x3����������ݻ���
genvar i;
generate
    for(i=1; i<3; i=i+1)
    begin: two_line_buffer  // ģ������Ϊtwo_line_buffer
        line_buffer_demosaic #(
            .IMG_HEIGHT (IMG_HEIGHT),
            .IMG_WIDTH  (IMG_WIDTH )
        ) line_buffer_demosaic_r1 (
            .I_clk     ( I_clk      ),    // ʱ���ź�
            .I_rst_n   ( I_rst_n    ),    // ��λ�źţ�����Ч
            .I_tuser   ( data[i-1][32] & valid[i-1]), // ����֡��ʼ�ź�
            .I_valid   ( valid[i-1] ),    // ������Ч�ź�
            .I_data    ( data[i-1]  ),    // ��������
            .O_valid   ( valid[i]   ),    // �����Ч�ź�
            .O_data    ( data[i]    )     // �������
        );           
    end        
endgenerate

// ������BRAM��ÿ��������ݶ����ӳ�һ�����ڣ�����Ҫʹ��Щ���ݶ��룬��Ҫ����Щ���ݽ����ӳٴ��������data[0]�ӳ�2�����ڣ�data[1]�ӳ�һ������
always @(posedge I_clk or negedge I_rst_n) begin
    if (!I_rst_n) begin
        data_d0_d <= 0;
        data_d0   <= 0;
        data_d1   <= 0;
        valid_d0  <= 0;
        valid_d1  <= 0;
    end
    else begin
        valid_d0  <= valid[0];
        valid_d1  <= valid_d0;
        data_d0_d <= data[0];
        data_d0   <= data_d0_d;
        data_d1   <= data[1];
    end
end

assign data_d2 = data[2];
/*********************************************************
***********3x3������γɣ��Լ��Ա߽�Ԫ�صĲ��䴦��***********
*********************************************************/

always @(posedge I_clk or negedge I_rst_n) begin 
    if(!I_rst_n) begin
        matrix_last_line_buffer <= 'b0 ;
        matrix_cur_line_buffer  <= 'b0 ;
        matrix_next_line_buffer <= 'b0 ;
    end else if(valid_d1) begin
        if(v_cnt == 0) begin  // ��һ��
            if(h_cnt == 0) begin  // ��һ�еĵ�һ������
                matrix_last_line_buffer <= {{24{1'b0}}, data_d0[31:24], data_d0[31:0], data_d0[31:0]} ;
                matrix_cur_line_buffer  <= {{24{1'b0}}, data_d0[31:24], data_d0[31:0], data_d0[31:0]} ;
                matrix_next_line_buffer <= {{24{1'b0}}, data_d0[31:24], data_d0[31:0], data_d0[31:0]} ;
            end else if(h_cnt == (IMG_WIDTH_4x - 1)) begin  // ��һ�е����һ������
                matrix_last_line_buffer <= {matrix_next_line_buffer [63:0], matrix_next_line_buffer[7:0], {24{1'b0}}} ;
                matrix_cur_line_buffer  <= {matrix_next_line_buffer [63:0], matrix_next_line_buffer[7:0], {24{1'b0}}} ;
                matrix_next_line_buffer <= {matrix_next_line_buffer [63:0], matrix_next_line_buffer[7:0], {24{1'b0}}} ;
            end else begin  // ��һ�е��м�����
                matrix_last_line_buffer <= {matrix_next_line_buffer[63:0], data_d0[31:0]} ;
                matrix_cur_line_buffer  <= {matrix_next_line_buffer[63:0], data_d0[31:0]} ;
                matrix_next_line_buffer <= {matrix_next_line_buffer[63:0], data_d0[31:0]} ;
            end
        end
        else if(v_cnt == 1) begin  // �ڶ���
            if(h_cnt == 0) begin  // �ڶ��еĵ�һ������
                matrix_last_line_buffer <= {{24{1'b0}}, data_d1[31:24], data_d1[31:0], data_d1[31:0]} ;
                matrix_cur_line_buffer  <= {{24{1'b0}}, data_d1[31:24], data_d1[31:0], data_d1[31:0]} ;
                matrix_next_line_buffer <= {{24{1'b0}}, data_d0[31:24], data_d0[31:0], data_d0[31:0]} ;
            end else if(h_cnt == (IMG_WIDTH_4x - 1)) begin  // �ڶ��е����һ������
                matrix_last_line_buffer <= {matrix_cur_line_buffer [63:0], matrix_cur_line_buffer[7:0], {24{1'b0}}} ;
                matrix_cur_line_buffer  <= {matrix_cur_line_buffer [63:0], matrix_cur_line_buffer[7:0], {24{1'b0}}} ;
                matrix_next_line_buffer <= {matrix_next_line_buffer [63:0], matrix_next_line_buffer[7:0], {24{1'b0}}} ;
            end else begin  // �ڶ��е��м�����
                matrix_last_line_buffer <= {matrix_cur_line_buffer[63:0],  data_d1[31:0]} ;
                matrix_cur_line_buffer  <= {matrix_cur_line_buffer[63:0],  data_d1[31:0]} ;
                matrix_next_line_buffer <= {matrix_next_line_buffer[63:0], data_d0[31:0]} ;
            end
        end
        else begin  // �����м��Ժ�
            if(h_cnt == 0) begin  // ��һ������
                matrix_last_line_buffer <= {{24{1'b0}}, data_d2[31:24], data_d2[31:0], data_d2[31:0]} ;
                matrix_cur_line_buffer  <= {{24{1'b0}}, data_d1[31:24], data_d1[31:0], data_d1[31:0]} ;
                matrix_next_line_buffer <= {{24{1'b0}}, data_d0[31:24], data_d0[31:0], data_d0[31:0]} ;
            end else if(h_cnt == (IMG_WIDTH_4x - 1)) begin  // ���һ������
                matrix_last_line_buffer <= {matrix_last_line_buffer[63:0], matrix_last_line_buffer[7:0], {24{1'b0}}} ;
                matrix_cur_line_buffer  <= {matrix_cur_line_buffer [63:0], matrix_cur_line_buffer [7:0], {24{1'b0}}} ;
                matrix_next_line_buffer <= {matrix_next_line_buffer[63:0], matrix_next_line_buffer[7:0], {24{1'b0}}} ;
            end else begin  // �м�����
                matrix_last_line_buffer <= {matrix_last_line_buffer[63:0], data_d2[31:0]} ;
                matrix_cur_line_buffer  <= {matrix_cur_line_buffer [63:0], data_d1[31:0]} ;
                matrix_next_line_buffer <= {matrix_next_line_buffer[63:0], data_d0[31:0]} ;
            end  
        end
    end
    else begin
        matrix_last_line_buffer <= 'b0 ;
        matrix_cur_line_buffer  <= 'b0 ;
        matrix_next_line_buffer <= 'b0 ;
    end 
end

// ����ˮƽ�ʹ�ֱ������
always @(posedge I_clk or negedge I_rst_n) begin
    if(!I_rst_n || zhenghe_tuser) begin
        h_cnt <= 0;
        v_cnt <= 0;
    end else begin
        h_cnt    <= valid_d1 ? ((h_cnt == IMG_WIDTH_4x - 1) ? 0 : h_cnt + 1) : h_cnt;
        v_cnt    <= valid_d1 && (h_cnt == IMG_WIDTH_4x - 1) ? ((v_cnt == IMG_HEIGHT - 1) ? 0 : v_cnt + 1) : v_cnt;
    end
end

// ������Ч�źŵ��ӳٴ���
always @(posedge I_clk or negedge I_rst_n) begin
    if(!I_rst_n) begin
        ch_valid <= 0;
        v_cnt_d <= 0;
    end else begin
        ch_valid <= valid_d1 ;
        v_cnt_d  <= v_cnt;
    end
end

assign bayer_ypos = bayer_ypos_d0 ;

// ��tuser��tlast�źŽ���ͬ���������ʱ���ݺ��������ź��Ѿ�ͬ��
always @(posedge I_clk or negedge I_rst_n) begin
    if (!I_rst_n) begin
        tlast_d0 <= 0;
        tlast_d1 <= 0;
        tlast_d2 <= 0;
        tuser_d0 <= 0;
        tuser_d1 <= 0;
        tuser_d2 <= 0;
    end
    else begin
        tlast_d0 <= zhenghe_tlast;
        tlast_d1 <= tlast_d0;
        tlast_d2 <= tlast_d1;
        tuser_d0 <= zhenghe_tuser;
        tuser_d1 <= tuser_d0;
        tuser_d2 <= tuser_d1;
    end
end

// ��ͬ������ź����
assign O_tuser  = tuser_d2;
assign O_tlast  = tlast_d2;
assign O_tvalid = ch_valid;


endmodule


