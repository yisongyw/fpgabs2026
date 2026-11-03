//-----------------------------------------------------------------------------
// Module     : eth_app
// Project    : HX1P35Aimg_process_create1
// Description: Ethernet application layer.
//
//   Wraps the board proven UDP/IP/ARP/MAC stack (uiudp_stack + rgmii_interface
//   from 33_ethernet_udp_led, HX1P35A family reference design) and adds :
//
//     * result streaming : eth_result_tx builds one UDP payload per valid
//       moment (tracker update / report request / SD image done) and a 125 MHz
//       streamer pushes it into the UDP transmit interface ;
//     * command reception : the UDP payload of every incoming packet is fed
//       into the SAME cmd_parser used by the UART, so both PC channels
//       implement identical KEY1..KEY4 / SD / CAM / LD / R? commands ;
//     * status strobes  : link / TX / RX / error activity for the LED view.
//
//   Network configuration (identical to the reference design) :
//       MAC  local  = 00:0A:35:01:FE:01
//       IP   local  = 192.168.1.10
//       UDP  local  = 8080
//       IP   dest   = 192.168.1.112
//       UDP  dest   = 8080
//-----------------------------------------------------------------------------
module eth_app #(
    parameter [47:0] MAC_LOCAL = 48'h000A_3501_FE01,
    parameter [31:0] IP_LOCAL  = {8'd192, 8'd168, 8'd1, 8'd10},
    parameter [31:0] IP_DEST   = {8'd192, 8'd168, 8'd1, 8'd112},
    parameter [15:0] UDP_PORT  = 16'd8080,
    parameter        TRK_W     = 70
)(
    input  wire              I_refclk,      // 50 MHz board oscillator
    input  wire              I_rst_n,       // board reset (S_100m_clk domain)
    input  wire              I_clk100,      // S_100m_clk

    // result source (S_100m_clk, values stable during one frame)
    input  wire              I_trig,
    input  wire [TRK_W-1:0]  I_track0,
    input  wire [TRK_W-1:0]  I_track1,
    input  wire [3:0]        I_trk_num,
    input  wire [3:0]        I_obj_num,
    input  wire [8:0]        I_th,
    input  wire [3:0]        I_style,
    input  wire [2:0]        I_alg,
    input  wire              I_cls_mode,
    input  wire              I_alarm,
    input  wire              I_sd_mode,

    // command events (S_100m_clk one shot pulses)
    output wire [3:0]        O_key_evt,
    output wire              O_sd_evt,
    output wire              O_cam_evt,
    output wire              O_ld_evt,
    output wire              O_rep_evt,
    output wire              O_cmd_valid,

    // status (S_100m_clk)
    output wire              O_link,
    output wire              O_tx_act,
    output wire              O_rx_act,
    output wire              O_err_act,

    // RGMII pins
    input  wire              I_rgmii_rxc,
    input  wire              I_rgmii_rx_ctl,
    input  wire [3:0]        I_rgmii_rxd,
    output wire              O_rgmii_txc,
    output wire              O_rgmii_tx_ctl,
    output wire [3:0]        O_rgmii_txd,
    output wire              O_phy_rst
);

    //-------------------------------------------------------------------------
    // clocks and resets
    //-------------------------------------------------------------------------
    wire S_clk125;
    wire S_lock125;

    PLL_eth u_pll_eth (
        .refclk    ( I_refclk ),
        .reset     ( ~I_rst_n ),
        .clk125_out( S_clk125 ),
        .lock      ( S_lock125 )
    );

    // 125 MHz domain reset (asynchronous assert, synchronous release)
    reg [1:0] r_rst125_sync = 2'b00;
    always @(posedge S_clk125 or negedge I_rst_n) begin
        if(!I_rst_n)
            r_rst125_sync <= 2'b00;
        else
            r_rst125_sync <= {r_rst125_sync[0], S_lock125};
    end
    wire S_eth_rst_n = r_rst125_sync[1];
    wire S_eth_rst   = ~S_eth_rst_n;      // active high, stack convention

    // PHY reset : O_phyrst = 1 -> PHY released (board pin V8, RESET_N)
    uiphyrst #(
        .CLK_FREQ ( 32'd125_000_000 )
    ) u_phyrst (
        .I_CLK   ( S_clk125   ),
        .I_rstn  ( I_rst_n & S_lock125 ),
        .I_phyrst( S_lock125  ),
        .O_phyrst( O_phy_rst  )
    );

    //-------------------------------------------------------------------------
    // RGMII <-> GMII
    //-------------------------------------------------------------------------
    wire       gmii_rvalid;
    wire [7:0] gmii_rdata;
    wire       gmii_rclk;
    wire       gmii_tvalid;
    wire [7:0] gmii_tdata;
    wire       S_link;

    rgmii_interface u_rgmii_interface (
        .speed_10_100    ( 1'b0         ),
        .gmii_tx_clk_d   ( S_clk125     ),
        .gmii_tx_reset_d ( S_eth_rst    ),
        .gmii_tx_reset   ( S_eth_rst    ),
        .gmii_tx_clk     ( S_clk125     ),
        .gmii_txd        ( gmii_tdata   ),
        .gmii_tx_en      ( gmii_tvalid  ),
        .gmii_tx_er      ( 1'b0         ),
        .gmii_rx_reset   ( S_eth_rst    ),
        .gmii_rxd        ( gmii_rdata   ),
        .gmii_rx_dv      ( gmii_rvalid  ),
        .gmii_rx_er      (              ),
        .gmii_rx_clk     ( gmii_rclk    ),
        .rgmii_rxc       ( I_rgmii_rxc      ),
        .rgmii_rx_ctl    ( I_rgmii_rx_ctl   ),
        .rgmii_rxd       ( I_rgmii_rxd      ),
        .rgmii_txc       ( O_rgmii_txc      ),
        .rgmii_tx_ctl    ( O_rgmii_tx_ctl   ),
        .rgmii_txd       ( O_rgmii_txd      ),
        .gmii_crs        (              ),
        .gmii_col        (              ),
        .link_status     ( S_link       ),
        .clock_speed     (              ),
        .duplex_status   (              )
    );

    //-------------------------------------------------------------------------
    // UDP / IP / ARP / MAC stack
    //-------------------------------------------------------------------------
    wire        W_udp_busy;
    wire        W_udp_req;
    wire        W_udp_valid;
    wire [7:0]  W_udp_data;
    wire [15:0] W_udp_len;

    wire        R_udp_valid;
    wire [7:0]  R_udp_data;
    wire        S_ip_rerror;
    wire        S_mac_rerror;

    uiudp_stack #(
        .CRC_GEN_EN      ( 1'b1 ),
        .INTER_FRAME_GAP ( 4'd12 )
    ) u_uiudp_stack (
        .I_uclk           ( S_clk125     ),
        .I_reset          ( S_eth_rst    ),
        .I_mac_local_addr ( MAC_LOCAL    ),
        .I_udp_local_port ( UDP_PORT     ),
        .I_ip_local_addr  ( IP_LOCAL     ),
        .I_udp_dest_port  ( UDP_PORT     ),
        .I_ip_dest_addr   ( IP_DEST      ),
        .O_W_udp_busy     ( W_udp_busy   ),
        .I_W_udp_req      ( W_udp_req    ),
        .I_W_udp_valid    ( W_udp_valid  ),
        .I_W_udp_data     ( W_udp_data   ),
        .I_W_udp_len      ( W_udp_len    ),
        .O_R_udp_valid    ( R_udp_valid  ),
        .O_R_udp_data     ( R_udp_data   ),
        .O_R_udp_len      (              ),
        .O_R_udp_src_port (              ),
        .I_gmii_rclk      ( gmii_rclk    ),
        .I_gmii_rvalid    ( gmii_rvalid  ),
        .I_gmii_rdata     ( gmii_rdata   ),
        .I_gmii_tclk      ( S_clk125     ),
        .O_gmii_tvalid    ( gmii_tvalid  ),
        .O_gmii_tdata     ( gmii_tdata   ),
        .O_ip_rerror      ( S_ip_rerror  ),
        .O_mac_rerror     ( S_mac_rerror )
    );

    //-------------------------------------------------------------------------
    // result payload builder (S_100m_clk) + shared RAM
    //-------------------------------------------------------------------------
    wire [5:0] r_rd_addr;
    wire [7:0] r_rd_data;
    wire       S_req_tgl;
    wire       S_ack_tgl;
    wire [5:0] S_pkt_len;

    eth_result_tx #(
        .TRK_W ( TRK_W )
    ) u_eth_result_tx (
        .I_clk      ( I_clk100   ),
        .I_rst_n    ( I_rst_n    ),
        .I_trig     ( I_trig     ),
        .I_track0   ( I_track0   ),
        .I_track1   ( I_track1   ),
        .I_trk_num  ( I_trk_num  ),
        .I_obj_num  ( I_obj_num  ),
        .I_th       ( I_th       ),
        .I_style    ( I_style    ),
        .I_alg      ( I_alg      ),
        .I_cls_mode ( I_cls_mode ),
        .I_alarm    ( I_alarm    ),
        .I_sd_mode  ( I_sd_mode  ),
        .I_rd_addr  ( r_rd_addr  ),
        .O_rd_data  ( r_rd_data  ),
        .O_req_tgl  ( S_req_tgl  ),
        .O_len      ( S_pkt_len  ),
        .I_ack_tgl  ( S_ack_tgl  )
    );

    //-------------------------------------------------------------------------
    // 125 MHz TX streamer : request -> stream payload bytes -> acknowledge
    //-------------------------------------------------------------------------
    localparam ST_IDLE  = 2'd0;
    localparam ST_REQ   = 2'd1;
    localparam ST_SEND  = 2'd2;
    localparam ST_WAIT  = 2'd3;

    reg [1:0]  r_st;
    reg [1:0]  r_req_sync;
    reg        r_req_d;
    reg [5:0]  r_cnt;
    reg        r_ack_tgl;
    reg        r_pre;
    reg [21:0] r_tx_stretch;

    wire S_req_edge = r_req_sync[1] ^ r_req_d;

    assign W_udp_req   = (r_st == ST_REQ);
    assign W_udp_valid = (r_st == ST_SEND);
    assign W_udp_data  = r_rd_data;
    assign W_udp_len   = {10'd0, S_pkt_len} + 16'd1;
    assign r_rd_addr   = r_cnt;

    always @(posedge S_clk125 or negedge S_eth_rst_n) begin
        if(!S_eth_rst_n) begin
            r_st         <= ST_IDLE;
            r_req_sync   <= 2'b00;
            r_req_d      <= 1'b0;
            r_cnt        <= 6'd0;
            r_ack_tgl    <= 1'b0;
            r_pre        <= 1'b0;
            r_tx_stretch <= 22'd0;
        end
        else begin
            r_req_sync <= {r_req_sync[0], S_req_tgl};
            r_req_d    <= r_req_sync[1];

            if(r_tx_stretch != 22'd0)
                r_tx_stretch <= r_tx_stretch - 22'd1;

            case(r_st)
                ST_IDLE: begin
                    if(S_req_edge) begin
                        r_st  <= ST_REQ;
                        r_cnt <= 6'd0;
                        r_pre <= 1'b0;
                    end
                end
                ST_REQ: begin
                    // wait for an idle UDP layer first (busy may come from ARP),
                    // then hold the request until the UDP layer grabs the packet
                    if(!r_pre) begin
                        if(!W_udp_busy)
                            r_pre <= 1'b1;
                    end
                    else if(W_udp_busy) begin
                        r_st         <= ST_SEND;
                        r_cnt        <= 6'd0;
                        r_tx_stretch <= 22'h3FFFFF;
                    end
                end
                ST_SEND: begin
                    if(r_cnt == S_pkt_len) begin
                        r_st <= ST_WAIT;
                    end
                    else begin
                        r_cnt <= r_cnt + 6'd1;
                    end
                end
                ST_WAIT: begin
                    if(!W_udp_busy) begin
                        r_ack_tgl <= S_req_tgl;
                        r_st      <= ST_IDLE;
                    end
                end
            endcase
        end
    end

    //-------------------------------------------------------------------------
    // command reception : UDP payload -> shared cmd_parser (125 MHz)
    //-------------------------------------------------------------------------
    wire [3:0] S125_key_evt;
    wire       S125_sd_evt, S125_cam_evt, S125_ld_evt, S125_rep_evt, S125_cmd;

    cmd_parser u_cmd_parser_eth (
        .I_clk      ( S_clk125    ),
        .I_rst_n    ( S_eth_rst_n ),
        .I_rx_valid ( R_udp_valid ),
        .I_rx_data  ( R_udp_data  ),
        .O_key_evt  ( S125_key_evt ),
        .O_sd_evt   ( S125_sd_evt  ),
        .O_cam_evt  ( S125_cam_evt ),
        .O_ld_evt   ( S125_ld_evt  ),
        .O_rep_evt  ( S125_rep_evt ),
        .O_cmd_valid( S125_cmd     )
    );

    //-------------------------------------------------------------------------
    // event bridge 125 MHz -> S_100m_clk (latched bundle + level handshake)
    //-------------------------------------------------------------------------
    reg [7:0] r_evt_lat;
    reg [7:0] r_pkt;
    reg       r_req;
    reg [1:0] r_ack_sync125;

    reg [1:0] r_req_sync100;
    reg       S_ack_100;
    reg [3:0] O_key_evt_r;
    reg       O_sd_evt_r, O_cam_evt_r, O_ld_evt_r, O_rep_evt_r, O_cmd_valid_r;

    wire [7:0] S125_evt = {S125_cmd, S125_rep_evt, S125_ld_evt, S125_cam_evt,
                           S125_sd_evt, S125_key_evt};

    always @(posedge S_clk125 or negedge S_eth_rst_n) begin
        if(!S_eth_rst_n) begin
            r_evt_lat   <= 8'd0;
            r_pkt       <= 8'd0;
            r_req       <= 1'b0;
            r_ack_sync125 <= 2'b00;
        end
        else begin
            r_ack_sync125 <= {r_ack_sync125[0], S_ack_100};
            if(r_req) begin
                if(r_ack_sync125[1] == r_req)
                    r_req <= 1'b0;
            end
            else if(r_evt_lat != 8'd0) begin
                r_pkt     <= r_evt_lat;
                r_evt_lat <= S125_evt;
                r_req     <= 1'b1;
            end
            else begin
                r_evt_lat <= S125_evt;
            end
        end
    end

    always @(posedge I_clk100 or negedge I_rst_n) begin
        if(!I_rst_n) begin
            r_req_sync100 <= 2'b00;
            S_ack_100     <= 1'b0;
            O_key_evt_r   <= 4'd0;
            O_sd_evt_r    <= 1'b0;
            O_cam_evt_r   <= 1'b0;
            O_ld_evt_r    <= 1'b0;
            O_rep_evt_r   <= 1'b0;
            O_cmd_valid_r <= 1'b0;
        end
        else begin
            r_req_sync100 <= {r_req_sync100[0], r_req};
            O_key_evt_r   <= 4'd0;
            O_sd_evt_r    <= 1'b0;
            O_cam_evt_r   <= 1'b0;
            O_ld_evt_r    <= 1'b0;
            O_rep_evt_r   <= 1'b0;
            O_cmd_valid_r <= 1'b0;
            if(r_req_sync100[1] != S_ack_100) begin
                O_key_evt_r   <= r_pkt[3:0];
                O_sd_evt_r    <= r_pkt[4];
                O_cam_evt_r   <= r_pkt[5];
                O_ld_evt_r    <= r_pkt[6];
                O_rep_evt_r   <= r_pkt[7];
                O_cmd_valid_r <= |r_pkt;
                S_ack_100     <= r_req_sync100[1];
            end
        end
    end

    assign O_key_evt   = O_key_evt_r;
    assign O_sd_evt    = O_sd_evt_r;
    assign O_cam_evt   = O_cam_evt_r;
    assign O_ld_evt    = O_ld_evt_r;
    assign O_rep_evt   = O_rep_evt_r;
    assign O_cmd_valid = O_cmd_valid_r;

    //-------------------------------------------------------------------------
    // status (level stroches stretched in 125 MHz, safe to resample)
    //-------------------------------------------------------------------------
    reg [21:0] r_rx_stretch;
    reg [21:0] r_er_stretch;

    always @(posedge S_clk125 or negedge S_eth_rst_n) begin
        if(!S_eth_rst_n) begin
            r_rx_stretch <= 22'd0;
            r_er_stretch <= 22'd0;
        end
        else begin
            if(R_udp_valid)
                r_rx_stretch <= 22'h3FFFFF;
            else if(r_rx_stretch != 22'd0)
                r_rx_stretch <= r_rx_stretch - 22'd1;

            if(S_ip_rerror | S_mac_rerror)
                r_er_stretch <= 22'h3FFFFF;
            else if(r_er_stretch != 22'd0)
                r_er_stretch <= r_er_stretch - 22'd1;
        end
    end

    assign O_link   = S_link;
    assign O_tx_act = (r_tx_stretch != 22'd0);
    assign O_rx_act = (r_rx_stretch != 22'd0);
    assign O_err_act= (r_er_stretch != 22'd0);

endmodule
