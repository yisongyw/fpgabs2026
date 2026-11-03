//-----------------------------------------------------------------------------
// Module     : eth_result_tx
// Project    : HX1P35Aimg_process_create1
// Description: detection / recognition result packet builder.
//
//   At every *valid moment* (tracker update at HDMI frame blanking, a remote
//   "R?" report request or an SD image completion) one UDP payload is built
//   and handed over to the Ethernet application (eth_app) which streams it
//   into the uiudp_stack transmit interface.
//
//   Payload (little endian, fixed 39 bytes) :
//       [ 0]    0xA5            magic
//       [ 1]    seq             frame / event counter
//       [ 2]    flags           [2:0] alg switch (SW1/2/3)
//                                        [3] motion alarm
//                                        [5:4] style level (KEY3)
//                                        [6] classifier mode (0 rule / 1 MLP)
//                                        [7] SD image mode
//       [ 3]    obj_num         number of valid objects (0 .. 2)
//       [ 4]    th[7:0]         algorithm threshold (KEY2), low byte
//       [ 5]    {7'd0, th[8]}
//       [ 6 .. 21] track 0 : valid, conf, mlp, rule, id, miss,
//                            xtl_lo, xtl_hi, xbr_lo, xbr_hi,
//                            ytl_lo, ytl_hi, ybr_lo, ybr_hi, 0, 0
//       [22 .. 37] track 1 : same
//       [38]    0x5A            tail
//
//   Clock domains : payload build = S_100m_clk ; read / stream = 125 MHz
//   (eth_app).  A two phase toggle handshake protects the shared 64 byte RAM.
//-----------------------------------------------------------------------------
module eth_result_tx #(
    parameter TRK_W = 70
)(
    input  wire              I_clk,        // S_100m_clk
    input  wire              I_rst_n,

    // one shot triggers (S_100m_clk domain)
    input  wire              I_trig,

    // result / status snapshot (stable during one frame)
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

    // shared payload RAM read port (125 MHz side, see handshakes below)
    input  wire [5:0]        I_rd_addr,
    output wire [7:0]        O_rd_data,

    output reg               O_req_tgl,    // toggles when a payload is ready
    output reg  [5:0]        O_len,        // payload length - 1 (38)
    input  wire              I_ack_tgl     // toggles when the streamer is done
);

    // record field offsets (must match ctrl/obj_tracker.v, CLS_W = 6)
    localparam T_MISS = 0;
    localparam T_CONF = 5;
    localparam T_MLP  = 12;
    localparam T_RULE = 18;
    localparam T_ID   = 24;
    localparam T_YBR  = 27;
    localparam T_YTL  = 37;
    localparam T_XBR  = 47;
    localparam T_XTL  = 58;
    localparam T_VAL  = 69;

    reg [7:0]  r_mem [0:63];
    reg [5:0]  r_wcnt;
    reg [7:0]  r_seq;
    reg        r_busy;       // 0 = idle, 1 = build / hand over
    reg        r_pub;        // 1 = published, waiting for the streamer
    reg [1:0]  r_ack_sync;
    wire       S_ack = r_ack_sync[1];

    assign O_rd_data = r_mem[I_rd_addr];

    // decode helpers
    wire [6:0] t0_conf = I_track0[T_CONF +: 7];
    wire [5:0] t0_mlp  = I_track0[T_MLP  +: 6];
    wire [5:0] t0_rule = I_track0[T_RULE +: 6];
    wire [2:0] t0_id   = I_track0[T_ID   +: 3];
    wire [3:0] t0_miss = I_track0[T_MISS +: 4];
    wire       t0_val  = I_track0[T_VAL];
    wire [6:0] t1_conf = I_track1[T_CONF +: 7];
    wire [5:0] t1_mlp  = I_track1[T_MLP  +: 6];
    wire [5:0] t1_rule = I_track1[T_RULE +: 6];
    wire [2:0] t1_id   = I_track1[T_ID   +: 3];
    wire [3:0] t1_miss = I_track1[T_MISS +: 4];
    wire       t1_val  = I_track1[T_VAL];

    // per byte value (combinational, addressed by r_wcnt)
    function [7:0] track_byte;
        input [6:0]  v_conf;
        input [5:0]  v_mlp;
        input [5:0]  v_rule;
        input [2:0]  v_id;
        input [3:0]  v_miss;
        input        v_val;
        input [10:0] v_xtl;
        input [10:0] v_xbr;
        input [9:0]  v_ytl;
        input [9:0]  v_ybr;
        input [4:0]  idx;
        begin
            case(idx)
                5'd0:  track_byte = {7'd0, v_val};
                5'd1:  track_byte = {1'b0, v_conf};
                5'd2:  track_byte = {2'd0, v_mlp};
                5'd3:  track_byte = {2'd0, v_rule};
                5'd4:  track_byte = {5'd0, v_id};
                5'd5:  track_byte = {4'd0, v_miss};
                5'd6:  track_byte = v_xtl[7:0];
                5'd7:  track_byte = {5'd0, v_xtl[10:8]};
                5'd8:  track_byte = v_xbr[7:0];
                5'd9:  track_byte = {5'd0, v_xbr[10:8]};
                5'd10: track_byte = v_ytl[7:0];
                5'd11: track_byte = {6'd0, v_ytl[9:8]};
                5'd12: track_byte = v_ybr[7:0];
                5'd13: track_byte = {6'd0, v_ybr[9:8]};
                default: track_byte = 8'd0;
            endcase
        end
    endfunction

    function [7:0] payload_byte;
        input [5:0] idx;
        begin
            case(idx)
                6'd0: payload_byte = 8'hA5;
                6'd1: payload_byte = r_seq;
                6'd2: payload_byte = {I_sd_mode, I_cls_mode, I_style[1:0],
                                      I_alarm, I_alg};
                6'd3: payload_byte = {4'd0, I_obj_num};
                6'd4: payload_byte = I_th[7:0];
                6'd5: payload_byte = {7'd0, I_th[8]};
                6'd6:  payload_byte = track_byte(t0_conf, t0_mlp, t0_rule, t0_id,
                        t0_miss, t0_val, I_track0[T_XTL +: 11], I_track0[T_XBR +: 11],
                        I_track0[T_YTL +: 10], I_track0[T_YBR +: 10], 5'd0);
                6'd7:  payload_byte = track_byte(t0_conf, t0_mlp, t0_rule, t0_id,
                        t0_miss, t0_val, I_track0[T_XTL +: 11], I_track0[T_XBR +: 11],
                        I_track0[T_YTL +: 10], I_track0[T_YBR +: 10], 5'd1);
                6'd8:  payload_byte = track_byte(t0_conf, t0_mlp, t0_rule, t0_id,
                        t0_miss, t0_val, I_track0[T_XTL +: 11], I_track0[T_XBR +: 11],
                        I_track0[T_YTL +: 10], I_track0[T_YBR +: 10], 5'd2);
                6'd9:  payload_byte = track_byte(t0_conf, t0_mlp, t0_rule, t0_id,
                        t0_miss, t0_val, I_track0[T_XTL +: 11], I_track0[T_XBR +: 11],
                        I_track0[T_YTL +: 10], I_track0[T_YBR +: 10], 5'd3);
                6'd10: payload_byte = track_byte(t0_conf, t0_mlp, t0_rule, t0_id,
                        t0_miss, t0_val, I_track0[T_XTL +: 11], I_track0[T_XBR +: 11],
                        I_track0[T_YTL +: 10], I_track0[T_YBR +: 10], 5'd4);
                6'd11: payload_byte = track_byte(t0_conf, t0_mlp, t0_rule, t0_id,
                        t0_miss, t0_val, I_track0[T_XTL +: 11], I_track0[T_XBR +: 11],
                        I_track0[T_YTL +: 10], I_track0[T_YBR +: 10], 5'd5);
                6'd12: payload_byte = track_byte(t0_conf, t0_mlp, t0_rule, t0_id,
                        t0_miss, t0_val, I_track0[T_XTL +: 11], I_track0[T_XBR +: 11],
                        I_track0[T_YTL +: 10], I_track0[T_YBR +: 10], 5'd6);
                6'd13: payload_byte = track_byte(t0_conf, t0_mlp, t0_rule, t0_id,
                        t0_miss, t0_val, I_track0[T_XTL +: 11], I_track0[T_XBR +: 11],
                        I_track0[T_YTL +: 10], I_track0[T_YBR +: 10], 5'd7);
                6'd14: payload_byte = track_byte(t0_conf, t0_mlp, t0_rule, t0_id,
                        t0_miss, t0_val, I_track0[T_XTL +: 11], I_track0[T_XBR +: 11],
                        I_track0[T_YTL +: 10], I_track0[T_YBR +: 10], 5'd8);
                6'd15: payload_byte = track_byte(t0_conf, t0_mlp, t0_rule, t0_id,
                        t0_miss, t0_val, I_track0[T_XTL +: 11], I_track0[T_XBR +: 11],
                        I_track0[T_YTL +: 10], I_track0[T_YBR +: 10], 5'd9);
                6'd16: payload_byte = track_byte(t0_conf, t0_mlp, t0_rule, t0_id,
                        t0_miss, t0_val, I_track0[T_XTL +: 11], I_track0[T_XBR +: 11],
                        I_track0[T_YTL +: 10], I_track0[T_YBR +: 10], 5'd10);
                6'd17: payload_byte = track_byte(t0_conf, t0_mlp, t0_rule, t0_id,
                        t0_miss, t0_val, I_track0[T_XTL +: 11], I_track0[T_XBR +: 11],
                        I_track0[T_YTL +: 10], I_track0[T_YBR +: 10], 5'd11);
                6'd18: payload_byte = track_byte(t0_conf, t0_mlp, t0_rule, t0_id,
                        t0_miss, t0_val, I_track0[T_XTL +: 11], I_track0[T_XBR +: 11],
                        I_track0[T_YTL +: 10], I_track0[T_YBR +: 10], 5'd12);
                6'd19: payload_byte = track_byte(t0_conf, t0_mlp, t0_rule, t0_id,
                        t0_miss, t0_val, I_track0[T_XTL +: 11], I_track0[T_XBR +: 11],
                        I_track0[T_YTL +: 10], I_track0[T_YBR +: 10], 5'd13);
                6'd20: payload_byte = 8'd0;
                6'd21: payload_byte = 8'd0;
                6'd22: payload_byte = track_byte(t1_conf, t1_mlp, t1_rule, t1_id,
                        t1_miss, t1_val, I_track1[T_XTL +: 11], I_track1[T_XBR +: 11],
                        I_track1[T_YTL +: 10], I_track1[T_YBR +: 10], 5'd0);
                6'd23: payload_byte = track_byte(t1_conf, t1_mlp, t1_rule, t1_id,
                        t1_miss, t1_val, I_track1[T_XTL +: 11], I_track1[T_XBR +: 11],
                        I_track1[T_YTL +: 10], I_track1[T_YBR +: 10], 5'd1);
                6'd24: payload_byte = track_byte(t1_conf, t1_mlp, t1_rule, t1_id,
                        t1_miss, t1_val, I_track1[T_XTL +: 11], I_track1[T_XBR +: 11],
                        I_track1[T_YTL +: 10], I_track1[T_YBR +: 10], 5'd2);
                6'd25: payload_byte = track_byte(t1_conf, t1_mlp, t1_rule, t1_id,
                        t1_miss, t1_val, I_track1[T_XTL +: 11], I_track1[T_XBR +: 11],
                        I_track1[T_YTL +: 10], I_track1[T_YBR +: 10], 5'd3);
                6'd26: payload_byte = track_byte(t1_conf, t1_mlp, t1_rule, t1_id,
                        t1_miss, t1_val, I_track1[T_XTL +: 11], I_track1[T_XBR +: 11],
                        I_track1[T_YTL +: 10], I_track1[T_YBR +: 10], 5'd4);
                6'd27: payload_byte = track_byte(t1_conf, t1_mlp, t1_rule, t1_id,
                        t1_miss, t1_val, I_track1[T_XTL +: 11], I_track1[T_XBR +: 11],
                        I_track1[T_YTL +: 10], I_track1[T_YBR +: 10], 5'd5);
                6'd28: payload_byte = track_byte(t1_conf, t1_mlp, t1_rule, t1_id,
                        t1_miss, t1_val, I_track1[T_XTL +: 11], I_track1[T_XBR +: 11],
                        I_track1[T_YTL +: 10], I_track1[T_YBR +: 10], 5'd6);
                6'd29: payload_byte = track_byte(t1_conf, t1_mlp, t1_rule, t1_id,
                        t1_miss, t1_val, I_track1[T_XTL +: 11], I_track1[T_XBR +: 11],
                        I_track1[T_YTL +: 10], I_track1[T_YBR +: 10], 5'd7);
                6'd30: payload_byte = track_byte(t1_conf, t1_mlp, t1_rule, t1_id,
                        t1_miss, t1_val, I_track1[T_XTL +: 11], I_track1[T_XBR +: 11],
                        I_track1[T_YTL +: 10], I_track1[T_YBR +: 10], 5'd8);
                6'd31: payload_byte = track_byte(t1_conf, t1_mlp, t1_rule, t1_id,
                        t1_miss, t1_val, I_track1[T_XTL +: 11], I_track1[T_XBR +: 11],
                        I_track1[T_YTL +: 10], I_track1[T_YBR +: 10], 5'd9);
                6'd32: payload_byte = track_byte(t1_conf, t1_mlp, t1_rule, t1_id,
                        t1_miss, t1_val, I_track1[T_XTL +: 11], I_track1[T_XBR +: 11],
                        I_track1[T_YTL +: 10], I_track1[T_YBR +: 10], 5'd10);
                6'd33: payload_byte = track_byte(t1_conf, t1_mlp, t1_rule, t1_id,
                        t1_miss, t1_val, I_track1[T_XTL +: 11], I_track1[T_XBR +: 11],
                        I_track1[T_YTL +: 10], I_track1[T_YBR +: 10], 5'd11);
                6'd34: payload_byte = track_byte(t1_conf, t1_mlp, t1_rule, t1_id,
                        t1_miss, t1_val, I_track1[T_XTL +: 11], I_track1[T_XBR +: 11],
                        I_track1[T_YTL +: 10], I_track1[T_YBR +: 10], 5'd12);
                6'd35: payload_byte = track_byte(t1_conf, t1_mlp, t1_rule, t1_id,
                        t1_miss, t1_val, I_track1[T_XTL +: 11], I_track1[T_XBR +: 11],
                        I_track1[T_YTL +: 10], I_track1[T_YBR +: 10], 5'd13);
                6'd36: payload_byte = 8'd0;
                6'd37: payload_byte = 8'd0;
                6'd38: payload_byte = 8'h5A;
                default: payload_byte = 8'd0;
            endcase
        end
    endfunction

    //   handshake : after the payload is written O_req_tgl is toggled ; the
    //   125 MHz streamer answers by driving I_ack_tgl to the SAME value, so
    //   "S_ack == O_req_tgl" means the payload was consumed (or not yet
    //   requested) and a new build may start.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            r_wcnt    <= 6'd0;
            r_seq     <= 8'd0;
            r_busy    <= 1'b0;
            r_pub     <= 1'b0;
            O_req_tgl <= 1'b0;
            O_len     <= 6'd38;
            r_ack_sync<= 2'b00;
        end
        else begin
            r_ack_sync <= {r_ack_sync[0], I_ack_tgl};

            if(!r_busy) begin
                if(I_trig) begin
                    r_busy <= 1'b1;
                    r_pub  <= 1'b0;
                    r_wcnt <= 6'd0;
                end
            end
            else if(r_wcnt != 6'd39) begin
                r_mem[r_wcnt] <= payload_byte(r_wcnt);
                r_wcnt <= r_wcnt + 6'd1;
            end
            else if(!r_pub) begin
                // payload fully written and the previous one consumed : publish
                if(S_ack == O_req_tgl) begin
                    O_req_tgl <= ~O_req_tgl;
                    O_len     <= 6'd38;
                    r_seq     <= r_seq + 8'd1;
                    r_pub     <= 1'b1;
                end
            end
            else if(S_ack == O_req_tgl) begin
                // streamer copied the request value -> it has finished the RAM
                r_busy <= 1'b0;
            end
        end
    end

endmodule
