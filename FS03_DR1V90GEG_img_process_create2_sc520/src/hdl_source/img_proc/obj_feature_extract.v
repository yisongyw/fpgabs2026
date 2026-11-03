//-----------------------------------------------------------------------------
// Module     : obj_feature_extract
// Project    : HX1P35Aimg_process_extend2
// Description: Turns the raw tile records of obj_detect into recognised targets.
//
//   For every tile that carries enough pixels:
//
//     1. compute 8 uint8 features  (size / fill / aspect / compactness /
//                                   mean Cb / mean Cr / skin share / texture)
//        using ONE shared serial arithmetic unit (ser_math) -> 0 DSP
//     2. classify them with the INT8 MLP        -> class + confidence [%]
//     3. classify them with a pure rule         -> class
//     4. keep the NOBJ largest targets in a list that the OSD draws
//
//   Both classifiers run every time, the KEY4 switch only selects which result
//   is drawn on the screen.  That way the two row status line can show
//   "RULE:xxxx" and "MLP:xxxx nn%" side by side.
//
//   Timing : every tile needs 1 multiplication + 6 divisions + one MLP run
//            = about 7 x 32 + 430 = 650 clocks.  Even all 12 tiles together
//            need less than 8000 clocks while one frame offers ~230000, i.e.
//            the whole recognition costs about 3 % of the frame time and adds
//            NO latency to the video path (the video stream is never stalled).
//
//   Output bus (per object, NOBJ entries, entry 0 = left most / largest).
//   The width and every field offset follow CLS_W (see the B_* localparams),
//   with CLS_W = 3 the layout is exactly the historical one :
//      [55]    valid
//      [54:44] xmin   [43:33] xmax   [32:23] ymin   [22:13] ymax
//      [12:10] rule class   [9:7] MLP class   [6:0] MLP confidence 0..99
//-----------------------------------------------------------------------------
module obj_feature_extract #(
    parameter MIN_AREA = 17'd500,       // minimum blob size in pixels
    parameter NOBJ     = 2,             // extend3 : 4 -> 2 objects (saves overlay / tracker resources)
    // ---- unified class parameters : the ONLY place the class count is set ---
    parameter CLS_W    = 6,             // bit width of one class index (2**CLS_W classes max)
    parameter NOUT     = 64,            // number of MLP classes (NOUT <= 2**CLS_W)
    parameter REC_W    = 50 + 2*CLS_W   // width of ONE object record (CLS_W=6 -> 62)
)(
    input  wire        I_clk,
    input  wire        I_rst_n,

    // tile records from obj_detect (handshake)
    input  wire        I_tile_valid,
    input  wire        I_tile_last,
    input  wire [16:0] I_cnt,
    input  wire [10:0] I_xmin,
    input  wire [10:0] I_xmax,
    input  wire [9:0]  I_ymin,
    input  wire [9:0]  I_ymax,
    input  wire [16:0] I_peri,
    input  wire [24:0] I_sum_cb,
    input  wire [24:0] I_sum_cr,
    input  wire [16:0] I_skin,
    input  wire [7:0]  I_ylo,
    input  wire [7:0]  I_yhi,

    output reg         O_tile_next,     // take the next record

    output reg  [3:0]  O_obj_num,       // targets found in this frame (0..12)
    output wire [NOBJ*REC_W-1:0] O_obj_bus
);

    //-------------------------------------------------------------------------
    // record layout - every offset is DERIVED from CLS_W, so the class count
    // can be changed without touching a single hard coded bit position.
    //   (CLS_W = 3 reproduces exactly the historical 56 bit record)
    //-------------------------------------------------------------------------
    localparam B_CONF = 0;                  // [ 6: 0]  confidence
    localparam B_MLP  = B_CONF + 7;         // [ 9: 7]  MLP class
    localparam B_RULE = B_MLP  + CLS_W;     // [12:10]  rule class
    localparam B_YMAX = B_RULE + CLS_W;     // [22:13]
    localparam B_YMIN = B_YMAX + 10;        // [32:23]
    localparam B_XMAX = B_YMIN + 10;        // [43:33]
    localparam B_XMIN = B_XMAX + 11;        // [54:44]
    localparam B_VAL  = B_XMIN + 11;        // [55]

    //-------------------------------------------------------------------------
    // classes (short English words, exactly what is printed on the screen)
    //-------------------------------------------------------------------------
    // improve1 : the rule classifier keeps its eight decisions, they are now
    // mapped onto the eight matching entries of the 64 class table
    // (class index = shape family * 8 + colour family, see osd_cls_lut) :
    //   round+skin   -> MAN ( 7)   round+red      -> BALL( 0)
    //   block+yellow -> BOX (10)   flat+red       -> CARD(16)
    //   tri+red      -> CONE(24)   irregular+blue-> TOY (52)
    //   tall+yellow  -> CUP (34)   irregular+brown-> DOG(55)
    //
    // improve4 : the eight rules and their thresholds are UNCHANGED, only the
    // indices they point at moved (64 classes instead of 32).  The MLP is the
    // path that resolves all 64 categories; the rule stays the fall back.
    localparam [CLS_W-1:0] CLS_MAN  = 6'd7;
    localparam [CLS_W-1:0] CLS_BALL = 6'd0;
    localparam [CLS_W-1:0] CLS_BOX  = 6'd10;
    localparam [CLS_W-1:0] CLS_CARD = 6'd16;
    localparam [CLS_W-1:0] CLS_TRI  = 6'd24;
    localparam [CLS_W-1:0] CLS_OBJ  = 6'd52;
    localparam [CLS_W-1:0] CLS_DOG  = 6'd55;   // quadruped / pet (brown, skin share 0)
    localparam [CLS_W-1:0] CLS_BOT  = 6'd34;   // cup / bottle / cylinder (tall)

    // states
    localparam [3:0] ST_WAIT  = 4'd0;
    localparam [3:0] ST_LATCH = 4'd1;
    localparam [3:0] ST_WH0   = 4'd2;
    localparam [3:0] ST_OPST  = 4'd3;
    localparam [3:0] ST_OPW   = 4'd4;
    localparam [3:0] ST_MLPST = 4'd5;
    localparam [3:0] ST_MLPW  = 4'd6;
    localparam [3:0] ST_NEXT  = 4'd7;

    // step 0 = w*h multiply, 1..6 = the six divisions
    localparam [2:0] OP_WH    = 3'd0;
    localparam [2:0] OP_FILL  = 3'd1;
    localparam [2:0] OP_ASPEC = 3'd2;
    localparam [2:0] OP_CMPCT = 3'd3;
    localparam [2:0] OP_CB    = 3'd4;
    localparam [2:0] OP_CR    = 3'd5;
    localparam [2:0] OP_SKIN  = 3'd6;

    //-------------------------------------------------------------------------
    // working copy of the tile record
    //-------------------------------------------------------------------------
    reg [16:0] w_cnt, w_peri, w_skin;
    reg [10:0] w_xmin, w_xmax;
    reg [9:0]  w_ymin, w_ymax;
    reg [24:0] w_cb, w_cr;
    reg [7:0]  w_ylo, w_yhi;

    wire [10:0] S_w = (w_xmax >= w_xmin) ? (w_xmax - w_xmin + 11'd1) : 11'd1;
    wire [9:0]  S_h = (w_ymax >= w_ymin) ? (w_ymax - w_ymin + 10'd1) : 10'd1;
    wire [10:0] S_dmin = (S_w < {1'b0, S_h}) ? S_w : {1'b0, S_h};
    wire [10:0] S_dmax = (S_w < {1'b0, S_h}) ? {1'b0, S_h} : S_w;

    //-------------------------------------------------------------------------
    // shared arithmetic unit
    //-------------------------------------------------------------------------
    reg  [31:0] r_a;
    reg  [23:0] r_b;
    reg         r_op;
    reg         r_start;
    wire [31:0] S_res;
    wire        S_done;

    ser_math u_math (
        .I_clk    ( I_clk    ),
        .I_rst_n  ( I_rst_n  ),
        .I_start  ( r_start  ),
        .I_op     ( r_op     ),
        .I_a      ( r_a      ),
        .I_b      ( r_b      ),
        .O_result ( S_res    ),
        .O_done   ( S_done   )
    );

    reg [31:0] r_wh;

    //-------------------------------------------------------------------------
    // features
    //-------------------------------------------------------------------------
    reg [7:0] f_size, f_fill, f_aspect, f_compct, f_cb, f_cr, f_skin, f_yrange;

    function [7:0] clog14;
        input [16:0] v;
        begin
            casex(v)
                17'b1_xxxx_xxxx_xxxx_xxxx: clog14 = 8'd16;
                17'b01_xxxx_xxxx_xxxx_xxx: clog14 = 8'd15;
                17'b001_xxxx_xxxx_xxxx_xx: clog14 = 8'd14;
                17'b0001_xxxx_xxxx_xxxx_x: clog14 = 8'd13;
                17'b00001_xxxx_xxxx_xxxx: clog14 = 8'd12;
                17'b000001_xxxx_xxxx_xxx: clog14 = 8'd11;
                17'b0000001_xxxx_xxxx_xx: clog14 = 8'd10;
                17'b00000001_xxxx_xxxx_x: clog14 = 8'd9;
                17'b000000001_xxxx_xxxx: clog14 = 8'd8;
                17'b0000000001_xxxx_xxx: clog14 = 8'd7;
                17'b00000000001_xxxx_xx: clog14 = 8'd6;
                17'b000000000001_xxxx_x: clog14 = 8'd5;
                17'b0000000000001_xxxx: clog14 = 8'd4;
                17'b00000000000001_xxx: clog14 = 8'd3;
                17'b000000000000001_xx: clog14 = 8'd2;
                17'b0000000000000001_x: clog14 = 8'd1;
                default:                  clog14 = 8'd0;
            endcase
        end
    endfunction

    function [7:0] sat8;
        input [31:0] v;
        begin
            sat8 = (|v[31:8]) ? 8'd255 : v[7:0];
        end
    endfunction

    //-------------------------------------------------------------------------
    // pure rule classification (no network at all - the fall back path)
    //-------------------------------------------------------------------------
    function [CLS_W-1:0] rule_cls;
        input [7:0] f0, f1, f2, f3, f4, f5, f6, f7;
        begin
            if(f6 >= 8'd110)                 rule_cls = CLS_MAN;    // skin
            else if(f4 >= 8'd215 && f5 >= 8'd95 && f5 <= 8'd130 && f2 < 8'd165) rule_cls = CLS_BOT;   // blue, tall
            else if(f4 >= 8'd110 && f4 <= 8'd135 && f5 >= 8'd80 && f5 <= 8'd115 && f6 < 8'd120) rule_cls = CLS_DOG;   // brown
            else if(f3 >= 8'd165)            rule_cls = CLS_OBJ;    // ragged
            else if(f2 <  8'd130)            rule_cls = CLS_CARD;   // elongated
            else if(f1 >= 8'd225)            rule_cls = CLS_BOX;    // ~ 1.00 fill
            else if(f1 >= 8'd165)            rule_cls = CLS_BALL;   // ~ 0.78 fill
            else                             rule_cls = CLS_TRI;    // ~ 0.50 fill
        end
    endfunction

    wire [CLS_W-1:0] S_rule = rule_cls(f_size, f_fill, f_aspect, f_compct,
                                       f_cb, f_cr, f_skin, f_yrange);

    //-------------------------------------------------------------------------
    // MLP
    //-------------------------------------------------------------------------
    reg             r_mlp_start;
    wire            S_mlp_valid;
    wire [CLS_W-1:0] S_mlp_cls;
    wire [6:0]      S_mlp_conf;

    wire [63:0] S_feat = { f_yrange, f_skin, f_cr, f_cb,
                           f_compct, f_aspect, f_fill, f_size };

    mlp_classifier #(
        .CLS_W   ( CLS_W        ),
        .NOUT    ( NOUT         )
    ) u_mlp (
        .I_clk   ( I_clk        ),
        .I_rst_n ( I_rst_n      ),
        .I_start ( r_mlp_start  ),
        .I_feat  ( S_feat[63:0] ),
        .O_valid ( S_mlp_valid  ),
        .O_class ( S_mlp_cls    ),
        .O_conf  ( S_mlp_conf   )
    );

    //-------------------------------------------------------------------------
    // object list (NOBJ largest targets)
    //-------------------------------------------------------------------------
    reg  [REC_W-1:0] obj     [0:NOBJ-1];
    reg  [16:0]      obj_area[0:NOBJ-1];

    wire [REC_W-1:0] S_new_rec = { 1'b1, w_xmin, w_xmax, w_ymin, w_ymax,
                                   S_rule, S_mlp_cls, S_mlp_conf };

    reg        r_in_frame;
    reg [2:0]  r_step;
    reg [3:0]  r_state;
    reg [3:0]  r_num;

    // insertion position : 0 = the new object is the largest one
    wire [2:0] S_pos =
        (w_cnt > obj_area[0]) ? 3'd0 :
        (NOBJ > 1 && w_cnt > obj_area[1]) ? 3'd1 :
        (NOBJ > 2 && w_cnt > obj_area[2]) ? 3'd2 :
        (NOBJ > 3 && w_cnt > obj_area[3]) ? 3'd3 : 3'd4;

    integer i;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            w_cnt    <= 17'd0;
            w_peri   <= 17'd0;
            w_skin   <= 17'd0;
            w_xmin   <= 11'd0;
            w_xmax   <= 11'd0;
            w_ymin   <= 10'd0;
            w_ymax   <= 10'd0;
            w_cb     <= 25'd0;
            w_cr     <= 25'd0;
            w_ylo    <= 8'd0;
            w_yhi    <= 8'd0;
            r_wh     <= 32'd0;
            r_step   <= 3'd0;
            r_state  <= ST_WAIT;
            r_start  <= 1'b0;
            r_op     <= 1'b0;
            r_a      <= 32'd0;
            r_b      <= 24'd0;
            r_mlp_start <= 1'b0;
            r_in_frame  <= 1'b0;
            r_num    <= 4'd0;
            O_tile_next <= 1'b0;
            O_obj_num   <= 4'd0;
            f_size   <= 8'd0;
            f_fill   <= 8'd0;
            f_aspect <= 8'd0;
            f_compct <= 8'd0;
            f_cb     <= 8'd0;
            f_cr     <= 8'd0;
            f_skin   <= 8'd0;
            f_yrange <= 8'd0;
            for(i = 0; i < NOBJ; i = i + 1) begin
                obj     [i] <= {REC_W{1'b0}};
                obj_area[i] <= 17'd0;
            end
        end
        else begin
            r_start     <= 1'b0;
            r_mlp_start <= 1'b0;
            O_tile_next <= 1'b0;

            case(r_state)
                //----------------------------------------------------------
                // wait for a tile record
                //----------------------------------------------------------
                ST_WAIT: begin
                    if(I_tile_valid) begin
                        if(!r_in_frame) begin
                            // first record of this frame -> start a new list
                            r_in_frame <= 1'b1;
                            r_num      <= 4'd0;
                            O_obj_num  <= 4'd0;
                            for(i = 0; i < NOBJ; i = i + 1) begin
                                obj     [i] <= {REC_W{1'b0}};
                                obj_area[i] <= 17'd0;
                            end
                        end
                        if(I_cnt < MIN_AREA) begin
                            // too small -> skip immediately
                            O_tile_next <= 1'b1;
                            if(I_tile_last) r_in_frame <= 1'b0;
                        end
                        else begin
                            r_state <= ST_LATCH;
                        end
                    end
                end

                //----------------------------------------------------------
                ST_LATCH: begin
                    w_cnt  <= I_cnt;
                    w_peri <= I_peri;
                    w_skin <= I_skin;
                    w_xmin <= I_xmin;
                    w_xmax <= I_xmax;
                    w_ymin <= I_ymin;
                    w_ymax <= I_ymax;
                    w_cb   <= I_sum_cb;
                    w_cr   <= I_sum_cr;
                    w_ylo  <= I_ylo;
                    w_yhi  <= I_yhi;
                    // features that need no arithmetic
                    f_size   <= (clog14(I_cnt) > 8'd15) ? 8'd255
                                                        : (clog14(I_cnt) << 4);
                    f_yrange <= (I_yhi > I_ylo) ? (I_yhi - I_ylo) : 8'd0;
                    r_state  <= ST_WH0;
                end

                //----------------------------------------------------------
                // bounding box area (one cycle after the record is latched,
                // so that S_w / S_h are built from the new values)
                //----------------------------------------------------------
                ST_WH0: begin
                    r_op     <= 1'b0;                       // multiply
                    r_a      <= {27'd0, S_w};
                    r_b      <= {14'd0, S_h};
                    r_start  <= 1'b1;
                    r_state  <= ST_OPW;
                    r_step   <= OP_WH;
                end

                //----------------------------------------------------------
                // wait for the serial unit
                //----------------------------------------------------------
                ST_OPW: begin
                    if(S_done) begin
                        if(r_step == OP_WH) begin
                            r_wh <= S_res;
                        end
                        else begin
                            case(r_step)
                                OP_FILL : f_fill   <= sat8(S_res);
                                OP_ASPEC: f_aspect <= sat8(S_res);
                                OP_CMPCT: f_compct <= sat8(S_res);
                                OP_CB   : f_cb     <= sat8(S_res);
                                OP_CR   : f_cr     <= sat8(S_res);
                                default : f_skin   <= sat8(S_res);
                            endcase
                        end

                        if(r_step == OP_SKIN) begin
                            r_state <= ST_MLPST;
                        end
                        else begin
                            r_step  <= r_step + 3'd1;
                            r_state <= ST_OPST;
                        end
                    end
                end

                //----------------------------------------------------------
                // launch the next serial operation
                //----------------------------------------------------------
                ST_OPST: begin
                    r_op    <= 1'b1;                       // divide
                    case(r_step)
                        OP_FILL : begin
                            r_a <= {15'd0, w_cnt, 8'd0};   // cnt * 256 / (w*h)
                            r_b <= (|r_wh[31:24]) ? 24'hffffff : r_wh[23:0];
                        end
                        OP_ASPEC: begin
                            r_a <= {21'd0, S_dmin, 8'd0};   // min*256 / max
                            r_b <= {13'd0, S_dmax};
                        end
                        OP_CMPCT: begin
                            r_a <= {18'd0, w_peri, 7'd0};   // peri*128 / 2(w+h)
                            r_b <= {11'd0, (S_w + {1'b0, S_h}), 1'b0};
                        end
                        OP_CB   : begin
                            r_a <= {7'd0, w_cb[24:0]};
                            r_b <= {7'd0, w_cnt};
                        end
                        OP_CR   : begin
                            r_a <= {7'd0, w_cr[24:0]};
                            r_b <= {7'd0, w_cnt};
                        end
                        default : begin
                            r_a <= {15'd0, w_skin, 8'd0};   // skin*256 / cnt
                            r_b <= {7'd0, w_cnt};
                        end
                    endcase
                    r_start <= 1'b1;
                    r_state <= ST_OPW;
                end

                //----------------------------------------------------------
                // run the network
                //----------------------------------------------------------
                ST_MLPST: begin
                    r_mlp_start <= 1'b1;
                    r_state     <= ST_MLPW;
                end

                ST_MLPW: begin
                    if(S_mlp_valid) begin
                        // keep the NOBJ largest targets
                        if(S_pos != 3'd4) begin
                            for(i = 0; i < NOBJ; i = i + 1) begin
                                if(i[2:0] > S_pos) begin
                                    obj     [i] <= obj     [i-1];
                                    obj_area[i] <= obj_area[i-1];
                                end
                                else if(i[2:0] == S_pos) begin
                                    obj     [i] <= S_new_rec;
                                    obj_area[i] <= w_cnt;
                                end
                            end
                        end
                        r_num   <= (r_num == 4'd15) ? 4'd15 : (r_num + 4'd1);
                        r_state <= ST_NEXT;
                    end
                end

                //----------------------------------------------------------
                ST_NEXT: begin
                    O_obj_num   <= r_num;
                    O_tile_next <= 1'b1;
                    if(I_tile_last) r_in_frame <= 1'b0;
                    r_state     <= ST_WAIT;
                end

                default: r_state <= ST_WAIT;
            endcase
        end
    end

    //-------------------------------------------------------------------------
    // output packing
    //-------------------------------------------------------------------------
    genvar g;
    generate
        for (g = 0; g < NOBJ; g = g + 1) begin : g_pack
            assign O_obj_bus[g*REC_W +: REC_W] = obj[g];
        end
    endgenerate

endmodule
