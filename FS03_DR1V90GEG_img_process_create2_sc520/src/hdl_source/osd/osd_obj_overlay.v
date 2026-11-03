//-----------------------------------------------------------------------------
// Module     : osd_obj_overlay
// Project    : HX1P35Aimg_process_improve1
// Description: Third OSD layer.  Draws the detection AND the tracking result
//              into the HDMI picture :
//
//   STATIC indicators of a detected target (extend2 / extend3, unchanged)
//                * a BW pixel wide bounding box around every target
//                * the class name of the target above the box (8 x 8 font)
//                * a 1 digit object ID right after the class name
//                * the ROI rectangle used by the alarm (red when a target is
//                  inside it, grey otherwise)
//                * every class has its own colour (32 classes in improve1)
//
//   DYNAMIC indicators of a TRACKED target (improve1, new)
//                * a 16 point trajectory trail of the box centres
//                * a crosshair on the current centre of the target
//                * a marker on the PREDICTED centre (motion prediction)
//                * three text lines below the box
//                      line 0 : ( cx , cy )   current position  (coordinates)
//                      line 1 : V( vx , vy )  velocity, pixel per frame
//                      line 2 : P( px , py )  predicted position (coordinates)
//                * while a target is occluded (1 .. 8 frames without a
//                  detection) the box is drawn DASHED at the extrapolated
//                  position - the visible proof of a real tracker
//
//   This is the "overlay of the detection result AND a text explanation" that
//   the competition asks for (extension requirement 2) plus the "tracking
//   marker or coordinate information" of advanced requirement 2.5 (2).
//
//   I_cls_sel selects which classifier drives the drawing
//        0 : pure rule      (KEY4 position 0)
//        1 : INT8 MLP       (KEY4 position 1)
//
//   I_osd_en (SW4) hides the whole layer, the picture then stays untouched.
//   Every static and every dynamic indicator of this layer is switched by SW4,
//   exactly like in extend3 and in the projects before it.
//-----------------------------------------------------------------------------
module osd_obj_overlay #(
    parameter NOBJ  = 2,          // drawn objects
    parameter NTRK  = 2,          // track slots (same number of records)
    parameter BW    = 12'd3,      // box line thickness in pixels
    parameter CLS_W = 6,          // bit width of a class index (2**CLS_W max)
    parameter REC_W = 50 + 2*CLS_W, // width of one object record
    parameter DYN_W = 72,         // width of one dynamic indicator record
    // improve2 : length of the trajectory trail.
    //   improve1 stored 16 centres.  Every stored centre costs a 21 bit
    //   equality comparator against the running pixel coordinate, i.e.
    //   16 x 2 objects x ~12 LUT = about 380 LUT - and LUTs are the bottleneck
    //   of this device while its DSP blocks are almost idle.  8 points still
    //   draw a perfectly readable trail.  Set 16 to restore the improve1 look.
    // parameter TRAJ_LEN = 8     // original (improve2)

    //-------------------------------------------------------------------------
    // Area / routability : 8 -> 6
    //   The device is at its limit : with the timing fixes the design reached
    //   20911 / 21216 slices (98.56 %) and the route could not close at all
    //   (route WNS -9.763 ns, 762 violated endpoints, route elapsed 5 h 29 min,
    //   while the same design routed fine at 96.74 %).
    //   Shortening the trail by two points removes
    //       2 objects x 2 points x 21 bit = 84 registers   (r_trax / r_tray)
    //       2 objects x 2 points x ~12 LUT = 48 LUT        (trail comparators)
    //   Nothing else changes : the trail is only two dots shorter, every other
    //   overlay feature (box, label, id digit, text lines, crosshair, predicted
    //   centre) is untouched.  Restore by setting it back to 8.
    //-------------------------------------------------------------------------
    // parameter TRAJ_LEN = 6     // first area step (8 -> 6)

    //-------------------------------------------------------------------------
    // Area / routability : 6 -> 4
    //   Run C (TRAJ_LEN = 6) reached 20797 / 21216 slices (98.03 %).
    //   The place stage was already clean (Setup WNS +930 ps, TNS 0,
    //   0 violated endpoint) but the router could not converge :
    //   route took 4 h 06 min and ended with WNS -2.854 ns on
    //   HDMI_PIXEL_CLK (57 violated endpoints) - pure congestion, the logic
    //   itself is fast enough.  Getting below ~97 % is what lets the router
    //   converge normally again (Run A at 96.74 % routed in 36 min).
    //   Two more points remove
    //       2 objects x 2 points x 21 bit = 84 registers   (r_trax / r_tray)
    //       2 objects x 2 points x ~12 LUT = 48 LUT        (trail comparators)
    //   i.e. about 115 slices : 20797 -> ~20680 (97.5 %).
    //   Nothing else changes : only the trail is two dots shorter.  Restore by
    //   setting it back to 8 (original) or 6.
    //-------------------------------------------------------------------------
    parameter TRAJ_LEN = 4
)(
    input  wire                 I_clk,
    input  wire                 I_rst_n,

    input  wire                 I_video_vsync,
    input  wire                 I_video_hsync,
    input  wire                 I_video_de,
    input  wire                 I_video_user,
    input  wire                 I_video_last,

    input  wire                 I_osd_en,      // SW4
    input  wire                 I_cls_sel,     // KEY4 : 0 rule / 1 MLP
    input  wire [NOBJ*REC_W-1:0] I_obj_bus,
    input  wire [NOBJ*3-1:0]    I_trk_id,      // object id per record
    input  wire [NTRK*DYN_W-1:0] I_trk_dyn,    // dynamic indicators (improve1)
    input  wire                 I_alarm,       // ROI alarm
    input  wire [11:0]          I_roi_x,
    input  wire [11:0]          I_roi_y,
    input  wire [11:0]          I_roi_w,
    input  wire [11:0]          I_roi_h,
    input  wire [23:0]          I_video_data,

    output reg                  O_video_vsync,
    output reg                  O_video_hsync,
    output reg                  O_video_de,
    output reg  [23:0]          O_video_data
);

    //-------------------------------------------------------------------------
    // record layout (identical to obj_feature_extract)
    //-------------------------------------------------------------------------
    localparam B_CONF  = 0;
    localparam B_MLP   = B_CONF + 7;
    localparam B_RULE  = B_MLP  + CLS_W;
    localparam B_YMAX  = B_RULE + CLS_W;
    localparam B_YMIN  = B_YMAX + 10;
    localparam B_XMAX  = B_YMIN + 10;
    localparam B_XMIN  = B_XMAX + 11;
    localparam B_VALID = B_XMIN + 11;

    // dynamic indicator layout (identical to obj_tracker)
    localparam D_LOST = 0;
    localparam D_AGE  = D_LOST + 1;
    localparam D_VY   = D_AGE  + 5;
    localparam D_VX   = D_VY   + 12;
    localparam D_PY   = D_VX   + 12;
    localparam D_PX   = D_PY   + 10;
    localparam D_CY   = D_PX   + 11;
    localparam D_CX   = D_CY   + 10;

    //-------------------------------------------------------------------------
    // colours
    //-------------------------------------------------------------------------
    localparam [23:0] C_TRAIL  = 24'h909090;    // grey   - trajectory dot
    localparam [23:0] C_PRED   = 24'hffff00;    // yellow - predicted centre
    localparam [23:0] C_TEXT   = 24'hffffff;    // white  - coordinate text

    localparam CHAR_SPACE = 8'h20;
    localparam CHAR_LBR   = 8'h28;   // '('
    localparam CHAR_RBR   = 8'h29;   // ')'
    localparam CHAR_COMMA = 8'h2c;   // ','
    localparam CHAR_PLUS  = 8'h2b;
    localparam CHAR_MINUS = 8'h2d;
    localparam CHAR_V     = 8'h56;
    localparam CHAR_P     = 8'h50;
    localparam CHAR_0     = 8'h30;

    // text block geometry : 3 lines of 8 x 8 glyphs, 9 pixel line pitch
    localparam [11:0] TXT_LINES = 12'd3;
    localparam [11:0] TXT_PITCH = 12'd9;
    localparam [11:0] TXT_W     = 12'd96;    // 12 characters
    localparam [11:0] TXT_H     = 12'd26;

    //-------------------------------------------------------------------------
    // pixel coordinates (same scheme as every other OSD layer)
    //-------------------------------------------------------------------------
    reg [11:0] S_x;
    reg [11:0] S_y;
    reg [11:0] S_x_1d, S_x_2d;
    reg [11:0] S_y_1d, S_y_2d;
    reg        S_de_1d,  S_de_2d;
    reg        S_vs_1d,  S_vs_2d;
    reg        S_hs_1d,  S_hs_2d;
    reg [23:0] S_data_1d, S_data_2d;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
        end
        else if(I_video_user) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
        end
        else if(I_video_de) begin
            if(I_video_last) begin
                S_x <= 12'd0;
                S_y <= S_y + 12'd1;
            end
            else begin
                S_x <= S_x + 12'd1;
            end
        end
    end

    //-------------------------------------------------------------------------
    // per object : extracted fields
    //-------------------------------------------------------------------------
    wire [REC_W-1:0] S_rec [0:NOBJ-1];
    wire [DYN_W-1:0] S_dyn [0:NOBJ-1];

    genvar gi;
    generate
        for (gi = 0; gi < NOBJ; gi = gi + 1) begin : g_unpack
            assign S_rec[gi] = I_obj_bus[gi*REC_W +: REC_W];
            assign S_dyn[gi] = I_trk_dyn[gi*DYN_W +: DYN_W];
        end
    endgenerate

    wire        S_valid [0:NOBJ-1];
    wire [10:0] S_xmin  [0:NOBJ-1];
    wire [10:0] S_xmax  [0:NOBJ-1];
    wire [9:0]  S_ymin  [0:NOBJ-1];
    wire [9:0]  S_ymax  [0:NOBJ-1];
    wire [CLS_W-1:0] S_rule [0:NOBJ-1];
    wire [CLS_W-1:0] S_mlp  [0:NOBJ-1];

    // dynamic indicators
    wire        S_lost [0:NOBJ-1];
    wire signed [11:0] S_vx   [0:NOBJ-1];
    wire signed [11:0] S_vy   [0:NOBJ-1];
    wire [10:0] S_pcx  [0:NOBJ-1];
    wire [9:0]  S_pcy  [0:NOBJ-1];
    wire [10:0] S_ccx  [0:NOBJ-1];
    wire [9:0]  S_ccy  [0:NOBJ-1];

    generate
        for (gi = 0; gi < NOBJ; gi = gi + 1) begin : g_fields
            // B_* are LSB indices -> the MSB of a W bit field is (B_* + W - 1)
            assign S_valid[gi] = S_rec[gi][B_VALID];
            assign S_xmin [gi] = S_rec[gi][B_XMIN + 10 -: 11];
            assign S_xmax [gi] = S_rec[gi][B_XMAX + 10 -: 11];
            assign S_ymin [gi] = S_rec[gi][B_YMIN +  9 -: 10];
            assign S_ymax [gi] = S_rec[gi][B_YMAX +  9 -: 10];
            assign S_rule [gi] = S_rec[gi][B_RULE + CLS_W - 1 -: CLS_W];
            assign S_mlp  [gi] = S_rec[gi][B_MLP  + CLS_W - 1 -: CLS_W];

            assign S_lost[gi]  = S_dyn[gi][D_LOST];
            assign S_vx  [gi]  = S_dyn[gi][D_VX + 12 - 1 -: 12];
            assign S_vy  [gi]  = S_dyn[gi][D_VY + 12 - 1 -: 12];
            assign S_pcx [gi]  = S_dyn[gi][D_PX + 11 - 1 -: 11];
            assign S_pcy [gi]  = S_dyn[gi][D_PY + 10 - 1 -: 10];
            assign S_ccx [gi]  = S_dyn[gi][D_CX + 11 - 1 -: 11];
            assign S_ccy [gi]  = S_dyn[gi][D_CY + 10 - 1 -: 10];
        end
    endgenerate

    //-------------------------------------------------------------------------
    // 32 class names / colours (one small memory per drawn object)
    //
    //   NOTE: this ROM output was earlier registered as well (to shave the
    //   first one or two levels of the S_obj56_r_reg -> S_data_2d_reg path).
    //   That stage was REVERTED : S_name (2 x 32) + S_clscol (2 x 24) = 112
    //   registers cost far more slices than the timing they recovered, and the
    //   resulting 98.56 % utilisation made the design nearly unroutable
    //   (route WNS -9.763 ns, 762 violated endpoints, 5 h 29 min route).
    //   The rest of the osd_obj_overlay pipeline (r_lbl_y / r_txt_y) is KEPT :
    //   that is the stage that really broke the long arithmetic chain.
    //-------------------------------------------------------------------------
    wire [31:0] S_name  [0:NOBJ-1];
    wire [23:0] S_clscol[0:NOBJ-1];

    generate
        for (gi = 0; gi < NOBJ; gi = gi + 1) begin : g_lut
            osd_cls_lut u_cls_lut (
                .I_cls   ( I_cls_sel ? S_mlp[gi] : S_rule[gi] ),
                .O_name  ( S_name  [gi] ),
                .O_color ( S_clscol[gi] )
            );
        end
    endgenerate

    //-------------------------------------------------------------------------
    // decimal conversion
    //
    //   The coordinates and the velocity of a track are constant for a whole
    //   frame, so ONE binary -> BCD converter is enough : it is time
    //   multiplexed over the (NTRK * 6) values at the beginning of every
    //   frame.  Instantiating one converter per value would cost about 12
    //   times the logic.
    //-------------------------------------------------------------------------
    function [10:0] abs_clamp;          // |v| limited to 999 (3 digits)
        input signed [11:0] v;
        reg [10:0] m;
        begin
            m = v[11] ? (~v[10:0] + 11'd1) : v[10:0];
            abs_clamp = (m > 11'd999) ? 11'd999 : m;
        end
    endfunction

    function [15:0] bcd11;              // 11 bit binary -> 4 BCD digits
        input [10:0] bin;
        reg [26:0] t;
        integer i;
        begin
            t = 27'd0;
            t[10:0] = bin;
            for (i = 0; i < 11; i = i + 1) begin
                if (t[11 +: 4] > 4) t[11 +: 4] = t[11 +: 4] + 4'd3;
                if (t[15 +: 4] > 4) t[15 +: 4] = t[15 +: 4] + 4'd3;
                if (t[19 +: 4] > 4) t[19 +: 4] = t[19 +: 4] + 4'd3;
                if (t[23 +: 4] > 4) t[23 +: 4] = t[23 +: 4] + 4'd3;
                t = t << 1;
            end
            bcd11 = t[26:11];
        end
    endfunction

    localparam NDEC = NTRK * 6;         // cx cy |vx| |vy| px py  per track

    reg [15:0] r_dec [0:NDEC-1];
    reg [3:0]  r_csel;
    reg        r_crun;
    reg [10:0] S_cval;
    reg        r_user_d;

    always @(*) begin
        case(r_csel)
            4'd0:  S_cval = S_ccx[0];
            4'd1:  S_cval = {1'd0, S_ccy[0]};
            4'd2:  S_cval = abs_clamp(S_vx[0]);
            4'd3:  S_cval = abs_clamp(S_vy[0]);
            4'd4:  S_cval = S_pcx[0];
            4'd5:  S_cval = {1'd0, S_pcy[0]};
            4'd6:  S_cval = S_ccx[1];
            4'd7:  S_cval = {1'd0, S_ccy[1]};
            4'd8:  S_cval = abs_clamp(S_vx[1]);
            4'd9:  S_cval = abs_clamp(S_vy[1]);
            4'd10: S_cval = S_pcx[1];
            default: S_cval = {1'd0, S_pcy[1]};
        endcase
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            r_crun <= 1'b0;
            r_csel <= 4'd0;
        end
        else begin
            r_user_d <= I_video_user;
            if (I_video_user && !r_user_d) begin
                r_crun <= 1'b1;
                r_csel <= 4'd0;
            end
            else if (r_crun) begin
                r_dec[r_csel] <= bcd11(S_cval);
                if (r_csel == NDEC[3:0] - 4'd1) begin
                    r_crun <= 1'b0;
                    r_csel <= 4'd0;
                end
                else begin
                    r_csel <= r_csel + 4'd1;
                end
            end
        end
    end

    //-------------------------------------------------------------------------
    // trajectory history : last 16 box centres per object
    //-------------------------------------------------------------------------
    reg [10:0] r_trax [0:NOBJ-1][0:TRAJ_LEN-1];
    reg [9:0]  r_tray [0:NOBJ-1][0:TRAJ_LEN-1];
    integer kt, ht;
    reg [10:0] tcx;
    reg [9:0]  tcy;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            for (kt = 0; kt < NOBJ; kt = kt + 1)
                for (ht = 0; ht < TRAJ_LEN; ht = ht + 1) begin
                    r_trax[kt][ht] <= 11'd0;
                    r_tray[kt][ht] <= 10'd0;
                end
        end else begin
            if (I_video_user && !r_user_d) begin   // frame start
                for (kt = 0; kt < NOBJ; kt = kt + 1) begin
                    if (S_valid[kt]) begin
                        tcx = (S_xmin[kt] + S_xmax[kt]) >> 1;
                        tcy = (S_ymin[kt] + S_ymax[kt]) >> 1;
                        for (ht = TRAJ_LEN-1; ht > 0; ht = ht - 1) begin
                            r_trax[kt][ht] <= r_trax[kt][ht-1];
                            r_tray[kt][ht] <= r_tray[kt][ht-1];
                        end
                        r_trax[kt][0] <= tcx;
                        r_tray[kt][0] <= tcy;
                    end
                end
            end
        end
    end

    //-------------------------------------------------------------------------
    // label position : above the box, or below it when the box touches the top
    //   text block : below the box, or above it when the box touches the bottom
    //-------------------------------------------------------------------------
    function [11:0] lbl_y;
        input [9:0] ymin;
        begin
            lbl_y = (ymin >= 10'd10) ? ({2'd0, ymin} - 12'd10) : 12'd2;
        end
    endfunction

    function [11:0] txt_y;
        input [9:0] ymin;
        input [9:0] ymax;
        reg [11:0] below;
        begin
            below = {2'd0, ymax} + 12'd3;
            if ((below + TXT_H) <= 12'd719) txt_y = below;
            else if ({2'd0, ymin} >= (TXT_H + 12'd1))
                txt_y = {2'd0, ymin} - TXT_H - 12'd1;
            else txt_y = 12'd2;
        end
    endfunction

    //-------------------------------------------------------------------------
    // frame static object geometry
    //
    //   lbl_y() and txt_y() depend ONLY on S_ymin / S_ymax, i.e. on the object
    //   box coordinates, and those are constant for a whole frame (see the
    //   comment on the BCD converter above : the trajectory history is updated
    //   once per frame at I_video_user).  They are therefore pre-computed into
    //   registers here, so the per-pixel hit test starts from a register
    //   instead of from the 3..4 level arithmetic chain of txt_y() / lbl_y().
    //
    //   This is the stage that splits the 16 level osd_obj_overlay -> video
    //   output path measured in phy_1/final_timing.rpt
    //        HDMI_PIXEL_CLK intra  SWNS -0.448 ns
    //        begin S_obj56_r_reg[37] -> end S_data_2d_reg[23], 61 % net delay
    //
    //   Because the inputs are frame static this adds exactly one clock of
    //   latency to the object geometry only.  The pixel stream (S_x_2d, S_y_2d,
    //   S_data_2d) is NOT touched, so the overlay stays aligned : at most one
    //   pixel of a frame can still use the previous frame's box geometry, which
    //   is invisible.
    //-------------------------------------------------------------------------
    reg [11:0] r_lbl_y [0:NOBJ-1];
    reg [11:0] r_txt_y [0:NOBJ-1];

    integer kg;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            for (kg = 0; kg < NOBJ; kg = kg + 1) begin
                r_lbl_y[kg] <= 12'd0;
                r_txt_y[kg] <= 12'd0;
            end
        end
        else begin
            for (kg = 0; kg < NOBJ; kg = kg + 1) begin
                r_lbl_y[kg] <= lbl_y(S_ymin[kg]);
                r_txt_y[kg] <= txt_y(S_ymin[kg], S_ymax[kg]);
            end
        end
    end

    // 3 x 5 digit font for the object id (0..7)
    function char_digit;
        input [2:0] d;
        input [1:0] x;   // 0..2
        input [2:0] y;   // 0..4
        reg [14:0] f;
        begin
            case(d)
                3'd0: f = 15'b111_101_101_101_111;
                3'd1: f = 15'b001_001_001_001_001;
                3'd2: f = 15'b111_001_111_100_111;
                3'd3: f = 15'b111_001_111_001_111;
                3'd4: f = 15'b101_101_111_001_001;
                3'd5: f = 15'b111_100_111_001_111;
                3'd6: f = 15'b111_100_111_101_111;
                3'd7: f = 15'b111_001_001_001_001;
                default: f = 15'b000_000_000_000_000;
            endcase
            char_digit = f[y*3 + x];
        end
    endfunction

    //-------------------------------------------------------------------------
    // combinational hit test
    //-------------------------------------------------------------------------
    reg [7:0]  S_char;
    reg [2:0]  S_row;
    reg [2:0]  S_col;
    reg [1:0]  S_di;
    reg [2:0]  S_dj;
    reg        S_lbl_hit;
    reg        S_box_hit;
    reg        S_trail_hit;
    reg        S_roi_hit;
    reg        S_cross_hit;
    reg        S_pred_hit;
    reg        S_txt_hit;
    reg        S_txt_panel;
    reg [23:0] S_color;
    reg [23:0] S_roi_col;
    reg        S_panel;
    reg [11:0] S_xoff;
    reg [11:0] S_yoff;
    reg [11:0] S_txoff;
    reg [11:0] S_tyoff;
    reg [1:0]  S_tline;
    reg [3:0]  S_tidx;
    reg [3:0]  S_trow;
    reg [11:0] S_dxc, S_dyc;
    reg signed [11:0] S_vsx, S_vsy;

    wire [7:0] S_row_bits;

    osd_text8_lib u_osd_text8_lib (
        .I_char     ( S_char      ),
        .I_row      ( S_row       ),
        .O_row_bits ( S_row_bits  )
    );

    integer k;

    always @(*) begin
        S_char      = CHAR_SPACE;
        S_row       = 3'd0;
        S_col       = 3'd0;
        S_di        = 2'd0;
        S_dj        = 3'd0;
        S_lbl_hit   = 1'b0;
        S_box_hit   = 1'b0;
        S_trail_hit = 1'b0;
        S_roi_hit   = 1'b0;
        S_cross_hit = 1'b0;
        S_pred_hit  = 1'b0;
        S_txt_hit   = 1'b0;
        S_txt_panel = 1'b0;
        S_color     = 24'hffffff;
        S_roi_col   = I_alarm ? 24'hff0000 : 24'h888888;
        S_panel     = 1'b0;
        // default for the pixel relative offsets.  Without it the synthesis tool
        // infers a latch on S_xoff / S_yoff (HDL-5369) because they are only
        // assigned inside the "S_de_2d && I_osd_en" branch further down.  They
        // are only read inside that same branch, so the latch was harmless, but
        // a default makes the block fully combinational and silences it.
        S_xoff      = 12'd0;
        S_yoff      = 12'd0;

        // ---- trajectory trail (grey dots at the previous centres) ----
        for (kt = 0; kt < NOBJ; kt = kt + 1)
            for (ht = 0; ht < TRAJ_LEN; ht = ht + 1)
                if ((S_x_2d == {1'd0, r_trax[kt][ht]}) &&
                    (S_y_2d == {2'd0, r_tray[kt][ht]}))
                    S_trail_hit = 1'b1;

        // ---- ROI rectangle outline ----
        if (S_de_2d && I_osd_en &&
            (S_x_2d >= I_roi_x) && (S_x_2d <= I_roi_x + I_roi_w) &&
            (S_y_2d >= I_roi_y) && (S_y_2d <= I_roi_y + I_roi_h)) begin
            if (((S_x_2d - I_roi_x) < 12'd2) ||
                ((I_roi_x + I_roi_w - S_x_2d) < 12'd2) ||
                ((S_y_2d - I_roi_y) < 12'd2) ||
                ((I_roi_y + I_roi_h - S_y_2d) < 12'd2))
                S_roi_hit = 1'b1;
        end

        if (S_de_2d && I_osd_en) begin
            for (k = 0; k < NOBJ; k = k + 1) begin
                S_xoff = S_x_2d - {1'd0, S_xmin[k]};
                S_yoff = S_y_2d - r_lbl_y[k];
                if (S_valid[k]) begin
                    //------------------------------------------------------
                    // bounding box : solid while locked, DASHED while the
                    // track is extrapolated through an occlusion
                    //------------------------------------------------------
                    if ((S_y_2d >= {2'd0, S_ymin[k]}) && (S_y_2d <= {2'd0, S_ymax[k]}) &&
                        (S_x_2d >= {1'd0, S_xmin[k]}) && (S_x_2d <= {1'd0, S_xmax[k]})) begin
                        if (((S_y_2d - {2'd0, S_ymin[k]}) < BW) ||
                            (({2'd0, S_ymax[k]} - S_y_2d) < BW) ||
                            ((S_x_2d - {1'd0, S_xmin[k]}) < BW) ||
                            (({1'd0, S_xmax[k]} - S_x_2d) < BW)) begin
                            if (!S_lost[k] ||
                                (S_xoff[2:0] < 3'd4) || (S_yoff[2:0] < 3'd4)) begin
                                S_box_hit = 1'b1;
                                S_color   = S_clscol[k];
                            end
                        end
                    end

                    //------------------------------------------------------
                    // crosshair on the current centre + marker on the
                    // predicted centre (dynamic indicators)
                    //------------------------------------------------------
                    S_dxc = (S_x_2d > {1'd0, S_ccx[k]}) ?
                            (S_x_2d - {1'd0, S_ccx[k]}) : ({1'd0, S_ccx[k]} - S_x_2d);
                    S_dyc = (S_y_2d > {2'd0, S_ccy[k]}) ?
                            (S_y_2d - {2'd0, S_ccy[k]}) : ({2'd0, S_ccy[k]} - S_y_2d);
                    if ((S_dxc <= 12'd4) && (S_dyc <= 12'd4) &&
                        ((S_dxc <= 12'd1) || (S_dyc <= 12'd1)))
                        S_cross_hit = 1'b1;
                    if ((S_x_2d >= {1'd0, S_pcx[k]} - 12'd2) &&
                        (S_x_2d <= {1'd0, S_pcx[k]} + 12'd2) &&
                        (S_y_2d >= {2'd0, S_pcy[k]} - 12'd2) &&
                        (S_y_2d <= {2'd0, S_pcy[k]} + 12'd2))
                        S_pred_hit = 1'b1;

                    //------------------------------------------------------
                    // class name label : 4 characters of 8 x 8 pixel
                    //------------------------------------------------------
                    if ((S_x_2d >= {1'd0, S_xmin[k]}) &&
                        (S_x_2d <  ({1'd0, S_xmin[k]} + 12'd32)) &&
                        (S_y_2d >= r_lbl_y[k]) &&
                        (S_y_2d <  (r_lbl_y[k] + 12'd8))) begin
                        S_panel = 1'b1;
                        if (!S_box_hit)
                            S_color = S_clscol[k];
                        S_col  = S_xoff[2:0];
                        S_row  = S_yoff[2:0];
                        S_char = S_name[k][(3 - S_xoff[4:3])*8 +: 8];
                        if (S_row_bits[7 - S_col])
                            S_lbl_hit = 1'b1;
                    end

                    //------------------------------------------------------
                    // object id digit (3 x 5, right after the name)
                    //------------------------------------------------------
                    if ((S_x_2d >= ({1'd0, S_xmin[k]} + 12'd33)) &&
                        (S_x_2d <  ({1'd0, S_xmin[k]} + 12'd36)) &&
                        (S_y_2d >= r_lbl_y[k]) &&
                        (S_y_2d <  (r_lbl_y[k] + 12'd5))) begin
                        S_panel = 1'b1;
                        if (!S_box_hit)
                            S_color = 24'hffffff;
                        S_di = S_x_2d - ({1'd0, S_xmin[k]} + 12'd33);
                        S_dj = S_y_2d - r_lbl_y[k];
                        if (char_digit(I_trk_id[k*3 +: 3], S_di, S_dj))
                            S_lbl_hit = 1'b1;
                    end

                    //------------------------------------------------------
                    // three text lines : (cx,cy) / V(vx,vy) / P(px,py)
                    //------------------------------------------------------
                    S_txoff = S_x_2d - {1'd0, S_xmin[k]};
                    S_tyoff = S_y_2d - r_txt_y[k];
                    if ((S_txoff < TXT_W) && (S_tyoff < TXT_H)) begin
                        S_txt_panel = 1'b1;
                        S_tline = (S_tyoff >= 12'd18) ? 2'd2 :
                                  (S_tyoff >= 12'd9)  ? 2'd1 : 2'd0;
                        S_tidx  = S_txoff[6:3];
                        S_col   = S_txoff[2:0];
                        S_trow  = (S_tline == 2'd2) ? (S_tyoff - 12'd18) :
                                  (S_tline == 2'd1) ? (S_tyoff - 12'd9)  :
                                                      S_tyoff;
                        S_char  = CHAR_SPACE;
                        if (S_trow < 4'd8) begin
                            S_row = S_trow[2:0];
                            case (S_tline)
                                // ---- line 0 : current position (cx,cy) ----
                                2'd0: begin
                                    case (S_tidx)
                                        4'd0:    S_char = CHAR_LBR;
                                        4'd1:    S_char = CHAR_0 + {1'd0, r_dec[k*6+0][15:12]};
                                        4'd2:    S_char = CHAR_0 + r_dec[k*6+0][11:8];
                                        4'd3:    S_char = CHAR_0 + r_dec[k*6+0][7:4];
                                        4'd4:    S_char = CHAR_0 + r_dec[k*6+0][3:0];
                                        4'd5:    S_char = CHAR_COMMA;
                                        4'd6:    S_char = CHAR_0 + r_dec[k*6+1][11:8];
                                        4'd7:    S_char = CHAR_0 + r_dec[k*6+1][7:4];
                                        4'd8:    S_char = CHAR_0 + r_dec[k*6+1][3:0];
                                        4'd9:    S_char = CHAR_RBR;
                                        default: S_char = CHAR_SPACE;
                                    endcase
                                end
                                // ---- line 1 : velocity V(vx,vy) ----
                                2'd1: begin
                                    S_vsx = S_vx[k];
                                    S_vsy = S_vy[k];
                                    case (S_tidx)
                                        4'd0:    S_char = CHAR_V;
                                        4'd1:    S_char = CHAR_LBR;
                                        4'd2:    S_char = S_vsx[11] ? CHAR_MINUS : CHAR_PLUS;
                                        4'd3:    S_char = CHAR_0 + r_dec[k*6+2][11:8];
                                        4'd4:    S_char = CHAR_0 + r_dec[k*6+2][7:4];
                                        4'd5:    S_char = CHAR_0 + r_dec[k*6+2][3:0];
                                        4'd6:    S_char = CHAR_COMMA;
                                        4'd7:    S_char = S_vsy[11] ? CHAR_MINUS : CHAR_PLUS;
                                        4'd8:    S_char = CHAR_0 + r_dec[k*6+3][11:8];
                                        4'd9:    S_char = CHAR_0 + r_dec[k*6+3][7:4];
                                        4'd10:   S_char = CHAR_0 + r_dec[k*6+3][3:0];
                                        4'd11:   S_char = CHAR_RBR;
                                        default: S_char = CHAR_SPACE;
                                    endcase
                                end
                                // ---- line 2 : predicted position (px,py) ----
                                default: begin
                                    case (S_tidx)
                                        4'd0:    S_char = CHAR_P;
                                        4'd1:    S_char = CHAR_LBR;
                                        4'd2:    S_char = CHAR_0 + {1'd0, r_dec[k*6+4][15:12]};
                                        4'd3:    S_char = CHAR_0 + r_dec[k*6+4][11:8];
                                        4'd4:    S_char = CHAR_0 + r_dec[k*6+4][7:4];
                                        4'd5:    S_char = CHAR_0 + r_dec[k*6+4][3:0];
                                        4'd6:    S_char = CHAR_COMMA;
                                        4'd7:    S_char = CHAR_0 + r_dec[k*6+5][11:8];
                                        4'd8:    S_char = CHAR_0 + r_dec[k*6+5][7:4];
                                        4'd9:    S_char = CHAR_0 + r_dec[k*6+5][3:0];
                                        4'd10:   S_char = CHAR_RBR;
                                        default: S_char = CHAR_SPACE;
                                    endcase
                                end
                            endcase
                            if (S_row_bits[7 - S_col])
                                S_txt_hit = 1'b1;
                        end
                    end
                end
            end
        end
    end

    //-------------------------------------------------------------------------
    // output alignment : everything delayed by 2 clocks
    //-------------------------------------------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x_1d     <= 12'd0;
            S_x_2d     <= 12'd0;
            S_y_1d     <= 12'd0;
            S_y_2d     <= 12'd0;
            S_de_1d    <= 1'b0;
            S_de_2d    <= 1'b0;
            S_vs_1d    <= 1'b0;
            S_vs_2d    <= 1'b0;
            S_hs_1d    <= 1'b0;
            S_hs_2d    <= 1'b0;
            S_data_1d  <= 24'd0;
            S_data_2d  <= 24'd0;
            O_video_vsync <= 1'b0;
            O_video_hsync <= 1'b0;
            O_video_de    <= 1'b0;
            O_video_data  <= 24'd0;
        end
        else begin
            S_x_1d    <= S_x;
            S_x_2d    <= S_x_1d;
            S_y_1d    <= S_y;
            S_y_2d    <= S_y_1d;

            S_vs_1d   <= I_video_vsync;
            S_vs_2d   <= S_vs_1d;
            S_hs_1d   <= I_video_hsync;
            S_hs_2d   <= S_hs_1d;
            S_de_1d   <= I_video_de;
            S_de_2d   <= S_de_1d;
            S_data_1d <= I_video_data;
            S_data_2d <= S_data_1d;

            O_video_vsync <= S_vs_2d;
            O_video_hsync <= S_hs_2d;
            O_video_de    <= S_de_2d;

            if (S_roi_hit)
                O_video_data <= S_roi_col;
            else if (S_box_hit || S_lbl_hit)
                O_video_data <= S_color;
            else if (S_txt_hit)
                O_video_data <= C_TEXT;
            else if (S_cross_hit)
                O_video_data <= S_color;
            else if (S_pred_hit)
                O_video_data <= C_PRED;
            else if (S_trail_hit)
                O_video_data <= C_TRAIL;
            else if (S_panel || S_txt_panel)
                // dark panel behind the letters -> readable on every picture
                O_video_data <= {3'b000, S_data_2d[23:3]};
            else
                O_video_data <= S_data_2d;
        end
    end

endmodule
