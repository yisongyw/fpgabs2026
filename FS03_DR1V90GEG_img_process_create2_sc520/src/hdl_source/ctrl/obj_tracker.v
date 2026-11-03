//==========================================================================
// obj_tracker.v  (HX1P35Aimg_process_improve1)   -- TRACKING v2 --
//--------------------------------------------------------------------------
// Cross-frame object tracker with temporal voting, motion prediction and a
// full set of DYNAMIC indicators (advanced requirement 2.5 (2) : "dynamic
// tracking of a target and overlay of a tracking marker or of coordinate
// information").
//
// The raw detector produces a fresh, independent list of objects every
// video frame.  Small per-frame variations (segmentation noise, partial
// occlusion, skin-colour flicker) make the *displayed* class / box jump
// around and occasionally flip to the wrong class.
//
// v1 (extend3, still fully contained in v2)
//   * nearest neighbour association on the bounding box
//   * 3 frame majority voting of the (rule, mlp) class -> stable class
//   * drop a track after MAX_MISS consecutive misses
//
// v2 (this file) - what has been added
//   1. the association distance is measured between CENTROIDS (v1 compared
//      the new centroid with the old TOP LEFT corner, which carries a
//      systematic error of half the box width and therefore lost large
//      objects much earlier than small ones)
//   2. a first order velocity is estimated and the PREDICTED position
//      (cx + vx, cy + vy) is used for the gating -> fast objects keep their ID
//           v' = (3*v + d) >> 2          (alpha-beta, shifts only, 0 DSP)
//   3. LIFE CYCLE / OCCLUSION : while a track is not matched (1 .. MAX_MISS
//      frames) it is extrapolated with its own velocity instead of being
//      dropped -> "the box becomes dashed but does not disappear", the
//      decisive proof that this is a tracker and not only a detector
//   4. the ROI alarm now triggers on the CENTROID (v1 required the whole box
//      to be inside the ROI, so a large object never triggered it)
//   5. the DYNAMIC indicators of every track are exported on O_trk_dyn :
//          current centre (cx, cy), velocity (vx, vy), PREDICTED centre
//          (px, py) and the "currently occluded" flag
//      -> osd_obj_overlay prints them next to the box, switched by SW4 like
//         every other overlay of this project.
//
// The module stays intentionally tiny (2 tracks) : it is evaluated once per
// frame during the vertical blanking interval of the HDMI clock domain and
// costs a few hundred LUTs.
//==========================================================================
`timescale 1ns / 1ps

module obj_tracker
#(
    parameter NOBJ  = 2,            // number of incoming objects per frame
    parameter NTRK  = 2,            // persistent track slots
    parameter CLS_W = 6,            // bit width of a class index (2**CLS_W max)
    parameter REC_W = 50 + 2*CLS_W, // width of one incoming object record
    parameter TRK_W = 58 + 2*CLS_W, // width of one extended track record
    parameter DYN_W = 72,           // width of one dynamic indicator record
    parameter GATE  = 96,           // NN association gate (L1 distance, pixels)
    parameter MAX_MISS = 8          // miss frames before a track is dropped
)
(
    input  wire                  I_clk,
    input  wire                  I_rst_n,         // active-LOW reset
    input  wire                  I_frame_start,   // 1-clock pulse, once per frame

    input  wire [NOBJ*REC_W-1:0] I_obj_bus,       // raw object records
    input  wire [3:0]            I_obj_num,

    // alarm region (pixel coordinates, inclusive box)
    input  wire [11:0]           I_roi_x,
    input  wire [11:0]           I_roi_y,
    input  wire [11:0]           I_roi_w,
    input  wire [11:0]           I_roi_h,

    output reg  [NTRK*TRK_W-1:0] O_trk_bus,       // extended, stable records
    output reg  [NOBJ*REC_W-1:0] O_obj56,         // overlay-compatible records
    output reg  [NTRK*DYN_W-1:0] O_trk_dyn,       // dynamic indicators
    output reg                   O_alarm,
    output reg  [3:0]            O_trk_num
);

    //----------------------------------------------------------------------
    // record layout - every offset is DERIVED from CLS_W, so the class
    // count can be changed without touching a single hard coded position.
    //----------------------------------------------------------------------
    localparam B_CONF = 0;                  // [ 6: 0]
    localparam B_MLP  = B_CONF + 7;
    localparam B_RULE = B_MLP  + CLS_W;
    localparam B_YMAX = B_RULE + CLS_W;
    localparam B_YMIN = B_YMAX + 10;
    localparam B_XMAX = B_YMIN + 10;
    localparam B_XMIN = B_XMAX + 11;
    localparam B_VAL  = B_XMIN + 11;

    // extended track record (TRK_W bit) : same fields + id + miss counter
    localparam T_MISS = 0;
    localparam T_ZERO = T_MISS + 4;
    localparam T_CONF = T_ZERO + 1;
    localparam T_MLP  = T_CONF + 7;
    localparam T_RULE = T_MLP  + CLS_W;
    localparam T_ID   = T_RULE + CLS_W;
    localparam T_YBR  = T_ID   + 3;
    localparam T_YTL  = T_YBR  + 10;
    localparam T_XBR  = T_YTL  + 10;
    localparam T_XTL  = T_XBR  + 11;
    localparam T_VAL  = T_XTL  + 11;

    // dynamic indicator record (DYN_W bit) - LSB first
    localparam D_LOST = 0;                  // 1 : currently occluded / extrapolated
    localparam D_AGE  = D_LOST + 1;         // 5 : frames since the track was born
    localparam D_VY   = D_AGE  + 5;         // 12 : signed, pixel / frame
    localparam D_VX   = D_VY   + 12;        // 12 : signed, pixel / frame
    localparam D_PY   = D_VX   + 12;        // 10 : predicted centre
    localparam D_PX   = D_PY   + 10;        // 11 : predicted centre
    localparam D_CY   = D_PX   + 11;        // 10 : current  centre
    localparam D_CX   = D_CY   + 10;        // 11 : current  centre

    //----------------------------------------------------------------------
    // field helpers on the incoming record
    //----------------------------------------------------------------------
    // NOTE: B_* are LSB indices, so the MSB of a W bit field is (B_* + W - 1)
    function [10:0]      f_xmin; input [REC_W-1:0] r; f_xmin = r[B_XMIN + 10 -: 11]; endfunction
    function [10:0]      f_xmax; input [REC_W-1:0] r; f_xmax = r[B_XMAX + 10 -: 11]; endfunction
    function [9:0]       f_ymin; input [REC_W-1:0] r; f_ymin = r[B_YMIN +  9 -: 10]; endfunction
    function [9:0]       f_ymax; input [REC_W-1:0] r; f_ymax = r[B_YMAX +  9 -: 10]; endfunction
    function [CLS_W-1:0] f_rule; input [REC_W-1:0] r; f_rule = r[B_RULE + CLS_W - 1 -: CLS_W]; endfunction
    function [CLS_W-1:0] f_mlp;  input [REC_W-1:0] r; f_mlp  = r[B_MLP  + CLS_W - 1 -: CLS_W]; endfunction
    function [6:0]       f_conf; input [REC_W-1:0] r; f_conf = r[B_CONF +  6 -: 7];  endfunction
    function             f_val;  input [REC_W-1:0] r; f_val  = r[B_VAL];            endfunction

    //----------------------------------------------------------------------
    // saturating helpers (0 .. 1279 / 0 .. 719), 13 bit signed input
    //----------------------------------------------------------------------
    function [10:0] sat11;
        input signed [12:0] v;
        begin
            if (v[12])                 sat11 = 11'd0;
            else if (v > 13'sd1279)    sat11 = 11'd1279;
            else                       sat11 = v[10:0];
        end
    endfunction

    function [9:0] sat10;
        input signed [12:0] v;
        begin
            if (v[12])                 sat10 = 10'd0;
            else if (v > 13'sd719)     sat10 = 10'd719;
            else                       sat10 = v[9:0];
        end
    endfunction

    // velocity limiter : +/- 1023 pixel per frame (3 digits + sign on screen)
    function signed [11:0] satv;
        input signed [11:0] v;
        begin
            if      (v >  12'sd1023) satv =  12'sd1023;
            else if (v < -12'sd1023) satv = -12'sd1023;
            else                     satv = v;
        end
    endfunction

    //----------------------------------------------------------------------
    // majority over a 3-deep history (tie -> keep first)
    //
    //   Bit exact identical to the old "2**CLS_W bucket counter" version,
    //   including the "all three differ -> keep the first argument" tie rule,
    //   but it needs only 3 comparators + 3 muxes instead of an 2**CLS_W
    //   entry counter.  The counter version was part of the obj_tracker
    //   critical path (HDMI_PIXEL_CLK, 13.33 ns).
    //----------------------------------------------------------------------
    function [CLS_W-1:0] majority3;
        input [CLS_W-1:0] a, b, c;
        begin
            if      (a == b) majority3 = a;
            else if (a == c) majority3 = a;
            else if (b == c) majority3 = b;
            else             majority3 = a;   // all differ -> first
        end
    endfunction

    //----------------------------------------------------------------------
    // track state
    //----------------------------------------------------------------------
    reg         r_val [0:NTRK-1];
    reg [10:0]  r_xtl [0:NTRK-1];   // track box (top-left / bottom-right)
    reg [10:0]  r_xbr [0:NTRK-1];
    reg [9:0]   r_ytl [0:NTRK-1];
    reg [9:0]   r_ybr [0:NTRK-1];
    reg [2:0]   r_id  [0:NTRK-1];
    reg [3:0]   r_miss[0:NTRK-1];
    reg [4:0]   r_age [0:NTRK-1];
    reg [6:0]   r_conf[0:NTRK-1];
    reg [CLS_W-1:0] r_hr [0:NTRK-1][0:2];   // rule history (3 deep)
    reg [CLS_W-1:0] r_hm [0:NTRK-1][0:2];   // mlp  history (3 deep)
    reg [2:0]   r_next_id;

    // v2 : motion state
    reg [10:0]  r_cx  [0:NTRK-1];   // centroid
    reg [9:0]   r_cy  [0:NTRK-1];
    reg signed [11:0] r_vx  [0:NTRK-1];   // velocity, pixel / frame
    reg signed [11:0] r_vy  [0:NTRK-1];
    reg [10:0]  r_px  [0:NTRK-1];   // predicted centre (this frame)
    reg [9:0]   r_py  [0:NTRK-1];

    integer t, k;
    reg [NOBJ*REC_W-1:0] objs;
    reg [10:0] cx [0:NOBJ-1];
    reg [9:0]  cy [0:NOBJ-1];
    reg        ova[0:NOBJ-1];
    reg [1:0]  match [0:NOBJ-1];     // 0 = none, else track+1
    reg        used  [0:NTRK-1];
    reg [11:0] dcx, dcy, dist, best_d;
    reg [1:0]  best_j;
    reg [3:0]  r_cnt;
    reg        matched_t;

    // ---- predicted centre of a track (centroid + velocity) ----
    reg [10:0] pcx [0:NTRK-1];
    reg [9:0]  pcy [0:NTRK-1];

    // ---- per track combinational helper values ----
    reg [10:0] hw [0:NTRK-1];
    reg [9:0]  hh [0:NTRK-1];

    // velocity estimator temporaries (15 bit so that 3*v + d cannot overflow)
    reg signed [14:0] tvx, tvy;

    //----------------------------------------------------------------------
    // improve2 : stage A pipeline registers (timing fix, see the always block)
    //   The frame update used to be ONE combinational block of 17 logic
    //   levels and was the only setup violation of the whole design.  These
    //   registers carry everything stage B needs one clock early.
    //----------------------------------------------------------------------
    reg [NOBJ*REC_W-1:0] r_objs;
    reg [10:0] r_cxs  [0:NOBJ-1];
    reg [9:0]  r_cys  [0:NOBJ-1];
    reg        r_ovas [0:NOBJ-1];
    reg [10:0] r_pcx  [0:NTRK-1];
    reg [9:0]  r_pcy  [0:NTRK-1];
    reg [10:0] r_hwr  [0:NTRK-1];
    reg [9:0]  r_hhr  [0:NTRK-1];
    reg [11:0] r_dist [0:NTRK-1][0:NOBJ-1];
    reg signed [14:0] r_pvx [0:NTRK-1];
    reg signed [14:0] r_pvy [0:NTRK-1];
    reg        r_fs1;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            for (t = 0; t < NTRK; t = t + 1) begin
                r_val[t]  <= 1'b0;
                r_miss[t] <= 4'd0;
                r_age[t]  <= 5'd0;
                r_id[t]   <= 3'd0;
                r_cx[t]   <= 11'd0;
                r_cy[t]   <= 10'd0;
                r_px[t]   <= 11'd0;
                r_py[t]   <= 10'd0;
                r_vx[t]   <= 12'sd0;
                r_vy[t]   <= 12'sd0;
            end
            r_next_id <= 3'd0;
            O_alarm   <= 1'b0;
            O_trk_num <= 4'd0;
            O_trk_dyn <= {NTRK*DYN_W{1'b0}};
            r_fs1     <= 1'b0;
        //======================================================================
        // improve2 (timing fix) : STAGE A  -  snapshot + prediction +
        //                         the per pair distances + the velocity term
        //
        //   The whole frame update used to be ONE combinational block of 17
        //   logic levels :
        //       r_vy -> sat -> |dx|+|dy| -> compare chain -> match -> multiplier
        //   It was the only setup violation of the design (WNS -12 ps on
        //   HDMI_PIXEL_CLK 75 MHz).  A violation that small makes the router
        //   rip up and retry again and again (place WNS -0.012 ns -> route
        //   WNS -4.9 ns, 7 hours) instead of converging in 25 minutes like
        //   improve1 (+217 ps).  Splitting the block into two stages of about
        //   6 and 7 levels costs one clock of a block that runs once per
        //   frame - 13 ns inside 16.7 ms - and does not change a single
        //   result.
        //======================================================================
        end else if (I_frame_start) begin
            objs = I_obj_bus;
            // snapshot incoming objects + centroid
            for (k = 0; k < NOBJ; k = k + 1) begin
                ova[k] = f_val(objs[k*REC_W +: REC_W]);
                cx[k]  = (f_xmin(objs[k*REC_W +: REC_W]) + f_xmax(objs[k*REC_W +: REC_W])) >> 1;
                cy[k]  = (f_ymin(objs[k*REC_W +: REC_W]) + f_ymax(objs[k*REC_W +: REC_W])) >> 1;
                r_ovas[k] <= ova[k];
                r_cxs[k]  <= cx[k];
                r_cys[k]  <= cy[k];
            end
            r_objs <= I_obj_bus;

            for (t = 0; t < NTRK; t = t + 1) begin
                // ---- 1st order prediction : p = c + v ----
                pcx[t] = sat11($signed({1'b0, r_cx[t]}) + r_vx[t]);
                pcy[t] = sat10($signed({2'b0, r_cy[t]}) + r_vy[t]);
                r_px[t]  <= pcx[t];
                r_py[t]  <= pcy[t];
                r_pcx[t] <= pcx[t];
                r_pcy[t] <= pcy[t];
                // half box size (kept while the track is extrapolated)
                hw[t]  = (r_xbr[t] - r_xtl[t]) >> 1;
                hh[t]  = (r_ybr[t] - r_ytl[t]) >> 1;
                r_hwr[t] <= hw[t];
                r_hhr[t] <= hh[t];
                // v' = (3*v + d) >> 2 : the part that does NOT depend on the
                // association result, computed one clock early
                r_pvx[t] <= (r_vx[t] * 15'sd3) - $signed({4'd0, r_cx[t]});
                r_pvy[t] <= (r_vy[t] * 15'sd3) - $signed({5'd0, r_cy[t]});
            end

            // every (track, object) distance, fully parallel : stage B only
            // has to compare them
            for (t = 0; t < NTRK; t = t + 1)
                for (k = 0; k < NOBJ; k = k + 1) begin
                    dcx = (cx[k] > pcx[t]) ? (cx[k] - pcx[t]) : (pcx[t] - cx[k]);
                    dcy = (cy[k] > pcy[t]) ? (cy[k] - pcy[t]) : (pcy[t] - cy[k]);
                    r_dist[t][k] <= dcx + dcy;
                end

            r_fs1 <= 1'b1;

        //======================================================================
        // improve2 : STAGE B  -  association + update + ageing + outputs
        //======================================================================
        end else if (r_fs1) begin
            r_fs1 <= 1'b0;

            objs = r_objs;
            for (k = 0; k < NOBJ; k = k + 1) begin
                ova[k]   = r_ovas[k];
                cx[k]    = r_cxs[k];
                cy[k]    = r_cys[k];
                match[k] = 2'd0;
            end
            for (t = 0; t < NTRK; t = t + 1) begin
                used[t] = 1'b0;
                pcx[t]  = r_pcx[t];
                pcy[t]  = r_pcy[t];
                hw[t]   = r_hwr[t];
                hh[t]   = r_hhr[t];
            end

            // ---- nearest-neighbour association on the PREDICTED centre ----
            //      improve2 : the distances were computed in stage A, only
            //      the compare chain is left here (4 levels instead of 12)
            for (k = 0; k < NOBJ; k = k + 1) begin
                if (ova[k]) begin
                    best_d = 12'hFFF;
                    best_j = 2'd0;
                    for (t = 0; t < NTRK; t = t + 1) begin
                        if (r_val[t] && !used[t]) begin
                            dist = r_dist[t][k];
                            if (dist < best_d) begin
                                best_d = dist;
                                best_j = t[1:0];
                            end
                        end
                    end
                    if (best_d < GATE) begin
                        match[k]    = best_j + 1'b1;
                        used[best_j] = 1'b1;
                    end
                end
            end

            // ---- update matched tracks (push history, estimate velocity) ----
            for (t = 0; t < NTRK; t = t + 1) begin
                for (k = 0; k < NOBJ; k = k + 1) begin
                    if (match[k] == (t + 1'b1)) begin
                        // v' = (3*v + d) >> 2      (shift + add only, 0 DSP)
                        // improve2 : (3*v - c) comes from stage A, so the
                        // multiplier no longer sits behind the association
                        tvx = r_pvx[t] + $signed({4'd0, cx[k]});
                        tvy = r_pvy[t] + $signed({5'd0, cy[k]});
                        r_vx[t] <= satv(tvx[13:2]);
                        r_vy[t] <= satv(tvy[13:2]);
                        r_xtl[t] <= f_xmin(objs[k*REC_W +: REC_W]);
                        r_xbr[t] <= f_xmax(objs[k*REC_W +: REC_W]);
                        r_ytl[t] <= f_ymin(objs[k*REC_W +: REC_W]);
                        r_ybr[t] <= f_ymax(objs[k*REC_W +: REC_W]);
                        r_cx[t]  <= cx[k];
                        r_cy[t]  <= cy[k];
                        r_conf[t]<= f_conf(objs[k*REC_W +: REC_W]);
                        r_miss[t]<= 4'd0;
                        if (r_age[t] != 5'd31) r_age[t] <= r_age[t] + 5'd1;
                        r_hr[t][0] <= f_rule(objs[k*REC_W +: REC_W]);
                        r_hr[t][1] <= r_hr[t][0];
                        r_hr[t][2] <= r_hr[t][1];
                        r_hm[t][0] <= f_mlp (objs[k*REC_W +: REC_W]);
                        r_hm[t][1] <= r_hm[t][0];
                        r_hm[t][2] <= r_hm[t][1];
                    end
                end
            end

            // ---- age / drop unmatched tracks (with velocity extrapolation) ----
            for (t = 0; t < NTRK; t = t + 1) begin
                matched_t = 1'b0;
                for (k = 0; k < NOBJ; k = k + 1)
                    if (match[k] == (t + 1'b1)) matched_t = 1'b1;
                if (r_val[t] && !matched_t) begin
                    if (r_miss[t] < MAX_MISS) begin
                        r_miss[t] <= r_miss[t] + 1'b1;
                        // ---- OCCLUSION : extrapolate, do NOT drop ----
                        // the centroid keeps the box size, so the dashed box
                        // that osd_obj_overlay draws stays as large as the
                        // object really is
                        pcx[t] = sat11($signed({1'b0, r_cx[t]}) + r_vx[t]);
                        pcy[t] = sat10($signed({2'b0, r_cy[t]}) + r_vy[t]);
                        r_cx[t] <= pcx[t];
                        r_cy[t] <= pcy[t];
                        r_xtl[t] <= sat11($signed({1'b0, pcx[t]}) - $signed({1'b0, hw[t]}));
                        r_xbr[t] <= sat11($signed({1'b0, pcx[t]}) + $signed({1'b0, hw[t]}));
                        r_ytl[t] <= sat10($signed({1'b0, pcy[t]}) - $signed({1'b0, hh[t]}));
                        r_ybr[t] <= sat10($signed({1'b0, pcy[t]}) + $signed({1'b0, hh[t]}));
                    end
                    else begin
                        r_val[t] <= 1'b0;
                    end
                end
            end

            // ---- unmatched objects : create a new track in a free slot ----
            for (k = 0; k < NOBJ; k = k + 1) begin
                if (ova[k] && (match[k] == 2'd0)) begin
                    for (t = 0; t < NTRK; t = t + 1) begin
                        // create at most ONE track per object : guard on used[] and match[]
                        if (!r_val[t] && !used[t] && (match[k] == 2'd0)) begin
                            r_val[t]  <= 1'b1;
                            r_id[t]   <= r_next_id;
                            r_miss[t] <= 4'd0;
                            r_age[t]  <= 5'd1;
                            r_xtl[t]  <= f_xmin(objs[k*REC_W +: REC_W]);
                            r_xbr[t]  <= f_xmax(objs[k*REC_W +: REC_W]);
                            r_ytl[t]  <= f_ymin(objs[k*REC_W +: REC_W]);
                            r_ybr[t]  <= f_ymax(objs[k*REC_W +: REC_W]);
                            r_cx[t]   <= cx[k];
                            r_cy[t]   <= cy[k];
                            r_vx[t]   <= 12'sd0;
                            r_vy[t]   <= 12'sd0;
                            r_conf[t] <= f_conf(objs[k*REC_W +: REC_W]);
                            r_hr[t][0]<= f_rule(objs[k*REC_W +: REC_W]);
                            r_hr[t][1]<= f_rule(objs[k*REC_W +: REC_W]);
                            r_hr[t][2]<= f_rule(objs[k*REC_W +: REC_W]);
                            r_hm[t][0]<= f_mlp (objs[k*REC_W +: REC_W]);
                            r_hm[t][1]<= f_mlp (objs[k*REC_W +: REC_W]);
                            r_hm[t][2]<= f_mlp (objs[k*REC_W +: REC_W]);
                            match[k]  =  t + 1'b1;   // binding : prevents a 2nd track for the same object
                            used[t]   =  1'b1;        // binding : prevents another object reusing this slot this frame
                            r_next_id <= r_next_id + 1'b1;
                        end
                    end
                end
            end

            // ---- build outputs + alarm ----
            r_cnt  = 4'd0;
            O_alarm <= 1'b0;
            for (t = 0; t < NTRK; t = t + 1) begin
                if (r_val[t]) begin
                    r_cnt = r_cnt + 1'b1;
                    O_trk_bus[t*TRK_W +: TRK_W] <=
                        { r_val[t], r_xtl[t], r_xbr[t], r_ytl[t],
                          r_ybr[t], r_id[t],
                          majority3(r_hr[t][0], r_hr[t][1], r_hr[t][2]),
                          majority3(r_hm[t][0], r_hm[t][1], r_hm[t][2]),
                          r_conf[t], 1'b0, r_miss[t] };
                    O_obj56[t*REC_W +: REC_W] <=
                        { r_val[t], r_xtl[t], r_xbr[t], r_ytl[t],
                          r_ybr[t],
                          majority3(r_hr[t][0], r_hr[t][1], r_hr[t][2]),
                          majority3(r_hm[t][0], r_hm[t][1], r_hm[t][2]),
                          r_conf[t] };
                    // ---- dynamic indicators of this track ----
                    O_trk_dyn[t*DYN_W +: DYN_W] <=
                        { r_cx[t], r_cy[t], r_px[t], r_py[t],
                          r_vx[t], r_vy[t], r_age[t],
                          (r_miss[t] != 4'd0) };
                    // centroid inside ROI ?  (v2 : the centroid decides, so a
                    // large object triggers the alarm as well)
                    if ((r_cx[t] >= I_roi_x[10:0]) &&
                        (r_cx[t] <= (I_roi_x[10:0] + I_roi_w[10:0])) &&
                        (r_cy[t] >= I_roi_y[9:0]) &&
                        (r_cy[t] <= (I_roi_y[9:0] + I_roi_h[9:0])))
                        O_alarm <= 1'b1;
                end else begin
                    O_trk_bus[t*TRK_W +: TRK_W] <= {TRK_W{1'b0}};
                    O_obj56[t*REC_W +: REC_W] <= {REC_W{1'b0}};
                    O_trk_dyn[t*DYN_W +: DYN_W] <= {DYN_W{1'b0}};
                end
            end
            O_trk_num <= r_cnt;
        end
    end

endmodule
