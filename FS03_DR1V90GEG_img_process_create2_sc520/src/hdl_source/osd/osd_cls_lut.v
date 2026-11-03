//-----------------------------------------------------------------------------
// Module     : osd_cls_lut
// Project    : HX1P35Aimg_process_create1
// Description: Class name / class colour lookup table for the 64 classes of the
//              improve4 classifier (improve1 .. improve3 only had 32).
//
//   improve4 doubles the number of recognised object categories from 32 to 64.
//   Writing the name table as a 64 branch `case` statement for every single
//   character would cost several hundred LUTs - and LUTs are the bottleneck of
//   the PH1P35 (the block RAMs are almost idle).  The table is therefore held in
//   two small memories (64 x 32 bit for the four ASCII characters and
//   64 x 24 bit for the colour) which the synthesiser implements in
//   distributed RAM : ~3.5 Kbit of memory instead of a wide mux tree.
//
//   The very same coding style (a `reg` array that is filled by an `initial`
//   block) is used by the Anlogic IP generator itself
//   (ip_source/w128_d512_fifo/soft_fifo_al_*.v -> ram_infer_*), so it is
//   synthesisable by TD and simulatable with Icarus Verilog.
//
//   64 object categories - 8 shape families x 8 colour families.  Every entry
//   carries the name of the everyday object that matches the combination best
//   (the people category "MAN" is one of them, as required by the topic).
//
//        class index = shape family * 8 + colour family
//        colour   0 red   1 orange  2 yellow  3 green  4 blue  5 purple  6 pink  7 brown
//
//        shape\colour   RED     ORANGE  YELLOW  GREEN   BLUE    PURPLE  PINK     BROWN
//        round          BALL    APPL    LEMON   LIME    PLUM    GRAPE   PEACH    MAN
//        block          BRICK   BOOK    BOX     CART    CHIP    SOAP    TILE     SAFE
//        flat           CARD    NOTE    RULER   LEAF    KEYB    BAG     TAPE     BELT
//        triangle       CONE    PLANT   SIGN    WEDGE   HAT     FLAG    SLICE    ROOF
//        tall cylinder  BOTTLE  TUBE    CUP     POT     PIPE    VASE    CANDLE   MUG
//        ring / hollow  RING    NUT     WASHER  ROPE    DISC    COIL    HOSE     BUN
//        irregular      TOY     TOOL    CLIP    DOG     CUSHION SOCKS   GLOVE    BREAD
//        spiky          STAR    TREE    BADGE   BIRD    CROWN   CACTUS  CORAL    CAT
//
//   Only four letters are drawn (8 x 8 OSD font), therefore every name is the
//   natural four letter abbreviation of the object (BOTTLE -> BOTL, CUSHION ->
//   CUSH, WASHER -> WASR ...).  The colour family is the same one the detector
//   measures (mean Cb / Cr inside the blob), so the box colour of a detected
//   object always agrees with its printed name.
//-----------------------------------------------------------------------------
module osd_cls_lut (
    input  wire [5:0]  I_cls,       // improve4 : 6 bit (64 classes)
    output wire [31:0] O_name,      // 4 ASCII characters, [7:0] = left most
    output wire [23:0] O_color
);

    // ASCII helpers (the OSD font holds A - Z and 0 - 9 only)
    localparam CH_A = 8'h41, CH_B = 8'h42, CH_C = 8'h43, CH_D = 8'h44;
    localparam CH_E = 8'h45, CH_F = 8'h46, CH_G = 8'h47, CH_H = 8'h48;
    localparam CH_I = 8'h49, CH_K = 8'h4B, CH_L = 8'h4C, CH_M = 8'h4D;
    localparam CH_N = 8'h4E, CH_O = 8'h4F, CH_P = 8'h50, CH_R = 8'h52;
    localparam CH_S = 8'h53, CH_T = 8'h54, CH_U = 8'h55, CH_V = 8'h56;
    localparam CH_W = 8'h57, CH_X = 8'h58, CH_Y = 8'h59, CH_SP = 8'h20;

    reg [31:0] r_name [0:63];
    reg [23:0] r_col  [0:63];

    initial begin
        // ---- 0 : round --------------------------------------------------
        r_name[ 0] = {CH_B, CH_A, CH_L, CH_L};   // BALL
        r_name[ 1] = {CH_A, CH_P, CH_P, CH_L};   // APPL  (apple, orange)
        r_name[ 2] = {CH_L, CH_E, CH_M, CH_N};   // LEMN  (lemon, yellow)
        r_name[ 3] = {CH_L, CH_I, CH_M, CH_E};   // LIME  (green)
        r_name[ 4] = {CH_P, CH_L, CH_U, CH_M};   // PLUM  (blue violet)
        r_name[ 5] = {CH_G, CH_R, CH_A, CH_P};   // GRAP  (grape, purple)
        r_name[ 6] = {CH_P, CH_E, CH_A, CH_H};   // PEAC  (peach, pink)
        r_name[ 7] = {CH_SP, CH_M, CH_A, CH_N};  //  MAN  (person, skin tone)
        // ---- 1 : block --------------------------------------------------
        r_name[ 8] = {CH_B, CH_R, CH_I, CH_K};   // BRCK  (brick, red)
        r_name[ 9] = {CH_B, CH_O, CH_O, CH_K};   // BOOK  (orange)
        r_name[10] = {CH_SP, CH_B, CH_O, CH_X};  //  BOX  (yellow box)
        r_name[11] = {CH_C, CH_A, CH_R, CH_T};   // CART  (green crate)
        r_name[12] = {CH_C, CH_H, CH_I, CH_P};   // CHIP  (blue chip)
        r_name[13] = {CH_S, CH_O, CH_A, CH_P};   // SOAP  (purple)
        r_name[14] = {CH_T, CH_I, CH_L, CH_E};   // TILE  (pink)
        r_name[15] = {CH_S, CH_A, CH_F, CH_E};   // SAFE  (brown)
        // ---- 2 : flat ---------------------------------------------------
        r_name[16] = {CH_C, CH_A, CH_R, CH_D};   // CARD  (red card)
        r_name[17] = {CH_N, CH_O, CH_T, CH_E};   // NOTE  (orange note)
        r_name[18] = {CH_R, CH_U, CH_L, CH_R};   // RULR  (ruler, yellow)
        r_name[19] = {CH_L, CH_E, CH_A, CH_F};   // LEAF  (green)
        r_name[20] = {CH_K, CH_E, CH_Y, CH_B};   // KEYB  (keyboard, blue)
        r_name[21] = {CH_B, CH_A, CH_G, CH_SP};  // BAG   (purple)
        r_name[22] = {CH_T, CH_A, CH_P, CH_E};   // TAPE  (pink)
        r_name[23] = {CH_B, CH_E, CH_L, CH_T};   // BELT  (brown leather)
        // ---- 3 : triangle -----------------------------------------------
        r_name[24] = {CH_C, CH_O, CH_N, CH_E};   // CONE  (red cone)
        r_name[25] = {CH_P, CH_L, CH_N, CH_T};   // PLNT  (plant, orange pot)
        r_name[26] = {CH_S, CH_I, CH_G, CH_N};   // SIGN  (yellow sign)
        r_name[27] = {CH_W, CH_E, CH_D, CH_G};   // WEDG  (wedge, green)
        r_name[28] = {CH_SP, CH_H, CH_A, CH_T};  //  HAT  (blue)
        r_name[29] = {CH_F, CH_L, CH_A, CH_G};   // FLAG  (purple)
        r_name[30] = {CH_S, CH_L, CH_I, CH_C};   // SLIC  (slice, pink)
        r_name[31] = {CH_R, CH_O, CH_O, CH_F};   // ROOF  (brown roof)
        // ---- 4 : tall cylinder ------------------------------------------
        r_name[32] = {CH_B, CH_O, CH_T, CH_L};   // BOTL  (bottle, red)
        r_name[33] = {CH_T, CH_U, CH_B, CH_E};   // TUBE  (orange)
        r_name[34] = {CH_SP, CH_C, CH_U, CH_P};  //  CUP  (yellow)
        r_name[35] = {CH_P, CH_O, CH_T, CH_SP};  // POT   (green)
        r_name[36] = {CH_P, CH_I, CH_P, CH_E};   // PIPE  (blue)
        r_name[37] = {CH_V, CH_A, CH_S, CH_E};   // VASE  (purple)
        r_name[38] = {CH_C, CH_A, CH_N, CH_D};   // CAND  (candle, pink)
        r_name[39] = {CH_M, CH_U, CH_G, CH_SP};  // MUG   (brown)
        // ---- 5 : ring / hollow ------------------------------------------
        r_name[40] = {CH_R, CH_I, CH_N, CH_G};   // RING  (red)
        r_name[41] = {CH_N, CH_U, CH_T, CH_SP};  // NUT   (orange)
        r_name[42] = {CH_W, CH_A, CH_S, CH_R};   // WASR  (washer, yellow)
        r_name[43] = {CH_R, CH_O, CH_P, CH_E};   // ROPE  (green)
        r_name[44] = {CH_D, CH_I, CH_S, CH_C};   // DISC  (blue disc)
        r_name[45] = {CH_C, CH_O, CH_I, CH_L};   // COIL  (purple)
        r_name[46] = {CH_H, CH_O, CH_S, CH_E};   // HOSE  (pink)
        r_name[47] = {CH_B, CH_U, CH_N, CH_SP};  // BUN   (brown crust)
        // ---- 6 : irregular ----------------------------------------------
        r_name[48] = {CH_SP, CH_T, CH_O, CH_Y};  //  TOY  (red toy)
        r_name[49] = {CH_T, CH_O, CH_O, CH_L};   // TOOL  (orange)
        r_name[50] = {CH_C, CH_L, CH_I, CH_P};   // CLIP  (yellow)
        r_name[51] = {CH_C, CH_U, CH_S, CH_H};   // CUSH  (cushion, green)
        r_name[52] = {CH_S, CH_O, CH_C, CH_K};   // SOCK  (blue socks)
        r_name[53] = {CH_G, CH_L, CH_O, CH_V};   // GLOV  (glove, purple)
        r_name[54] = {CH_B, CH_R, CH_E, CH_D};   // BRED  (bread, pink dough)
        r_name[55] = {CH_SP, CH_D, CH_O, CH_G};  //  DOG  (brown pet)
        // ---- 7 : spiky --------------------------------------------------
        r_name[56] = {CH_S, CH_T, CH_A, CH_R};   // STAR  (red)
        r_name[57] = {CH_T, CH_R, CH_E, CH_E};   // TREE  (orange foliage)
        r_name[58] = {CH_B, CH_A, CH_D, CH_G};   // BADG  (badge, yellow)
        r_name[59] = {CH_C, CH_A, CH_C, CH_T};   // CACT  (cactus, green)
        r_name[60] = {CH_C, CH_R, CH_O, CH_W};   // CROW  (crown, blue)
        r_name[61] = {CH_C, CH_O, CH_R, CH_L};   // CORL  (coral, purple)
        r_name[62] = {CH_B, CH_I, CH_R, CH_D};   // BIRD  (pink flamingo)
        r_name[63] = {CH_C, CH_A, CH_T, CH_SP};  // CAT   (brown cat)

        // ---- colours : one clearly distinguishable colour per class -------
        r_col[ 0] = 24'hff2020;   // BALL   red
        r_col[ 1] = 24'hff7010;   // APPL   orange
        r_col[ 2] = 24'hffe020;   // LEMN   yellow
        r_col[ 3] = 24'h40ff40;   // LIME   green
        r_col[ 4] = 24'h7030c0;   // PLUM   violet
        r_col[ 5] = 24'h9040d0;   // GRAP   purple
        r_col[ 6] = 24'hff90a0;   // PEAC   pink
        r_col[ 7] = 24'hff00ff;   // MAN    magenta (person, like extend3)
        r_col[ 8] = 24'hc03010;   // BRCK   brick red
        r_col[ 9] = 24'hff8020;   // BOOK   orange
        r_col[10] = 24'hffff00;   // BOX    yellow
        r_col[11] = 24'h00a040;   // CART   crate green
        r_col[12] = 24'h0080ff;   // CHIP   blue
        r_col[13] = 24'hb060c0;   // SOAP   purple
        r_col[14] = 24'hffa0c0;   // TILE   pink
        r_col[15] = 24'h906020;   // SAFE   brown
        r_col[16] = 24'hff4000;   // CARD   red
        r_col[17] = 24'hffa040;   // NOTE   orange
        r_col[18] = 24'hffff80;   // RULR   pale yellow
        r_col[19] = 24'h20e020;   // LEAF   leaf green
        r_col[20] = 24'h40c0ff;   // KEYB   cyan blue
        r_col[21] = 24'ha040ff;   // BAG    violet
        r_col[22] = 24'hff70b0;   // TAPE   pink
        r_col[23] = 24'ha06030;   // BELT   leather brown
        r_col[24] = 24'he02020;   // CONE   traffic red
        r_col[25] = 24'hff9000;   // PLNT   orange
        r_col[26] = 24'hffd040;   // SIGN   amber
        r_col[27] = 24'h00ff80;   // WEDG   spring green
        r_col[28] = 24'h2060ff;   // HAT    blue
        r_col[29] = 24'h8000ff;   // FLAG   violet
        r_col[30] = 24'hff60c0;   // SLIC   pink
        r_col[31] = 24'hc04000;   // ROOF   roof brown
        r_col[32] = 24'hff0080;   // BOTL   pink red
        r_col[33] = 24'hff6000;   // TUBE   orange
        r_col[34] = 24'he0e020;   // CUP    yellow
        r_col[35] = 24'h00ff00;   // POT    green
        r_col[36] = 24'h0040ff;   // PIPE   blue
        r_col[37] = 24'hb040ff;   // VASE   purple
        r_col[38] = 24'hffc0d0;   // CAND   candle pink
        r_col[39] = 24'hb07040;   // MUG    brown
        r_col[40] = 24'hff5050;   // RING   light red
        r_col[41] = 24'hffc080;   // NUT    sand
        r_col[42] = 24'hc0c020;   // WASR   brass yellow
        r_col[43] = 24'h80ff80;   // ROPE   light green
        r_col[44] = 24'h8080ff;   // DISC   periwinkle
        r_col[45] = 24'hc060ff;   // COIL   light purple
        r_col[46] = 24'hffa8c0;   // HOSE   pale pink
        r_col[47] = 24'hd0a060;   // BUN    crust brown
        r_col[48] = 24'hff1040;   // TOY    toy red
        r_col[49] = 24'hffa020;   // TOOL   orange
        r_col[50] = 24'h00ffff;   // CLIP   cyan
        r_col[51] = 24'h40c0a0;   // CUSH   jade
        r_col[52] = 24'h60a0ff;   // SOCK   denim blue
        r_col[53] = 24'hd070ff;   // GLOV   light purple
        r_col[54] = 24'hffe0c0;   // BRED   dough pink
        r_col[55] = 24'hb06030;   // DOG    dog brown
        r_col[56] = 24'hff8c00;   // STAR   orange
        r_col[57] = 24'h2f8f2f;   // TREE   forest green
        r_col[58] = 24'hffe800;   // BADG   gold
        r_col[59] = 24'h30a030;   // CACT   cactus green
        r_col[60] = 24'h40e0ff;   // CROW   sky blue
        r_col[61] = 24'hff70d0;   // CORL   coral pink
        r_col[62] = 24'hff5060;   // BIRD   flamingo pink
        r_col[63] = 24'hffa850;   // CAT    light brown
    end

    assign O_name  = r_name[I_cls];
    assign O_color = r_col [I_cls];

endmodule
