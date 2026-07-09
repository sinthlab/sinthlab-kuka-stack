// =====================================================================
//  Apple-pluck end-effector for the KUKA LBR iiwa7   (parametric DRAFT v0.6)
//  sinthlab-kuka-stack / end_effector_design
//
//  ELECTRONICS HUB that bolts to the iiwa7 *media flange (electric)*, routes its
//  power/data wiring, drives a NeoPixel cue, and carries the control electronics.
//  v0.6: the EXTENSION is gone — the apple core base bolts straight to the cover —
//  and the apple rides on a HEIGHT-ADJUSTABLE shaft (detent pin). Stack:
//
//        [ iiwa7 media flange (electric) ]   8x screws, 24V + data
//                 |  cable bundle THROUGH the base centre
//        (1) BASE HUB    8-hole flange mount + central cable bore; FOUR board pockets
//              ├─ Metro M4 AirLift (4000)        ┐ tucked toward the centre,
//              ├─ PSM-B05 24V->5V converter      │ long side tangential
//              ├─ Pixel Shifter (6066)           │ (small breakouts in the ±90° gaps)
//              ├─ DRV2605L ERM/LRA driver (2305) ┘
//              └─ wire channels linking bore <-> converter <-> board <-> centre riser
//        (2) COVER       closes the pockets · seats the NeoPixel ring in a groove on its TOP
//                        (opaque, same material as the base) · 4x M3 down to the base
//                 ▼ BOLTED FLANGE: apple CORE BASE screws down into the cover (3x M3 + spigot)
//        (3) APPLE = (3a) CORE BASE  flange + sleeve boss with a HEIGHT-SET pin hole; wiring bore
//                  + (3b) PETG SHAFT rod with detent holes (pick the height); slides into the boss and
//                                    runs up into the ball, ending in a flange the TPU anchors on
//                  + (3c) TPU BALL   LOWER cup FUSED to the shaft (dual-material print); solid cap embeds
//                                    the flange; OPEN top loads the ERM + sensor (all bigger than the bore)
//                  + (3d) TPU CAP    press-fit dome that closes the ball; wires / FSR tail / tube exit the bore
//        ( + ) CLEAR CASING BOX   5-sided clear box (OPEN on the flange side) shrouding the whole
//                                  electronics stack; top covers the ring, clamps 3x M3 into the cover
//
//  !!!  VERIFY every flange/component dimension vs the real datasheets before printing.
//  Export one part at a time, e.g.:
//     openscad -D 'part="base"'  -o base.stl  apple_pluck_end_effector.scad
// =====================================================================

part = "assembly"; // [assembly, base, cover, apple_core_base, adj_shaft, apple_ball, apple_cap, apple, apple_section, section, electronics_mock, casing]

/* [Robot media flange — MEASURED off the flange: Ø62 bolt circle, 8x M6, Ø34 central electric bore] */
// Mating pattern: 8 x M6 (Ø6) fixing holes on a Ø62 bolt circle ("ee circle"), a Ø34 central electric
// opening ("inside electric"), and the mounting ring between them is 14 mm wide ((62-34)/2). The base
// bottom mates on that Ø34..Ø62 ring.
flange_bolt_n      = 8;     // 8 x M6 fixing holes on the bolt circle
flange_pcd         = 62;    // bolt pitch-circle diameter [mm] ("ee circle")
flange_bolt_clear  = 6.6;   // M6 clearance hole, normal fit [mm]
flange_cbore_d     = 11.0;  // head-access well Ø for an M6 cap/cheese head (head Ø10) [mm]
flange_mount_t     = 8.0;   // mounting-wall thickness under the M6 head [mm] — the bolt spans THIS (+ the
                            // washer) then threads into the flange, so a short M6x16 reaches (~6.4 mm bite).
                            // Head recesses into and drives from the TOP, down the Ø11 access well.
flange_washer_d    = 13.0;  // washer-seat Ø under the head [mm] — clears a steel M6 flat washer (OD ~12)
flange_washer_t    = 2.0;   // washer-seat depth [mm] (M6 washer ~1.6 thick); spreads bolt torque on the wall
flange_first_angle = 22.5;  // first-hole angle [deg] — VERIFY clocking vs the flange
flange_center_d    = 34.0;  // central electric opening Ø [mm] ("inside electric"; clears the connector)
flange_center_h    = 6.0;   // opening/recess depth into the base [mm] — VERIFY the connector protrusion
// The Ø34 opening is a 6 mm recess at the flange face (kept below the board pockets, so the tucked
// boards are unaffected); the Ø16 cable bore continues up the centre for the apple wiring.

/* [Central cabling] */
cable_bore_d   = 16;   // central pass-through for the media-flange bundle [mm]

/* [Standard fastener — ONE M3 spec used for EVERYTHING except the M6 robot flange] */
// The whole tool uses just two thread sizes: M6 at the robot flange (fixed by the flange), and M3
// everywhere else (base<->cover, cover<->apple-core, board mounts, the height pin and the ball lock).
// Heads are CAP or CHEESE (same Ø5.5/Ø10 head, so the counterbores fit either): the M3 assortment kit
// is cap-head (2.5 mm hex key); cheese-head (screwdriver) drops into the same holes. Change M3 here in
// one place; every joint below derives from it.
m3_clear   = 3.4;  // M3 clearance hole [mm]
m3_insert  = 4.6;  // M3 heat-set INSERT bore [mm] (Bambu M3x5x4: M3, 5.0 mm OD, 4 mm long -> ~4.6 hole; EVERY screw-into-plastic joint)
m3_cbore   = 6.5;  // M3 cap/cheese-head counterbore Ø [mm] (clears the Ø5.5 head)
m3_cbore_h = 3.0;  // M3 counterbore depth [mm] (>= head height, so the head sits recessed)

/* [Base electronics hub] */
base_d     = 170;   // base outer Ø [mm]  (three big boards at 0/90/180, two breakouts in the 270 quadrant)
base_h     = 30;    // base height = compartment depth [mm]
base_floor = 5;     // floor thickness above the flange face [mm]
base_wall  = 3;     // structural wall / bridge thickness [mm]

/* [NeoPixel ring — Adafruit 2874, 60x5050 RGBW (buy 4x QUARTER-rings) — groove on the COVER TOP  https://www.adafruit.com/product/2874] */
// Product 2874 is ONE QUARTER of a 60-LED ring (15 LEDs, ~4500K natural-white RGBW). Buy FOUR and
// butt them into the full circle. Note this is a big ring (Ø157 = 6.2") that nearly fills the base.
ring_od        = 157.0; // assembled ring outer Ø [mm]  (Adafruit 2874 full ring = 157 mm / 6.2")
ring_id        = 145.0; // assembled ring inner Ø [mm]  (Adafruit 2874 full ring = 145 mm / 5.7")
ring_pcb_t     = 3.25;  // ring overall thickness [mm]   (PCB + LEDs, 2874 = 3.25 mm / 0.13")
ring_clear     = 0.6;   // radial clearance per side [mm]
ring_groove_h  = 4.0;   // top-facing groove depth to seat the ring [mm] (LEDs face up; clear box shields)
ring_wire_d    = 6.0;   // lead pass-through under the groove, one per quarter junction [mm]

/* [Cover plate] */
cover_plate_t  = 6.0;   // cover thickness [mm] — opaque (same material as base); top ring groove + 4x M3

/* [Control board — Adafruit Metro M4 Express AirLift Lite (4000) https://www.adafruit.com/product/4000] */
board_l        = 72.0;  // PCB length [mm] (Arduino Metro footprint; board overall 72 x 54 x 15)
board_w        = 54.0;  // PCB width  [mm]
board_clear_h  = 16;    // tallest-component clearance above the PCB [mm] (overall ht ~15)
board_standoff = 6;     // standoff post height under the board [mm] (tall enough to seat a 5 mm M3 insert)
board_screw_d  = m3_insert;  // M3 heat-set insert into the printed standoffs -- VERIFY: real Metro/Arduino holes are NOT symmetric; set standoff XY to the footprint
board_angle    = 0;     // angular position of the board compartment [deg]
board_radius   = 40;    // compartment centre radius [mm] (tucked toward centre; long side tangential)

/* [DC-DC converter — PSM-B05-1224-05, 12/24V->5V 5A 25W, IP68 potted block] */
conv_l         = 63;    // [mm]
conv_w         = 53;    // [mm]
conv_clear_h   = 20;    // [mm]
conv_angle     = 180;   // angular position of the converter compartment [deg]
conv_radius    = 40;    // compartment centre radius [mm] (tucked toward centre; long side tangential)

/* [Optocoupler board — buy one that fits this pocket: footprint <= opto_l x opto_w] */
opto_l         = 70.0;  // [mm] tangential (side-to-side) — MAX board length that fits the 12 o'clock slot
opto_w         = 34.0;  // [mm] radial (in-out) — MAX board width (limited by the Metro/converter reach + base edge)
opto_clear     = 16;    // tallest-component clearance above the PCB [mm]
opto_angle     = 90;    // angular position [deg] (12 o'clock, between the Metro at 0 and converter at 180)
opto_radius    = 55;    // compartment centre radius [mm] (pushed out so it clears the two big boards)

/* [Small breakouts — pockets in the 270 quadrant (the gap left by the three big boards)] */
shifter_l      = 25.5; shifter_w = 15.0; shifter_h = 10.2;  // Adafruit Pixel Shifter (6066): NeoPixel 3.3->5V data line
shifter_angle  = 250;  shifter_radius = 45;  shifter_clear = 12;   // pocket placement [deg/mm] + component clearance
haptic_drv_l   = 25.8; haptic_drv_w = 17.8; haptic_drv_h = 4.6;  // Adafruit DRV2605L (2305): ERM/LRA actuator driver (I2C)
haptic_angle   = 290;  haptic_radius  = 45;  haptic_clear  = 10;   // pocket placement [deg/mm] + component clearance

/* [Base <-> cover fastening screws] */
cover_screw_n       = 4;     // screws joining the cover down to the base
cover_screw_bcd     = 130;   // their bolt-circle Ø [mm] (in the ±45° gaps, clear of the tucked boards)
cover_screw_a0      = 45;    // first-screw angle [deg]
cover_screw_d       = m3_clear;    // clearance hole in the cover (M3) [mm]
cover_screw_pilot   = m3_insert;   // heat-set insert bore in the base (M3) [mm]
cover_screw_cbore   = m3_cbore;    // counterbore Ø for the head [mm]
cover_screw_cbore_h = m3_cbore_h;  // counterbore depth [mm]
cover_screw_depth   = 14;          // pilot-hole depth into the base [mm]

/* [Cover<->Apple-core joint — bolted flange (apple core base bolts straight to the cover)] */
sj_flange_d   = 44;   // flange OD [mm] (fits inside the ring ID; clears the centre boss + screw heads)
sj_flange_t   = 8;    // flange thickness [mm]
sj_screw_n    = 3;    // screws
sj_screw_bcd  = 36;   // bolt-circle Ø [mm] (r=18: head counterbore clears the Ø26 boss, stays inside the flange)
sj_screw_a0   = 0;    // first-screw angle [deg]
sj_register_d = 24;   // centring spigot/recess Ø [mm]

/* [Shared bolted-flange screw spec (M3)] */
joint_screw_d       = m3_clear;    // clearance hole (M3) [mm]
joint_screw_pilot   = m3_insert;   // heat-set insert bore in the cover boss (M3) [mm]
joint_screw_depth   = 7;           // pilot depth (<= flange_t, so it stays in solid material) [mm]
joint_screw_cbore   = m3_cbore;    // head counterbore Ø [mm]
joint_screw_cbore_h = m3_cbore_h;  // counterbore depth [mm]
joint_register_h    = 2.5;  // spigot/recess depth [mm]

/* [Apple — CORE BASE (bolts to cover) + height-ADJUSTABLE SHAFT + FUSED soft BALL]
    The shaft (PETG) and ball (TPU) print as ONE dual-material object on the H2D: export adj_shaft.stl
    (PETG) + apple_ball.stl (TPU), import BOTH at the same origin in Bambu Studio and assign filaments.
    The PETG shaft runs up into the ball and ends in a low FLANGE embedded in a solid TPU cap, so the
    pull load is carried mechanically (NOT by PETG<->TPU adhesion). The ball is hollow + closed
    (smooth, no stem); a feed bore up the shaft opens into the cavity for the ERM + sensor + wiring. */
apple_d        = 45;   // TPU ball outer Ø [mm]
apple_wall     = 3;    // TPU ball shell wall [mm]
apple_grooves  = true; // grip grooves on the ball sides
adj_shaft_d    = 14;   // PETG support-shaft Ø [mm]
adj_shaft_len  = 90;   // shaft length, bottom -> ball centre [mm] (telescopes into the boss for height)
adj_fit_clear  = 0.6;  // boss-bore clearance over the shaft (Ø) [mm] (snug slide fit)
core_boss_d    = 20;   // core-base sleeve boss OD [mm]
core_boss_h    = 30;   // sleeve boss height above the flange [mm]
core_pin_z     = 22;   // HEIGHT-SET pin hole height above the flange top [mm]
hpin_d         = m3_clear;  // height-set / detent pin Ø [mm] (M3)
adj_pin0       = 6;    // first detent hole, height above the shaft bottom [mm]
adj_step       = 7;    // spacing between detent holes [mm]
adj_n          = 4;    // number of detent holes  (-> apple height in discrete steps)
sensor_bore_d  = 9;    // feed bore up the shaft [mm] (wiring + small ERM/sensor; keep <= adj_shaft_d - 4 for a solid wall)
arm_flange_d   = 22;   // PETG armature flange Ø [mm] — embedded in the TPU cap (mechanical anchor for the pull)
arm_flange_t   = 3;    // armature flange thickness [mm]
arm_cap        = 8;    // solid TPU cap thickness below the flange [mm] (embeds it; the upper ball is hollow)
/* [Re-openable press-fit cap — the ball SPLITS so you can load the ERM (Ø10) + a force sensor (FSR head
    Ø18, or the MPRLS board 17.8 mm) — all far bigger than the bore. Lower ball (fused to the shaft) is an
    open cup; a separate TPU cap press-fits on a rim rebate. Tune cap_lip_clear on a test print; add a dab
    of silicone if the pull unseats it. Wires / FSR tail / pressure tube still run down the Ø9 bore. */
cap_split      = 6;    // split plane above the ball centre [mm] (TPU cap = everything above it)
cap_lip_h      = 6;    // press-fit skirt / rim-rebate depth [mm]
cap_lip_clear  = 0.2;  // skirt-to-rebate clearance per side [mm] (TPU press-fit; tune on a test print)

/* [Clear casing BOX — a 5-sided clear box, OPEN on the arm-flange side, that shrouds the whole
    electronics stack. Built by hand from acrylic sheet (the STL is a dimensional build reference, not a
    printed part). The TOP face lies on the cover top over the ring (apple pokes through its centre) and
    clamps down with 3x M3 into the cover just outside the apple-core boss — same 3-screw fixing as before;
    the 4 walls drop from there to the flange face. */
case_box_side  = 178;   // outer square side [mm] (encloses the Ø170 base: ~1 mm gap + a wall each side)
case_box_wall  = 3;     // wall & top-plate thickness [mm]
case_top_bore  = 44.8;  // centre hole Ø in the top [mm] (clears the Ø44 apple-core boss poking through)
disc_screw_n   = 3;     // clamp screws down into the cover (through the top face)
disc_screw_bcd = 56;    // clamp bolt-circle Ø [mm] (just outside the boss / apple-core joint)
disc_screw_a0  = 60;    // first clamp-screw angle [deg] (clocked between the 3 apple-core joint screws)
disc_screw_depth = 5;   // clamp pilot depth into the cover (< cover_plate_t) [mm]
case_preview   = true;  // show the casing box (transparent) in the assembly preview

/* [Quality] */
$fn = 96;
eps = 0.02;

// ---- derived ----
ring_o_wall_r = ring_od / 2 + ring_clear;
ring_i_wall_r = ring_id / 2 - ring_clear;
cover_z       = base_h;                                   // cover sits on top of the base
cover_top_z   = cover_z + cover_plate_t;                  // cover top plate face (ring groove + boss live here)
cover_face_z  = cover_top_z + sj_flange_t;                // top mating face of the cover's joint boss
apple_base_z  = cover_face_z;                             // apple CORE BASE bottom flange sits here (no extension)

// [x,y] of the i-th base<->cover screw (rim bolt-circle)
function cover_screw_pos(i) =
    let(a = cover_screw_a0 + i * 360 / cover_screw_n)
        [cover_screw_bcd / 2 * cos(a), cover_screw_bcd / 2 * sin(a)];

// [x,y] of the i-th screw on a joint bolt-circle (n screws, Ø bcd, first at a0)
function jscrew_pos(i, n, bcd, a0) =
    let(a = a0 + i * 360 / n) [bcd / 2 * cos(a), bcd / 2 * sin(a)];

// =====================================================================
//  Bolted-flange joint helpers  (local frame: the flange occupies z = 0..flange_t)
// =====================================================================

// LOWER side of a joint: pilot holes drilled DOWN from the top mating face (at ztop),
// plus the centering RECESS. Cable bore is cut separately by the caller.
module joint_lower_cuts(ztop, n, bcd, a0, reg_d) {
    for (i = [0 : n - 1])
        translate([jscrew_pos(i,n,bcd,a0)[0], jscrew_pos(i,n,bcd,a0)[1], ztop - joint_screw_depth])
            cylinder(h = joint_screw_depth + eps, d = joint_screw_pilot);
    translate([0, 0, ztop - joint_register_h])
        cylinder(h = joint_register_h + eps, d = reg_d + 0.4);   // recess (+clearance)
}

// UPPER side of a joint: clearance holes through a flange of thickness `t` whose TOP is at ztop,
// with head counterbores from the top. Caller adds the centering spigot + cable bore.
module joint_upper_cuts(ztop, t, n, bcd, a0) {
    for (i = [0 : n - 1]) {
        translate([jscrew_pos(i,n,bcd,a0)[0], jscrew_pos(i,n,bcd,a0)[1], ztop - t - eps])
            cylinder(h = t + 2 * eps, d = joint_screw_d);
        translate([jscrew_pos(i,n,bcd,a0)[0], jscrew_pos(i,n,bcd,a0)[1], ztop - joint_screw_cbore_h])
            cylinder(h = joint_screw_cbore_h + eps, d = joint_screw_cbore);
    }
}

// Centering spigot that protrudes DOWN from a flange bottom at z=0 (annulus; cable bore cut later).
module joint_spigot(reg_d) {
    translate([0, 0, -joint_register_h]) cylinder(h = joint_register_h + eps, d = reg_d);
}

// A standoff post with a screw pilot, for mounting PCBs inside a compartment.
module screw_boss(h = board_standoff, d_out = 9, d_hole = board_screw_d) {
    difference() {
        cylinder(h = h, d = d_out);
        translate([0, 0, 1]) cylinder(h = h, d = d_hole);
    }
}

// Straight wire channel (cut) between two points at a given z, as a hull of cylinders.
module wire_channel(p0, p1, d = 7, z = base_floor + 3) {
    hull() {
        translate([p0[0], p0[1], z]) cylinder(h = d, d = d, center = true);
        translate([p1[0], p1[1], z]) cylinder(h = d, d = d, center = true);
    }
}

// --- PCB pocket helpers: a board centred at radius R on the `ang` axis, with its LONG side `Lt`
//     TANGENTIAL and short (radial) side `Wr`, so it tucks toward the hub centre. ---
function pcb_center(ang, R) = [R * cos(ang), R * sin(ang)];

// open-top pocket cut (clear = component height above the board, standoff = post height under it)
module pcb_pocket_cut(ang, R, Lt, Wr, clear, standoff) {
    translate([pcb_center(ang,R)[0], pcb_center(ang,R)[1], base_h - (clear + standoff)])
        rotate([0, 0, ang + 90])
            translate([-Lt/2 - 2, -Wr/2 - 2, 0])
                cube([Lt + 4, Wr + 4, clear + standoff + eps]);
}
// four corner standoffs under the board
module pcb_standoffs(ang, R, Lt, Wr, clear, standoff) {
    for (s = [[1,1],[1,-1],[-1,1],[-1,-1]])
        translate([pcb_center(ang,R)[0], pcb_center(ang,R)[1], base_h - clear - standoff])
            rotate([0, 0, ang + 90])
                translate([s[0]*(Lt/2 - 5), s[1]*(Wr/2 - 5), 0]) screw_boss();
}
// solid board mock (fit check only), PCB bottom at zbot
module pcb_mock(ang, R, Lt, Wr, zbot, t, col) {
    color(col) translate([pcb_center(ang,R)[0], pcb_center(ang,R)[1], zbot])
        rotate([0, 0, ang + 90]) translate([-Lt/2, -Wr/2, 0]) cube([Lt, Wr, t]);
}

// =====================================================================
//  Flange mount cuts (8x M6 on Ø62 + Ø34 electric opening + Ø16 cable bore)
// =====================================================================
module flange_mount_cuts() {
    for (i = [0 : flange_bolt_n - 1])
        rotate([0, 0, flange_first_angle + i * 360 / flange_bolt_n])
            translate([flange_pcd / 2, 0, 0]) {
                // M6 clearance through the mounting wall (flange face -> under the head)
                translate([0, 0, -eps]) cylinder(h = base_h + 2 * eps, d = flange_bolt_clear);
                // steel-washer seat on the wall top: spreads bolt torque so the printed wall can't crush
                translate([0, 0, flange_mount_t]) cylinder(h = flange_washer_t + eps, d = flange_washer_d);
                // head-access well cut DOWN from the TOP: drop the M6 screw in and drive from the top (hex key or screwdriver)
                translate([0, 0, flange_mount_t + flange_washer_t]) cylinder(h = base_h - flange_mount_t - flange_washer_t + eps, d = flange_cbore_d);
            }
    translate([0, 0, -eps]) cylinder(h = flange_center_h, d = flange_center_d);  // Ø34 electric opening (recess)
    translate([0, 0, -eps]) cylinder(h = base_h + 2 * eps, d = cable_bore_d);    // Ø16 central cable bore
}

// =====================================================================
//  (1) BASE HUB
// =====================================================================
module base() {
    board_xy   = pcb_center(board_angle,   board_radius);
    conv_xy    = pcb_center(conv_angle,    conv_radius);
    opto_xy    = pcb_center(opto_angle,    opto_radius);
    shifter_xy = pcb_center(shifter_angle, shifter_radius);
    haptic_xy  = pcb_center(haptic_angle,  haptic_radius);
    zc         = base_h - 8;   // wiring-channel height: within every pocket's depth, pierces the dividers

    difference() {
        cylinder(h = base_h, d = base_d);                             // solid puck (centre = hub)
        flange_mount_cuts();
        // FIVE board pockets (ring lives in the cover): three big boards at 0/90/180, two small
        // breakouts in the 270 quadrant. Long side tangential, tucked toward the centre.
        pcb_pocket_cut(board_angle,   board_radius,   board_l,      board_w,      board_clear_h, board_standoff);
        pcb_pocket_cut(conv_angle,    conv_radius,    conv_l,       conv_w,       conv_clear_h,  2);
        pcb_pocket_cut(opto_angle,    opto_radius,    opto_l,       opto_w,       opto_clear,    board_standoff);
        pcb_pocket_cut(shifter_angle, shifter_radius, shifter_l,    shifter_w,    shifter_clear, board_standoff);
        pcb_pocket_cut(haptic_angle,  haptic_radius,  haptic_drv_l, haptic_drv_w, haptic_clear,  board_standoff);
        // --- wiring distribution (channels through the pocket dividers, at height zc) ---
        //  24 V media-flange bundle -> converter; then 5 V fanned OUT from the converter to every
        //  consumer; data fanned out from the Metro board; the centre riser feeds the ring + apple.
        wire_channel([0,0],    conv_xy,    7, zc);                 // 24 V in  -> converter
        wire_channel(conv_xy,  board_xy,   6, zc);                 // 5 V -> Metro
        wire_channel(conv_xy,  opto_xy,    6, zc);                 // 5 V -> optocoupler
        wire_channel(conv_xy,  shifter_xy, 5, zc);                 // 5 V -> Pixel Shifter
        wire_channel(conv_xy,  haptic_xy,  5, zc);                 // 5 V -> DRV2605L
        wire_channel(conv_xy,  [0,0],      6, zc);                 // 5 V up the centre riser -> ring + apple
        wire_channel(board_xy, opto_xy,    5, zc);                 // data -> optocoupler
        wire_channel(board_xy, shifter_xy, 5, zc);                 // data -> Pixel Shifter (ring DIN)
        wire_channel(board_xy, haptic_xy,  5, zc);                 // I2C  -> DRV2605L
        wire_channel(board_xy, [0,0],      5, zc);                 // data up the centre riser -> apple
        // cover-fastening screw pilots (rim, in the ±45° gaps)
        for (i = [0 : cover_screw_n - 1])
            translate([cover_screw_pos(i)[0], cover_screw_pos(i)[1], base_h - cover_screw_depth])
                cylinder(h = cover_screw_depth + eps, d = cover_screw_pilot);
    }
    // PCB standoffs added AFTER the pocket cut so they survive as islands on the pocket floor (each takes
    // an M3 heat-set insert). Only the SCREWED boards get posts — the Metro + optocoupler; the potted
    // converter and the two tiny breakouts sit in their pockets (tape / headers), no posts. Placeholder
    // XY: set to each board's real footprint (fit-note 3).
    pcb_standoffs(board_angle, board_radius, board_l, board_w, board_clear_h, board_standoff);
    pcb_standoffs(opto_angle,  opto_radius,  opto_l,  opto_w,  opto_clear,    board_standoff);
}

// =====================================================================
//  (2) COVER : closes the board pockets, seats the ring on top, BOLTED-FLANGE face for the apple core base
// =====================================================================
module base_cover() {
    cover_t   = cover_plate_t;
    face_top  = cover_t + sj_flange_t;        // top mating face of the apple-core joint boss (local frame)
    difference() {
        union() {
            cylinder(h = cover_t, d = base_d);                            // lid plate (opaque, same as base)
            translate([0, 0, cover_t - eps])                              // apple-core joint boss (sits inside the ring)
                cylinder(h = sj_flange_t, d = sj_flange_d);
        }
        translate([0, 0, -eps])                                           // central cable bore (wiring up to the apple)
            cylinder(h = face_top + 2*eps, d = cable_bore_d);
        joint_lower_cuts(face_top, sj_screw_n, sj_screw_bcd, sj_screw_a0, sj_register_d);  // pilots + recess for the apple core base
        // NeoPixel ring groove: annular pocket cut DOWN from the TOP face — the ring drops in from
        // above, LEDs up; the clear casing box covers it. The cover is opaque (no diffuser skin).
        translate([0, 0, cover_t - ring_groove_h])
            difference() {
                cylinder(h = ring_groove_h + eps, r = ring_o_wall_r);
                translate([0,0,-eps]) cylinder(h = ring_groove_h + 3*eps, r = ring_i_wall_r);
            }
        // lead pass-throughs under the groove: one per quarter junction so each 15-LED arc gets a
        // local power/data drop to the base electronics (60 RGBW LEDs pull real current at full white)
        for (a = [0, 90, 180, 270])
            rotate([0, 0, a]) translate([(ring_i_wall_r + ring_o_wall_r)/2, 0, -eps])
                cylinder(h = cover_t + 2*eps, d = ring_wire_d);
        // base<->cover screws: clearance through + head counterbore from the top
        for (i = [0 : cover_screw_n - 1]) {
            translate([cover_screw_pos(i)[0], cover_screw_pos(i)[1], -eps])
                cylinder(h = cover_t + 2*eps, d = cover_screw_d);
            translate([cover_screw_pos(i)[0], cover_screw_pos(i)[1], cover_t - cover_screw_cbore_h])
                cylinder(h = cover_screw_cbore_h + eps, d = cover_screw_cbore);
        }
        // casing-box clamp pilots: 3x M3 heat-set inserts DOWN from the cover top, just outside the boss
        for (i = [0 : disc_screw_n - 1])
            rotate([0, 0, disc_screw_a0 + i * 360 / disc_screw_n])
                translate([disc_screw_bcd/2, 0, cover_t - disc_screw_depth])
                    cylinder(h = disc_screw_depth + eps, d = m3_insert);
    }
}

// =====================================================================
//  (3) APPLE = CORE BASE (bolts to cover) + ADJUSTABLE SHAFT (height) + soft BALL
//      Apple height is set by which detent hole in the shaft the boss pin engages.
// =====================================================================
module grip_grooves(z0) {
    for (a = [-1.2, -0.6, 0, 0.6, 1.2])
        let(off = a * apple_d * 0.14,
            r   = sqrt(max(0, (apple_d * apple_d / 4) - off * off)))
            translate([0, 0, z0 + apple_d / 2 + off])
                rotate_extrude() translate([r, 0]) circle(r = 1.2);
}

// (3a) APPLE CORE BASE — flange (bolts DOWN into the cover) + sleeve boss that takes the shaft + height pin.
module apple_core_base() {
    boss_top = sj_flange_t + core_boss_h;
    difference() {
        union() {
            cylinder(h = sj_flange_t, d = sj_flange_d);                               // flange (-> cover)
            translate([0,0,sj_flange_t - eps]) cylinder(h = core_boss_h + eps, d = core_boss_d); // sleeve boss
            joint_spigot(sj_register_d);                                              // centring spigot (down into cover)
        }
        translate([0,0,-joint_register_h - eps])                                      // shaft bore + wiring path, all the way through
            cylinder(h = joint_register_h + boss_top + eps, d = adj_shaft_d + adj_fit_clear);
        joint_upper_cuts(sj_flange_t, sj_flange_t, sj_screw_n, sj_screw_bcd, sj_screw_a0);  // bolts -> cover
        translate([0,0,sj_flange_t + core_pin_z]) rotate([0,90,0])                     // transverse HEIGHT-SET pin hole
            cylinder(h = core_boss_d + 2*eps, d = hpin_d, center = true);
    }
}

// (3b) PETG SHAFT + ARMATURE — the rod (detent holes set the height) continues up into the ball and ends
//      in a FLANGE the TPU moulds onto; a feed bore runs the full length up into the ball cavity.
module adj_shaft_solid() {   // OUTER envelope of the PETG part (no bore/holes) — the TPU moulds around this
    union() {
        cylinder(h = adj_shaft_len, d = adj_shaft_d);                                     // shaft, bottom -> ball centre
        translate([0,0,adj_shaft_len - eps]) cylinder(h = arm_flange_t + eps, d = arm_flange_d);  // armature anchor flange
    }
}
module adj_shaft() {
    difference() {
        adj_shaft_solid();
        translate([0,0,-eps])                                                         // feed bore -> opens into the ball cavity
            cylinder(h = adj_shaft_len + arm_flange_t + 2*eps, d = sensor_bore_d);
        for (k = [0 : adj_n - 1])                                                     // detent holes (engage the boss pin)
            translate([0,0,adj_pin0 + k*adj_step]) rotate([0,90,0])
                cylinder(h = adj_shaft_d + 2*eps, d = hpin_d, center = true);
    }
}

// (3c) TPU BALL — LOWER cup, fused to the shaft. Open-top so the ERM + force sensor drop in; the PETG
//      flange is embedded in the solid cap below; a rim rebate takes the press-fit cap. Grooves continue.
module apple_ball() {
    ball_c   = adj_shaft_len - arm_cap + apple_d/2;   // ball sits low on the shaft: flange is near the BOTTOM
    fl_top   = adj_shaft_len + arm_flange_t;          // flange top = cavity floor
    inner_d  = apple_d - 2 * apple_wall;              // hollow inner Ø
    split_z  = ball_c + cap_split;                    // ball splits here; the TPU cap covers everything above
    rebate_d = apple_d - 2 * apple_wall;              // straight-wall socket at the rim for the cap skirt
    difference() {
        translate([0,0,ball_c]) sphere(d = apple_d);                                  // ball body
        intersection() {                                                              // hollow the dome above the flange
            translate([0,0,ball_c]) sphere(d = inner_d);                              //   (solid cap below embeds the flange)
            translate([0,0,fl_top]) cylinder(h = apple_d, d = apple_d + 1);
        }
        translate([0,0,split_z]) cylinder(h = apple_d, d = apple_d + 2);              // slice OPEN at the split (cap takes above)
        translate([0,0,split_z - cap_lip_h]) cylinder(h = cap_lip_h + eps, d = rebate_d);  // rim rebate = socket for the skirt
        adj_shaft_solid();                                                            // carve the PETG armature (shared boundary)
        if (apple_grooves) grip_grooves(ball_c - apple_d/2);                          // grip grooves on the sphere sides
    }
}

// (3d) TPU CAP — separate press-fit dome that closes the ball after loading electronics; its skirt plugs
//      the lower-ball rebate. Same material as the ball (solvent/heat-weld it for a permanent smooth ball).
module apple_cap() {
    ball_c  = adj_shaft_len - arm_cap + apple_d/2;
    inner_d = apple_d - 2 * apple_wall;
    split_z = ball_c + cap_split;
    skirt_d = (apple_d - 2 * apple_wall) - 2 * cap_lip_clear;
    union() {
        difference() {                                                                // dome shell above the split
            intersection() {
                translate([0,0,ball_c]) sphere(d = apple_d);
                translate([0,0,split_z]) cylinder(h = apple_d, d = apple_d + 2);
            }
            translate([0,0,ball_c]) sphere(d = inner_d);
            if (apple_grooves) grip_grooves(ball_c - apple_d/2);
        }
        translate([0,0,split_z - cap_lip_h])                                          // press-fit skirt (2 mm wall)
            difference() {
                cylinder(h = cap_lip_h + eps, d = skirt_d);
                translate([0,0,-eps]) cylinder(h = cap_lip_h + 3*eps, d = skirt_d - 4);
            }
    }
}

// Shaft insertion (preview): bottom z so the middle detent hole lines up with the boss pin (a mid height).
function adj_shaft_seat_z() = (sj_flange_t + core_pin_z) - (adj_pin0 + floor(adj_n/2) * adj_step);

// Apple core base + shaft (at a mid height) + ball assembled (preview / section).
module apple_assembled() {
    apple_core_base();
    sh = adj_shaft_seat_z();
    translate([0,0,sh]) { adj_shaft(); apple_ball(); apple_cap(); }   // shaft + fused lower ball + press-fit cap
}

// =====================================================================
//  Optional: mock electronics to check fit (NOT for printing)
// =====================================================================
module electronics_mock() {
    // NeoPixel ring — seated in the groove on the cover TOP (LEDs up)
    color("green") translate([0, 0, cover_top_z - ring_groove_h + ring_pcb_t])
        difference() { cylinder(h = ring_pcb_t, r = ring_od/2); cylinder(h = ring_pcb_t+eps, r = ring_id/2); }
    // five boards tucked in the base (long side tangential)
    pcb_mock(board_angle,   board_radius,   board_l,      board_w,      base_h - board_clear_h, 1.6,              "steelblue");
    pcb_mock(conv_angle,    conv_radius,    conv_l,       conv_w,       base_h - conv_clear_h,  conv_clear_h*0.6, "dimgray");
    pcb_mock(opto_angle,    opto_radius,    opto_l,       opto_w,       base_h - opto_clear,    1.6,              "seagreen");
    pcb_mock(shifter_angle, shifter_radius, shifter_l,    shifter_w,    base_h - shifter_clear, shifter_h,        "darkorange");
    pcb_mock(haptic_angle,  haptic_radius,  haptic_drv_l, haptic_drv_w, base_h - haptic_clear,  haptic_drv_h,     "purple");
}

// Clear casing BOX (built by hand from acrylic sheet) — 5 sides, OPEN on the arm-flange side (bottom).
// Local frame: top plate at z=0..wall; the 4 walls hang DOWN to z=-cover_top_z (the flange face). The
// top carries the apple centre hole + 3 clamp holes — the same 3-screw fixing as before.
module casing() {
    S = case_box_side; w = case_box_wall; H = cover_top_z;
    color("lightcyan", 0.30)
    difference() {
        union() {
            translate([-S/2, -S/2, 0]) cube([S, S, w]);                    // top plate (lies on the cover top)
            translate([-S/2, -S/2, -H])                                    // 4 walls, open top & bottom
                difference() {
                    cube([S, S, H + eps]);
                    translate([w, w, -eps]) cube([S - 2*w, S - 2*w, H + 2*eps]);
                }
        }
        translate([0, 0, -eps]) cylinder(h = w + 2*eps, d = case_top_bore);        // apple centre hole
        for (i = [0 : disc_screw_n - 1])                                            // 3 clamp holes -> cover
            rotate([0, 0, disc_screw_a0 + i * 360 / disc_screw_n])
                translate([disc_screw_bcd/2, 0, -eps])
                    cylinder(h = w + 2*eps, d = m3_clear);
    }
}

// =====================================================================
//  Assembly preview
// =====================================================================
module assembly() {
    color("silver")   base();
    %electronics_mock();
    color("gainsboro", 0.7) translate([0, 0, cover_z]) base_cover();
    translate([0, 0, apple_base_z]) {
        color("slategray") apple_core_base();                       // rigid core base (bolts to cover)
        sh = adj_shaft_seat_z();
        translate([0,0,sh]) {
            color("dimgray")        adj_shaft();                    // PETG shaft + armature
            color("firebrick", 0.9) apple_ball();                   // TPU lower ball (fused)
            color("indianred", 0.85) apple_cap();                   // TPU press-fit cap
        }
    }
    if (case_preview) translate([0, 0, cover_top_z]) casing();      // clear casing box (preview)
    %translate([0, 0, -1]) cylinder(h = 1, d = base_d + 6);         // robot flange face marker
}

// ---- render the selected part ----
if      (part == "base")             base();
else if (part == "cover")            base_cover();
else if (part == "apple_core_base")  apple_core_base();
else if (part == "adj_shaft")        adj_shaft();
else if (part == "apple_ball")       apple_ball();
else if (part == "apple_cap")        apple_cap();
else if (part == "apple")            apple_assembled();
else if (part == "electronics_mock") electronics_mock();
else if (part == "casing")           casing();
else if (part == "apple_section")    // half-cut: shaft + fused lower ball + press-fit cap + cavity + flange
    difference() { apple_assembled(); translate([0, -base_d, -40]) cube([base_d, 2*base_d, 260]); }
else if (part == "section")          // half-cut of the whole stack (shows the full cable path)
    difference() { assembly(); translate([0, -base_d, -20]) cube([base_d, 2*base_d, 360]); }
else                                 assembly();
