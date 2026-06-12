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
//                  + (3b) ADJ SHAFT  rod with a column of detent holes (pick the height) + a side
//                                    window + the ball lock hole; slides into the boss
//                  + (3c) BALL       hollow grippable sphere (soft); socket sleeves the shaft top +
//                                    cross-pin lock; ERM motor + pressure sensor sit in the cavity
//        ( + ) PLEXIGLASS CASING   user-cut clear shroud seated in a groove on the base rim,
//                                  shielding the electronics + ring (shaft exits the top)
//
//  !!!  VERIFY every flange/component dimension vs the real datasheets before printing.
//  Export one part at a time, e.g.:
//     openscad -D 'part="base"'  -o base.stl  apple_pluck_end_effector.scad
// =====================================================================

part = "assembly"; // [assembly, base, cover, apple_core_base, adj_shaft, apple_ball, apple, apple_section, section, electronics_mock, casing]

/* [Robot media flange (electric) — VERIFY vs iiwa7 datasheet] */
flange_bolt_n      = 8;     // media flange electric: 8 mounting screws
flange_pcd         = 63;    // bolt pitch-circle diameter [mm]   (VERIFY)
flange_bolt_clear  = 4.5;   // screw clearance hole (e.g. M4) [mm] (VERIFY size)
flange_cbore_d     = 8.5;   // counterbore for screw head [mm]
flange_cbore_h     = 4.5;   // counterbore depth [mm]
flange_first_angle = 22.5;  // angle of the first hole [deg]      (VERIFY)
flange_center_d    = 31.6;  // central centering-boss recess Ø [mm] (VERIFY)
flange_center_h    = 4.0;   // recess depth [mm]

/* [Central cabling] */
cable_bore_d   = 16;   // central pass-through for the media-flange bundle [mm]
shaft_bore_d   = 8;    // wiring channel up the adjustable apple shaft [mm] (a few leads: ERM + sensor)

/* [Base electronics hub] */
base_d     = 170;   // base outer Ø [mm]  (three big boards at 0/90/180, two breakouts in the 270 quadrant)
base_h     = 30;    // base height = compartment depth [mm]
base_floor = 5;     // floor thickness above the flange face [mm]
base_wall  = 3;     // structural wall / bridge thickness [mm]

/* [NeoPixel ring — Adafruit 1768, 24x5050 — seated OPEN in a groove on the COVER TOP  https://www.adafruit.com/product/1768] */
ring_od        = 66.0;  // ring outer Ø [mm]   (Adafruit 1768 = 66.04 mm)
ring_id        = 52.0;  // ring inner Ø [mm]   (Adafruit 1768 = 52.07 mm)
ring_pcb_t     = 1.6;   // PCB thickness [mm]
ring_clear     = 0.6;   // radial clearance per side [mm]
ring_groove_h  = 4.0;   // top-facing groove depth to seat the ring [mm] (LEDs face up; plexiglass shields)
ring_wire_d    = 6.0;   // pass-through under the groove for the ring leads -> base [mm]

/* [Cover plate] */
cover_plate_t  = 6.0;   // cover thickness [mm] — opaque (same material as base); top ring groove + 4x M3

/* [Control board — Adafruit Metro M4 Express AirLift Lite (4000) https://www.adafruit.com/product/4000] */
board_l        = 72.0;  // PCB length [mm] (Arduino Metro footprint; board overall 72 x 54 x 15)
board_w        = 54.0;  // PCB width  [mm]
board_clear_h  = 16;    // tallest-component clearance above the PCB [mm] (overall ht ~15)
board_standoff = 4;     // standoff post height under the board [mm]
board_screw_d  = 3.2;   // M3 mount [mm]  -- VERIFY: real Metro/Arduino holes are NOT symmetric; set standoff XY to the footprint
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
cover_screw_d       = 3.4;   // clearance hole in the cover (M3) [mm]
cover_screw_pilot   = 2.5;   // pilot hole in the base (self-tapping M3) [mm]
cover_screw_cbore   = 6.5;   // counterbore Ø for the head [mm]
cover_screw_cbore_h = 3.0;   // counterbore depth [mm]
cover_screw_depth   = 14;    // pilot-hole depth into the base [mm]

/* [Cover<->Apple-core joint — bolted flange (apple core base bolts straight to the cover)] */
sj_flange_d   = 44;   // flange OD [mm] (fits inside the ring ID; clears the centre boss + screw heads)
sj_flange_t   = 8;    // flange thickness [mm]
sj_screw_n    = 3;    // screws
sj_screw_bcd  = 36;   // bolt-circle Ø [mm] (r=18: head counterbore clears the Ø26 boss, stays inside the flange)
sj_screw_a0   = 0;    // first-screw angle [deg]
sj_register_d = 24;   // centring spigot/recess Ø [mm]

/* [Shared bolted-flange screw spec (M3)] */
joint_screw_d       = 3.4;  // clearance hole (M3) [mm]
joint_screw_pilot   = 2.5;  // pilot (self-tapping M3) [mm]  (open to insert OD for heat-set)
joint_screw_depth   = 7;    // pilot depth (<= flange_t, so it stays in solid material) [mm]
joint_screw_cbore   = 6.5;  // head counterbore Ø [mm]
joint_screw_cbore_h = 3.0;  // counterbore depth [mm]
joint_register_h    = 2.5;  // spigot/recess depth [mm]

/* [Apple — CORE BASE (bolts to cover) + height-ADJUSTABLE SHAFT + soft BALL] */
apple_d        = 45;   // ball outer Ø [mm]
apple_wall     = 3;    // ball shell wall [mm]
apple_stem_d   = 9;    // top stem Ø [mm]
apple_stem_h   = 16;   // top stem height [mm]
apple_grooves  = true; // grip grooves on the ball
adj_shaft_d    = 14;   // adjustable support-shaft Ø [mm] (THIN so it doesn't occlude the ring at an angle)
adj_shaft_len  = 90;   // shaft length [mm]
adj_fit_clear  = 0.6;  // boss-bore clearance over the shaft (Ø) [mm] (snug slide fit)
core_boss_d    = 20;   // core-base sleeve boss OD [mm] (slim; sits well inside the ring; bore = shaft + fit clear)
core_boss_h    = 30;   // sleeve boss height above the flange [mm]
core_pin_z     = 22;   // HEIGHT-SET pin hole height above the flange top [mm] (near the boss top)
hpin_d         = 3.4;  // height-set / detent pin Ø [mm]
adj_pin0       = 6;    // first detent hole, height above the shaft bottom [mm]
adj_step       = 7;    // spacing between detent holes [mm]
adj_n          = 4;    // number of detent holes  (-> apple-to-base gap ~17..38 mm, default ~24 mm)
ball_socket_depth = 38;// how deep the ball socket sleeves the shaft top [mm]
ball_fit_clear = 0.4;  // ball-socket radial clearance over the shaft [mm]
ball_lock_h    = 8;    // ball lock-hole height above the ball base [mm] (in the solid lower shell)
core_window_d  = 8;    // wiring side-window Ø in the shaft (releases wires into the cavity) [mm]
window_from_top= 15;   // side-window centre, distance below the shaft top [mm] (inside the cavity)
lock_pin_d     = 3.4;  // ball<->shaft cross-pin / M3 lock Ø [mm]

/* [Plexiglass casing — user-cut clear shroud over the electronics + ring; this models its SEAT] */
case_wall      = 2.0;  // shroud wall thickness [mm]
case_fit       = 0.3;  // radial clearance per side in the seat groove [mm]
case_seat_in   = 8.0;  // seat-groove centre, distance inward from the base edge [mm]
case_seat_h    = 4.0;  // seat-groove depth into the base top [mm]
case_h         = 34;   // shroud height above the base top [mm] (clears the cover + ring; shaft exits top)
case_preview   = true; // show the shroud (transparent) in the assembly preview

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
case_seat_r   = base_d / 2 - case_seat_in;                // plexiglass-shroud seat-groove centre radius

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
module screw_boss(h = board_standoff, d_out = 7, d_hole = board_screw_d) {
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
//  Flange mount cuts (8 bolts + centering recess + central cable bore)
// =====================================================================
module flange_mount_cuts() {
    for (i = [0 : flange_bolt_n - 1])
        rotate([0, 0, flange_first_angle + i * 360 / flange_bolt_n])
            translate([flange_pcd / 2, 0, 0]) {
                translate([0, 0, -eps]) cylinder(h = base_h + 2 * eps, d = flange_bolt_clear);
                translate([0, 0, -eps]) cylinder(h = flange_cbore_h,   d = flange_cbore_d);
            }
    translate([0, 0, -eps]) cylinder(h = flange_center_h, d = flange_center_d);  // centering recess
    translate([0, 0, -eps]) cylinder(h = base_h + 2 * eps, d = cable_bore_d);    // central cable bore
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
        union() {
            cylinder(h = base_h, d = base_d);                         // solid puck (centre = hub)
            pcb_standoffs(board_angle,   board_radius,   board_l,      board_w,      board_clear_h, board_standoff);
            pcb_standoffs(conv_angle,    conv_radius,    conv_l,       conv_w,       conv_clear_h,  2);
            pcb_standoffs(opto_angle,    opto_radius,    opto_l,       opto_w,       opto_clear,    board_standoff);
            pcb_standoffs(shifter_angle, shifter_radius, shifter_l,    shifter_w,    shifter_clear, board_standoff);
            pcb_standoffs(haptic_angle,  haptic_radius,  haptic_drv_l, haptic_drv_w, haptic_clear,  board_standoff);
        }
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
        // plexiglass-shroud seat groove (annular channel in the base top rim)
        translate([0, 0, base_h - case_seat_h])
            difference() {
                cylinder(h = case_seat_h + eps, r = case_seat_r + case_wall/2 + case_fit);
                translate([0,0,-eps]) cylinder(h = case_seat_h + 3*eps, r = case_seat_r - case_wall/2 - case_fit);
            }
    }
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
        // above, LEDs up; a plexiglass shroud shields it. The cover is opaque (no diffuser skin).
        translate([0, 0, cover_t - ring_groove_h])
            difference() {
                cylinder(h = ring_groove_h + eps, r = ring_o_wall_r);
                translate([0,0,-eps]) cylinder(h = ring_groove_h + 3*eps, r = ring_i_wall_r);
            }
        // lead pass-through under the groove: ring power/data drop down to the base electronics
        translate([(ring_i_wall_r + ring_o_wall_r)/2, 0, -eps])
            cylinder(h = cover_t + 2*eps, d = ring_wire_d);
        // base<->cover screws: clearance through + head counterbore from the top
        for (i = [0 : cover_screw_n - 1]) {
            translate([cover_screw_pos(i)[0], cover_screw_pos(i)[1], -eps])
                cylinder(h = cover_t + 2*eps, d = cover_screw_d);
            translate([cover_screw_pos(i)[0], cover_screw_pos(i)[1], cover_t - cover_screw_cbore_h])
                cylinder(h = cover_screw_cbore_h + eps, d = cover_screw_cbore);
        }
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

// (3b) ADJUSTABLE SHAFT — rod with a column of detent holes (pick the height) + side window + ball lock hole.
module adj_shaft() {
    difference() {
        cylinder(h = adj_shaft_len, d = adj_shaft_d);
        translate([0,0,-eps]) cylinder(h = adj_shaft_len + 2*eps, d = shaft_bore_d);   // wiring bore (full length)
        for (k = [0 : adj_n - 1])                                                      // detent holes (engage the boss pin)
            translate([0,0,adj_pin0 + k*adj_step]) rotate([0,90,0])
                cylinder(h = adj_shaft_d + 2*eps, d = hpin_d, center = true);
        translate([0,0,adj_shaft_len - window_from_top]) rotate([0,90,0])              // wiring side-window -> ball cavity
            cylinder(h = adj_shaft_d + 2*eps, d = core_window_d, center = true);
        translate([0,0,adj_shaft_len - (ball_socket_depth - ball_lock_h)]) rotate([0,90,0])  // ball lock hole (aligns w/ ball)
            cylinder(h = adj_shaft_d + 2*eps, d = lock_pin_d, center = true);
    }
}

// (3c) APPLE BALL — soft/separate material: hollow grippable sphere; socket sleeves the shaft top.
//      Lower shell is solid (for the lock pin); the cavity (ERM motor + sensor) is the annulus above it.
module apple_ball() {
    sph_z    = apple_d / 2;                       // sphere centre (ball base z=0 = where the shaft top enters)
    z_cav    = ball_lock_h + 6;                   // cavity floor: keep the lower shell solid for the lock
    socket_d = adj_shaft_d + 2 * ball_fit_clear;  // sleeve bore over the shaft
    difference() {
        union() {
            translate([0,0,sph_z]) sphere(d = apple_d);                               // body
            translate([0,0,sph_z + apple_d/2 - apple_wall])
                cylinder(h = apple_stem_h + apple_wall, d = apple_stem_d);            // solid top stem
        }
        intersection() {                                                             // hollow cavity ABOVE z_cav only
            translate([0,0,sph_z]) sphere(d = apple_d - 2 * apple_wall);
            translate([0,0,z_cav]) cylinder(h = apple_d, d = apple_d + 1);
        }
        if (apple_grooves) grip_grooves(0);
        translate([0,0,-eps]) cylinder(h = ball_socket_depth + 1, d = socket_d);      // central socket (sleeves the shaft top)
        translate([0,0,ball_lock_h]) rotate([0,90,0])                                 // transverse lock hole (aligns w/ shaft)
            cylinder(h = apple_d + 2*eps, d = lock_pin_d, center = true);
    }
}

// Shaft insertion (preview): bottom z so the middle detent hole lines up with the boss pin (a mid height).
function adj_shaft_seat_z() = (sj_flange_t + core_pin_z) - (adj_pin0 + floor(adj_n/2) * adj_step);

// Apple core base + shaft (at a mid height) + ball assembled (preview / section).
module apple_assembled() {
    apple_core_base();
    sh = adj_shaft_seat_z();
    translate([0,0,sh]) adj_shaft();
    translate([0,0,sh + adj_shaft_len - ball_socket_depth]) apple_ball();
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

// Plexiglass shroud (user-cut; preview only) — a clear tube seated in the base-rim groove.
module casing() {
    color("lightcyan", 0.25)
        translate([0, 0, base_h - case_seat_h])
            difference() {
                cylinder(h = case_seat_h + case_h, r = case_seat_r + case_wall/2);
                translate([0,0,-eps]) cylinder(h = case_seat_h + case_h + 2*eps, r = case_seat_r - case_wall/2);
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
        color("dimgray")        translate([0,0,sh]) adj_shaft();    // height-adjustable shaft
        color("firebrick", 0.9) translate([0,0,sh + adj_shaft_len - ball_socket_depth]) apple_ball();  // soft ball
    }
    if (case_preview) casing();                                     // plexiglass shroud (preview)
    %translate([0, 0, -1]) cylinder(h = 1, d = base_d + 6);         // robot flange face marker
}

// ---- render the selected part ----
if      (part == "base")             base();
else if (part == "cover")            base_cover();
else if (part == "apple_core_base")  apple_core_base();
else if (part == "adj_shaft")        adj_shaft();
else if (part == "apple_ball")       apple_ball();
else if (part == "apple")            apple_assembled();
else if (part == "electronics_mock") electronics_mock();
else if (part == "casing")           casing();
else if (part == "apple_section")    // half-cut: shaft sleeved by the ball + cavity + lock pin + boss
    difference() { apple_assembled(); translate([0, -base_d, -40]) cube([base_d, 2*base_d, 260]); }
else if (part == "section")          // half-cut of the whole stack (shows the full cable path)
    difference() { assembly(); translate([0, -base_d, -20]) cube([base_d, 2*base_d, 360]); }
else                                 assembly();
