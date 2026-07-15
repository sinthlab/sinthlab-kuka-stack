// =====================================================================
//  Apple-pluck end-effector for the KUKA LBR iiwa7   (parametric DRAFT v0.6)
//  sinthlab-kuka-stack / end_effector_design
//
//  ELECTRONICS HUB that bolts to the iiwa7 *media flange (electric)*, routes its
//  power/data wiring, drives a NeoPixel cue, and carries the control electronics.
//  v0.6: the EXTENSION is gone — the apple core base bolts straight to the cover —
//  and the apple rides on a HEIGHT-ADJUSTABLE shaft (detent pin). Stack:
//
//        [ iiwa7 media flange (electric) ]   8x M6 on a Ø51 pitch circle, 24V + data
//        (0) FLANGE PLATE  thick adapter disc: takes the 8x M6 (heads buried flush) + the Ø30 cable bore.
//                          The base bolts onto ITS top face with 4x M3 — so the M6 can be driven with the
//                          plate bare, and the printed base never carries the M6 clamp load.
//                 |  cable bundle THROUGH the Ø30 base centre
//        (1) BASE HUB    central Ø30 bore + hub wire ports; FIVE flat-floored board pockets.
//              ├─ Metro M4 AirLift + power adapter   92 x 50  ┐  ALL VELCRO'D — no screw bosses.
//              ├─ Tobsun 24V->5V converter           60 x 55  │  Packed Cartesian (not on a bolt
//              ├─ Optocoupler board                  95 x 75  │  circle): it is the 95x75 opto that
//              ├─ Level shifter (ring data)          26 x 18  │  forces the Ø225 base.
//              ├─ DRV2605L haptic driver             26 x 18  ┘
//              └─ hub ports + wire channels linking bore <-> converter <-> board <-> centre riser
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

part = "assembly"; // [assembly, flange_plate, base, cover, apple_core_base, adj_shaft, apple_ball, apple_cap, apple, apple_section, section, electronics_mock, casing]

/* [Robot media flange — MEASURED: 8x M6 on a Ø51 bolt circle, Ø34 central electric bore]
    THE ONE NUMBER THAT MUST MATCH THE ROBOT IS THE PITCH CIRCLE: 51 mm, centre-to-centre across two
    diametrically opposite holes (measured directly). Do NOT set it from an outer-edge span: that span is
    (pitch circle + hole Ø), and the plate's hole is deliberately BIGGER than the flange's, so the two
    spans differ by the clearance and are not comparable.

      on the ROBOT : pcd 51 + Ø6 tapped hole      -> outer-edge span 57  (what you measure on the arm)
      on the PLATE : pcd 51 + Ø7 clearance hole   -> outer-edge span 58  (0.5 mm of radial slop per side)
      plate TOP    : pcd 51 + Ø12.5 head well     -> outer-edge span 63.5 <-- NOT the bolt hole. This is
                     the counterbore the M6 cap head drops into, 10 mm above the mating face. Measuring
                     the plate from the top hits THIS, which is why it reads ~63.

    History: Ø62 pcd (span 68.6, measured ~70) did not bolt on; Ø50.4 came from wrongly subtracting the
    plate's Ø6.6 clearance instead of the flange's Ø6 thread. */
flange_bolt_n      = 8;     // 8 x M6 fixing holes on the bolt circle
flange_pcd         = 51.0;  // bolt pitch-circle diameter [mm] — MEASURED centre-to-centre
flange_bolt_clear  = 7.0;   // M6 clearance hole, ISO 273 COARSE fit [mm]. Coarse on purpose: it absorbs
                            // the ±0.5 mm uncertainty in the measured pcd AND the fact that FDM holes
                            // print 0.2-0.4 mm undersize (a Ø6.6 hole prints ~6.3 and binds on an M6).
                            // The Ø12 steel washer under the head covers a Ø7 hole with room to spare.
flange_cbore_d     = 12.5;  // head-access well Ø for the M6 head [mm]. MEASURED head ~10; at Ø11 the well
                            // printed ~10.6 (FDM shrinks holes 0.2-0.4) and the head jammed part-way.
                            // Ø12.5 -> ~12.1 printed -> ~1 mm clearance/side, and clears a hex head's
                            // ~11.5 mm across-corners too. If your head is bigger, raise this (keep it
                            // below flange_washer_d so the washer still has a shoulder to seat on).
flange_mount_t     = 8.0;   // mounting-wall thickness under the M6 head [mm] — the bolt spans THIS (+ the
                            // washer) then threads into the flange, so a short M6x16 reaches (~6.4 mm bite).
flange_washer_d    = 14.0;  // washer-seat Ø under the head [mm] — clears a steel M6 flat washer (OD ~12)
                            // and stays > flange_cbore_d so its top edge is a shoulder that traps the washer
flange_washer_t    = 2.0;   // washer-seat depth [mm] (M6 washer ~1.6 thick); spreads bolt torque on the wall
flange_head_h      = 6.0;   // M6 cap-head height [mm] — buried in the plate so its top face stays flat
flange_first_angle = 22.5;  // first-hole angle [deg] — VERIFY clocking vs the flange
flange_center_d    = 34.0;  // central electric opening Ø [mm] ("inside electric"; clears the connector)
flange_center_h    = 6.0;   // opening/recess depth into the plate bottom [mm] — VERIFY connector protrusion

/* [(0) FLANGE PLATE — thick adapter disc: bolts to the robot, the base bolts onto IT]
    Split out of the base so the 8x M6 can be driven with the plate bare (heads recess flush into its top
    face), and so the printed base never carries the M6 clamp load. Assembly order: plate -> robot, base
    -> plate, THEN populate the electronics. */
plate_d        = 110;  // adapter plate Ø [mm] (covers the M6 pattern + the base's M3 bolt circle)
plate_t        = 16;   // plate thickness [mm] = mount wall 8 + washer seat 2 + M6 head 6 (head sits flush)
plate_screw_n  = 4;    // M3 screws holding the base down onto the plate
plate_screw_bcd = 80;  // their bolt-circle Ø [mm] (clear of the M6 washer seats, well inside the plate rim)
plate_screw_a0 = 45;   // first-screw angle [deg]
plate_screw_depth = 6; // M3 insert bore depth into the plate top [mm] (takes a 4 mm insert)
plate_head_z   = 5;    // M3 head seat height above the base bottom [mm] — the screw spans 5 mm of base
                       // floor, so M3x10 = 5 (base) + 5 (into the plate insert). An access well is bored
                       // straight up from the head to the base top, so the screws are reachable from
                       // inside the open compartment (before the boards are velcro'd down).

/* [Central cabling] */
cable_bore_d   = 30;   // BASE central pass-through for the media-flange bundle [mm] (power/data out of the robot)
apple_bore_d   = 16;   // COVER + apple-core central bore [mm] (only the apple's own wiring runs up here,
                       // so it stays small — a Ø30 bore here would swallow the Ø24 centring spigot)

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

/* [Base electronics hub]
    Ø225 is set by the optocoupler (95 x 75). Its pocket must clear the central bore (r = 18) and still
    fit inside the rim, so its far corners land at r = sqrt((18+77)^2 + 48.5^2) = 106.7 -> Ø225 leaves
    2.8 mm of margin. It does NOT fit on the old Ø170 base in any orientation: the usable radial band
    there was only (85-3) - 18 = 64 mm against the 75 mm the board needs. */
base_d     = 225;   // base outer Ø [mm]
base_h     = 30;    // base height = compartment depth [mm]
base_floor = 8;     // floor thickness [mm] — the DEEPEST pocket (converter, 22) sets this: 30 - 22 = 8
base_wall  = 3;     // structural wall / bridge thickness [mm]
hub_wall   = 3;     // material between the central bore and the nearest board pocket [mm]
pocket_clear = 1.0; // pocket oversize per side [mm] (drop-in locating recess; boards are velcro'd, not screwed)

/* [Wire distribution through the central hub]
    The flange bundle arrives up the Ø30 bore; these radial ports let it be fanned out of the hub in any
    direction (and let the Metro reach the other boards) instead of only through the point-to-point
    channels below. */
hub_port_n = 8;     // radial wire ports around the central bore
hub_port_d = 6;     // port Ø [mm]

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

/* [BOARD LAYOUT — Cartesian, velcro-mounted]
    NO screw bosses anywhere: every board is stuck down on hook-and-loop, so each pocket is just a
    shallow locating recess with a flat floor. `*_pos` = pocket centre [x,y]; `*_rot` = 0 puts the LONG
    side along x, 90 puts it along y. `*_clear` = pocket DEPTH below the base top = velcro (~2) + PCB +
    tallest component. Layout is packed Cartesian (not on a bolt circle) because three large rectangles
    reaching in toward a central bore cannot be spaced angularly: their combined angular width at the hub
    exceeds 360°. Verified: no overlaps, all corners inside r=109.5, all clear of the Ø30 bore. */
velcro_t       = 2;     // hook-and-loop pad thickness under every board [mm] (included in each *_clear)

// Control board — Adafruit Metro M4 Express AirLift Lite (4000), + its power adapter
board_l        = 92.0;  board_w = 50.0;   // footprint [mm]
board_clear_h  = 19;    // pocket depth [mm] (velcro 2 + PCB + ~15 tall components)
board_pos      = [32.0, -44.0];  board_rot = 0;

// DC-DC converter — Tobsun 24V->5V potted block
conv_l         = 60.0;  conv_w = 55.0;    // footprint [mm]
conv_clear_h   = 22;    // pocket depth [mm] — the DEEPEST pocket; it sets base_floor (30 - 22 = 8)
conv_pos       = [47.0, 44.5];   conv_rot = 0;

// Optocoupler board — the big one; it is what forces the Ø225 base
opto_l         = 95.0;  opto_w = 75.0;    // footprint [mm]
opto_clear     = 18;    // pocket depth [mm]
opto_pos       = [-56.5, 0.0];   opto_rot = 90;   // long side RADIAL-ish (along y), tucked to the -x side

// Level shifter (NeoPixel 3.3->5V data line)
shifter_l      = 26.0; shifter_w = 18.0; shifter_h = 10.2;
shifter_clear  = 12;    // pocket depth [mm]
shifter_pos    = [-46.0, 62.0];  shifter_rot = 0;

// Haptic driver — DRV2605L, drives the ERM in the apple
haptic_drv_l   = 26.0; haptic_drv_w = 18.0; haptic_drv_h = 4.6;
haptic_clear   = 9;     // pocket depth [mm]
haptic_pos     = [-46.0, -62.0]; haptic_rot = 0;

/* [Base <-> cover fastening screws] */
cover_screw_n       = 4;     // screws joining the cover down to the base
cover_screw_bcd     = 200;   // their bolt-circle Ø [mm] — at r=100, OUTSIDE the Ø157 ring and in the four
                             // solid spokes the board layout leaves at 0/90/180/270 (verified clear)
cover_screw_a0      = 0;     // first-screw angle [deg]
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
case_box_side  = 233;   // outer square side [mm] (encloses the Ø225 base: ~1 mm gap + a wall each side)
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

// Straight wire channel (cut) between two points at a given z, as a hull of cylinders.
module wire_channel(p0, p1, d = 7, z = base_floor + 3) {
    hull() {
        translate([p0[0], p0[1], z]) cylinder(h = d, d = d, center = true);
        translate([p1[0], p1[1], z]) cylinder(h = d, d = d, center = true);
    }
}

// --- PCB pocket helpers (Cartesian): board of footprint L x W centred at `pos`, turned by `rot`.
//     No standoffs — the boards are velcro'd to the flat pocket floor. ---

// open-top locating pocket; `depth` = velcro + PCB + tallest component, measured down from the base top
module pcb_pocket_cut(pos, L, W, rot, depth) {
    l = L + 2 * pocket_clear;
    w = W + 2 * pocket_clear;
    translate([pos[0], pos[1], base_h - depth])
        rotate([0, 0, rot])
            translate([-l/2, -w/2, 0])
                cube([l, w, depth + eps]);
}
// solid board mock (fit check only), PCB bottom sitting on the velcro at the pocket floor
module pcb_mock(pos, L, W, rot, zbot, t, col) {
    color(col) translate([pos[0], pos[1], zbot])
        rotate([0, 0, rot]) translate([-L/2, -W/2, 0]) cube([L, W, t]);
}

// =====================================================================
//  (0) FLANGE PLATE — bolts to the robot (8x M6); the base bolts onto it (4x M3)
// =====================================================================
module flange_plate() {
    difference() {
        cylinder(h = plate_t, d = plate_d);
        // 8x M6 down into the robot flange. Head + washer are BURIED, so the plate top stays flat for
        // the base to sit on: wall 0..8, washer seat 8..10, head 10..16 (flush with the top face).
        for (i = [0 : flange_bolt_n - 1])
            rotate([0, 0, flange_first_angle + i * 360 / flange_bolt_n])
                translate([flange_pcd / 2, 0, 0]) {
                    translate([0, 0, -eps]) cylinder(h = plate_t + 2*eps, d = flange_bolt_clear);
                    translate([0, 0, flange_mount_t]) cylinder(h = flange_washer_t + eps, d = flange_washer_d);
                    translate([0, 0, flange_mount_t + flange_washer_t])
                        cylinder(h = flange_head_h + eps, d = flange_cbore_d);
                }
        translate([0, 0, -eps]) cylinder(h = flange_center_h, d = flange_center_d);  // connector recess (bottom face)
        translate([0, 0, -eps]) cylinder(h = plate_t + 2*eps, d = cable_bore_d);     // Ø30 cable bore
        // M3 heat-set inserts in the TOP face: the base screws down into these
        for (i = [0 : plate_screw_n - 1])
            rotate([0, 0, plate_screw_a0 + i * 360 / plate_screw_n])
                translate([plate_screw_bcd/2, 0, plate_t - plate_screw_depth])
                    cylinder(h = plate_screw_depth + eps, d = m3_insert);
    }
}

// =====================================================================
//  (1) BASE HUB — sits on the plate; carries the five velcro'd boards
// =====================================================================
module base() {
    zc = base_h - 8;   // wiring-channel height: above every pocket floor, pierces the solid spokes

    difference() {
        cylinder(h = base_h, d = base_d);                             // solid puck (centre = hub)
        translate([0, 0, -eps]) cylinder(h = base_h + 2*eps, d = cable_bore_d);   // Ø30 central cable bore
        // base -> plate screws: M3 clearance through the floor, head seated at plate_head_z, and an
        // access well bored straight up to the base top so a hex key reaches the head from inside the
        // open compartment. Two of the four land under a board pocket, which is why the base must be
        // bolted to the plate BEFORE the boards are velcro'd down.
        for (i = [0 : plate_screw_n - 1])
            rotate([0, 0, plate_screw_a0 + i * 360 / plate_screw_n])
                translate([plate_screw_bcd/2, 0, 0]) {
                    translate([0, 0, -eps]) cylinder(h = plate_head_z + eps, d = m3_clear);
                    translate([0, 0, plate_head_z]) cylinder(h = base_h - plate_head_z + eps, d = m3_cbore);
                }
        // FIVE board pockets — flat-floored locating recesses (velcro, no screws)
        pcb_pocket_cut(board_pos,   board_l,      board_w,      board_rot,   board_clear_h);
        pcb_pocket_cut(conv_pos,    conv_l,       conv_w,       conv_rot,    conv_clear_h);
        pcb_pocket_cut(opto_pos,    opto_l,       opto_w,       opto_rot,    opto_clear);
        pcb_pocket_cut(shifter_pos, shifter_l,    shifter_w,    shifter_rot, shifter_clear);
        pcb_pocket_cut(haptic_pos,  haptic_drv_l, haptic_drv_w, haptic_rot,  haptic_clear);
        // --- wiring distribution ---
        // Radial ports out of the central hub: the flange bundle can be fanned out in ANY direction
        // instead of only along the point-to-point channels.
        for (i = [0 : hub_port_n - 1])
            rotate([0, 0, i * 360 / hub_port_n])
                translate([0, 0, zc]) rotate([0, 90, 0])
                    cylinder(h = base_d/2, d = hub_port_d);
        // Point-to-point channels through the solid spokes between pockets, at height zc.
        wire_channel([0,0],      conv_pos,    7, zc);   // 24 V in  -> converter
        wire_channel(conv_pos,   board_pos,   6, zc);   // 5 V -> Metro
        wire_channel(conv_pos,   opto_pos,    6, zc);   // 5 V -> optocoupler
        wire_channel(conv_pos,   shifter_pos, 5, zc);   // 5 V -> level shifter
        wire_channel(conv_pos,   haptic_pos,  5, zc);   // 5 V -> DRV2605L
        wire_channel(conv_pos,   [0,0],       6, zc);   // 5 V up the centre riser -> ring + apple
        wire_channel(board_pos,  opto_pos,    5, zc);   // data -> optocoupler
        wire_channel(board_pos,  shifter_pos, 5, zc);   // data -> level shifter (ring DIN)
        wire_channel(board_pos,  haptic_pos,  5, zc);   // I2C  -> DRV2605L
        wire_channel(board_pos,  [0,0],       5, zc);   // data up the centre riser -> apple
        // cover-fastening screw pilots (heat-set inserts, in the four solid spokes at 0/90/180/270)
        for (i = [0 : cover_screw_n - 1])
            translate([cover_screw_pos(i)[0], cover_screw_pos(i)[1], base_h - cover_screw_depth])
                cylinder(h = cover_screw_depth + eps, d = cover_screw_pilot);
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
        translate([0, 0, -eps])                                           // central bore (wiring up to the apple)
            cylinder(h = face_top + 2*eps, d = apple_bore_d);
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
    // five boards velcro'd into their pockets (PCB bottom sits on the velcro pad)
    pcb_mock(board_pos,   board_l,      board_w,      board_rot,   base_h - board_clear_h + velcro_t, 1.6,              "steelblue");
    pcb_mock(conv_pos,    conv_l,       conv_w,       conv_rot,    base_h - conv_clear_h  + velcro_t, conv_clear_h*0.6, "dimgray");
    pcb_mock(opto_pos,    opto_l,       opto_w,       opto_rot,    base_h - opto_clear    + velcro_t, 1.6,              "seagreen");
    pcb_mock(shifter_pos, shifter_l,    shifter_w,    shifter_rot, base_h - shifter_clear + velcro_t, shifter_h,        "darkorange");
    pcb_mock(haptic_pos,  haptic_drv_l, haptic_drv_w, haptic_rot,  base_h - haptic_clear  + velcro_t, haptic_drv_h,     "purple");
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
    color("dimgray")  translate([0, 0, -plate_t]) flange_plate();   // adapter plate: robot <-> base
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
    %translate([0, 0, -plate_t - 1]) cylinder(h = 1, d = plate_d + 6);  // robot flange face marker
}

// ---- render the selected part ----
if      (part == "flange_plate")     flange_plate();
else if (part == "base")             base();
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
