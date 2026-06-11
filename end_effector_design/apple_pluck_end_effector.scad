// =====================================================================
//  Apple-pluck end-effector for the KUKA LBR iiwa7   (parametric DRAFT v0.3)
//  sinthlab-kuka-stack / end_effector_design
//
//  ELECTRONICS HUB that bolts to the iiwa7 *media flange (electric)*, routes its
//  power/data wiring, drives a NeoPixel cue, and carries the control board +
//  DC-DC converter. v0.3: all shaft joints are now BOLTED FLANGES (no slip
//  coupler) — every joint is N x M3 on a bolt circle, centred by a spigot, with
//  the cable bore down the middle. Stack:
//
//        [ iiwa7 media flange (electric) ]   8x screws, 24V + data
//                 |  cable bundle THROUGH the base centre
//        (1) BASE HUB    8-hole flange mount + central cable bore
//              ├─ NeoPixel ring (Adafruit 1768) in a concentric groove
//              ├─ compartment: Metro AirLift board
//              ├─ compartment: Tobsun 24V->5V converter
//              └─ wire channels linking bore <-> converter <-> board <-> ring
//        (2) COVER       closes compartments · diffuses ring · 4x M3 down to base
//                 ▼ BOLTED FLANGE: extension screws down into the cover (N x M3 + spigot)
//        (3) EXTENSION   shaft; cable bore + SIDE EXIT hole; flange at each end
//                 ▼ BOLTED FLANGE: apple CORE screws down into the extension top
//        (4) APPLE = (4a) CORE  flange + full-length support shaft (rigid); wiring up the centre,
//                                released into the cavity via a side window
//                  + (4b) BALL  hollow grippable sphere (soft/separate material); sleeves the core
//                                shaft + cross-pin/M3 lock; sensor + actuator sit in the cavity
//
//  Two joint sizes: the cover<->extension flange is sized to the shaft (sj_*, Ø44);
//  the extension<->apple flange is wider so a driver clears the sphere (aj_*, Ø60,
//  bolt circle r=25 > apple r=22.5 + driver). Both are centred by a spigot, with the
//  cable bore down the middle.
//
//  !!!  VERIFY every flange/component dimension vs the real datasheets before printing.
//  Export one part at a time, e.g.:
//     openscad -D 'part="base"'  -o base.stl  apple_pluck_end_effector.scad
// =====================================================================

part = "assembly"; // [assembly, base, cover, extension, apple_core, apple_ball, apple, apple_section, section, electronics_mock]

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
shaft_bore_d   = 11;   // cable channel up the shaft (extension) [mm]
shaft_exit_d   = 13;   // SIDE exit hole in the shaft to pull the bundle out [mm]
shaft_exit_z   = 28;   // exit-hole centre height above the shaft-rod base [mm]

/* [Base electronics hub] */
base_d     = 122;   // base outer Ø [mm]  (driven by the Metro board footprint)
base_h     = 30;    // base height = compartment depth [mm]
base_floor = 5;     // floor thickness above the flange face [mm]
base_wall  = 3;     // structural wall / bridge thickness [mm]

/* [NeoPixel ring — Adafruit 1768, 24x5050  https://www.adafruit.com/product/1768] */
ring_od        = 66.0;  // ring outer Ø [mm]   (VERIFY)
ring_id        = 52.0;  // ring inner Ø [mm]   (VERIFY)
ring_pcb_t     = 1.6;   // PCB thickness [mm]
ring_clear     = 0.6;   // radial clearance per side [mm]
ring_groove_h  = 4.0;   // groove depth to seat the PCB [mm]
diffuser_t     = 1.5;   // diffuser window thickness over the LEDs [mm] (print clear/white)

/* [Metro AirLift board — Arduino Uno footprint — VERIFY your board] */
board_l        = 68.6;  // [mm]
board_w        = 53.3;  // [mm]
board_clear_h  = 22;    // tallest-component clearance above the board [mm]
board_standoff = 4;     // standoff post height under the board [mm]
board_screw_d  = 3.2;   // M3 mount [mm]
board_angle    = 0;     // angular position of the board compartment [deg]
board_radius   = 33;    // compartment centre radius from hub axis [mm]

/* [Tobsun 24V->5V converter — VERIFY your model] */
conv_l         = 60;    // [mm]
conv_w         = 30;    // [mm]
conv_clear_h   = 18;    // [mm]
conv_angle     = 180;   // angular position of the converter compartment [deg]
conv_radius    = 36;    // compartment centre radius [mm]

/* [Base <-> cover fastening screws] */
cover_screw_n       = 4;     // screws joining the cover down to the base
cover_screw_bcd     = 110;   // their bolt-circle Ø [mm] (solid rim, clear of compartments)
cover_screw_a0      = 45;    // first-screw angle [deg]
cover_screw_d       = 3.4;   // clearance hole in the cover (M3) [mm]
cover_screw_pilot   = 2.5;   // pilot hole in the base (self-tapping M3) [mm]
cover_screw_cbore   = 6.5;   // counterbore Ø for the head [mm]
cover_screw_cbore_h = 3.0;   // counterbore depth [mm]
cover_screw_depth   = 14;    // pilot-hole depth into the base [mm]

/* [Cover<->Extension joint — bolted flange, sized to the shaft] */
sj_flange_d   = 44;   // flange OD [mm] (just clears the shaft + screw heads — daintier than the apple joint)
sj_flange_t   = 8;    // flange thickness [mm]
sj_screw_n    = 3;    // screws
sj_screw_bcd  = 34;   // bolt-circle Ø [mm] (r=17 > shaft r13 -> heads sit on the ledge, clear of the shaft)
sj_screw_a0   = 0;    // first-screw angle [deg]
sj_register_d = 24;   // centring spigot/recess Ø [mm]

/* [Extension<->Apple joint — bolted flange, MUST clear the apple sphere] */
aj_flange_d   = 60;   // flange OD [mm]
aj_flange_t   = 8;    // flange thickness [mm]
aj_screw_n    = 4;    // screws
aj_screw_bcd  = 50;   // bolt-circle Ø [mm] (r=25; even a straight hex driver clears the apple r22.5)
aj_screw_a0   = 45;   // first-screw angle [deg]
aj_register_d = 24;   // centring spigot/recess Ø [mm]

/* [Shared shaft-joint screw spec (M3) + shaft rod] */
joint_screw_d       = 3.4;  // clearance hole (M3) [mm]
joint_screw_pilot   = 2.5;  // pilot (self-tapping M3) [mm]  (open to insert OD for heat-set)
joint_screw_depth   = 7;    // pilot depth (<= flange_t, so it stays in solid material) [mm]
joint_screw_cbore   = 6.5;  // head counterbore Ø [mm]
joint_screw_cbore_h = 3.0;  // counterbore depth [mm]
joint_register_h    = 2.5;  // spigot/recess depth [mm]
shaft_d             = 26;   // shaft-rod diameter between joints [mm]

/* [Standoff (shaft rod length)] */
standoff_len   = 90;   // shaft-rod length between the two extension flanges [mm]

/* [Apple — split into a structural CORE (rigid) + a separate hollow BALL (soft material)] */
apple_d        = 45;   // ball outer Ø [mm]
apple_wall     = 3;    // ball shell wall [mm]
apple_stem_d   = 9;    // top stem Ø [mm]
apple_stem_h   = 16;   // top stem height [mm]
apple_grooves  = true; // grip grooves on the ball
core_d         = 20;   // CORE support-shaft Ø (runs the full inner height of the ball) [mm]
core_len       = 42;   // support-shaft length above the core flange [mm] (reaches near the ball top)
ball_fit_clear = 0.4;  // ball-socket radial clearance over the shaft [mm]
core_window_d  = 10;   // wiring side-window in the shaft (releases wires into the cavity) [mm]
core_window_z  = 34;   // window height above the core flange top [mm]
lock_pin_d     = 3.4;  // transverse lock hole through ball+shaft (Ø3 dowel = snap-in / M3 screw = bolted) [mm]
lock_pin_z     = 14;   // lock-hole height above the core flange top [mm] (low, away from the grip)

/* [Quality] */
$fn = 96;
eps = 0.02;

// ---- derived ----
ring_o_wall_r = ring_od / 2 + ring_clear;
ring_i_wall_r = ring_id / 2 - ring_clear;
cover_z       = base_h;                                   // cover sits on top of the base
cover_face_z  = cover_z + base_floor + sj_flange_t;       // top mating face of the cover's joint boss
ext_base_z    = cover_face_z;                             // extension bottom flange sits here
ext_top_z     = ext_base_z + sj_flange_t + standoff_len + aj_flange_t;  // extension top mating face
apple_base_z  = ext_top_z;                                // apple bottom flange sits here

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
    board_xy = [board_radius * cos(board_angle), board_radius * sin(board_angle)];
    conv_xy  = [conv_radius  * cos(conv_angle),  conv_radius  * sin(conv_angle)];

    difference() {
        union() {
            cylinder(h = base_h, d = base_d);                         // solid puck (centre = hub)
            for (s = [[1,1],[1,-1],[-1,1],[-1,-1]])                   // PCB standoffs in the board pocket
                translate([board_xy[0] + s[0]*(board_l/2 - 5),
                           board_xy[1] + s[1]*(board_w/2 - 5),
                           base_h - board_clear_h - board_standoff])
                    rotate([0,0,board_angle]) screw_boss();
        }
        flange_mount_cuts();

        // NeoPixel ring groove (concentric, top rim)
        translate([0, 0, base_h - ring_groove_h])
            difference() {
                cylinder(h = ring_groove_h + eps, r = ring_o_wall_r);
                translate([0,0,-eps]) cylinder(h = ring_groove_h + 3*eps, r = ring_i_wall_r);
            }
        // board compartment (open top)
        translate([board_xy[0], board_xy[1], base_h - (board_clear_h + board_standoff)])
            rotate([0,0,board_angle])
                translate([-board_l/2 - 2, -board_w/2 - 2, 0])
                    cube([board_l + 4, board_w + 4, board_clear_h + board_standoff + eps]);
        // converter compartment (open top)
        translate([conv_xy[0], conv_xy[1], base_h - (conv_clear_h + 2)])
            rotate([0,0,conv_angle])
                translate([-conv_l/2 - 2, -conv_w/2 - 2, 0])
                    cube([conv_l + 4, conv_w + 4, conv_clear_h + 2 + eps]);
        // wire channels (bridges)
        wire_channel([0,0],    conv_xy,  7);
        wire_channel(conv_xy,  board_xy, 7);
        wire_channel(board_xy, [ (ring_o_wall_r+ring_i_wall_r)/2, 0 ], 6);
        wire_channel(board_xy, [0,0],    6);
        // cover-fastening screw pilots (solid rim)
        for (i = [0 : cover_screw_n - 1])
            translate([cover_screw_pos(i)[0], cover_screw_pos(i)[1], base_h - cover_screw_depth])
                cylinder(h = cover_screw_depth + eps, d = cover_screw_pilot);
    }
}

// =====================================================================
//  (2) COVER : closes compartments, diffuses ring, BOLTED-FLANGE face for the extension
// =====================================================================
module base_cover() {
    cover_t   = base_floor;
    face_top  = cover_t + sj_flange_t;        // top mating face of the joint boss (local frame)
    difference() {
        union() {
            cylinder(h = cover_t, d = base_d);                            // lid plate
            translate([0, 0, cover_t - eps])                              // joint flange boss (lower side)
                cylinder(h = sj_flange_t, d = sj_flange_d);
        }
        translate([0, 0, -eps])                                           // central cable bore
            cylinder(h = face_top + 2*eps, d = cable_bore_d);
        joint_lower_cuts(face_top, sj_screw_n, sj_screw_bcd, sj_screw_a0, sj_register_d);  // pilots + recess for the extension
        // ring diffuser window
        translate([0, 0, cover_t - diffuser_t])
            difference() {
                cylinder(h = diffuser_t + eps, r = ring_o_wall_r);
                translate([0,0,-eps]) cylinder(h = diffuser_t + 3*eps, r = ring_i_wall_r);
            }
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
//  (3) EXTENSION : bottom flange (screws DOWN into cover) + shaft rod + top flange (apple screws in)
//      Local frame: bottom-flange base at z=0; top mating face at z = 2*flange_t + standoff_len.
// =====================================================================
module extension(len = standoff_len) {
    top_face = sj_flange_t + len + aj_flange_t;
    difference() {
        union() {
            cylinder(h = sj_flange_t, d = sj_flange_d);                               // bottom flange (-> cover)
            translate([0,0,sj_flange_t - eps]) cylinder(h = len + 2*eps, d = shaft_d); // shaft rod
            translate([0,0,top_face - aj_flange_t]) cylinder(h = aj_flange_t, d = aj_flange_d); // top flange (-> apple)
            joint_spigot(sj_register_d);                                              // centring spigot (down into cover)
        }
        translate([0,0,-joint_register_h - eps])                                      // cable bore through everything
            cylinder(h = top_face + joint_register_h + 2*eps, d = shaft_bore_d);
        joint_upper_cuts(sj_flange_t, sj_flange_t, sj_screw_n, sj_screw_bcd, sj_screw_a0);  // bottom flange screws -> cover
        joint_lower_cuts(top_face, aj_screw_n, aj_screw_bcd, aj_screw_a0, aj_register_d);   // top flange pilots <- apple
        // SIDE exit hole in the shaft rod
        translate([0, 0, sj_flange_t + shaft_exit_z]) rotate([0, 90, 0])
            cylinder(h = shaft_d + 2*eps, d = shaft_exit_d, center = true);
    }
}

// =====================================================================
//  (4) APPLE : bottom flange (screws DOWN into extension) + neck + hollow sphere + tip port
//      Local frame: bottom-flange base at z=0; sphere sits on the neck above the flange.
// =====================================================================
module grip_grooves(z0) {
    for (a = [-1.2, -0.6, 0, 0.6, 1.2])
        let(off = a * apple_d * 0.14,
            r   = sqrt(max(0, (apple_d * apple_d / 4) - off * off)))
            translate([0, 0, z0 + apple_d / 2 + off])
                rotate_extrude() translate([r, 0]) circle(r = 1.2);
}

// (4a) APPLE CORE — rigid: flange (bolts to extension) + full-length support shaft + wiring + lock hole.
module apple_core() {
    shaft_top = aj_flange_t + core_len;
    difference() {
        union() {
            cylinder(h = aj_flange_t, d = aj_flange_d);                               // flange (-> extension)
            translate([0,0,aj_flange_t - eps]) cylinder(h = core_len + eps, d = core_d); // full support shaft
            joint_spigot(aj_register_d);                                              // centring spigot (down)
        }
        translate([0,0,-joint_register_h - eps])                                      // wiring bore up the core
            cylinder(h = joint_register_h + shaft_top + eps, d = shaft_bore_d);
        joint_upper_cuts(aj_flange_t, aj_flange_t, aj_screw_n, aj_screw_bcd, aj_screw_a0);  // bolts -> extension
        translate([0,0,aj_flange_t + core_window_z]) rotate([0,90,0])                 // wiring side-window -> cavity
            cylinder(h = core_d + 2*eps, d = core_window_d, center = true);
        translate([0,0,aj_flange_t + lock_pin_z]) rotate([0,90,0])                    // transverse lock hole
            cylinder(h = core_d + 2*eps, d = lock_pin_d, center = true);
    }
}

// (4b) APPLE BALL — soft/separate material: hollow grippable sphere that sleeves the core shaft.
//      Lower shell is solid (for the lock pin); the cavity (sensor/actuator) is the annulus above it.
module apple_ball() {
    sph_z    = apple_d / 2;                       // sphere centre (ball base z=0 sits on the core flange top)
    z_cav    = lock_pin_z + 6;                    // cavity floor: keep the lower shell solid for the lock
    socket_d = core_d + 2 * ball_fit_clear;       // sleeve bore over the shaft
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
        translate([0,0,-eps]) cylinder(h = core_len + 1, d = socket_d);               // central socket (sleeves the shaft)
        translate([0,0,lock_pin_z]) rotate([0,90,0])                                  // transverse lock hole (aligns w/ core)
            cylinder(h = apple_d + 2*eps, d = lock_pin_d, center = true);
    }
}

// Apple core + ball assembled (preview / section).
module apple_assembled() {
    apple_core();
    translate([0,0,aj_flange_t]) apple_ball();
}

// =====================================================================
//  Optional: mock electronics to check fit (NOT for printing)
// =====================================================================
module electronics_mock() {
    color("green") translate([0,0,base_h - ring_groove_h])
        difference() { cylinder(h = ring_pcb_t, r = ring_od/2); cylinder(h = ring_pcb_t+eps, r = ring_id/2); }
    color("steelblue") translate([board_radius*cos(board_angle), board_radius*sin(board_angle), base_h - board_clear_h])
        rotate([0,0,board_angle]) translate([-board_l/2,-board_w/2,0]) cube([board_l, board_w, 1.6]);
    color("dimgray") translate([conv_radius*cos(conv_angle), conv_radius*sin(conv_angle), base_h - conv_clear_h])
        rotate([0,0,conv_angle]) translate([-conv_l/2,-conv_w/2,0]) cube([conv_l, conv_w, conv_clear_h*0.6]);
}

// =====================================================================
//  Assembly preview
// =====================================================================
module assembly() {
    color("silver")   base();
    %electronics_mock();
    color("gainsboro", 0.6) translate([0, 0, cover_z]) base_cover();
    color("dimgray")  translate([0, 0, ext_base_z]) extension(standoff_len);
    translate([0, 0, apple_base_z]) {
        color("slategray")      apple_core();                       // rigid structural core
        color("firebrick", 0.9) translate([0, 0, aj_flange_t]) apple_ball();  // soft grippable ball
    }
    %translate([0, 0, -1]) cylinder(h = 1, d = base_d + 6);   // robot flange face marker
}

// ---- render the selected part ----
if      (part == "base")             base();
else if (part == "cover")            base_cover();
else if (part == "extension")        extension(standoff_len);
else if (part == "apple_core")       apple_core();
else if (part == "apple_ball")       apple_ball();
else if (part == "apple")            apple_assembled();
else if (part == "electronics_mock") electronics_mock();
else if (part == "apple_section")    // half-cut: support shaft sleeved by the ball + cavity + lock pin
    difference() { apple_assembled(); translate([0, -base_d, -20]) cube([base_d, 2*base_d, 200]); }
else if (part == "section")          // half-cut of the whole stack (shows the full cable path)
    difference() { assembly(); translate([0, -base_d, -20]) cube([base_d, 2*base_d, 320]); }
else                                 assembly();
