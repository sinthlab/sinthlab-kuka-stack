// =====================================================================
//  Apple-pluck end-effector for the KUKA LBR iiwa7   (parametric DRAFT v0.1)
//  sinthlab-kuka-stack / end_effector_design
//
//  A modular, 3D-printable tool that bolts to the iiwa7 tool flange and
//  presents a compliant "apple" for the monkey to pull. Three printed parts
//  couple together:
//
//        [ robot flange ]
//              |  bolts (M6 x N on PCD)
//        (1) ADAPTER      female socket ^
//              |  slip stud + M4 set screw
//        (2) EXTENSION    sets the standoff length
//              |  slip stud + M4 set screw
//        (3) APPLE        hollow sphere, silicone-coatable
//
//  The "pluck" is the CABINET's Cartesian-impedance displacement (see the
//  control stack), so the apple is rigidly attached — no detach mechanism.
//  Swap apples / extensions to change size, texture, and standoff.
//
//  Finishing: print hollow, then brush/dip 2 layers of food-safe silicone
//  for a compliant, grippable, cleanable skin (cf. Barra et al. 2019).
//
//  !!!  VERIFY the "Robot flange" section against YOUR iiwa7 R800 datasheet /
//       ISO 9409-1-50-4-M6 before printing the adapter. Values are NOMINAL.
//  Export one part at a time, e.g.:
//     openscad -D 'part="apple"'   -o apple.stl     apple_pluck_end_effector.scad
//     openscad -D 'part="adapter"' -o adapter.stl   apple_pluck_end_effector.scad
// =====================================================================

part = "assembly"; // [assembly, adapter, extension, apple]

/* [Robot flange  — VERIFY vs iiwa7 datasheet] */
flange_plate_d    = 63;    // adapter outer diameter [mm]
flange_plate_t    = 8;     // adapter plate thickness [mm]
flange_pcd        = 50;    // bolt pitch-circle diameter (ISO 9409-1-50) [mm]
flange_bolt_n     = 4;     // number of mounting bolts
flange_bolt_clear = 6.6;   // M6 clearance hole [mm]
flange_cbore_d    = 11;    // counterbore for M6 socket-head screw [mm]
flange_cbore_h    = 6.5;   // counterbore depth [mm]
flange_center_d   = 31.6;  // central recess for robot centering boss (~Ø31.5) [mm]
flange_center_h   = 4;     // recess depth [mm]
flange_pin_d      = 6.2;   // locating-pin clearance hole [mm] (set 0 to disable)
flange_pin_pcd    = 50;    // pin location circle diameter [mm]  (VERIFY)
flange_pin_offset = 45;    // pin angle from first bolt [deg]    (VERIFY)

/* [Coupler  — shared stud/socket between parts] */
coupler_bore   = 14;   // socket bore diameter [mm]
coupler_clear  = 0.3;  // slip clearance (stud Ø = bore - 2*clear) [mm]
coupler_engage = 22;   // stud length / socket depth [mm]
coupler_wall   = 4;    // wall thickness around the socket [mm]
setscrew_d     = 4.3;  // M4 radial set-screw clearance [mm]

/* [Standoff] */
standoff_len   = 90;   // flange-face -> apple-base distance (extension length) [mm]

/* [Apple / fruit] */
apple_d        = 45;   // apple outer diameter [mm]
apple_wall     = 3;    // shell wall thickness (printable + silicone) [mm]
apple_stem_d   = 9;    // top stem diameter [mm]
apple_stem_h   = 16;   // top stem height [mm]
apple_grooves  = true; // shallow grip grooves around the apple

/* [Quality] */
$fn = 96;
eps = 0.02;

// ---- derived ----
stud_d = coupler_bore - 2 * coupler_clear;   // male stud diameter
boss_d = coupler_bore + 2 * coupler_wall;    // outer diameter of socket bosses / rods

// =====================================================================
//  Shared coupler features
// =====================================================================

// Male stud built upward from z=0, chamfered at the insertion tip (z=0).
module male_stud(d = stud_d, h = coupler_engage) {
    translate([0, 0, 1]) cylinder(h = h - 1, d = d);
    cylinder(h = 1, d1 = d - 2, d2 = d);     // lead-in chamfer
}

// Female socket as a cut volume: bore + one radial M4 set-screw at mid-depth.
module socket_cut(bore = coupler_bore, depth = coupler_engage) {
    translate([0, 0, -eps]) cylinder(h = depth + 2 * eps, d = bore);
    translate([0, 0, depth / 2]) rotate([0, 90, 0])
        cylinder(h = boss_d, d = setscrew_d, center = true);
}

// =====================================================================
//  (1) Flange adapter
//      Robot face = bottom (z=0); tool side = top (+Z) with a female socket.
// =====================================================================
module adapter() {
    socket_z0 = flange_plate_t + coupler_wall;   // socket floor
    difference() {
        union() {
            cylinder(h = flange_plate_t, d = flange_plate_d);                  // plate
            translate([0, 0, flange_plate_t - eps])
                cylinder(h = coupler_wall + coupler_engage, d = boss_d);       // socket boss
        }
        // centering recess on the robot face (bottom)
        translate([0, 0, -eps]) cylinder(h = flange_center_h, d = flange_center_d);
        // mounting bolts: clearance + counterbore from the robot face
        for (i = [0 : flange_bolt_n - 1]) rotate([0, 0, i * 360 / flange_bolt_n])
            translate([flange_pcd / 2, 0, 0]) {
                translate([0, 0, -eps]) cylinder(h = flange_plate_t + 2 * eps, d = flange_bolt_clear);
                translate([0, 0, -eps]) cylinder(h = flange_cbore_h, d = flange_cbore_d);
            }
        // locating pin
        if (flange_pin_d > 0)
            rotate([0, 0, flange_pin_offset]) translate([flange_pin_pcd / 2, 0, -eps])
                cylinder(h = flange_plate_t + 2 * eps, d = flange_pin_d);
        // tool-side female socket (opens upward)
        translate([0, 0, socket_z0]) socket_cut();
    }
}

// =====================================================================
//  (2) Standoff extension : male stud (down) + female socket (up)
// =====================================================================
module extension(len = standoff_len) {
    union() {
        translate([0, 0, -coupler_engage]) male_stud();          // male, inserts down
        difference() {
            cylinder(h = len, d = boss_d);                       // standoff rod
            translate([0, 0, len - coupler_engage]) socket_cut(); // female, opens up
        }
    }
}

// =====================================================================
//  (3) Apple : hollow sphere + male stud (down) + top stem + grip grooves
// =====================================================================
module grip_grooves() {
    for (a = [-1.2, -0.6, 0, 0.6, 1.2])
        let(off = a * apple_d * 0.14,
            r   = sqrt(max(0, (apple_d * apple_d / 4) - off * off)))
            translate([0, 0, apple_d / 2 + off])
                rotate_extrude() translate([r, 0]) circle(r = 1.2);
}

module apple() {
    union() {
        difference() {
            union() {
                translate([0, 0, apple_d / 2]) sphere(d = apple_d);                       // body
                translate([0, 0, apple_d - apple_wall])
                    cylinder(h = apple_stem_h + apple_wall, d = apple_stem_d);            // top stem
            }
            translate([0, 0, apple_d / 2]) sphere(d = apple_d - 2 * apple_wall);          // cavity
            if (apple_grooves) grip_grooves();
            // vent hole down through the stem into the cavity (for dip-coating / printing)
            translate([0, 0, apple_d - 2 * apple_wall])
                cylinder(h = apple_stem_h + 2 * apple_wall + eps, d = 3);
        }
        // solid stud + internal post (added AFTER hollowing => bonds to the shell)
        translate([0, 0, -coupler_engage])
            male_stud(h = coupler_engage + apple_wall + apple_d * 0.30);
    }
}

// =====================================================================
//  Assembly preview
// =====================================================================
module assembly() {
    ext_base = flange_plate_t + coupler_wall + coupler_engage;   // adapter boss top
    color("silver")  adapter();
    color("dimgray") translate([0, 0, ext_base]) extension(standoff_len);
    color("firebrick", 0.9) translate([0, 0, ext_base + standoff_len]) apple();
    // translucent marker for the robot flange face (z=0 plane)
    %translate([0, 0, -1]) cylinder(h = 1, d = flange_plate_d + 6);
}

// ---- render the selected part ----
if      (part == "adapter")   adapter();
else if (part == "extension") extension(standoff_len);
else if (part == "apple")     apple();
else                          assembly();
