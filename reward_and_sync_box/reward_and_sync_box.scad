// Reward and sync box: front and rear panels of the Hammond RM2U1908SBK, with every cut-out, plus an
// assembly view of the box with the Beckhoff sync stack inside; and a 3D-printed stand-in box that
// holds the same parts. The one place the holes are defined.
//
// Panels: 415.30 x 84.53 x 1.6 mm aluminium (Hammond drawing RM2U1908SBK, section CC-CC).
// Coordinates: mm from the panel's bottom-left corner, SEEN FROM OUTSIDE; the outside face is +Z.
//
// Exports (from this folder):
//   openscad -D 'part="front"'     -o front_panel.stl          reward_and_sync_box.scad
//   openscad -D 'part="rear"'      -o rear_panel.stl           reward_and_sync_box.scad
//   openscad -D 'part="assembly"'  -o reward_and_sync_box.stl  reward_and_sync_box.scad
//   openscad -D 'part="print_box"' -o print_box.stl            reward_and_sync_box.scad
//   openscad -D 'part="print_lid"' -o print_lid.stl            reward_and_sync_box.scad

part = "assembly";  // front | rear | assembly | print_box | print_lid | print_assembly

$fn = 96;

// ---- enclosure -------------------------------------------------------------------------------------
PANEL_W = 415.30;
PANEL_H = 84.53;
PANEL_T = 1.6;            // 0.064" 5052-H32
MARGIN_X = 15;            // keep holes clear of the frame lips and panel screws
MARGIN_Y = 8;
BOX_W = 421.65;           // extruded frame, outside
BOX_H = 88.14;
BOX_D = 203.20;
EAR_W = 482.61;           // with the rack ears

// ---- cut-outs: ["round", dia] | ["d", dia, across_flat] | ["keyed", dia, to_notch_bottom, notch_w]
//                | ["neutrik_d", dia] ------------------------------------------------------------------
TOGGLE   = ["round", 6.5];              // C&K 7101SYZQE: 1/4-40 bushing
LED      = ["round", 12.7];             // VCC 1091M5-24V: 0.50" cut-out
BNC      = ["d", 9.90, 8.84];           // TE 5227726-1 isolated BNC: single-D (Protocase cut-out library)
M16      = ["keyed", 18.0, 19.1, 2.8];  // Amphenol T3263000U: C091 catalog p.11, notch down
BREAKER  = ["round", 16.0];             // TE W28-XQ1A-1: snap-in, 0.625-0.635"
DC_JACK  = ["round", 8.0];              // Switchcraft 721A: 5/16"-32 bushing
ETHERCON = ["neutrik_d", 24.0];         // Neutrik NE8FDP (D-size): Ø24 + 2 x Ø3.2 at (-9.5, +12), (+9.5, -12)

// [label, x, y, cut-out, part]
// front: the reward controls and inputs, then the sync stack's EtherCAT in and TTL outputs
FRONT = [
    ["POWER",        40,  38, TOGGLE,   "C&K 7101SYZQE"],
    ["",             40,  64, LED,      "VCC 1091M5-24V"],
    ["FLUSH",        80,  38, TOGGLE,   "C&K 7101SYZQE"],
    ["",             80,  64, LED,      "VCC 1091M5-24V"],
    ["NSP",          125, 42, BNC,      "TE 5227726-1"],
    ["PEDAL",        155, 42, BNC,      "TE 5227726-1"],
    ["BUTTON",       185, 42, BNC,      "TE 5227726-1"],
    ["ETHERCAT X65", 240, 42, ETHERCON, "Neutrik NE8FDP"],
    ["SYNC 1",       280, 42, BNC,      "TE 5227726-1"],
    ["SYNC 2",       310, 42, BNC,      "TE 5227726-1"],
    ["SYNC 3",       340, 42, BNC,      "TE 5227726-1"],
    ["SYNC 4",       370, 42, BNC,      "TE 5227726-1"],
];
// rear: what stays plugged in — the 24 V input, a breaker for each branch, and the valve
REAR = [
    ["24 V IN",       35,  42, DC_JACK,  "Switchcraft 721A"],
    ["BREAKER VALVE", 90,  42, BREAKER,  "TE W28-XQ1A-1"],
    ["BREAKER SYNC",  150, 42, BREAKER,  "TE W28-XQ1A-1"],
    ["VALVE",         210, 42, M16,      "Amphenol T3263000U"],
];
// Beckhoff stack on a DIN rail lying on the box floor (box coordinates, see assembly()):
// EK1100 44 + EL9505 12 + EL2124 12 + EL9011 ~12 mm along the rail, 100 mm across, 68 mm tall.
STACK_AT = [290, 45];     // box x, y of the stack's corner nearest the front-left: behind the SYNC jacks
STACK = [80, 100, 68];
RAIL = [120, 35, 7.5];
LABEL_Y = 16;
LABEL_SIZE = 4;
ENGRAVE = 0.3;            // label depth
FONT = "Liberation Sans:style=Bold";

// ---- 2D ----------------------------------------------------------------------------------------------
module cutout(c) {
    if (c[0] == "round") circle(d = c[1]);
    else if (c[0] == "d")
        intersection() {
            circle(d = c[1]);
            translate([-c[1], -c[1]]) square([2 * c[1], c[1] + c[2] - c[1] / 2]);   // flat on top
        }
    else if (c[0] == "keyed")
        union() {
            circle(d = c[1]);
            translate([-c[3] / 2, c[1] / 2 - c[2]]) square([c[3], c[2] - c[1] / 2]);  // notch at bottom
        }
    else if (c[0] == "neutrik_d")       // seen from the front: screws top-left and bottom-right
        union() {
            circle(d = c[1]);
            for (p = [[-9.5, 12], [9.5, -12]]) translate(p) circle(d = 3.2);
        }
}

module holes(items) { for (it = items) translate([it[1], it[2]]) cutout(it[3]); }

module labels(items, y = LABEL_Y) {
    for (it = items) if (it[0] != "")
        translate([it[1], y]) text(it[0], size = LABEL_SIZE, font = FONT, halign = "center");
}

// ---- 2U panels and assembly --------------------------------------------------------------------------
module panel_3d(items) {
    difference() {
        cube([PANEL_W, PANEL_H, PANEL_T]);
        translate([0, 0, -1]) linear_extrude(PANEL_T + 2) holes(items);
        translate([0, 0, PANEL_T - ENGRAVE]) linear_extrude(ENGRAVE + 0.01) labels(items);
    }
}

// Rough bodies behind the panel, to check clearances in the assembly (not to scale in detail).
// They start 0.2 mm inside the panel so the assembly is one valid solid.
module parts_behind(items) {
    for (it = items) translate([it[1], it[2], 0.2]) mirror([0, 0, 1]) {
        c = it[3];
        if (c == TOGGLE)   translate([-6.5, -4.5, 0]) cube([13, 9, 12]);
        if (c == LED)      cylinder(d = 12, h = 20);
        if (c == BNC)      cylinder(d = 11, h = 18);
        if (c == M16)      cylinder(d = 18, h = 30);
        if (c == BREAKER)  cylinder(d = 15, h = 38);
        if (c == DC_JACK)  cylinder(d = 9, h = 20);
        if (c == ETHERCON) translate([-13.85, -16.35, 0]) cube([27.7, 32.7, 35]);
    }
}

module beckhoff_stack() {   // DIN rail on the floor, modules standing on it
    color("silver") translate([STACK_AT[0] - (RAIL[0] - STACK[0]) / 2, STACK_AT[1] + (STACK[1] - RAIL[1]) / 2, 1.5])
        cube(RAIL);
    color("lightgray") translate([STACK_AT[0], STACK_AT[1], 1.5 + RAIL[2] - 0.2]) cube(STACK);
}

module assembly() {
    x0 = (BOX_W - PANEL_W) / 2;
    z0 = (BOX_H - PANEL_H) / 2;
    color("dimgray") difference() {                                         // frame, lid and floor
        cube([BOX_W, BOX_D, BOX_H]);
        translate([1.8, -1, 1.8]) cube([BOX_W - 3.6, BOX_D + 2, BOX_H - 3.6]);
    }
    ear = (EAR_W - BOX_W) / 2;
    color("black") for (s = [0, 1])                                          // rack ears
        translate([s ? BOX_W - 0.5 : -ear, -3.2, 0]) cube([ear + 0.5, 3.7, BOX_H]);
    // front panel: outside face toward -Y
    color("silver") translate([x0, PANEL_T, z0]) rotate([90, 0, 0]) panel_3d(FRONT);
    color("orange") translate([x0, PANEL_T, z0]) rotate([90, 0, 0]) parts_behind(FRONT);
    // rear panel: seen from outside (+Y), so mirrored left-right in box coordinates
    color("silver") translate([x0 + PANEL_W, BOX_D - PANEL_T, z0]) rotate([90, 0, 180]) panel_3d(REAR);
    color("orange") translate([x0 + PANEL_W, BOX_D - PANEL_T, z0]) rotate([90, 0, 180]) parts_behind(REAR);
    color("steelblue") translate([BOX_W / 2 - 22, 60, 1.5]) cube([44, 58, 25]);   // SSR on the floor
    beckhoff_stack();
}

// ---- 3D-printed stand-in box (reward and sync) -------------------------------------------------------
// Open-top box + screw-on lid, 180 x 170 x 85 mm (fits an 11 x 11 in bed). Holds everything the 2U box
// does: the reward parts, the SSR and the Beckhoff stack on its DIN rail.
//   front wall: upper row POWER + LED, FLUSH + LED, NSP, PEDAL, BUTTON; lower row ETHERCAT X65, SYNC 1-4
//   rear wall:  24 V IN, BREAKER VALVE, BREAKER SYNC, VALVE
//   floor:      SSR (2 x M4), DIN rail (2 x M4)
// The lid screws down with 4 x M3 x 10 self-tapping screws into the corner posts.
PB_W = 180;  PB_D = 170;  PB_H = 85;    // box outside (without lid or ears)
PB_T = 2.5;                            // walls, floor, lid
PB_POST = 8;                           // corner posts, square
PB_THIN = 1.5;                         // wall left at each breaker: the W28 snaps into 0.81-1.57 mm
SSR_HOLES = 47.6;                      // Crydom panel-mount SSR hole spacing

// walls: [label, u, v, cut-out, label v], u along the wall seen from outside, v = height above the bottom
PB_FRONT = [
    ["POWER",        22,  56, TOGGLE,   44],
    ["",             22,  71, LED,      0],
    ["FLUSH",        52,  56, TOGGLE,   44],
    ["",             52,  71, LED,      0],
    ["NSP",          90,  56, BNC,      44],
    ["PEDAL",        120, 56, BNC,      44],
    ["BUTTON",       150, 56, BNC,      44],
    ["ETHERCAT X65", 30,  26, ETHERCON, 6],
    ["SYNC 1",       70,  26, BNC,      6],
    ["SYNC 2",       100, 26, BNC,      6],
    ["SYNC 3",       130, 26, BNC,      6],
    ["SYNC 4",       160, 26, BNC,      6],
];
PB_REAR = [
    ["24 V IN",       20,  42, DC_JACK,  22],
    ["BREAKER VALVE", 57,  42, BREAKER,  22],
    ["BREAKER SYNC",  108, 42, BREAKER,  22],
    ["VALVE",         155, 42, M16,      22],
];
SSR_AT = [32, 74];                     // SSR centre on the floor
PB_STACK_AT = [75, 40];                // Beckhoff stack corner; rail runs front to back
PB_RAIL_HOLES = [40, 120];             // DIN rail screw holes, y, on the rail's centre line

module wall_labels(items) {
    for (it = items) if (it[0] != "")
        translate([it[1], it[4]]) text(it[0], size = LABEL_SIZE, font = FONT, halign = "center");
}

// Place a 2D wall drawing on a face, seen from outside: front (y = 0), rear (y = PB_D).
module on_front(depth) {
    translate([0, -0.01, 0]) rotate([90, 0, 0]) mirror([0, 0, 1]) linear_extrude(depth + 0.02) children();
}
module on_rear(depth) {
    translate([PB_W, PB_D + 0.01, 0]) rotate([90, 0, 180]) mirror([0, 0, 1]) linear_extrude(depth + 0.02) children();
}

module print_box() {
    difference() {
        union() {
            difference() {
                cube([PB_W, PB_D, PB_H]);
                translate([PB_T, PB_T, PB_T]) cube([PB_W - 2 * PB_T, PB_D - 2 * PB_T, PB_H]);
            }
            for (x = [PB_T, PB_W - PB_T - PB_POST], y = [PB_T, PB_D - PB_T - PB_POST])
                translate([x, y, 0]) cube([PB_POST, PB_POST, PB_H]);
        }
        // pilot holes for the lid screws
        for (x = [PB_T + PB_POST / 2, PB_W - PB_T - PB_POST / 2], y = [PB_T + PB_POST / 2, PB_D - PB_T - PB_POST / 2])
            translate([x, y, PB_H - 12]) cylinder(d = 2.8, h = 13);
        // front and rear cut-outs, through the wall
        on_front(PB_T + 1) holes(PB_FRONT);
        on_rear(PB_T + 1) holes(PB_REAR);
        // thin the rear wall to PB_THIN around each breaker, from inside
        for (it = PB_REAR) if (it[3] == BREAKER)
            translate([PB_W - it[1], PB_D - PB_T - 0.01, it[2]]) rotate([-90, 0, 0])
                cylinder(d = 26, h = PB_T - PB_THIN + 0.01);
        // engraved labels
        on_front(ENGRAVE) wall_labels(PB_FRONT);
        on_rear(ENGRAVE) wall_labels(PB_REAR);
        // SSR and DIN rail mounting holes
        for (s = [-1, 1]) translate([SSR_AT[0], SSR_AT[1] + s * SSR_HOLES / 2, -1]) cylinder(d = 4.3, h = PB_T + 2);
        for (y = PB_RAIL_HOLES) translate([PB_STACK_AT[0] + STACK[1] / 2, y, -1]) cylinder(d = 4.3, h = PB_T + 2);
    }
}

module print_lid() {   // plain cover with the four screw holes
    difference() {
        cube([PB_W, PB_D, PB_T]);
        for (x = [PB_T + PB_POST / 2, PB_W - PB_T - PB_POST / 2], y = [PB_T + PB_POST / 2, PB_D - PB_T - PB_POST / 2])
            translate([x, y, -1]) cylinder(d = 3.4, h = PB_T + 2);
    }
}

module print_assembly() {
    color("lightgray") print_box();
    color("white", 0.6) translate([0, 0, PB_H]) print_lid();
    color("steelblue") translate([SSR_AT[0] - 22, SSR_AT[1] - 29, PB_T - 0.2]) cube([44, 58, 25]);
    // Beckhoff stack, rail front to back: modules 100 mm across (x), 80 mm along the rail (y)
    color("silver") translate([PB_STACK_AT[0] + STACK[1] / 2 - RAIL[1] / 2, 30, PB_T - 0.2]) cube([RAIL[1], 100, RAIL[2]]);
    color("orange") translate([PB_STACK_AT[0], PB_STACK_AT[1], PB_T + RAIL[2] - 0.4]) cube([STACK[1], STACK[0], STACK[2]]);
}

// ---- checks ------------------------------------------------------------------------------------------
function reach(c) = c[0] == "keyed" ? max(c[1] / 2, c[2] - c[1] / 2)
                  : c[0] == "neutrik_d" ? 12 + 1.6 : c[1] / 2;
module check(items, name, w, h, mx, my) {
    for (it = items) {
        r = reach(it[3]);
        assert(it[1] - r >= mx && it[1] + r <= w - mx, str(name, " ", it[0], ": too near a side"));
        assert(it[2] - r >= my && it[2] + r <= h - my, str(name, " ", it[0], ": too near top/bottom"));
    }
}
check(FRONT, "front", PANEL_W, PANEL_H, MARGIN_X, MARGIN_Y);
check(REAR, "rear", PANEL_W, PANEL_H, MARGIN_X, MARGIN_Y);
check(PB_FRONT, "print front", PB_W, PB_H, PB_T + PB_POST, PB_T + 2);
check(PB_REAR, "print rear", PB_W, PB_H, PB_T + PB_POST, PB_T + 2);
assert(PB_W <= 279 && PB_D <= 279, "print box larger than an 11 x 11 in bed");
assert(PB_T + RAIL[2] + STACK[2] < PB_H - 2, "Beckhoff stack taller than the print box");

// ---- output ------------------------------------------------------------------------------------------
if (part == "front") panel_3d(FRONT);
else if (part == "rear") panel_3d(REAR);
else if (part == "assembly") assembly();
else if (part == "print_box") print_box();
else if (part == "print_lid") print_lid();
else if (part == "print_assembly") print_assembly();
else assert(false, str("unknown part: ", part));
