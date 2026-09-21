// =====================================================================
//  Apple-pluck end-effector for the KUKA LBR iiwa7   (parametric DRAFT v0.14)
//  sinthlab-kuka-stack / end_effector_design
//
//  ELECTRONICS HUB that bolts to the iiwa7 *media flange (electric)*, routes its
//  power/data wiring, drives a NeoPixel cue, and carries the control electronics.
//
//  v0.7 changes (all driven by the v0.6 build):
//    * Ø225 -> Ø188. The 95x75 optocoupler is GONE (a floating dry contact / RS-422 link
//      needs none), and it was the only thing forcing Ø225. The base is now sized by the
//      ring groove + rim, with ~10.5 mm of rim outboard of the groove.
//    * The base is now TWO STACKED TIERS. Shrinking the diameter cost board area, so the
//      electronics split across two pucks that bolt together (4x M3).
//    * Ring groove grown Ø157/145 -> Ø165/152 (+~3 LEDs of arc) — four butted quarter-rings
//      did not close in the v0.6 groove.
//    * Tobsun pocket 60x55 -> 70x65 and level shifter 26x18 -> 36x28 (both were too small).
//    * New pockets: RS-422 transceiver (MIKROE-2821) and MPRLS pressure sensor.
//    * PERIMETER CABLE TRENCH on tier 1 — somewhere for the ~44 cm tool-connector bundle
//      to turn and be dressed, which v0.6 had no provision for at all.
//    * The adjustable shaft is GONE. The apple core base + shaft are now ONE PETG part
//      (the detent-pin joint printed finicky and wobbled).
//
//        [ iiwa7 media flange (electric) ]   8x M6 on a Ø51 pitch circle, 24V + data
//        (0) FLANGE PLATE  thick adapter disc: takes the 8x M6 (heads buried flush) + the Ø30 cable bore.
//                          Tier 1 bolts onto ITS top face with 4x M3.
//                 |  cable bundle THROUGH the Ø30 tier-1 centre
//        (1a) BASE TIER 1  Ø30 central bore (the flange bundle lands here) + PERIMETER CABLE TRENCH.
//              ├─ Metro M4 AirLift + power adapter   92 x 50   ALL VELCRO'D — no screw bosses.
//              ├─ RS-422 transceiver (MIKROE-2821)   45 x 28
//              └─ MPRLS pressure sensor              20 x 19
//        (1b) BASE TIER 2  stacks on tier 1 (4x M3); Ø20 bore — only ring/apple wiring goes higher.
//              ├─ Tobsun 24V->5V converter           70 x 65   <- deepest pocket, sets tier-2 height
//              ├─ Level shifter (ring data)          36 x 28
//              └─ DRV2605L haptic driver             28 x 20
//        (2) COVER       closes tier 2 · seats the NeoPixel ring in a groove on its TOP
//                        (opaque, same material as the base) · 4x M3 down into tier 2
//                 ▼ BOLTED FLANGE: apple STEM screws down into the cover (3x M3 + spigot)
//        (3) APPLE = (3a) STEM      ONE PETG part: cover flange + shaft + armature flange
//                  + (3b) TPU BALL  LOWER cup FUSED to the stem (dual-material print); solid cap
//                                   embeds the armature flange; OPEN top loads the ERM + sensor
//                  + (3c) TPU CAP   press-fit dome that closes the ball; wires / FSR tail / tube exit the bore
//        ( + ) CLEAR CASING BOX   5-sided clear box (OPEN on the flange side) shrouding the whole
//                                  electronics stack; top covers the ring, clamps 3x M3 into the cover
//
//  Board layout is VERIFIED numerically (no overlaps, every pocket corner inside the rim, every
//  pocket clear of the tier bore, every screw clear of every pocket). Re-run that check after ANY
//  size change — the packing has only 2-5 mm of margin in places.
//
//  !!!  VERIFY every flange/component dimension vs the real datasheets before printing.
//  Export one part at a time, e.g.:
//     openscad -D 'part="base_tier1"'  -o base_tier1.stl  apple_pluck_end_effector.scad
// =====================================================================

part = "assembly"; // [assembly, flange_plate, base_tier1, base_tier2, cover, apple_stem, apple_ball, apple_cap, apple, apple_section, section, electronics_mock, casing]

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
plate_d        = 124;  // adapter plate Ø [mm]. Grown from 110: the plate is what SUPPORTS tier 1's
                       // floor, and beyond its edge the floor is an unsupported cantilever out to
                       // r 94. Ø110 left 39 mm of overhang; Ø124 leaves 32 mm.
plate_t        = 16;   // plate thickness [mm] = mount wall 8 + washer seat 2 + M6 head 6 (head sits flush)
plate_screw_n  = 6;    // M3 screws holding tier 1 down onto the plate (was 4 — more, and evenly
                       // spread, so the joint that carries the whole tool is not held on four points)
plate_screw_bcd = 100; // their bolt-circle Ø [mm]. Grown from 80: the joint reacts the apple's moment
                       // as a couple across this circle, so a wider circle is directly less force per
                       // screw (-20%). Still clear of the M6 washer seats (out to r 32.5), 12.8 mm
                       // inside the Ø124 plate rim, and clear of the nearest feed channel.
plate_screw_a0 = 15;   // first-screw angle [deg]. 15 threads the six between BOTH the feed channels
                       // at 30/210 (4.7 mm clear) and the tier-2 drops at 0/90/180/270 (16.2 mm), so
                       // nothing lands a wire on top of a screw head. At a0=0 the drops at 0 and 180
                       // came down 0.25 mm inside the screw well.
plate_screw_depth = 6; // M3 insert bore depth into the plate top [mm] (takes a 4 mm insert)
plate_head_z   = 10;   // M3 head seat height above the tier-1 bottom [mm]. DOUBLED from 5: this is the
                       // thickness of printed material the head bears on, and it was the thinnest
                       // section in the whole load path — 5 mm of PETG with a Ø6.5 well directly above
                       // it. Now 10 mm below the head, 4 mm of well above. Needs M3x16 (10 through the
                       // floor + 6 into the plate insert), NOT the M3x10 used elsewhere.

/* [Central cabling] */
cable_bore_d   = 30;   // BASE central pass-through for the media-flange bundle [mm] (power/data out of the robot)
tier2_bore_d   = 20;   // TIER-2 central bore [mm] — only ring + apple wiring passes above tier 1, so
                       // it is 10 mm smaller than tier 1's. That reclaimed radius is what lets the
                       // 70x65 Tobsun fit inside Ø188 at all.
apple_bore_d   = 16;   // COVER + apple-stem central bore [mm] (only the apple's own wiring runs up here,
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

/* [Base electronics hub — TWO STACKED TIERS, ONE OPEN COMPARTMENT EACH]
    Ø188 is set by the RING: groove outer r = 83.5, + rim + wall = r 94. The old Ø225 existed only to
    swallow the 95x75 optocoupler, which is gone.

    v0.8: the per-board pockets are GONE. Each tier is now ONE open compartment with a flat floor, and
    every board is velcro'd wherever it fits — nothing is keyed to a rectangle any more, so boards can
    be swapped, moved or added without touching the CAD. What is left standing in the compartment is
    only what has to be: four screw pillars per tier. Wiring lives BELOW board level, in channels sunk
    into the floor (see the trench block), so a board can be stuck down on top of a cable run. */
base_d       = 188;  // base outer Ø [mm] — BOTH tiers. HELD at 188 through the v0.14 strength work.
                     // The 4 mm hub wall makes the Tobsun the binding part: at a Ø20 bore it needs
                     // r 86.4 against the 88 available, so 1.6 mm of margin. That is real but thin —
                     // any further growth of hub_wall, conv_l/w or tier2_bore_d and the base has to
                     // grow with it, or the Tobsun and the Metro have to swap tiers.
base1_floor  = 14;   // TIER-1 floor thickness [mm]. This is the part that bolts to the flange plate,
                     // and the v0.6 prototype BROKE at that joint. It is thicker than tier 2's on
                     // purpose: a 20 N pull at the apple is 4.1 N.m here, and while the M3s only see
                     // ~17 N each, the printed material around them is what fails. 14 also leaves
                     // 8 mm of solid floor where a 6 mm cable channel crosses (was 4).
base2_floor  = 10;   // TIER-2 floor thickness [mm] — carries far less, no need to pay the height
base_floor   = base1_floor;  // legacy alias used by the fit mock
comp1_depth  = 19;   // tier-1 compartment depth below its top face [mm] (tallest = Metro, ~19)
comp2_depth  = 22;   // tier-2 compartment depth below its top face [mm] (tallest = Tobsun, 22)
comp_ro      = 88;   // compartment outer radius [mm] — leaves a 6 mm rim wall at Ø188
base_wall    = 3;    // structural wall / bridge thickness [mm]
hub_wall     = 4;    // material between a tier's central bore and the open compartment [mm]. Raised
                     // from 3: the bore ports now take 39-55% of this tube's circumference.
pocket_clear = 1.0;  // clearance allowance used by the layout check and the fit mock

/* [Screw pillars — the only obstructions left in the compartments]
    With the pockets gone there is no solid top face to put a heat-set insert in, so each screw
    position gets a Ø12 pillar standing from the compartment floor to the tier top. Four per tier.
    They are placed clear of every reference board position (see check_layout.py). */
pillar_d     = 12;   // screw-pillar Ø [mm]
fillet_r     = 3;    // fillet radius where the compartment walls meet the floor, and at pillar bases.
                     // Sharp internal corners are where a flat-printed part cracks along its layers.

/* [Tier 1 <-> Tier 2 fixing — a proper bolt circle]
    v0.8 had these at 0/180/225/315, which is a trapezoid: the clamp load was lopsided and the joint
    could rock about the two close-together screws. They are now a symmetric square at 45/135/225/315.
    r = 75 is the smallest radius that clears a Metro-sized board: a 94 x 52 pocket tangent to the Ø30
    bore spans |x| <= 47, so a pillar at r = 75 sits at |x| = 53 — outside it by 6 mm. Anything nearer
    the centre lands underneath the Metro no matter how it is clocked. */
tier_screw_bcd = 150;   // Ø [mm] -> r 75
tier_screw_a0  = 45;    // first pillar angle [deg]
tier_screw_n   = 4;
tier_screw_pos = [for (i = [0 : tier_screw_n - 1])
                    let(a = tier_screw_a0 + i * 360 / tier_screw_n)
                      [tier_screw_bcd/2 * cos(a), tier_screw_bcd/2 * sin(a)]];
tier_screw_depth = 14;  // heat-set insert bore depth into a tier-1 pillar [mm]
tier_head_z      = 5;   // head seat height above the tier-2 floor bottom [mm] (head ends up recessed)

/* [Wire pass-throughs — how anything actually gets OUT of a bore, or between tiers]
    Merging the pockets into one open compartment left each central bore walled in by a 3 mm tube
    running the full height of its tier: wire could come up the bore but had no way into the
    compartment, and nothing could cross between tiers except back down the middle. Two sets of holes
    fix that.

    1. RADIAL BORE PORTS — cut through the hub wall with their bottoms flush to the compartment floor,
       so wire leaves the bore already lying flat and stays flat. Tier 1 gets 4 (the flange bundle
       fans out here). Tier 2 gets 2 large ones: that is the path for the apple's ERM leads, the
       pressure tube and the ring data, which all arrive up the riser and have to reach chips sitting
       out on the floor.
    2. TIER DROPS — vertical holes through tier 2's floor so wiring reaches tier 1's boards directly
       instead of going back through the bore. Four, symmetric, at 0/90/180/270, i.e. in the gaps
       between the pillars at 45/135/225/315.

    Tier 1 needs no drops: its compartment is open on top, so anything through a tier-2 drop lands
    straight in it. */
hub_port_n  = 4;     // radial wire ports through the TIER-1 bore wall
hub_port_d  = 10;    // port Ø [mm]
hub_port_a0 = 30;    // MUST EQUAL radial_a0 (checked by check_layout.py). Clocked onto the feed
                     // channels: Two of the four ports sit directly over
                     // a channel, so port and channel merge into ONE large opening running from the
                     // bore straight out to the trench — the bundle leaves the bore and is already in
                     // its channel, with no step and no corner. The other two (at +90/+270 from
                     // those) are plain ports for everything that is not going to a trench.
t2_port_n   = 4;     // radial wire ports through the TIER-2 bore wall (apple wiring -> tier-2 chips)
t2_port_d   = 10;    // port Ø [mm]
t2_port_a0  = 0;     // first port angle [deg]
drop_n      = 4;     // vertical pass-throughs, tier-2 compartment -> tier-1 compartment
drop_d      = 10;    // drop Ø [mm]
drop_bcd    = 116;   // Ø [mm] -> r 58: 3 mm inboard of the tier-1 trenches (r 66), and 45 deg off
                     // every pillar
drop_a0     = 0;     // first drop angle [deg] — between the pillars

/* [CABLE MANAGEMENT — sunk into the floor, so it runs UNDER the boards]
    v0.6 had nowhere for the ~44 cm, 16-wire tool-connector breakout to go. v0.7 added a perimeter
    trench; v0.8 sinks it BELOW the compartment floor and adds RADIAL channels that run from the
    central bore all the way out to it. So the bundle comes up the bore, drops into a radial channel,
    runs out to the perimeter, and turns into the ring trench — all below board level, with boards
    velcro'd on the flat floor over the top of it.

    The trench floor sits `trench_sink` below the compartment floor, leaving
    (base_floor - trench_sink) mm of solid floor underneath.

    TWO PERIMETER TRENCHES, not one ring. The tool-connector bundle splits in half and each half
    goes into its own pocket on opposite sides of the tier, which is easier to dress and to stuff
    than one long ring — and it puts the slack back out at the PERIMETER where it belongs, clear of
    the middle of the floor. Each trench fills the gap between two screw pillars, so the pillars sit
    in the two remaining gaps and nothing has to get past them.

    One radial channel feeds each trench, so the split happens at the bore and never crosses back. */
trench_ri    = 66;   // trench inner radius [mm]
trench_ro    = 86;   // trench outer radius [mm] — 2 mm shy of the compartment wall
trench_n     = 2;    // number of perimeter trenches (opposite sides)
trench_a0    = 0;    // centre angle of the first trench [deg] — mid-gap between two pillars
trench_span  = 74;   // angular width of each trench [deg] — PEG TO PEG. The pillars sit at
                     // 45/135/225/315 and a Ø12 pillar at r 75 subtends 9.2 deg, so its near edge is
                     // at 40.4 deg. Ending the trench at 37 deg leaves ~4.5 mm of wall between the
                     // trench end and the pillar, which is as far as it can run without undercutting
                     // the thing that carries tier 2. 98 mm of run per trench.
trench_sink  = 6;    // depth BELOW the compartment floor [mm] -> 4 mm of floor left under it
trench_tie_n = 3;    // cable-tie slots per trench
radial_n     = 2;    // radial cable channels — ONE to each trench
radial_w     = 10;   // radial channel width [mm] — takes half the bundle
radial_a0    = 30;   // first radial channel angle [deg]. Each channel now meets its trench at the
                     // trench's END, not its middle: the bundle enters at one end and runs the whole
                     // length, instead of arriving in the centre and having to be dressed both ways.
                     // Trench 1 spans -30..+30 and trench 2 spans 150..210, so channels at 30 and 210
                     // land on one end of each — and the two channels form a single diagonal through
                     // the bore. 15 deg off the nearest pillar = 8.4 mm of clear floor.

/* [BOARD REFERENCE PLACEMENTS — no pockets are cut for these]
    Every board is velcro'd to the flat compartment floor, so these numbers no longer shape the print.
    They are kept because they are still the verified answer to "does this all actually fit?" —
    check_layout.py holds each one against the compartment wall, the tier bore and the screw pillars,
    and electronics_mock draws them. Move a board on the bench and nothing needs reprinting; move it
    here and re-run the check if you want the record to stay true.
    `*_rot` = 0 puts the LONG side along x. `*_clear` is the height the board needs above the floor,
    which is what sets each tier's compartment depth. */
velcro_t = 2;   // hook-and-loop pad thickness under every board [mm] (included in each *_clear)

// ---- TIER 1 (bore Ø30) ----
// Control board — Adafruit Metro M4 Express AirLift Lite (4000). NOTE: 92 x 50 is the BARE board;
// if its power adapter has to live beside it, that is extra floor area, not a bigger rectangle.
board_l        = 92.0;  board_w = 50.0;
board_clear_h  = 19;
board_pos      = [0.0, -44.0];  board_rot = 0;

// RS-422 transceiver — MikroElektronika RS485 3 Click (MIKROE-2821), 42.9 x 25.4, full duplex, 3.3 V,
// SN65HVD31. Its screw terminals take the CTR pairs straight off the tool connector, so it belongs on
// tier 1 near the bore.
rs422_l        = 45.0;  rs422_w = 28.0;
rs422_clear    = 14;
rs422_pos      = [0.0, 33.0];   rs422_rot = 0;

// Pressure sensor — Adafruit MPRLS (3965), 17.8 x 16.7 x 7.5. I2C; a Ø2-3 tube runs from its port up
// the centre riser to the sealed cavity in the apple.
mprls_l        = 20.0;  mprls_w = 19.0;
mprls_clear    = 12;
mprls_pos      = [-42.0, 28.0]; mprls_rot = 0;

// ---- TIER 2 (bore Ø20) ----
// DC-DC converter — Tobsun 24V->5V potted block. The tallest thing in the tool at 22 mm, which is
// what sets comp2_depth.
conv_l         = 70.0;  conv_w = 65.0;
conv_clear_h   = 22;
conv_pos       = [0.0, 46.5];   conv_rot = 0;

// NeoPixel level shifter (3.3 -> 5 V ring data).
shifter_l      = 36.0; shifter_w = 28.0; shifter_h = 10.2;
shifter_clear  = 12;
shifter_pos    = [0.0, -30.0];  shifter_rot = 0;

// Haptic driver — DRV2605L, drives the ERM in the apple.
haptic_drv_l   = 28.0; haptic_drv_w = 20.0; haptic_drv_h = 4.6;
haptic_clear   = 9;
haptic_pos     = [-48.0, -20.0]; haptic_rot = 0;

/* [NeoPixel ring — Adafruit 2874, 60x5050 RGBW (buy 4x QUARTER-rings) — groove on the COVER TOP]
    v0.6 cut the groove at the datasheet Ø157/145 and four butted quarter-rings did not close in it.
    The groove is now ~3 LEDs of arc bigger: at the old mean radius 75.5 the 60 LEDs sit at 7.9 mm
    of arc each, so 3 more LEDs = 23.7 mm of circumference = +3.8 mm of radius. Clearance per side
    is also up from 0.6 to 1.0 so the segments drop in rather than being pressed in. */
ring_od        = 165.0; // GROOVE outer Ø [mm]  (the ring PCB itself is 157 — see ring_pcb_od)
ring_id        = 152.0; // GROOVE inner Ø [mm]  (the ring PCB itself is 145 — see ring_pcb_id)
ring_pcb_od    = 157.0; // actual Adafruit 2874 full-ring outer Ø [mm] — for the fit mock only
ring_pcb_id    = 145.0; // actual Adafruit 2874 full-ring inner Ø [mm] — for the fit mock only
// The channel is now 8.5 mm wide against the ring's 6 mm, so the four quarter-arcs can sit anywhere
// from r 72.5..78.5 (hard against the inner wall) out to r 77.5..83.5 (hard against the outer wall).
// Sitting further out is what buys the extra circumference: at r 79 the 60-LED ring needs 496 mm of
// arc against the 474 mm it needs at r 75.5 — about 3 LEDs' worth, which is the gap that stopped the
// four segments closing in v0.6. Push them outward as you seat them.
ring_pcb_t     = 3.25;  // ring overall thickness [mm]   (PCB + LEDs, 2874 = 3.25 mm / 0.13")
ring_clear     = 1.0;   // radial clearance per side [mm]
ring_groove_h  = 4.5;   // top-facing groove depth to seat the ring [mm] (LEDs face up; clear box shields)
ring_wire_d    = 6.0;   // lead pass-through under the groove, one per quarter junction [mm]

/* [Cover plate] */
cover_plate_t  = 6.0;   // cover thickness [mm] — opaque (same material as the base)

/* [Base <-> cover fastening screws] */
cover_screw_n       = 4;     // screws joining the cover down to the base
cover_screw_bcd     = 120;   // their bolt-circle Ø [mm] — at r=60, INSIDE the ring. Outboard is not
                             // possible: the gap between the groove outer (r 83.5) and the rim inner
                             // (r 91) is 7.5 mm and an M3 counterbore is 6.5 wide, which leaves 0.5 mm
                             // a side.
                             // WAS 138 (r=69) and that was WRONG: the Ø12 cover pillar reached r 75,
                             // and the tier-screw counterbore at r 75 starts at r 71.75 — the two
                             // overlapped by 3.25 mm at the same four angles, so the counterbore ate
                             // into the pillar. r=60 puts the pillar at 54..66, a clear 5.75 mm inboard
                             // of the counterbore. check_layout.py now tests for this.
cover_screw_a0      = 45;    // first-screw angle [deg] — 45 clears the Tobsun (|x| <= 36) on tier 2
cover_screw_d       = m3_clear;    // clearance hole in the cover (M3) [mm]
cover_screw_pilot   = m3_insert;   // heat-set insert bore in the base (M3) [mm]
cover_screw_cbore   = m3_cbore;    // counterbore Ø for the head [mm]
cover_screw_cbore_h = m3_cbore_h;  // counterbore depth [mm]
cover_screw_depth   = 14;          // pilot-hole depth into the TIER-2 top [mm]

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

/* [Apple — ONE-PIECE PETG STEM + FUSED soft BALL]
    v0.6 split this into a core base, a separate PETG rod, and an M3 detent pin that set the height.
    In practice that joint was finicky to assemble and the apple wobbled on it: a Ø14 rod in a Ø14.6
    bore has ~0.3 mm of radial slop, and at a 100 mm lever that is over a degree of rock before the
    pin even starts to wear. The height adjustment is not worth that, so the flange, the shaft and
    the armature flange are now ONE printed part.

    The stem (PETG) and ball (TPU) still print as ONE dual-material object on the H2D: export
    apple_stem.stl (PETG) + apple_ball.stl (TPU), import BOTH at the same origin in Bambu Studio and
    assign filaments. The PETG shaft runs up into the ball and ends in a low FLANGE embedded in a
    solid TPU cap, so the pull load is carried mechanically (NOT by PETG<->TPU adhesion). */
apple_d        = 45;   // TPU ball outer Ø [mm]
apple_wall     = 3;    // TPU ball shell wall [mm]
apple_grooves  = true; // grip grooves on the ball sides
stem_shaft_d   = 14;   // PETG shaft Ø [mm]
stem_shaft_len = 100;  // shaft length, cover flange TOP -> armature flange [mm]. Sets apple height,
                       // which is now FIXED: change this number and reprint to move the apple.
sensor_bore_d  = 9;    // feed bore up the stem [mm] (wiring + ERM + FSR tail + pressure tube;
                       // keep <= stem_shaft_d - 4 for a solid wall)
arm_flange_d   = 22;   // PETG armature flange Ø [mm] — embedded in the TPU cap (anchors the pull)
arm_flange_t   = 3;    // armature flange thickness [mm]
arm_cap        = 8;    // solid TPU cap thickness below the flange [mm] (embeds it; upper ball hollow)
stem_fillet_r  = 6;    // fillet radius where the shaft meets the flange [mm] — this joint carries
                       // the whole pull moment, and a sharp internal corner is where it would crack
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
case_box_side  = 198;   // outer square side [mm] (encloses the Ø188 base: ~2 mm gap + a wall each side)
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
base1_h       = base1_floor + comp1_depth;                // tier-1 height [mm]
base2_h       = base2_floor + comp2_depth;                // tier-2 height [mm]
base_h        = base1_h + base2_h;                        // total base stack height [mm]
tier2_z       = base1_h;                                  // tier 2 sits on top of tier 1
cover_z       = base_h;                                   // cover sits on top of tier 2
cover_top_z   = cover_z + cover_plate_t;                  // cover top plate face (ring groove + boss live here)
cover_face_z  = cover_top_z + sj_flange_t;                // top mating face of the cover's joint boss
apple_base_z  = cover_face_z;                             // apple STEM bottom flange sits here

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

// ---------------------------------------------------------------------
//  Shared tier features
// ---------------------------------------------------------------------

// The single open compartment: an annulus from the hub wall out to comp_ro, cut down from the top.
module compartment_cut(top_z, depth, bore_d) {
    translate([0, 0, top_z - depth])
        difference() {
            cylinder(h = depth + eps, r = comp_ro);
            translate([0, 0, -eps]) cylinder(h = depth + 3*eps, r = bore_d/2 + hub_wall);
        }
}

// Radial cable channels sunk INTO the floor, running from the central bore out to `r_out`.
// Wires drop in here and boards velcro down on the flat floor over the top of them.
module radial_channels(bore_d, r_out, fl, a0 = radial_a0) {
    for (i = [0 : radial_n - 1])
        rotate([0, 0, a0 + i * 360 / radial_n])
            translate([bore_d/2 - 2, -radial_w/2, fl - trench_sink])
                cube([r_out - bore_d/2 + 2, radial_w, trench_sink + eps]);
}

// Radial ports through a tier's central-bore wall. Bottom flush with the compartment floor, so wire
// leaves the bore already lying on the floor rather than having to climb over the hub.
module bore_ports(bore_d, n, d, fl, a0) {
    for (i = [0 : n - 1])
        rotate([0, 0, a0 + i * 360 / n])
            translate([0, 0, fl + d/2])
                rotate([0, 90, 0])
                    cylinder(h = bore_d/2 + hub_wall + 4, d = d);
}

// A screw pillar standing from the compartment floor to the tier top (added back, then bored).
module screw_pillars(top_z, positions, fl) {
    for (pp = positions)
        translate([pp[0], pp[1], fl - eps]) {
            cylinder(h = top_z - fl + eps, d = pillar_d);
            // flared base — a pillar meeting a flat floor at 90 deg is a crack waiting to happen
            cylinder(h = fillet_r, d1 = pillar_d + 2 * fillet_r, d2 = pillar_d);
        }
}

// Fillets in the two concave corners where the compartment walls meet the floor. Added back AFTER the
// compartment is cut and BEFORE the holes, so ports and channels still cut cleanly through them.
module compartment_fillets(fl, bore_d, f = fillet_r) {
    ri = bore_d/2 + hub_wall;
    rotate_extrude() {
        translate([comp_ro - f, fl])                       // outer: against the rim
            difference() { square([f, f]); translate([0, f]) circle(r = f); }
        translate([ri, fl])                                // inner: against the hub tube
            difference() { square([f, f]); translate([f, f]) circle(r = f); }
    }
}

// =====================================================================
//  (1a) BASE TIER 1 — sits on the flange plate. Ø30 bore, one open compartment,
//       perimeter cable trench + radial channels sunk into the floor.
//       Reference boards: Metro M4 AirLift, RS-422 click, MPRLS pressure sensor.
// =====================================================================
module base_tier1() {
    sink_z = base1_floor - trench_sink;   // floor of the trenches and the feed channels

    difference() {
        union() {
            // blank, compartment, then the fillets and pillars added back into it
            difference() {
                cylinder(h = base1_h, d = base_d);
                compartment_cut(base1_h, comp1_depth, cable_bore_d);
            }
            compartment_fillets(base1_floor, cable_bore_d);
            screw_pillars(base1_h, tier_screw_pos, base1_floor);
        }

        // ---- everything below is cut LAST, so it goes through the fillets too ----
        translate([0, 0, -eps]) cylinder(h = base1_h + 2*eps, d = cable_bore_d);   // Ø30 bore

        // radial ports out of the bore. Two are clocked onto the feed channels, so bore, port and
        // channel become one continuous opening running out to the trench.
        bore_ports(cable_bore_d, hub_port_n, hub_port_d, base1_floor, hub_port_a0);

        // ...and BRIDGE those two down to the channel floor. A port is a circle sitting tangent to
        // the compartment floor, so on its own it pinches to zero width exactly where the channel
        // meets it — leaving a thin web between the two that the wire would have to climb over.
        // This squares the opening off from the channel floor up to the port's widest point, so the
        // channel and the port really are ONE hole.
        for (i = [0 : radial_n - 1])
            rotate([0, 0, radial_a0 + i * 360 / radial_n])
                translate([0, -radial_w/2, sink_z])
                    cube([cable_bore_d/2 + hub_wall + 4, radial_w,
                          base1_floor + hub_port_d/2 - sink_z]);

        // two perimeter trenches, each filling the gap between two screw pillars
        for (t = [0 : trench_n - 1])
            rotate([0, 0, trench_a0 + t * 360 / trench_n - trench_span / 2])
                rotate_extrude(angle = trench_span)
                    translate([trench_ri, sink_z])
                        square([trench_ro - trench_ri, trench_sink + eps]);

        // one feed channel out to each trench
        radial_channels(cable_bore_d, trench_ro, base1_floor);

        // cable-tie slots across each trench floor
        for (t = [0 : trench_n - 1])
            for (k = [0 : trench_tie_n - 1])
                rotate([0, 0, trench_a0 + t * 360 / trench_n
                              - trench_span/2 + (k + 0.5) * trench_span / trench_tie_n])
                    translate([(trench_ri + trench_ro)/2, 0, sink_z - 1.5])
                        cube([trench_ro - trench_ri + 6, 3.5, 3 + eps], center = true);

        // tier1 -> plate screws. The head bears on plate_head_z mm of printed floor — that section
        // was the weakest link in the v0.6 prototype, so it is now 10 mm, not 5. Drive these BEFORE
        // anything is velcro'd over them. Takes M3x16, not the M3x10 used elsewhere.
        for (i = [0 : plate_screw_n - 1])
            rotate([0, 0, plate_screw_a0 + i * 360 / plate_screw_n])
                translate([plate_screw_bcd/2, 0, 0]) {
                    translate([0, 0, -eps]) cylinder(h = plate_head_z + eps, d = m3_clear);
                    translate([0, 0, plate_head_z])
                        cylinder(h = base1_floor - plate_head_z + eps, d = m3_cbore);
                }

        // insert bores down the pillars
        for (pp = tier_screw_pos)
            translate([pp[0], pp[1], base1_h - tier_screw_depth])
                cylinder(h = tier_screw_depth + eps, d = m3_insert);
    }
}

// =====================================================================
//  (1b) BASE TIER 2 — stacks on tier 1. Ø20 bore, one open compartment, flat floor.
//       Reference boards: Tobsun converter (deepest), level shifter, DRV2605L.
// =====================================================================
module base_tier2() {
    difference() {
        union() {
            difference() {
                cylinder(h = base2_h, d = base_d);
                compartment_cut(base2_h, comp2_depth, tier2_bore_d);
            }
            compartment_fillets(base2_floor, tier2_bore_d);
            screw_pillars(base2_h, [for (i = [0 : cover_screw_n - 1]) cover_screw_pos(i)], base2_floor);
        }

        translate([0, 0, -eps]) cylinder(h = base2_h + 2*eps, d = tier2_bore_d);  // Ø20 bore

        // 4 radial ports out of the riser: the apple's ERM leads, the pressure tube and the ring data
        // all arrive up the middle and have to reach chips sitting out on the floor
        bore_ports(tier2_bore_d, t2_port_n, t2_port_d, base2_floor, t2_port_a0);

        // vertical drops through the floor -> straight into tier 1's compartment
        for (i = [0 : drop_n - 1])
            rotate([0, 0, drop_a0 + i * 360 / drop_n])
                translate([drop_bcd/2, 0, -eps])
                    cylinder(h = base2_floor + 2*eps, d = drop_d);

        // tier2 -> tier1 screws: clearance through the floor, head recessed into it
        for (pp = tier_screw_pos)
            translate([pp[0], pp[1], 0]) {
                translate([0, 0, -eps]) cylinder(h = tier_head_z + eps, d = m3_clear);
                translate([0, 0, tier_head_z])
                    cylinder(h = base2_floor - tier_head_z + eps, d = m3_cbore);
            }

        // insert bores down the cover-screw pillars
        for (i = [0 : cover_screw_n - 1])
            translate([cover_screw_pos(i)[0], cover_screw_pos(i)[1], base2_h - cover_screw_depth])
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

// (3a) APPLE STEM — ONE PETG part: cover flange + shaft + armature flange.
//      Replaces v0.6's core base + separate rod + detent pin, which wobbled on the pin clearance.
//      Apple height is now fixed by stem_shaft_len (reprint to change it).
module stem_solid() {   // OUTER envelope of the shaft + armature ONLY — the TPU ball moulds around this
    union() {
        translate([0, 0, sj_flange_t - eps])
            cylinder(h = stem_shaft_len + eps, d = stem_shaft_d);
        translate([0, 0, sj_flange_t + stem_shaft_len - eps])
            cylinder(h = arm_flange_t + eps, d = arm_flange_d);
    }
}
module apple_stem() {
    top_z = sj_flange_t + stem_shaft_len + arm_flange_t;
    difference() {
        union() {
            cylinder(h = sj_flange_t, d = sj_flange_d);      // flange -> bolts down into the cover
            stem_solid();                                    // shaft + armature anchor flange
            joint_spigot(sj_register_d);                     // centring spigot (down into the cover)
            // fillet at the shaft/flange junction — this corner carries the entire pull moment
            rotate_extrude()
                translate([stem_shaft_d/2, sj_flange_t])
                    difference() {
                        square([stem_fillet_r, stem_fillet_r]);
                        translate([stem_fillet_r, stem_fillet_r]) circle(r = stem_fillet_r);
                    }
        }
        // feed bore, full length: wiring + ERM leads + FSR tail + pressure tube
        translate([0, 0, -joint_register_h - eps])
            cylinder(h = joint_register_h + top_z + 2*eps, d = sensor_bore_d);
        joint_upper_cuts(sj_flange_t, sj_flange_t, sj_screw_n, sj_screw_bcd, sj_screw_a0);  // bolts -> cover
    }
}

// (3b) TPU BALL — LOWER cup, fused to the stem. Open-top so the ERM + force sensor drop in; the PETG
//      armature flange is embedded in the solid cap below; a rim rebate takes the press-fit cap.
module apple_ball() {
    ball_c   = sj_flange_t + stem_shaft_len - arm_cap + apple_d/2;  // ball sits low: flange near the BOTTOM
    fl_top   = sj_flange_t + stem_shaft_len + arm_flange_t;         // armature flange top = cavity floor
    inner_d  = apple_d - 2 * apple_wall;
    split_z  = ball_c + cap_split;
    rebate_d = apple_d - 2 * apple_wall;
    difference() {
        translate([0,0,ball_c]) sphere(d = apple_d);                                  // ball body
        intersection() {                                                              // hollow above the flange
            translate([0,0,ball_c]) sphere(d = inner_d);
            translate([0,0,fl_top]) cylinder(h = apple_d, d = apple_d + 1);
        }
        translate([0,0,split_z]) cylinder(h = apple_d, d = apple_d + 2);              // slice OPEN at the split
        translate([0,0,split_z - cap_lip_h]) cylinder(h = cap_lip_h + eps, d = rebate_d);  // rim rebate
        stem_solid();                                                                 // carve the PETG stem
        if (apple_grooves) grip_grooves(ball_c - apple_d/2);
    }
}

// (3c) TPU CAP — separate press-fit dome that closes the ball after loading electronics.
module apple_cap() {
    ball_c  = sj_flange_t + stem_shaft_len - arm_cap + apple_d/2;
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
        translate([0,0,split_z - cap_lip_h])                                          // press-fit skirt
            difference() {
                cylinder(h = cap_lip_h + eps, d = skirt_d);
                translate([0,0,-eps]) cylinder(h = cap_lip_h + 3*eps, d = skirt_d - 4);
            }
    }
}

// Apple stem + fused ball + press-fit cap assembled (preview / section). One frame, no seat offset.
module apple_assembled() {
    apple_stem();
    apple_ball();
    apple_cap();
}

// =====================================================================
//  Optional: mock electronics to check fit (NOT for printing)
// =====================================================================
module electronics_mock() {
    // NeoPixel ring — seated in the groove on the cover TOP (LEDs up)
    color("green") translate([0, 0, cover_top_z - ring_groove_h + ring_pcb_t])
        difference() { cylinder(h = ring_pcb_t, r = ring_pcb_od/2); cylinder(h = ring_pcb_t+eps, r = ring_pcb_id/2); }
    // tier 1 — every board sits on the SAME flat floor now (velcro, no pockets)
    pcb_mock(board_pos,   board_l,      board_w,      board_rot,   base_floor + velcro_t, 1.6,              "steelblue");
    pcb_mock(rs422_pos,   rs422_l,      rs422_w,      rs422_rot,   base_floor + velcro_t, 1.6,              "seagreen");
    pcb_mock(mprls_pos,   mprls_l,      mprls_w,      mprls_rot,   base_floor + velcro_t, 7.5,              "gold");
    // tier 2
    translate([0, 0, tier2_z]) {
        pcb_mock(conv_pos,    conv_l,       conv_w,       conv_rot,    base2_floor + velcro_t, conv_clear_h*0.6, "dimgray");
        pcb_mock(shifter_pos, shifter_l,    shifter_w,    shifter_rot, base2_floor + velcro_t, shifter_h,        "darkorange");
        pcb_mock(haptic_pos,  haptic_drv_l, haptic_drv_w, haptic_rot,  base2_floor + velcro_t, haptic_drv_h,     "purple");
    }
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
    color("dimgray")  translate([0, 0, -plate_t]) flange_plate();   // adapter plate: robot <-> tier 1
    color("silver")   base_tier1();
    color("lightgray") translate([0, 0, tier2_z]) base_tier2();
    %electronics_mock();
    color("gainsboro", 0.7) translate([0, 0, cover_z]) base_cover();
    translate([0, 0, apple_base_z]) {
        color("slategray")      apple_stem();                       // ONE PETG part: flange + shaft
        color("firebrick", 0.9) apple_ball();                       // TPU lower ball (fused)
        color("indianred", 0.85) apple_cap();                       // TPU press-fit cap
    }
    if (case_preview) translate([0, 0, cover_top_z]) casing();      // clear casing box (preview)
    %translate([0, 0, -plate_t - 1]) cylinder(h = 1, d = plate_d + 6);  // robot flange face marker
}

// ---- render the selected part ----
if      (part == "flange_plate")     flange_plate();
else if (part == "base_tier1")       base_tier1();
else if (part == "base_tier2")       base_tier2();
else if (part == "cover")            base_cover();
else if (part == "apple_stem")       apple_stem();
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
