# Apple-Pluck End-Effector (parametric, DRAFT v0.6)

A 3D-printable tool that bolts to the **KUKA LBR iiwa7 media flange (electric)** and presents a
compliant **"apple"** for the monkey to pull. It is also an **electronics hub**: it routes the
media-flange power/data, drives a **NeoPixel ring** visual cue, and carries the control board, the
DC-DC converter, and two small breakouts, with wiring extending to an **ERM/LRA actuator** and a
**pressure sensor** in the apple. Authored in **OpenSCAD** so it's text-based, parametric, and
version-controlled.

> **v0.15 — the COVER was the real weak link**
> Tracing the pull path properly: **the apple stem bolts to the cover on a Ø36 circle, so its screws
> carry 78 N each — 5.6× what the base-to-flange screws see.** The same 4.2 N·m reacted across a
> third of the diameter. The cover at 6 mm was thin in three independent places at once:
>
> | | was | now |
> |---|---|---|
> | bending just outside the Ø44 boss | 15.9 MPa, **SF 3.1** | 7.1 MPa, **SF 7.1** |
> | material under the ring groove | **1.5 mm** | 4.5 mm |
> | material under the casing insert | **1.0 mm** | 4.0 mm |
> | boss-to-plate step | **sharp 90°** | **R4 fillet** |
>
> All four fixed by `cover_plate_t` **6 → 9 mm** plus the fillet, for about **70 g**. The casing clamp
> circle moved Ø56 → Ø64 so its inserts clear the new fillet. SF 3.1 was before allowing for the part
> being flat-printed, where layer-direction strength is nearer 60–70% of bulk — so the real margin was
> closer to 2, on the joint that carries the most force in the whole tool.
>
> Print settings changed with it: the cover is now **6 walls / 8 shells / 40 %**, and tier 2 nudged to
> **6 / 6-6 / 35 %**. Neither tier is decoration — the full moment passes through both.
>
> **v0.14 — STRENGTH, after the v0.6 prototype broke at the base↔flange joint**
> The screws were never the problem: a 20 N pull at the apple is 4.19 N·m at that joint, which is only
> ~14 N per M3 on the tension side. **The printed material around them was.** What changed:
>
> | | was | now | why |
> |---|---|---|---|
> | material under each screw head | **5 mm** | **10 mm** | the thinnest section in the entire load path — 5 mm of PETG with a Ø6.5 well directly above it. Needs **M3×16** here, not M3×10. |
> | tier-1 floor | 10 mm | **14 mm** | and 8 mm (was 4) where a cable channel crosses it |
> | flange plate Ø | 110 | **124** | the plate is what *supports* tier 1's floor; beyond its rim the floor is an unsupported cantilever. 39 mm of overhang → 32 mm. |
> | plate bolt circle Ø | 80 | **100** | the joint reacts the moment as a couple across this circle — wider circle, −20% force per screw |
> | hub wall | 3 mm | **4 mm** | the bore ports take 39–55% of that tube's circumference |
> | internal corners | sharp | **R3 fillets** | compartment walls→floor and pillar bases. A flat-printed disc cracks along its layers, and sharp internal corners are where that starts. |
> | base Ø | 188 | **188 — HELD** | |
>
> **The base did not grow.** The 4 mm hub wall makes the Tobsun the binding part: it needs r 86.4
> against the 88 available, so **1.6 mm**. That margin is real — the earlier "0.3 mm" reading was a
> checker bug, double-counting a 1 mm-per-side pocket allowance on a design that no longer has
> pockets. The fit test now uses the raw footprint; clearance still applies to board-to-board and
> board-to-pillar spacing. Any further growth in `hub_wall`, `conv_l/w` or `tier2_bore_d` and either
> the base grows or the Tobsun and Metro swap tiers.
>
> **The bore port and its feed channel are now genuinely one opening.** A port is a circle sitting
> tangent to the compartment floor, so on its own it pinches to zero width exactly where the channel
> meets it, leaving a thin web the wire would have to climb over. The two aligned ports are now
> squared off from the channel floor up to the port's widest point — continuous from z 8 to z 24,
> full 10 mm width all the way.
>
> **Plate screws re-clocked to 15°** so the six thread between both the feed channels (4.7 mm clear)
> and the tier-2 drops (16.2 mm) — at 0° a drop came down 0.25 mm inside a screw well.
>
> **Geometry is only half of this — see [Print settings](#print-settings) below.** The part is printed
> flat, so the moment at the bolt circle pulls directly against the layer bonds, which is the weakest
> axis by a wide margin and the most likely way the prototype actually failed.
>
> **v0.13 — what changed**
> - **Tier-1 bore ports clocked onto the feed channels.** The two are now at the same angle, so port
>   and channel merge into **one continuous opening running from the bore straight out to the trench**
>   — the bundle leaves the bore already in its channel, with no step and no corner to fight. The
>   other two ports (at +90/+270) stay as plain ports for anything not headed to a trench.
>   `check_layout.py` now pins `hub_port_a0 == radial_a0` so they cannot drift apart.
> - **Tier-2 bore ports 2 → 4.**
> - **Trenches extended peg to peg.** Span **60° → 74°**: a Ø12 pillar at r 75 subtends 9.2°, so its
>   near edge is at 40.4°, and ending the trench at 37° leaves ~4.5 mm of wall — as far as it can run
>   without undercutting the thing that carries tier 2. **98 mm of run per trench, 196 mm total**
>   (was 159).
>
> **v0.12 — what changed**
> - **Wire had no way out of either bore.** Merging the pockets into one compartment (v0.8) left each
>   central bore walled in by a 3 mm tube running the full height of its tier — wire could come up the
>   bore but could not get into the compartment, and nothing could cross tiers except back down the
>   middle. Fixed with:
>   - **4× Ø10 radial ports** through tier 1's bore wall (39% of the hub circumference), bottoms flush
>     with the floor so wire leaves the bore already lying flat;
>   - **2× Ø10 radial ports** through tier 2's bore wall — the path for the apple's ERM leads, the
>     pressure tube and the ring data coming up the riser to the DRV2605L and the level shifter;
>   - **4× Ø10 vertical drops** through tier 2's floor at r 58, at 0/90/180/270 (between the pillars),
>     so tier-2 wiring reaches tier-1 boards directly. Tier 1 needs no drops — its compartment is open
>     on top, so anything through a drop lands straight in it.
>
> **v0.11 — what changed**
> - **BUG FIX: the cover pillars and the tier-screw counterbores were intersecting.** Both sat at
>   45/135/225/315 on tier 2, the Ø12 cover pillar reached r 75, and the tier-screw counterbore at
>   r 75 starts at r 71.75 — a **3.25 mm overlap**, so the counterbore was cutting into the pillar
>   that carries the cover. Cover bolt circle **Ø138 → Ø120** (r 60): the pillar now ends at r 66,
>   a clear 5.75 mm inboard. `check_layout.py` now tests for this explicitly.
> - **Each feed channel now meets its trench at an END, not the middle**, so the bundle enters at one
>   end and runs the full length instead of arriving centrally and needing dressing both ways. The two
>   channels sit at 30°/210°, forming a single diagonal through the bore. Trench span **72° → 60°** to
>   open up the 15° the channel needs beside each pillar.
> - **Flange-plate screws 4 → 6** (Ø80, starting at 0°), so the joint carrying the whole tool is not
>   held on four points. a0 = 0 keeps every one of them ≥ 11.7 mm clear of a radial channel; at 45°
>   two would come within 2 mm.
>
> **v0.10 — what changed**
> - **Two perimeter trenches instead of one ring.** v0.9 moved the trench inboard to make room for
>   symmetric pillars, which pushed the slack store into the middle of the floor where it was in the
>   way. It is back at the **perimeter (r 66→86)** and split in **two 72° pockets on opposite sides**,
>   each filling the gap between two screw pillars. Split the tool-connector bundle in half and stuff
>   one half in each — easier to dress than one long ring, and the pillars sit in the two remaining
>   gaps so nothing has to get past them. 96 mm of run each, 191 mm total.
> - **Two radial channels, one feeding each trench**, so the split happens at the bore and never
>   crosses back. 10 mm wide (was 8) to take half a bundle each.
> - **Tier 2's floor is now completely flat** — its channels are gone. Nothing on that tier needed
>   routing under a board; the riser comes up in the middle and the boards sit around it.
>
> **v0.9 — what changed**
> - **Tier screws are a symmetric bolt circle now.** v0.8 had them at 0/180/225/315 — a trapezoid,
>   so the clamp load was lopsided and the joint could rock about the two close-together screws.
>   They are now **Ø150 at 45/135/225/315**. r = 75 is the smallest radius that clears a Metro-sized
>   board (a 94 × 52 pocket tangent to the Ø30 bore spans |x| ≤ 47, so r = 75 puts the pillar at
>   |x| = 53); anything nearer the centre lands underneath the Metro however it is clocked.
> - *(The trench rework in v0.9 was superseded by v0.10 — see above.)*
>
> **v0.8 — what changed**
> - **The per-board pockets are gone.** Each tier is now **ONE open compartment** with a flat floor.
>   Every board is velcro'd wherever it fits, so boards can be swapped, moved or added without
>   touching the CAD. The only things left standing are **four Ø12 screw pillars per tier** — with no
>   solid top face left, that is where the heat-set inserts have to live.
> - **Cable management moved BELOW board level.** The perimeter trench is now **sunk into the floor**
>   rather than standing proud of it, and **6 radial channels** run from the central bore out to it.
>   The bundle comes up the bore, drops into a radial channel, runs out to the perimeter and turns
>   into the ring trench — all under the boards, which velcro down on the flat floor over the top.
> - **Floor 8 → 10 mm** so the sunk channels leave 4 mm of solid floor beneath them. Tiers are now
>   29 + 32 mm.
>
> **v0.7 — what changed** (all driven by building v0.6)
> - **Ø225 → Ø188, and two stacked tiers.** The 95 × 75 optocoupler is gone (a floating dry contact,
>   and the RS-422 link that replaced it, need none) and it was the only thing forcing Ø225. Shrinking
>   the disc cost board area, so the electronics now split over **two pucks that bolt together**.
> - **Ring groove grown** Ø157/145 → **Ø165/152**: four butted quarter-rings would not close in the
>   v0.6 groove, which was cut to the bare datasheet size.
> - **Pockets resized to the real parts:** Tobsun 60 × 55 → **70 × 65**, level shifter 26 × 18 → **36 × 28**.
>   In v0.6 the Metro ended up living in the optocoupler's slot and the Tobsun did not fit its own.
> - **New pockets** for the **RS-422 transceiver** (MIKROE-2821) and the **MPRLS pressure sensor**.
> - **Perimeter cable trench** on tier 1 — v0.6 had nowhere for the ~44 cm tool-connector bundle to go.
> - **The adjustable shaft is gone.** Apple core base + shaft are now **one PETG part**; the detent
>   joint printed finicky and the apple wobbled on the pin clearance.
> - **`check_layout.py`** verifies the packing against the .scad. Run it after any size change.
>
> **v0.6 — what changed**
> - **No more extension.** The apple **core base bolts straight to the cover**; the height is set by a
>   separate **adjustable shaft** (a detent pin in one of several holes → discrete height steps).
> - **Big ring on the cover *top*.** The NeoPixel ring is now the large **Ø157 (6.2″) 60-LED RGBW**
>   ring (Adafruit 2874 — buy **4× quarter-arcs**). It seats LEDs-up in a groove near the cover rim.
>   The cover is **opaque (same material as the base)** — no translucent skin.
> - **Clear casing box.** A hand-built **5-sided clear box, open on the arm-flange side**, shrouds the
>   whole electronics stack. Its top lies over the ring and clamps by **3× M3 into the cover just outside
>   the apple-core boss**; the 4 walls drop to the flange face. The apple pokes up through its centre.
>   (The .scad models the box + a preview; the STL is a build reference — you make it by hand from acrylic.)
> - **Separate flange plate.** A thick **Ø110 × 16** adapter disc takes the **8× M6** into the robot
>   (heads buried flush) and the **Ø30** cable bore; the base bolts onto **it** with 4× M3. The printed
>   base no longer carries the M6 clamp load, and you never reach past a board to a flange bolt.
> - **TWO stacked tiers, one open compartment each**, everything **velcro'd** (no pockets, no standoffs,
>   no board screws). Tier 1 carries the Metro M4, the RS-422 transceiver and the MPRLS pressure sensor;
>   tier 2 the Tobsun converter, the level shifter and the DRV2605L. Those placements are a *reference*,
>   not a mould — nothing is keyed to a rectangle any more.
>   **The optocoupler is gone** — a floating dry contact, and the RS-422 link that replaced it, need
>   none — and it was the only thing forcing Ø225. The base is now **Ø188**, sized by the ring groove.
> - **Power distribution.** The 24 V media-flange bundle goes to the converter; **5 V is then fanned
>   out from the converter** to every board and up the centre riser (ring + apple), with data fanned
>   out from the Metro — real channels through the pocket dividers, not just the central bore.

## Design at a glance
The apple is three printed parts that stack on the cover; the **base** is the electronics hub:

```
[ iiwa7 media flange (electric) ]   8× M6 on Ø51 (pitch circle) · Ø34 electric bore · 24 V + data bundle
  (0) FLANGE PLATE    (Ø110 × 16 thick) takes the 8× M6 — heads + washers BURIED so its top face is
                      flat — plus the Ø30 cable bore. The base bolts onto IT with 4× M3 (Ø80 circle).
                      Bolt the plate to the robot BARE, then drop the base on: the printed base never
                      carries the M6 clamp load, and you never reach past boards to a flange bolt.
        │  cable bundle up the Ø30 base centre
  (1a) BASE TIER 1    (Ø188 × 33) Ø30 central bore — the flange bundle lands HERE. ONE open
                      compartment, r 18 → 88, flat floor. Standing in it: 4× Ø12 screw pillars on a
                      SYMMETRIC Ø150 bolt circle (45/135/225/315).
                      Sunk INTO the floor (so wiring runs UNDER the boards):
        ├─ 2 PERIMETER TRENCHES  r 66→86, 6 deep, 74° each (PEG TO PEG) centred at 0° and 180° =
        │   98 mm of run apiece (196 total), 3 cable-tie slots each. Split the bundle, one half per side.
        ├─ 2 FEED CHANNELS  10 wide, at 30°/210° — one diagonal line through the bore, each meeting
        │   its trench at an END so the bundle runs the trench's full length
        ├─ 4× Ø10 RADIAL BORE PORTS  at 30/120/210/300, bottoms flush with the floor. The two at
        │   30/210 sit ON the feed channels and are squared down to the channel floor, so bore,
        │   port and channel are ONE continuous opening with no web to climb
        ├─ the rest of the floor is FLAT
        └─ reference boards (velcro anywhere): Metro M4 AirLift 92 × 50 · RS-422 click 45 × 28 ·
           MPRLS pressure 20 × 19
  (1b) BASE TIER 2    (Ø188 × 32) stacks on tier 1 (4× M3×16 at explicit points). Ø20 bore — only
                      ring/apple wiring goes higher, and those 5 mm of reclaimed radius per side
                      are what let the 70 × 65 Tobsun fit inside Ø188 at all. ONE open compartment,
                      r 13 → 88, 4× Ø12 pillars carrying the COVER inserts, 6 radial floor channels.
        └─ reference boards: Tobsun 70 × 65 (22 tall — sets the tier depth) · level shifter 36 × 28 ·
           DRV2605L 28 × 20
        ├─ POWER DISTRIBUTION: 24 V up the riser from tier 1 → converter, then 5 V back down the
        │   riser to tier 1 and up to the ring; data from the Metro
  (2) COVER           seats the NeoPixel ring in a Ø165/152 groove on its TOP (LEDs up, opaque cover) ·
                      closes tier 2 · 4× M3 down into tier 2 at Ø138 — INSIDE the ring, because the
                      7.5 mm between the groove and the rim will not take an M3 counterbore ·
                      4 ring-lead pass-throughs (one per quarter) · 3× M3 pilots for the casing box
        ▼  BOLTED FLANGE: apple STEM screws down into the cover (3× M3 + centring spigot)
  (3a) APPLE STEM       ONE PETG part: cover flange + Ø14 shaft + armature flange, filleted at the
                        root. Replaces v0.6's core base + separate rod + detent pin, which wobbled.
                        Apple height is now FIXED by stem_shaft_len — reprint to change it.
  (3b) TPU BALL         LOWER cup FUSED to the stem (dual-material print); solid cap embeds the
                        armature flange; OPEN top loads the ERM + force sensor
  (3c) TPU CAP          press-fit dome closes the ball; wires / FSR tail / pressure tube exit the bore
  ( + ) CLEAR CASING BOX  5-sided clear box (open on flange side) over the whole stack; top clamps 3× M3 at Ø56
```

> **Joints.** The cover↔core-base joint is a **bolted flange** (Ø44, 3×M3 on a bolt circle, centred by
> a spigot, cable bore down the middle). The flange sits at the centre of the big ring (ID Ø145), with
> loads of clear space between the apple mount and the ring.
>
> **Fixed height (v0.7).** The flange, shaft and armature flange are **one PETG part** (`apple_stem`),
> filleted R6 at the root. The v0.6 detent joint is gone: a Ø14 rod in a Ø14.6 bore carries ~0.3 mm of
> radial slop, which at a 100 mm lever is over a degree of rock before the pin starts to wear, and it
> was fiddly to assemble. Height is now set by **`stem_shaft_len` (100 mm)** — change it and reprint.
>
> **Fused apple (PETG + TPU).** The shaft + LOWER ball print as **one dual-material object** on the H2D.
> The PETG shaft runs up into the ball and ends in a low **flange**; a solid TPU **cap** embeds it, so the
> pull load is carried **mechanically**, not by the PETG↔TPU bond. The ball's **top is a separate press-fit
> TPU cap**: pop it, drop in the ERM + force sensor (all bigger than the bore), route their wires / FSR tail
> / pressure tube down the **Ø9 shaft bore**, press the cap back on (silicone it if the pull unseats it).

## Power / data flow
```
24 V (cabinet, via X650/X651) ─► Tobsun 24 V→5 V ─► 5 V ─┬─► Metro M4 AirLift board
                                                 ├─► NeoPixel ring   (via Pixel Shifter: 3.3 V data → 5 V)
                                                 ├─► DRV2605L driver ─► ERM/LRA motor (actuator, apple cavity)
                                                 └─► pressure sensor (apple cavity)
   data: Metro 3.3 V ─► Pixel Shifter ─► ring DIN ;  Metro I²C ─► DRV2605L ;  sensor ─► Metro ADC/I²C
```
> **Ring current.** The 60-LED RGBW ring can pull **~3.5 A at 5 V** at full-white — inject 5 V at each
> of the 4 quarter pass-throughs (don't feed 60 LEDs through one arc). Size the converter for it, and
> keep brightness capped in firmware and size the 5 V wiring for the load.
The **Metro M4 Express AirLift** runs everything: it drives the NeoPixel ring through the **Pixel
Shifter** (its 3.3 V data needs shifting to 5 V), commands the **DRV2605L** over I²C to run the
haptic **actuator** (an ERM/LRA motor in the apple), and reads the **pressure sensor**. The
**Tobsun converter** steps that 24 V down to 5 V; that **5 V rail is distributed from it** out
to every board and up the centre riser (ring + apple) through channels in the base. All five boards
live in the base; only the ERM motor + pressure sensor sit up in the apple cavity.

## The media flange is a pass-through — the as-built wiring

This robot has the **Media flange Inside electric**: two supply voltages, two analog/CAT5
interfaces, an internal connector, and **no electronics of its own** (KUKA media-flange manual V10,
20 Oct 2021, §2.1.9). It is a conduit from interface A1 at the rear of the base frame (Fig. 5-1) to
the tool connector at the wrist.

**As found on this arm:**

```
   [ robot base, interface A1, rear of base frame ]

     X31  ══ robot data cable ══► Sunrise cabinet          (already connected)

     X651 ══ KUKA data cable X650/X651 ══► cabinet         CONNECTED
          └─ carries 24 V + GND (pins 5/6) and EtherCAT (pins 9-12), Fig. 5-63
                                     │
     X76  ── your trigger wiring ────┤                     WAS CAPPED — 12+3 bypack fitted
                                     ▼
   [ tool connector at the flange face ]   16-way breakout, ~44 cm
```

Three consequences that shape the whole design:

1. **The 24 V is already there, from the cabinet.** The X650/X651 data cable puts 24 V on X651
   pin 5 and GND on pin 6, which the flange passes to **tool connector pins 1/2**. Verified in
   practice: the Metro board and the ring have both been run from it. No external PSU is needed.
   (KUKA's parts list for this flange calls for a *connector bypack* on X651 rather than the data
   cable, so this is an off-book but functionally clean configuration — the pin roles line up.)
2. **EtherCAT is live at the wrist**, on tool pins 3-6. Keep those insulated. It is also the best
   trigger path this robot could have — an EtherCAT slave at the tool would give cabinet-native
   timing and a return channel for sensors — but that is a hardware decision, parked for now.
3. **The cabinet cannot drive a cue line through the flange itself.** The manual's list of flanges
   with configurable I/O (§7.1) does not include Inside electric, so no `MediaFlangeIOGroup` is
   generated and Sunrise has no flange output to assert. The trigger is driven by whatever you
   land on **X76**, whose CTR pairs pass to tool pins 9-16.

### Tool connector pinout

The breakout is **12× AWG26 signal + 4× AWG18 power**, arriving as three 2-wire bundles and two
3-wire bundles (each 3-wire group is a shielded pair plus its drain).

| Tool pin | Colour | Signal | Via | Rating / use |
|---|---|---|---|---|
| **1** | RD | **Power1** | X651 5 | 60 V / 8 A — **live 24 V from the cabinet** |
| **2** | BK | **GND1** | X651 6 | |
| 3 | YE | CAT5 TXP | X651 9 | **live EtherCAT — insulate** |
| 4 | OG | CAT5 TXN | X651 11 | **live EtherCAT — insulate** |
| 5 | WH | CAT5 RXP | X651 10 | **live EtherCAT — insulate** |
| 6 | BU | CAT5 RXN | X651 12 | **live EtherCAT — insulate** |
| 7 | RD | Power2 | X76 A | 60 V / 5 A — unused; fallback supply if the cabinet 24 V ever won't carry the ring |
| 8 | BK | GND2 | X76 B | |
| **9** | BK | **CTR1_1** | X76 1 | **cue trigger** (shielded pair) |
| **10** | BU | **CTR1_2** | X76 2 | |
| 11 | WH | CTR1_3 | X76 3 | drain — bond at ONE end only |
| 12 | GN | CTR2_4 | X76 4 | spare pair — haptic trigger |
| 13 | YE | CTR2_5 | X76 5 | |
| 14 | WH | CTR2_6 | X76 6 | drain |
| 15 | RD | CTR3_7 | X76 7 | spare pair — **reserve for the force sensor** |
| 16 | BU | CTR3_8 | X76 8 | |

> **Colour is ambiguous — meter it.** WH, BU and RD each appear three times. The bundle grouping
> disambiguates, but map the breakout to X651/X76 with a continuity test before landing anything.
> The manual is also internally inconsistent here: the connection table says "6× AWG28" on X76
> while the wiring diagram (Fig. 5-39) labels eight CTR conductors. The meter settles it.

> **Why convert locally rather than feed 5 V down Power1?** Voltage drop. AWG18
> over the robot's internal harness plus the 44 cm breakout drops roughly 0.5 V at the ring's worst
> case — and the drop *varies with how much of the ring is lit*, so cue colour would shift with cue
> settings. Converting at the tool keeps the ring on a stiff 5 V, and the 24 V feed carries ~1.25 A
> instead of ~4.8 A for the same power. Measure the real round-trip resistance (short Power1 to
> GND1 at the tool, measure across X651 5–6) before revisiting this.

### Commissioning the trigger — do it in this order

Each step proves one thing and has a stop condition. Do not skip ahead.

**0. Prep — arm de-energised.** Mount the tool connector **flush** (4× M2×16, 0.35 N·m; if it is
not flush, contact is not guaranteed and every later step will lie to you). Individually insulate
all 16 breakout wires; uninsulate only what each step needs. **Pins 3–6 stay capped permanently —
live EtherCAT.** X651 remains plugged into the cabinet, so pins 1/2 and 3–6 may be live whenever the
cabinet has power.

**1. Crimp X76.** Contacts **1**, **2**, **3**, and **9** (screen). Leave A/B/C and 4–8 empty.
Do not terminate the far end yet — it depends on step 4.

**2. Ring it out — arm de-energised.** Two measurements, because one is not enough:

| At the base | At the tool | Proves |
|---|---|---|
| short X76 **1 ↔ 2** | continuity **pin 9 ↔ pin 10** | both conductors are through |
| short X76 **1 ↔ 3** | continuity **pin 9 ↔ pin 11** | contact 1 really is pin 9, i.e. **no swap** |

The first test passes identically whether or not 1 and 2 are swapped; only the second catches it.
A swap does not matter for a bare contact closure but is fatal for an optocoupler's LED.
While you are here, short tool 9 to tool 10 and measure across X76 1–2 for the **round-trip
resistance** — that number is still unmeasured and every voltage-drop estimate depends on it.

**3. Passive jumper test — no optocoupler, no source, no voltage.** Wire tool pin 9 → Metro **D2**
and tool pin 10 → Metro **GND**. The firmware's `active_low` default plus D2's internal pull-up
means a bare short is a valid trigger.

```bash
ros2 run sinthlab_bringup check_cue_wiring.py          # or: python3 diagnostics/check_cue_wiring.py
```

It pre-flights the board (catches `enabled=0`, an invisible cue, and **inverted polarity**), then
announces every edge as you make and break the short at X76. This is the step that finally verifies
`CUE_PIN = board.D2` against real hardware — the last unverified assumption in the firmware.

> **Stop condition:** no edges means wiring, not firmware. The tool prints the likely causes ranked.

**4. Fit the RS‑422 link.** The cabinet cannot drive the trigger — the Sunrise project has no
generated I/O groups — so the ROS computer drives it. That was going to be a USB relay closing
X76 1/2 as a dry contact; it is now a **full‑duplex RS‑422 serial link**, because the pressure sensor
needs a data channel off the tool anyway and a relay would have been both redundant and 5–15 ms
slower than the wire:

| | Part |
|---|---|
| ROS box | StarTech **ICUSB422IS** — isolated USB↔RS‑422, FTDI FT232RL |
| Tool | MikroE **MIKROE‑2821** RS485 3 Click — SN65HVD31, full duplex, 3.3 V |

The MikroE's screw terminals take the CTR pairs from the tool connector; its header pins go to the
Metro's UART (3.3V, GND, TX/D1, RX/D0 — **check D0/D1 are free** on the AirLift Lite before wiring).
It's a **crossover**: the adapter's TX pair lands on the MikroE's RX pair and vice versa. Put the
differential signals on the **shielded** CTR pairs. Set the FTDI `latency_timer` to 1 on the ROS box,
or you get 16 ms of buffering back. Full detail in
[§6.7 of the top-level README](../README.md#67-end-effector-board--the-visual-cue).

The jumper test in step 3 still stands as the first check of the harness — it proves continuity
before any electronics are involved.

### Handling limits from the manual

- **Tool connector: max 100 mating cycles.** Do not prototype by repeatedly unplugging it.
- Connect/disconnect **de-energised only**, and insulate every unused cable end — a loose end shorts
  and damages the flange.
- The connector must sit **flush** with the flange face or contact is not guaranteed.
  4× **M2×16**, torque **0.35 N·m** (class 8.8) per §12.1.
- Minimum breakout bending radius **5.85 mm**; strain-relieve on the tool side.
- IP54 requires sealing between flange and tool. Flange (230 g) and tool-connector weights are
  accounted for automatically by Sunrise.
- **Stopping distances:** the manual's tables (§4.16.3, §4.16.5) list which flanges they cover, and
  plain *Inside electric* is in neither — only the NE and NE II variants are. It is 230 g with the
  same payload table as the Basic-flange, so those figures very probably apply, but confirm with
  KUKA before relying on them in a risk assessment.

## Components (dimensions locked from datasheets)
| Item | Part / link | Size (mm) | Drives parameter |
|------|-------------|-----------|------------------|
| NeoPixel ring, 60×5050 RGBW (**buy 4× quarter-rings**) | [Adafruit 2874](https://www.adafruit.com/product/2874) | PCB Ø157 / Ø145 × 3.25 (6.2″) — **groove is cut Ø165 / Ø152** | `ring_pcb_od/id`, `ring_od`, `ring_id`, `ring_groove_h` |
| Control board (+ its power adapter) | [Metro M4 Express AirLift Lite (4000)](https://www.adafruit.com/product/4000) | **92 × 50** | `board_l/w`, `board_pos` |
| DC-DC converter | **Tobsun 24 V→5 V** | **70 × 65** (pocket; was 60 × 55 and did not take the real part) | `conv_l/w`, `conv_pos` |
| NeoPixel level shifter | [Pixel Shifter (6066)](https://www.adafruit.com/product/6066) (3.3→5 V data) | **36 × 28** (pocket; was 26 × 18) | `shifter_*` |
| Actuator driver | [DRV2605L (2305)](https://www.adafruit.com/product/2305) (ERM/LRA driver, I²C) | **28 × 20** (pocket) | `haptic_*` |
| **RS-422 transceiver** | [MIKROE-2821 RS485 3 Click](https://www.mikroe.com/rs485-3-click) — SN65HVD31, full duplex, 3.3 V | **42.9 × 25.4** (pocket 45 × 28) | `rs422_l/w`, `rs422_pos` |
| Haptic actuator | [Vibrating Mini Motor Disc (1201)](https://www.adafruit.com/product/1201) (coin ERM) | **Ø10 × 2.7** | apple cavity |
| Force sensor **A** | [FSR — Alpha MF01A round, high-force 1–98 N (5475)](https://www.adafruit.com/product/5475) | **Ø18 head** (~Ø15 active) · 60 long · 0.56 thin | apple cavity (head) + tail down bore |
| Force sensor **B** | [MPRLS ported pressure sensor (3965)](https://www.adafruit.com/product/3965), I²C 0–25 PSI | board **17.8 × 16.7 × 7.5** (pocket 20 × 19) · port Ø2.5 | `mprls_*` on **tier 1** + Ø2–3 tube up the riser to a *sealed* cavity |
| ~~Height-set pin~~ | **removed in v0.7** — the stem is one piece now | — | — |

> **Fit notes (read before printing):**
> 1. **Base is Ø188, and the RING is why — not a board.** The groove outer sits at r = 83.5; add 7.5 mm
>    of solid rim and a 3 mm wall and you get r = 94. The old Ø225 existed only to swallow the 95 × 75
>    optocoupler, which is gone. Run **`python3 check_layout.py`** after *any* size change: the packing
>    has only 3.3 mm of margin at the Tobsun and the script checks every pocket, screw and the trench.
> 2. **Two tiers, and the bore sizes are load-bearing decisions.** Tier 1 keeps the Ø30 bore because the
>    flange bundle lands there. Tier 2 drops to Ø20 because only ring/apple wiring goes higher — and
>    those 5 mm of reclaimed radius per side are exactly what lets the 70 × 65 Tobsun fit. At Ø30 its
>    corners land at **r = 90.1** and it does **not** fit inside Ø188 in any orientation. If you ever
>    enlarge tier 2's bore, the converter has to move back down or the base has to grow.
> 3. **The layout is Cartesian, not polar.** Large rectangles reaching in toward the bore cannot be
>    placed on a bolt circle: at the hub their angular widths exceed 360°. Each board has an explicit
>    `*_pos` = [x, y] and `*_rot`.
> 4. **Every screw now lands on a pillar.** With no pockets there is no solid top face to put a
>    heat-set insert in, so each tier carries **4× Ø12 pillars** from its compartment floor to its top:
>    tier 1's on a Ø150 circle at 45/135/225/315 (taking the tier-2 screws), tier 2's on Ø138 at the
>    same angles (taking the cover screws). Both are **symmetric bolt circles** — v0.8's trapezoid put
>    the clamp load off-centre. They are the only obstructions left in either compartment, and the
>    checker holds every reference board clear of them, verifies the spacing is even, and confirms
>    they sit outboard of the trench and clear of every radial channel.
> 5. **Cover screws moved INSIDE the ring (Ø138).** Outboard is no longer possible: the gap between the
>    groove outer (r 83.5) and the rim inner (r 91) is 7.5 mm, and an M3 counterbore is 6.5 wide — half
>    a millimetre a side. At r = 69 the counterbore ends at 72.25, clear of the groove at 75.
> 6. **The ring groove is deliberately oversized.** v0.6 cut it at the datasheet Ø157/145 and four
>    butted quarter-arcs would not close. The channel is now **8.5 mm wide against the ring's 6 mm**, so
>    the arcs can sit anywhere from hard against the inner wall (r 72.5–78.5) out to hard against the
>    outer (r 77.5–83.5). Sitting further out is what buys the circumference: at r 79 the 60 LEDs need
>    **496 mm** of arc against **474 mm** at r 75.5 — about three LEDs, which is the gap that stopped
>    them closing. **Seat the segments pushed outward.**
> 7. **No pockets and no board screws.** Each tier is ONE open compartment (r 18/13 → 88) with a flat
>    floor; every board is **velcro'd** wherever it fits. The board dimensions in the .scad are a
>    *reference placement* that `check_layout.py` validates and `electronics_mock` draws — they do not
>    shape the print, so you can move or swap a board on the bench without reprinting anything. Each
>    `*_clear` is the height that board needs, and the tallest on each tier sets the compartment depth:
>    Metro 19 → `comp1_depth`, Tobsun 22 → `comp2_depth`, both over a 10 mm floor.
> 8a. **Everything needs a hole to get through.** Each bore is a 3 mm tube the full height of its
>    tier, and tier 2's floor is solid: without holes, wire can reach neither the compartment nor the
>    other tier. **4× Ø10 radial ports** in tier 1's bore wall and **2× Ø10** in tier 2's, all with
>    their bottoms flush to the compartment floor so wire leaves lying flat rather than climbing over
>    the hub. On tier 1 two of the four are **clocked onto the feed channels**, so bore, port and
>    channel form one continuous opening out to the trench. **4× Ø10 drops** through tier 2's floor at
>    r 58 (0/90/180/270, between the pillars) for tier-2 → tier-1 wiring. The ports take 39% of
>    tier 1's hub circumference and **55% of tier 2's** (four 8 mm posts left in a 3 mm tube — thin;
>    drop `t2_port_d` to 8 if it prints badly) —
>    `check_layout.py` caps that, because the hub tube is also part of the face tier 2 lands on.
>    **Note:** the reference Tobsun placement sits over the drop at 90°; three of the four are clear,
>    or slide the Tobsun.
>
> 8. **Cable management runs UNDER the boards, and splits the bundle in two.** v0.6 had no provision
>    at all: the ~44 cm, 16-wire breakout arrived up the bore with nowhere to turn. Tier 1 now has
>    **two perimeter trenches** (r 66→86, 20 wide, 60° each, centred at 0° and 180°, **80 mm of run
>    apiece**, 3 tie slots each), each fed by its **own 10 mm channel** from the central bore. The
>    channels sit at 30°/210° — a single diagonal through the bore — and each meets its trench at an
>    **END**, so the bundle enters at one end and runs the full length rather than landing in the
>    middle. Split the bundle in half at the bore and dress one half into each side; each trench fills
>    the gap between two screw pillars, so nothing has to get past a pillar. All of it is **sunk 6 mm below the compartment floor**, so a board velcros straight over a
>    cable run, with 4 mm of solid floor left underneath. **Tier 2 has no channels at all** — its
>    floor is flat, because nothing there needs routing under a board.
> 9. **Assembly order is forced: plate → robot → tier 1 → boards → tier 2 → boards → cover.** The 8 M6
>    live in the **separate flange plate**, driven with the plate bare. Tier 1 then bolts down with
>    **6× M3 (Ø80 circle, first at 0°)**, their heads recessed into the floor. The compartment is open now so every
>    head is in plain sight — but they still sit **under where boards go**, so drive them before you
>    velcro anything down.
> 10. **Thin shaft + close apple (by design).** The stem is **Ø14** with a **Ø9 feed bore**, filleted
>    **R6** at the root — it sits at the very centre, far inside the ring (ID r ≈ 75), so it doesn't
>    occlude the LEDs. Height is now **fixed** at `stem_shaft_len = 100` (apple centre 202 mm above the
>    robot flange). The adjustable detent joint is gone: a Ø14 rod in a Ø14.6 bore has ~0.3 mm of radial
>    slop, which at a 100 mm lever is over a degree of rock before the pin even starts to wear.
> 11. **Shaft strength — re-check, the lever got longer.** v0.6 quoted a ~60 mm moment arm at the default
>    detent height. The v0.7 stem is a **fixed 100 mm** shaft, so at Ø14/Ø9-bore a 20 N pull at the ball
>    gives roughly **10 MPa** of bending stress at the root against ~30–50 MPa for PETG — a safety factor
>    of about **3–5**, down from v0.6's 5–8, with proportionally more deflection. Still adequate, but the
>    margin is thinner and it now depends on the **root fillet** forming properly. Print the **stem solid
>    / high-perimeter**, and if you shorten `stem_shaft_len` the margin improves as the square of the
>    change. The ball↔stem **flange anchor (Ø22)** still carries the pull in bearing on the solid TPU
>    cap, not on the PETG↔TPU bond.
> 12. **Apple cavity + access.** The cavity (dome ≈ **Ø39 × 31 mm** above the flange) fits the **ERM
>    (Ø10)**, the **FSR head (Ø18)**, or the **MPRLS board (17.8 mm)** with room to spare — but all are
>    bigger than the Ø9 bore, so you load them through the **open top** and close the **press-fit cap**
>    (`cap_*`); only wires / the FSR tail / a Ø2–3 mm pressure tube run down the bore. **MPRLS route:** keep
>    the board in the base and run a tube to the cavity — the cavity is the pressure chamber, so **seal it
>    airtight** (raw FDM TPU is porous: coat the inside or drop in a small bladder). Adhere the ERM to the
>    **inner TPU wall** so its buzz reaches the grip.

## Fasteners — order list (BOM)
The whole tool uses **two thread sizes** — **M6** (robot flange only) and **M3** (everything else) —
plus **one insert size** (M3 heat-set). Screw heads can be **cap (hex/Allen) or cheese (screwdriver)** —
both share the Ø5.5/Ø10 head, so the counterbores fit either. The build here uses **cap-head M3** (Allen).

| Fastener | Spec | Qty | Where / notes |
|----------|------|-----|---------------|
| M6 cap-head screw | **M6 × 16** | 8 | **Flange plate** → into the robot flange's **own tapped holes** (Ø51 pitch circle; Ø7.0 clearance holes). Head sits on a **steel M6 washer** in the Ø13 seat, buried flush in the 16 mm plate. Spans the 8 mm wall + washer → **~6.4 mm thread bite**; **verify the flange tap ≥ 7 mm** (or use M6×18 for ~8 mm). |
| M3 cap-head screw | **M3 × 16** | 6 | **tier 1→flange plate** (Ø100 circle): 10 mm of floor under the head + 6 mm into the plate insert. **Longer than the rest on purpose** — the 5 mm of material v0.6 had under these heads is where it broke. |
| M3 cap-head screw | **M3 × 10** | 7 | 4× cover→tier 2 (Ø120), 3× apple-stem→cover. Thread into heat-set inserts. |
| M3 cap-head screw | **M3 × 16** | 4 | **Tier 2 → tier 1** at the four explicit `tier_screw_pos` points: 5 mm of tier-2 floor + up to 12 mm into the tier-1 insert. |
| M3 cap-head screw | **M3 × 6** | 3 | Casing-box top → cover. **Short on purpose** — the cover pilot is only 5 mm deep in the 6 mm cover, so M3×10 would bottom out and never clamp. |
| M3 brass heat-set insert | **Bambu M3×5×4** (M3, 5.0 mm OD, 4 mm long) | 20 | 6× **flange plate** + 4× **tier-1 pillars** (tier 2) + 4× tier-2 pillars (cover) + 3× cover boss + 3× cover top. Printed hole is **Ø4.6 (`m3_insert`)**; the 4 mm length seats in every pilot (shallowest = 5 mm cover-top). |
| ~~Height-set pin~~ | — | 0 | **Removed in v0.7** — the stem is one piece; height is fixed by `stem_shaft_len`. |
| Steel washer, M6 | flat, OD ~12 (DIN 125) | 8 | **Under each M6 head** in the plate — spreads bolt torque so the printed 8 mm wall can't crush. Seats in the Ø14 recess (`flange_washer_*`). |
| Washer, M3 | small | 3 | Under the casing-top screws — spread the clamp load on the clear sheet. |
| **Hook-and-loop (velcro) pads** | ~2 mm thick | 6 boards | **Every board is stuck down, not screwed.** Pocket depths already allow for the pad (`velcro_t`). |

> **Shopping summary:** **M3 cap-head** — M3×10 (×11), M3×6 (×3, casing), and one **M3×25**;
> **M6×16** (×8) + **8 steel M6 washers**; **M3 heat-set inserts** (**Bambu M3×5×4**, ×14); **M3 nyloc
> nut** (×1, for the height pin) + **3 M3 washers** (casing) + **velcro** for the five boards.
> **Every screw-into-plastic joint takes the same M3 insert; there's no self-tapping and no board screws.**

## Files
| File | What |
|------|------|
| `apple_pluck_end_effector.scad` | the parametric model (base + cover + apple core base + adj shaft + ball + casing + assembly) |
| `README.md` | this file |

## Render / export
Open `apple_pluck_end_effector.scad` in the **OpenSCAD GUI** (Customizer exposes every parameter), or
export each printed part headless:

```bash
openscad -D 'part="flange_plate"'    -o flange_plate.stl    apple_pluck_end_effector.scad   # bolts to the robot
openscad -D 'part="base_tier1"'      -o base_tier1.stl      apple_pluck_end_effector.scad   # lower tier + cable trench
openscad -D 'part="base_tier2"'      -o base_tier2.stl      apple_pluck_end_effector.scad   # upper tier
openscad -D 'part="cover"'           -o cover.stl           apple_pluck_end_effector.scad
# FUSED APPLE — export stem + lower ball, import at the SAME origin in Bambu Studio, assign filaments:
openscad -D 'part="apple_stem"'      -o apple_stem.stl      apple_pluck_end_effector.scad   # PETG (flange + shaft + armature)
openscad -D 'part="apple_ball"'      -o apple_ball.stl      apple_pluck_end_effector.scad   # TPU  (lower ball, moulds onto it)
openscad -D 'part="apple_cap"'       -o apple_cap.stl       apple_pluck_end_effector.scad   # TPU  (press-fit cap, separate print)
# previews only: 'part="assembly"' (default), 'part="apple"' (stem + fused ball + cap),
# 'part="apple_section"' / 'part="section"' (half-cuts), 'part="electronics_mock"' (board/ring fit),
# 'part="casing"' (clear casing box — reference only; you build it by hand from clear acrylic sheet)
```

> The **apple_ball** has a sphere, so on the CGAL backend (OpenSCAD 2021.x) F6/STL render is slow
> (minutes); the **Manifold** backend (2023.06+, *Preferences → Features → Manifold*) renders it in
> seconds. Use F5 preview / `part="apple_section"` to check geometry without the wait.

## ⚠️ Verify before printing
- **Flange interface (`flange_*`) — the pitch circle is Ø51, and that is the ONLY number that must match
  the robot.** Measure it **centre-to-centre** across two diametrically opposite holes. Do **not** set it
  from an outer-edge span: that span is `pcd + hole Ø`, and the plate's hole is deliberately *bigger* than
  the flange's tapped hole, so the two spans are not comparable. **Measuring the plate from the top gives
  ~63 mm — that is the Ø12.5 cap-head counterbore, not the bolt hole**; it sits 10 mm above the mating face
  and never touches the robot.

  | where you measure | feature | Ø | centre-to-centre | outer-edge span |
  |---|---|---|---|---|
  | the **robot flange** | tapped M6 hole | 6 | **51** | 57 |
  | plate, **mating face** (z=0) | M6 clearance hole | **7.0** | **51** | 58 |
  | plate, washer recess (z=8) | washer seat | 14 | 51 | 65 |
  | plate, **top face** (z=16) | M6 head well | **12.5** | 51 | **63.5** ← what you measured |

  The clearance hole is **Ø7.0 (ISO 273 coarse)** on purpose: it absorbs the ±0.5 mm uncertainty in the
  measured pitch circle *and* the fact that FDM holes print 0.2–0.4 mm undersize (a Ø6.6 hole prints ~6.3
  and binds on an M6). The Ø12 steel washer under the head covers it easily.
  *History: Ø62 pcd → 68.6 mm span (measured ~70), would not bolt on. Ø50.4 came from wrongly subtracting
  the plate's Ø6.6 clearance instead of the flange's Ø6 thread — 0.6 mm out, which eats all the M6 slop.*

  Still confirm: the **exact hole clocking** (8 holes evenly at 45°; check `flange_first_angle`), the
  **connector protrusion depth** (drives `flange_center_h` — a 6 mm Ø34 recess in the **plate bottom**),
  and the **M6 tap depth** — the **M6×16** head sits on a steel washer (Ø13 seat) and the shank spans
  `flange_mount_t = 8 mm` + ~1.6 mm washer, leaving **~6.4 mm** of bite, so verify the tap is **≥ 7 mm**
  (use **M6×18** for ~8 mm). **Print the flange plate alone first** — it is small and cheap, and it is the
  only part whose fit to the robot can fail.
- **Central bore is now Ø30** (`cable_bore_d`, was Ø16) — the media-flange power/data bundle comes up
  through it. The **cover** keeps a separate **Ø16** bore (`apple_bore_d`) for the apple's own wiring: a
  Ø30 bore there would swallow the Ø24 centring spigot. Eight **radial hub ports** (Ø6) let the bundle be
  fanned out of the hub in any direction rather than only along the point-to-point channels.
- **Board fit:** all five footprints are locked (see the table). After **any** size change re-check the
  layout — three big rectangles around a central bore collide easily. `part="electronics_mock"` shows
  the boards in place.
- **Stem strength:** the shaft is one piece with the flange now, so there is no fit to test — but the
  **root fillet is the critical feature**. Print the stem solid / high-perimeter and check the fillet
  actually formed; a sharp internal corner there is where it would crack under the 15–20 N pull.
- **Feed bore vs wiring:** `sensor_bore_d = 9 mm` carries the ERM leads, the FSR tail, or a Ø2–3 mm
  pressure tube — the bulky parts load through the press-fit cap, not the bore. Widen it (watch the shaft
  wall) only if your *tail/tube bundle* is fat; tune `cap_lip_clear` for the cap's press-fit on a test print.
- **Strain relief:** add a clamp / grommet at the central bore so cable load isn't on the connectors.

## Printing & finishing
- **Base:** print flat-face down; PETG/PLA+/nylon, ≥40 % infill (structural).
- **Cover:** opaque, same material as the base; print **top-face up** so the ring groove + screw
  counterbores are open on top (the ring drops in from above). The clear casing box shields it.
- **Apple CORE BASE (rigid):** print **flange-down**; high perimeter/infill so the boss + height-pin
  hole are strong.
- **FUSED APPLE (PETG stem + TPU lower ball, one print):** import **apple_stem.stl** and **apple_ball.stl**
  at the same origin in Bambu Studio, assign **PETG** to the shaft and **TPU** to the ball, and print on the
  **H2D (dual nozzle)**. Print it **on its side** (shaft horizontal, support under ball + shaft) or shaft-
  **upright** with a brim — either way the PETG↔TPU boundary is inside the ball. Print the **PETG shaft
  solid / high-perimeter** (load path); the **TPU ball** at low infill stays soft and grippable. Keep the
  ball **rounded and smooth** — no sharp edges or pinch points.
- **APPLE CAP (`apple_cap.stl`, TPU, separate):** the top dome; print **dome-up** (skirt on the bed). It
  **press-fits** onto the lower ball's rim rebate — tune `cap_lip_clear` on a test print; solvent/heat-weld
  or silicone it for a permanent smooth ball. Round the seam. Load the ERM + sensor through the open ball,
  route wires / FSR tail / pressure tube down the shaft bore, then fit the cap.
- **Clear casing box:** built by hand from **clear acrylic sheet** (`case_box_*` params) — a 5-sided box
  **233 × 233** outer, **~36 mm tall**, **open on the flange side**, walls/top **3 mm**. The top has a
  Ø45 centre hole (clears the apple boss) + 3 clamp holes at Ø56; the walls shroud the base down to the
  flange. Set it over the stack so the top covers the ring, then screw the top down with **3× M3 into the
  cover**. Round all edges near the animal. (`part="casing"` exports the box as a dimensional reference;
  cut/bond it from sheet stock.)

## Assembly (after printing)
Everything is bolted (no glue). **Mount the empty base to the robot first**, then populate it — the
boards tuck in over the flange bolts.

> **Two thread sizes only.** **M6×16** at the robot flange (8×, fixed by the flange) and **M3 everywhere
> else** — base↔cover, cover↔core-base, the casing box, all board mounts, and the height pin. The
> single M3 spec lives in `m3_clear / m3_insert / m3_cbore / m3_cbore_h`; every joint derives
> from it, so one box of M3 screws + a bag of M3 heat-set inserts + a strip of M6×16 does the whole build.
> See **Fasteners — order list** above for exact lengths and quantities.

1. **Prepare the threads.** Every screw-into-plastic pilot is modelled at **Ø4.6 (`m3_insert`)** for an
   **M3 heat-set insert (Bambu M3×5×4: 5.0 mm OD, 4 mm long → Ø4.6 hole)** — press one into each:
   **4× flange plate (base mount), 4× base top (cover), 3× cover boss, 3× cover top (casing)** = **14**.
   No board inserts — the boards are velcro'd. No self-tapping anywhere. (The height pin is a separate
   **M3×25 screw + nyloc nut**.)
2. **Bolt the flange plate to the robot — bare.** **8× M6×16** through the plate into the flange's tapped
   holes, each on a steel washer in its Ø13 seat; the heads bury flush in the plate's top face. The Ø34
   recess clears the electric connector. Pull the power/data bundle up the **Ø30** bore.
3. **Bolt TIER 1 onto the plate, THEN populate it.** 6× M3×10 down the Ø80 circle into the plate's
   inserts; the heads recess into the floor. **Do this before any board goes in** — the compartment is
   open, but those screws sit under where boards land. Then dress the tool-connector bundle: bring it
   up the Ø30 bore, drop it into a **radial floor channel**, run it out to the **perimeter trench**,
   and tie it down at the slots. All of it sits below the floor line, so boards go on top of it. Now
   **velcro** the Metro, RS-422 and MPRLS down wherever they suit — clear of the four pillars — and
   land the CTR pairs on the RS-422 screw terminals.
4. **Stack TIER 2 and populate it.** Drop tier 2 on and drive **4× M3×16** down at the four
   `tier_screw_pos` points, into the pillars in tier 1 (heads recess into tier 2's floor). Pass the
   24 V pair up the central riser first. Then velcro the **Tobsun, level shifter and DRV2605L** down —
   again, anywhere clear of tier 2's four pillars — and fan **5 V out from the converter** to every
   board, back **down the riser to tier 1**, and up to the ring, using the radial floor channels.
   Leave a service loop at the riser; run the ERM + sensor leads and the pressure tube up the centre.
5. **Seat the ring + close the cover.** Butt the **4 quarter-arcs** into the full ring and drop it into
   the cover's **top groove** (LEDs up). The groove is 8.5 mm wide against the ring's 6 mm — **seat the
   arcs pushed OUTWARD, against the outer wall**, which is where the extra circumference is and what
   lets the four segments actually close. Solder the arc-to-arc joints and feed each arc's power/data
   leads down its **quarter pass-through**. Then lower the cover and drive the **4× M3** at **Ø138**
   (inside the ring, not at the rim) down into tier 2.
6. **Fit the casing box.** Lower the clear box over the stack (open side down to the flange) so its top
   covers the ring, line up the 3 top holes with the **Ø56** pilots, and drive **3× M3** down into the
   cover. (Do this before the apple parts — the box's Ø45 top hole won't pass over an assembled apple.)
7. **Attach the apple stem.** It pokes up through the box's top centre hole; set its flange on the cover
   boss (the **spigot** centres it) and drive the **3× M3** down into the cover boss. Pass the apple
   wiring up through its bore. The ball is already fused to it — there is nothing to slide or pin.
8. **Load the electronics + close the ball.** With the **cap off**, seat the **ERM (Ø10)** against the inner
   TPU wall and the **force sensor** in the open cavity (FSR head against the wall behind a backing; or, for
   the MPRLS route, just the pressure tube — sensor stays in the base). Route the leads / FSR tail / tube
   **down the stem bore**, then **press the cap on** (silicone it if needed). Confirm the **cap + ball
   can't pull off** by hand.
9. **Connect + commission.** Connect the media-flange power/data, **re-calibrate the tool load** in
   Sunrise (electronics + apple add mass — see main repo README §2 "Tool Load Data"), and first
   power-up / move in **T1** with a hand on the E-stop.

Disassembly is the reverse: pull the apple cap → withdraw the apple electronics via the bore → unbolt the
stem (3× M3) → unscrew the casing box → cover → tier 2 (4× M3, boards can stay velcro'd) → tier 1. No glue
in the stack; the stem + fused ball is one printed part (reprint to change the apple height).

## Print settings

Geometry did not break the last one on its own. A flat-printed disc under a bending moment fails
**along its layer lines**, and every number below is aimed at that.

### Every tier is in the pull path

Worth being explicit, because it is easy to assume only the bottom of the stack is structural. A pull
at the apple travels **apple → stem → cover → tier 2 → tier 1 → flange plate → robot**. All 4.2 N·m
of it passes through every one of those joints. What differs is the lever each joint reacts it on:

| Joint | Bolt circle | Force per screw |
|---|---|---|
| **apple stem → cover** | **Ø36** | **78 N** |
| cover → tier 2 | Ø120 | 18 N |
| tier 2 → tier 1 | Ø150 | 14 N |
| tier 1 → flange plate | Ø100 | 14 N |

**The most-loaded joint in the tool is the apple stem to the cover**, at 5.6× the screw force of the
one that actually broke — purely because the same moment is reacted across a third of the diameter.
That is why the cover is printed structural and why v0.15 took it from 6 mm to 9 mm.

Tier 2 carries the same moment but on a 10 mm floor spanning only 15 mm from pillar to tier screw,
which works out at **SF ≈ 24**. It does not need the full structural profile — but it is not
free-standing decoration either, so do not go below the settings in the table.

### The three things that matter most

1. **Dry the filament.** PETG is hygroscopic and wet PETG loses a large fraction of its layer
   adhesion — it prints with a rough, foamy texture and snaps cleanly between layers. If the
   prototype sat on an open spool for weeks, this alone could explain the failure. Dry at 65 °C for
   6–8 h and print from a dry box.
2. **Solid layers beat infill.** A plate in bending works like an I-beam: the top and bottom solid
   skins carry almost all the stress and the infill mostly just holds them apart. Adding top/bottom
   layers buys far more than adding infill.
3. **Run hot, cool little.** Layer adhesion rises with nozzle temperature and falls with part
   cooling. For the load-bearing parts go to the top of the PETG range and keep the fan low.

### Per part

| Part | Material | Layer | Walls | Top / bottom | Infill | Notes |
|---|---|---|---|---|---|---|
| **flange_plate** | PETG (**PAHT-CF / PET-CF if you have it**) | 0.20 mm | **8** | **8 / 8** | **60 %** gyroid | Highest-stressed part. Mating face down. |
| **base_tier1** | PETG (**PAHT-CF / PET-CF if you have it**) | 0.20 mm | **8** | **8 / 8** | **50 %** gyroid | **This is the one that broke.** Floor down on the plate. |
| **base_tier2** | PETG | 0.20 mm | **6** | **6 / 6** | **35 %** gyroid | In the load path (see below), but with large margins. |
| **cover** | PETG | 0.20 mm | **6** | **8 / 8** | **40 %** gyroid | **The apple bolts to this.** Opaque filament — not translucent. |
| **apple_stem** | PETG | **0.15 mm** | 6 | 6 / 6 | **100 %** | See below — this one is orientation-critical. |
| **apple_ball / apple_cap** | TPU 95A | 0.20 mm | 3 | 4 / 4 | 15 % gyroid | Slow (≤ 30 mm/s), retraction near zero. |

**Temperatures.** PETG: nozzle **250 °C**, bed 80 °C, **fan 20–30 %** (not 100 %). TPU: nozzle
230 °C, bed 45 °C, fan 50 %. Enclosure closed for both, no draught.

### The apple stem needs a decision

The stem is a 100 mm cantilever, and printed upright its layers lie **perpendicular to the bending
stress** — the worst possible orientation. Upright is also what the dual-material print with the TPU
ball requires. So:

- Keep it upright for the fused print, and lean hard on layer adhesion: **0.15 mm layers, 255 °C,
  fan off for the first 20 mm above the flange**, 100 % infill. Print it alone on the plate so each
  layer has time to cool evenly without the fan.
- If it still breaks at the root, print the stem **separately and lying down** (strong axis), and
  bond the TPU ball on mechanically — the Ø22 armature flange already carries the pull in bearing,
  not adhesion, so a separately printed ball is not a downgrade.

### Do not

- **Do not use PLA** for the plate or tier 1. Stiffer than PETG on paper, but brittle — it fails
  suddenly rather than bending, which is the wrong failure mode next to an animal.
- **Do not reduce the wall count to save time** on the plate or tier 1. Walls and solid skins are
  doing the structural work; infill percentage is the least important number in the table.

### Bambu Studio, step by step (H2D)

Sizes, so you know what you are looking at on the plate:

| Part | Footprint | Height | Material | Prints with |
|---|---|---|---|---|
| `flange_plate` | Ø124 | 16 | PETG | — |
| `base_tier1` | Ø188 | 33 | PETG | — |
| `base_tier2` | Ø188 | 32 | PETG | — |
| `cover` | Ø188 | 17 | PETG | — |
| `apple_stem` | Ø44 | 113.5 | PETG | **fused with `apple_ball`** |
| `apple_ball` | Ø45 | 27.4 | TPU 95A | **fused with `apple_stem`** |
| `apple_cap` | Ø43 | 22.5 | TPU 95A | — |
| `casing` | 198 × 198 | 74 | *not printed* | acrylic, by hand |

#### 0. Before you open Bambu Studio

- **Dry the PETG.** 65 °C for 6–8 h. This matters more than any slicer setting — see above.
- **Use the textured PEI plate for PETG.** PETG bonds *too well* to smooth PEI and can tear the
  coating off. If you only have smooth PEI, put a glue-stick layer down as a release agent.
- **TPU must come off the external spool holder, not the AMS.** Soft filament buckles in the AMS
  path.
- Optional but worth it for `base_tier1` and `flange_plate`: fit the **0.6 mm nozzle**. Fatter
  extrusions bond better between layers, which is precisely the failure mode being designed against.
  Nothing in this design needs finer than 0.6 — the smallest features are Ø3.4 holes.

#### 1. Make a process preset once, reuse it

Load any part, then in the right-hand parameter panel:

- **Quality → Layer height** `0.20`
- **Strength → Wall loops** `8`
- **Strength → Top shell layers** `8` · **Bottom shell layers** `8`
- **Strength → Sparse infill density** `50 %` · **Sparse infill pattern** `Gyroid`
- **Others → Brim type** `Outer brim only`, **Brim width** `5 mm`

Save it as a preset (the 💾 next to the process dropdown) called something like
`0.20 Structural PETG`. Now each part below is just "load this preset, change these two things".

#### 2. Filament preset

Duplicate the Bambu PETG profile and save it as `PETG - high strength`:

- **Filament → Nozzle temperature** `250 °C` (both layers)
- **Filament → Bed temperature** `80 °C`
- **Cooling → Minimum fan speed** `20 %` · **Maximum fan speed** `30 %`
- **Cooling → Keep fan always on** OFF

The low fan is deliberate. Part cooling makes PETG look nicer and bond worse, and every structural
part here fails along layer boundaries.

#### 3. Per part

**`flange_plate`** — highest-stressed part.
1. Import, **Place on bed**. It lands mating-face down, which is what you want.
2. Process: `0.20 Structural PETG`, change **Sparse infill density → 60 %**.
3. No supports. The M6 head wells are flat-bottomed and print fine.

**`base_tier1`** — the part that broke last time.
1. Import, **Place on bed** (floor down).
2. Process: `0.20 Structural PETG` as-is (8 walls, 8/8 shells, 50 %).
3. No supports needed: the trenches, channels and bore ports are all open-topped or short bridges.
4. Print it **alone on the plate**. It is the one part where a failed layer matters.

**`base_tier2`** — same shape, lower stress but still in the pull path.
1. Import, **Place on bed**.
2. Process: `0.20 Structural PETG`, then reduce **Wall loops → 5**, **Top/Bottom shell layers → 6**,
   **Sparse infill density → 30 %**.

**`cover`** — **more structural than it looks; the apple bolts to it.**
1. Import, **Place on bed** (ring groove facing up).
2. Process: **Wall loops 6**, **Top/Bottom 8**, **Sparse infill 40 %**. The top and bottom shells are
   the ones doing the work here: a plate in bending behaves like an I-beam and the solid skins carry
   almost all of it, so shell count buys more than infill percentage.
3. Use an **opaque** filament. The cover is the light barrier behind the ring — a translucent one
   will glow.

**`apple_cap`** — TPU, prints alone.
1. Import. It arrives at assembly height; **Place on bed** drops it. It should sit **skirt down,
   dome up**. Check this — dome-down needs supports and ruins the press-fit surface.
2. Assign the **TPU filament / nozzle**.
3. **Wall loops 3**, **Top/Bottom 4**, **Sparse infill 15 %**, **Layer height 0.20**.
4. **Speed → set everything ≤ 30 mm/s.** TPU does not survive fast corners.
5. Retraction near zero in the TPU filament profile.

#### 4. The fused apple — the only tricky one

`apple_stem` (PETG) and `apple_ball` (TPU) are modelled in one coordinate frame and **must stay
aligned**. Do not import them separately and let auto-arrange move them.

1. **File → Import → Import 3MF/STL**, select **both** `apple_stem.stl` and `apple_ball.stl` at once.
2. When asked *"Multiple objects detected — load as a single object with multiple parts?"* choose
   **Yes**. Their relative positions are now locked.
3. In the object list, expand the object. Assign:
   - `apple_stem` → the **PETG** nozzle
   - `apple_ball` → the **TPU** nozzle
4. **Place on bed.** The stem's Ø24 centring spigot lands first — a small footprint for a 131 mm
   tall print, so: **Others → Brim type `Outer brim only`, Brim width `8 mm`.**
5. Process changes for this plate:
   - **Quality → Layer height `0.15`** (more layers, more bond area at the root)
   - **Strength → Sparse infill density `100 %`** — the shaft is Ø14 with a Ø9 bore, so it is nearly
     all perimeter anyway
   - In the **PETG filament preset for this plate only**: **Minimum and Maximum fan speed `0 %`**.
     Layer adhesion up the shaft is the whole ball game here.
6. **Print it alone on the plate.** Nothing else should be stealing layer time.
7. No supports. The ball's lower cup is fused to the shaft and its overhang is progressive.

> **If it still snaps at the shaft root:** printed upright, the layers lie perpendicular to the
> bending stress — the worst possible orientation, and one no slicer setting fully fixes. Fall back
> to printing `apple_stem` **separately, lying flat** (strong axis, needs supports under the flange)
> and fitting the ball mechanically. The Ø22 armature flange carries the pull in *bearing*, not
> adhesion, so a separately printed ball costs nothing structurally.

#### 5. Check on the first layer

- **Brim stuck down all the way round** on the apple print — that part is top-heavy.
- **No gaps at the Ø30 bore wall** on `base_tier1`. It is a 4 mm ring and the first layer is where
  under-extrusion shows.
- If the first layer looks glassy and translucent rather than matte, the bed is too hot or the
  filament is wet.

## Safety (animal subject + electronics)
- Lightweight keeps tool inertia low (better impedance behavior; gentler on contact) — the electronics
  add mass, so re-check the FRI load data / tool calibration after fitting (main repo README §2).
- The apple sits on a **thin Ø14 PETG shaft** (with a Ø22 flange anchoring the TPU ball), now integral
  with the cover flange and filleted R6 at the root. Re-check the margin after any change to
  `stem_shaft_len`; print the stem solid, and confirm the **TPU ball can't pull off the flange by hand**
  before use.
- Keep **5 V/24 V wiring** sealed under the cover and inside the clear casing box, strain-relieved;
  nothing the animal can reach or pull. The box now shrouds the sides down to the flange (no exposed
  outer edge), but it's still clamped only at the centre — bond or edge-screw the box to the flange if
  the animal could lever it.
- The cabinet's Cartesian impedance + Sunrise safety limits are the real safety layer — this tool just
  needs to be smooth, light, robustly attached (all flange bolts torqued), and electrically safe.
- First mount/move in **T1** with a hand on the E-stop.

## Reference
Barra et al., *A versatile robotic platform for … reaching and grasping tasks in monkeys*,
J. Neural Eng. 17(1):016004 (2019/2020) — same iiwa + macaque platform; silicone-over-3D-print objects
with integrated grip sensing. (Their Zenodo deposit is software-only — no object CAD — so this tool is
drawn from scratch.)
