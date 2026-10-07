# Apple-Pluck End-Effector (parametric)

A 3D-printable tool that bolts to the **KUKA LBR iiwa7 media flange (electric)** and presents a
compliant **"apple"** for the monkey to pull. It is also an **electronics hub**: it routes the
media-flange power/data, drives a **NeoPixel ring** visual cue, and carries the control board, the
DC-DC converter, and small breakouts, with wiring extending to an **ERM/LRA actuator** and a
**pressure sensor** in the apple. Authored in **OpenSCAD** so it's text-based, parametric, and
version-controlled. `check_layout.py` verifies the geometry against the `.scad`; run it after any
size change.

## Design at a glance
The tool is a stack of four printed discs — flange plate, two base tiers, cover — with the apple on
top and a clear casing box over the lot:

```
[ iiwa7 media flange (electric) ]   8× M6 on Ø51 (pitch circle) · Ø34 electric bore · 24 V + data bundle
  (0) FLANGE PLATE    Ø124 × 16. Takes the 8× M6 — heads + washers BURIED so its top face is flat —
                      plus the Ø30 cable bore. Tier 1 bolts onto IT with 6× M3 on a Ø100 circle
                      (first at 15°). Bolt the plate to the robot BARE, then drop the base on: the
                      printed base never carries the M6 clamp load, and you never reach past boards
                      to a flange bolt.
        │  cable bundle up the Ø30 centre
  (1a) BASE TIER 1    Ø188 × 33 (14 mm floor + 19 mm compartment). Ø30 central bore — the flange
                      bundle lands HERE. ONE open compartment, r 19 → 88, flat floor. Standing in it:
                      4× Ø12 screw pillars on a SYMMETRIC Ø150 bolt circle (45/135/225/315).
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
  (1b) BASE TIER 2    Ø188 × 32 (10 mm floor + 22 mm compartment). Stacks on tier 1 with 4× M3×14
                      into tier 1's pillars. Ø20 bore — only ring/apple wiring goes higher, and those
                      5 mm of radius per side are what let the 70 × 65 Tobsun fit inside Ø188 at all.
                      ONE open compartment, r 14 → 88, flat floor (no channels).
        ├─ 4× Ø12 PILLARS on Ø120 (45/135/225/315) carrying the COVER inserts
        ├─ 4× Ø10 RADIAL BORE PORTS at 0/90/180/270 — apple / ring wiring out of the riser
        ├─ 4× Ø10 DROPS through the floor at r 58 (0/90/180/270) — tier-2 → tier-1 wiring
        └─ reference boards: Tobsun 70 × 65 (22 tall — sets the tier depth) · level shifter 36 × 28 ·
           DRV2605L 28 × 20
  (2) COVER           Ø188 × 9. Seats the NeoPixel ring in a Ø165/152 groove on its TOP (LEDs up,
                      opaque cover) · closes tier 2 · 4× M3 down into tier 2 at Ø120 — INSIDE the
                      ring · 4 ring-lead pass-throughs (one per quarter) · 3× M3 pilots on Ø64 for
                      the casing box · Ø44 boss (R4 fillet to the plate) for the apple stem
        ▼  BOLTED FLANGE: apple STEM screws down into the cover boss (3× M3 on Ø36 + Ø24 centring spigot)
  (3a) APPLE STEM     ONE PETG part: Ø44 cover flange + 100 mm shaft TAPERING Ø24 → Ø14 (Ø9 feed bore,
                      R2 root fillet), then a Ø20 SEAT and a Ø12 TIP up through the ball. Apple height is FIXED
                      by stem_shaft_len — reprint to change it. Apple centre sits 212 mm above the flange.
  (3b) TPU BALL       Ø45, printed alone. Flat base sits on the seat; Ø12 hole for the tip; solid floor;
                      OPEN top loads the ERM + force sensor. 45° teardrop neck, so it prints unsupported.
  (3c) RETAINER       small PETG ring (Ø22 × 3 + Ø16 hub), dropped in through the open top and GLUED onto
                      the tip — it clamps the ball's floor down onto the seat.
  (3d) TPU CAP        press-fit dome closes the ball; wires / FSR tail / pressure tube exit the bore
  ( + ) CLEAR CASING BOX  198 × 198, 5-sided (open on the flange side), over the whole stack; its top
                      clamps with 3× M3 into the cover on Ø64
```

> **Joints.** The cover↔stem joint is a **bolted flange** (Ø44, 3× M3 on a Ø36 bolt circle, centred by
> a Ø24 spigot, cable bore down the middle). The flange sits at the centre of the big ring (ID Ø145), with
> plenty of clear space between the apple mount and the ring.
>
> **Fixed height.** The flange, tapered shaft, seat and tip are **one PETG part** (`apple_stem`). One piece has no radial slop to rock on; height is set by
> **`stem_shaft_len` (100 mm)** — change it and reprint.
>
> **The apple prints as four single-material parts:** stem (PETG), ball and
> cap (TPU), retainer (PETG). The ball slides down the tip onto the stem's **seat**; the **retainer**
> drops in through the open top and is **glued onto the tip** — PETG to PETG, a strong bond — clamping the
> ball's solid floor between retainer and seat. **Pulled, the floor bears on the retainer; pushed, the
> base bears on the seat.** No load rides on a TPU bond (TPU barely bonds to anything). Then load the
> ERM + force sensor through the open top, route their wires / FSR tail / pressure tube down the **Ø9
> bore**, and press the cap on.


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
**Tobsun converter** steps the 24 V down to 5 V; that **5 V rail is distributed from it** out
to every board and up the centre riser (ring + apple). All the boards live in the base; only the
ERM motor + pressure sensor sit up in the apple cavity.

## The media flange is a pass-through — the as-built wiring

This robot has the **Media flange Inside electric**: two supply voltages, two analog/CAT5
interfaces, an internal connector, and **no electronics of its own** (KUKA media-flange manual V10,
20 Oct 2021, §2.1.9). It is a conduit from interface A1 at the rear of the base frame (Fig. 5-1) to
the tool connector at the wrist.

**On this arm:**

```
   [ robot base, interface A1, rear of base frame ]

     X31  ══ robot data cable ══► Sunrise cabinet          connected
     X651 ══ KUKA data cable X650/X651 ══► cabinet         connected
          └─ carries 24 V + GND (pins 5/6) and EtherCAT (pins 9-12), Fig. 5-63
                                     │
     X76  ── your trigger wiring ────┤                     12+3 bypack fitted
                                     ▼
   [ tool connector at the flange face ]   16-way breakout, ~44 cm
```

Three consequences that shape the whole design:

1. **The 24 V is already there, from the cabinet.** The X650/X651 data cable puts 24 V on X651
   pin 5 and GND on pin 6, which the flange passes to **tool connector pins 1/2**. The Metro board
   and the ring both run from it; no external PSU is needed. (KUKA's parts list for this flange
   calls for a *connector bypack* on X651 rather than the data cable, so this is an off-book but
   functionally clean configuration — the pin roles line up.)
2. **EtherCAT is live at the wrist**, on tool pins 3-6. Keep those insulated. An EtherCAT slave at
   the tool would give cabinet-native timing and a return channel for sensors — a hardware option
   not pursued at present.
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
| 7 | RD | Power2 | X76 A | 60 V / 5 A — unused; fallback supply if the cabinet 24 V won't carry the ring |
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
A swap does not matter for a bare contact closure but is fatal for a polarised receiver.
While you are here, short tool 9 to tool 10 and measure across X76 1–2 for the **round-trip
resistance** — every voltage-drop estimate depends on that number.

**3. Passive jumper test — no source, no voltage.** Wire tool pin 9 → Metro **D2** and tool pin 10 →
Metro **GND**. The firmware's `active_low` default plus D2's internal pull-up means a bare short is a
valid trigger.

```bash
ros2 run sinthlab_bringup check_cue_wiring.py          # or: python3 diagnostics/check_cue_wiring.py
```

It pre-flights the board (catches `enabled=0`, an invisible cue, and **inverted polarity**), then
announces every edge as you make and break the short at X76. This step verifies
`CUE_PIN = board.D2` against real hardware.

> **Stop condition:** no edges means wiring, not firmware. The tool prints the likely causes ranked.

**4. Fit the RS‑422 link (parts on order).** The cabinet cannot drive the trigger — the Sunrise
project has no generated I/O groups — so the ROS computer drives it, over a **full‑duplex RS‑422
serial link**. The same link carries the pressure sensor's data off the tool:

| | Part |
|---|---|
| ROS box | StarTech **ICUSB422IS** — isolated USB↔RS‑422, FTDI FT232RL |
| Tool | MikroE **MIKROE‑2821** RS485 3 Click — SN65HVD31, full duplex, 3.3 V |

The MikroE's screw terminals take the CTR pairs from the tool connector; its header pins go to the
Metro's UART (3.3V, GND, TX/D1, RX/D0 — **check D0/D1 are free** on the AirLift Lite before wiring).
It's a **crossover**: the adapter's TX pair lands on the MikroE's RX pair and vice versa. Put the
differential signals on the **shielded** CTR pairs. Set the FTDI `latency_timer` to 1 on the ROS box,
or you get 16 ms of buffering. Full detail in
[§6.7 of the top-level README](../README.md#67-end-effector-board--the-visual-cue).

The jumper test in step 3 is the first check of the harness either way — it proves continuity
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
| DC-DC converter | **Tobsun 24 V→5 V** | **70 × 65** | `conv_l/w`, `conv_pos` |
| NeoPixel level shifter | [Pixel Shifter (6066)](https://www.adafruit.com/product/6066) (3.3→5 V data) | **36 × 28** | `shifter_*` |
| Actuator driver | [DRV2605L (2305)](https://www.adafruit.com/product/2305) (ERM/LRA driver, I²C) | **28 × 20** | `haptic_*` |
| **RS-422 transceiver** | [MIKROE-2821 RS485 3 Click](https://www.mikroe.com/rs485-3-click) — SN65HVD31, full duplex, 3.3 V | **42.9 × 25.4** (footprint 45 × 28) | `rs422_l/w`, `rs422_pos` |
| Haptic actuator | [Vibrating Mini Motor Disc (1201)](https://www.adafruit.com/product/1201) (coin ERM) | **Ø10 × 2.7** | apple cavity |
| Force sensor **A** | [FSR — Alpha MF01A round, high-force 1–98 N (5475)](https://www.adafruit.com/product/5475) | **Ø18 head** (~Ø15 active) · 60 long · 0.56 thin | apple cavity (head) + tail down bore |
| Force sensor **B** | [MPRLS ported pressure sensor (3965)](https://www.adafruit.com/product/3965), I²C 0–25 PSI | board **17.8 × 16.7 × 7.5** (footprint 20 × 19) · port Ø2.5 | `mprls_*` on **tier 1** + Ø2–3 tube up the riser to a *sealed* cavity |

> **Fit notes (read before printing):**
> 1. **Base is Ø188, and the RING is why — not a board.** The groove outer sits at r = 83.5; add solid
>    rim and a wall and you arrive at r = 94. Run **`python3 check_layout.py`** after *any* size change:
>    the tightest fit is the Tobsun at **1.6 mm** of margin (corner r 86.4 against 88), and the script
>    checks every board, screw, port, drop and trench.
> 2. **Two tiers, and the bore sizes are load-bearing decisions.** Tier 1 has the Ø30 bore because the
>    flange bundle lands there. Tier 2 has Ø20 because only ring/apple wiring goes higher — and those
>    5 mm of radius per side are exactly what lets the 70 × 65 Tobsun fit. At Ø30 its corners land at
>    **r = 90.1** and it does **not** fit inside Ø188 in any orientation. Any growth in `hub_wall`,
>    `conv_l/w` or `tier2_bore_d` means the base grows or the Tobsun and Metro swap tiers.
> 3. **The layout is Cartesian, not polar.** Large rectangles reaching in toward the bore cannot be
>    placed on a bolt circle: at the hub their angular widths exceed 360°. Each board has an explicit
>    `*_pos` = [x, y] and `*_rot`.
> 4. **Every screw lands on a pillar.** Each compartment is open, so there is no solid top face for a
>    heat-set insert; each tier carries **4× Ø12 pillars** from its floor to its top — tier 1's on
>    Ø150 at 45/135/225/315 (taking the tier-2 screws), tier 2's on Ø120 at the same angles (taking the
>    cover screws). Both are **symmetric bolt circles**, so the clamp load is centred. They are the
>    only obstructions in either compartment; the checker holds every reference board clear of them,
>    verifies even spacing, confirms they sit between the trenches and clear of the channels, and
>    that the tier-2 pillars (to r 66) clear the tier-1 screw counterbores (from r 71.75).
> 5. **Cover screws are INSIDE the ring (Ø120).** Outboard is not possible: the gap between the
>    groove outer (r 83.5) and the rim inner (r 91) is 7.5 mm, and an M3 counterbore is 6.5 wide. At
>    r = 60 the counterbore ends at 63.25, clear of the groove at 75.
> 6. **The ring groove is deliberately oversized.** The channel is **8.5 mm wide against the ring's
>    6 mm**, so the arcs can sit anywhere from hard against the inner wall (r 72.5–78.5) out to hard
>    against the outer (r 77.5–83.5). Sitting further out is what buys the circumference: at r 79 the
>    60 LEDs need **496 mm** of arc against **474 mm** at r 75.5 — about three LEDs, enough to stop four
>    butted quarter-arcs closing. **Seat the segments pushed outward.**
> 7. **No pockets and no board screws.** Each tier is ONE open compartment (r 19 → 88 on tier 1,
>    r 14 → 88 on tier 2) with a flat floor; every board is **velcro'd** wherever it fits. The board
>    dimensions in the .scad are a *reference placement* that `check_layout.py` validates and
>    `electronics_mock` draws — they do not shape the print, so you can move or swap a board on the
>    bench without reprinting anything. Each `*_clear` is the height that board needs, and the tallest
>    on each tier sets the compartment depth: Metro 19 → `comp1_depth`, Tobsun 22 → `comp2_depth`.
> 8. **Every wire has a hole to get through.** Each bore is a 4 mm tube the full height of its tier:
>    - **4× Ø10 radial ports** in tier 1's bore wall and **4× Ø10** in tier 2's, all with their bottoms
>      flush to the compartment floor so wire leaves lying flat rather than climbing over the hub.
>    - On tier 1 two of the four are **clocked onto the feed channels** (`hub_port_a0 == radial_a0`,
>      enforced by the checker), so bore, port and channel form one continuous opening out to the
>      trench, 10 mm wide from the channel floor up.
>    - **4× Ø10 drops** through tier 2's floor at r 58 (0/90/180/270, between the pillars) for
>      tier-2 → tier-1 wiring. The reference Tobsun placement sits over the drop at 90°; the other
>      three are clear, or slide the Tobsun.
>
>    The ports take a large share of each hub's circumference, and the hub is also part of the face
>    the tier above lands on; `check_layout.py` caps it. Drop `t2_port_d` to 8 if tier 2's hub prints
>    badly.
> 9. **Cable management runs UNDER the boards, and splits the bundle in two.** Tier 1 has **two
>    perimeter trenches** (r 66→86, 74° each, centred at 0° and 180°, **98 mm of run apiece**, 3 tie
>    slots each), each fed by its **own 10 mm channel** from the central bore. The channels sit at
>    30°/210° — a single diagonal through the bore — and each meets its trench at an **END**, so the
>    bundle enters at one end and runs the full length. Split the bundle in half at the bore and dress
>    one half into each side; each trench fills the gap between two screw pillars (4.4 mm of wall to
>    the nearest pillar), so nothing has to get past a pillar. All of it is **sunk 6 mm below the
>    compartment floor**, so a board velcros straight over a cable run, with 8 mm of solid floor left
>    underneath. **Tier 2 has no channels** — nothing there needs routing under a board.
> 10. **Assembly order is forced: plate → robot → tier 1 → boards → tier 2 → boards → cover.** The 8 M6
>    live in the **separate flange plate**, driven with the plate bare. Tier 1 then bolts down with
>    **6× M3×14 on the Ø100 circle (first at 15°)**, their heads recessed into the floor. The
>    compartment is open so every head is in plain sight — but they sit **under where boards go**, so
>    drive them before you velcro anything down.
> 11. **Tapered shaft, close apple.** The stem **tapers from Ø24 at the cover flange to Ø14 under the
>    ball** (`stem_base_d` → `stem_shaft_d`), with a **Ø9 feed bore** and an R2 root fillet. The base is
>    as wide as it can be: base + fillet reach r 14, just inside the joint screws' counterbores (r 14.75),
>    which need straight-down access for the key. It sits at the very centre, far inside the ring (ID r ≈ 75), so it doesn't occlude the
>    LEDs. Height is **fixed** at `stem_shaft_len = 100` (apple centre 212 mm above the robot flange).
> 12. **Shaft strength and stiffness.** The pull's bending moment is largest at the root, which is
>    why the shaft is thickest there: Ø24 is **8.6× stiffer** than a straight Ø14 (stiffness goes with
>    Ø⁴), so the apple wobbles far less. A 20 N pull at the ball now peaks at about **2.4 MPa** of
>    bending stress, near the top of the shaft (Ø15.5), against ~30–50 MPa for PETG — a safety factor
>    above **10** (a straight Ø14 shaft reached 10 MPa at the root). Print the **stem solid /
>    high-perimeter**. The ball is held by the **Ø22 retainer** bearing on its solid TPU
>    floor, not by any bond to the TPU.
> 13. **Apple cavity + access.** The cavity (dome ≈ **Ø39 × 31 mm** above the flange) fits the **ERM
>    (Ø10)**, the **FSR head (Ø18)**, or the **MPRLS board (17.8 mm)** with room to spare — but all are
>    bigger than the Ø9 bore, so you load them through the **open top** and close the **press-fit cap**
>    (`cap_*`); only wires / the FSR tail / a Ø2–3 mm pressure tube run down the bore. **MPRLS route:**
>    keep the board in the base and run a tube to the cavity — the cavity is the pressure chamber, so
>    **seal it airtight** (raw FDM TPU is porous: coat the inside or drop in a small bladder). Adhere
>    the ERM to the **inner TPU wall** so its buzz reaches the grip.

## Strength — the pull path

A pull at the apple travels **apple → stem → cover → tier 2 → tier 1 → flange plate → robot**. A 20 N
pull is about **4.2 N·m** of moment, and all of it passes through every one of those joints. What
differs is the bolt circle each joint reacts it on:

| Joint | Bolt circle | Force per screw (tension side) |
|---|---|---|
| **apple stem → cover** | **Ø36** | **78 N** |
| cover → tier 2 | Ø120 | 18 N |
| tier 2 → tier 1 | Ø150 | 14 N |
| tier 1 → flange plate | Ø100 | 14 N |

**The most-loaded joint in the tool is the apple stem to the cover** — the same moment reacted across
a third of the diameter. So the cover is structural:

- `cover_plate_t` is **9 mm**: bending just outside the Ø44 boss is ~7 MPa (**SF ≈ 7** on bulk PETG,
  nearer 4–5 once flat-print layer strength is allowed for);
- **4.5 mm** of material under the ring groove and **4 mm** under each casing insert;
- an **R4 fillet** where the boss meets the plate, instead of a sharp step.

**At the base↔flange joint** the screws carry only ~14 N each; what matters is the printed material
around them:

- **10 mm of floor under each M3 head** (hence **M3×14** there: 10 through the floor + the full 4 mm insert), and 8 mm where a cable channel
  crosses;
- a **14 mm** tier-1 floor;
- a **Ø124** plate, which supports tier 1's floor so it overhangs by only 32 mm;
- a **Ø100** bolt circle — wider circle, lower force per screw;
- a **4 mm** hub wall around the bore ports;
- **R3 fillets** at every compartment-wall and pillar-base corner. A flat-printed disc cracks
  along its layers, and sharp internal corners are where that starts.

Tier 2 carries the same moment but on a 10 mm floor spanning only 15 mm from pillar to tier screw,
which works out at **SF ≈ 24**. It does not need the full structural profile — but it is in the
load path, so do not go below its print settings.

**Geometry is only half of this — see [Print settings](#print-settings).** The discs print flat, so
the moment at each bolt circle pulls directly against the layer bonds, the weakest axis by a wide
margin.

## Fasteners — order list (BOM)
The whole tool uses **two thread sizes** — **M6** (robot flange only) and **M3** (everything else) —
plus **one insert size** (M3 heat-set). Screw heads can be **cap (hex/Allen) or cheese (screwdriver)** —
both share the Ø5.5/Ø10 head, so the counterbores fit either. The build here uses **cap-head M3** (Allen).

| Fastener | Spec | Qty | Where / notes |
|----------|------|-----|---------------|
| M6 cap-head screw | **M6 × 16** | 8 | **Flange plate** → into the robot flange's **own tapped holes** (Ø51 pitch circle; Ø7.0 clearance holes). Head sits on a **steel M6 washer** in the Ø14 seat, buried flush in the 16 mm plate. Spans the 8 mm wall + washer → **~6.4 mm thread bite**; **verify the flange tap ≥ 7 mm** (or use M6×18 for ~8 mm). |
| M3 cap-head screw | **M3 × 14** | 14 | **One length for the base and cover joints.** Each engages its whole 4 mm insert and stops short of the hole bottom: **tier 1 → flange plate** (6×, Ø100: 10 mm of floor under the head — the critical section of the base joint — + 4 into the 6 mm plate hole, 2 mm spare); **tier 2 → tier 1** (4×, Ø150: 5 + 9 into the 14 mm pillar hole); **cover → tier 2** (4×, Ø120: 6 of the 9 mm cover + 8 into the 14 mm hole). **Not M3 × 16** at the plate: it needs the whole 6 mm hole, so it bottoms out before the head clamps and leaves a gap. |
| M3 cap-head screw | **M3 × 10** | 3 | **Apple stem → cover boss** (Ø36): 5 mm of the stem flange under the head + 5 mm into the boss (7 mm bore). **Not longer** — an M3 × 12 or longer bottoms out in the 7 mm hole and lifts the stem off the cover. |
| M3 cap-head screw | **M3 × 6** | 3 | Casing-box top → cover (Ø64). **Short on purpose** — the cover pilot is only 5 mm deep, so M3×10 would bottom out and never clamp. |
| M3 brass heat-set insert | **Bambu M3×5×4** (M3, 5.0 mm OD, 4 mm long) | 20 | 6× **flange plate** + 4× **tier-1 pillars** (tier 2) + 4× tier-2 pillars (cover) + 3× cover boss (stem) + 3× cover top (casing). Printed hole is **Ø4.6 (`m3_insert`)**; the 4 mm length seats in every pilot (shallowest = 5 mm cover top). |
| Steel washer, M6 | flat, OD ~12 (DIN 125) | 8 | **Under each M6 head** in the plate — spreads bolt torque so the printed 8 mm wall can't crush. Seats in the Ø14 recess (`flange_washer_*`). |
| Washer, M3 | small | 3 | Under the casing-top screws — spread the clamp load on the clear sheet. |
| **Hook-and-loop (velcro) pads** | ~2 mm thick | 6 boards | **Every board is stuck down, not screwed.** Board heights already allow for the pad (`velcro_t`). |

> **Shopping summary:** **M3 cap-head** — M3×14 (×14), M3×10 (×3), M3×6 (×3); **M6×16** (×8) +
> **8 steel M6 washers**; **M3 heat-set inserts** (**Bambu M3×5×4**, ×20); **3 M3 washers** (casing)
> + **velcro** for the six boards. **Every screw-into-plastic joint takes the same M3 insert; there is
> no self-tapping and there are no board screws.**

## Files
| File | What |
|------|------|
| `apple_pluck_end_effector.scad` | the parametric model (flange plate, both base tiers, cover, apple stem, ball, cap, casing, assembly) |
| `check_layout.py` | verifies board fit, pillars, ports, drops, trenches, screw clearances and the strength section against the `.scad` |
| `README.md` | this file |

## Render / export
Open `apple_pluck_end_effector.scad` in the **OpenSCAD GUI** (Customizer exposes every parameter), or
export each printed part headless:

```bash
openscad -D 'part="flange_plate"'    -o flange_plate.stl    apple_pluck_end_effector.scad   # bolts to the robot
openscad -D 'part="base_tier1"'      -o base_tier1.stl      apple_pluck_end_effector.scad   # lower tier + cable trenches
openscad -D 'part="base_tier2"'      -o base_tier2.stl      apple_pluck_end_effector.scad   # upper tier
openscad -D 'part="cover"'           -o cover.stl           apple_pluck_end_effector.scad
# APPLE — four single-material prints:
openscad -D 'part="apple_stem"'      -o apple_stem.stl      apple_pluck_end_effector.scad   # PETG (flange + shaft + seat + tip)
openscad -D 'part="apple_ball"'      -o apple_ball.stl      apple_pluck_end_effector.scad   # TPU  (lower ball)
openscad -D 'part="apple_retainer"'  -o apple_retainer.stl  apple_pluck_end_effector.scad   # PETG (glued onto the tip)
openscad -D 'part="apple_cap"'       -o apple_cap.stl       apple_pluck_end_effector.scad   # TPU  (press-fit cap)
# previews only: 'part="assembly"' (default), 'part="apple"' (stem + ball + retainer + cap),
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
  ~63 mm — that is the Ø12.5 cap-head well, not the bolt hole**; it sits 10 mm above the mating face
  and never touches the robot.

  | where you measure | feature | Ø | centre-to-centre | outer-edge span |
  |---|---|---|---|---|
  | the **robot flange** | tapped M6 hole | 6 | **51** | 57 |
  | plate, **mating face** (z=0) | M6 clearance hole | **7.0** | **51** | 58 |
  | plate, washer recess (z=8) | washer seat | 14 | 51 | 65 |
  | plate, **top face** (z=16) | M6 head well | **12.5** | 51 | **63.5** |

  The clearance hole is **Ø7.0 (ISO 273 coarse)** on purpose: it absorbs the ±0.5 mm uncertainty in the
  measured pitch circle *and* the fact that FDM holes print 0.2–0.4 mm undersize (a Ø6.6 hole prints ~6.3
  and binds on an M6). The Ø12 steel washer under the head covers it easily. Set the pitch circle from
  the flange's **tapped** holes (Ø6), never from the plate's clearance holes.

  Also confirm: the **exact hole clocking** (8 holes evenly at 45°; check `flange_first_angle`), the
  **connector protrusion depth** (drives `flange_center_h` — a 6 mm Ø34 recess in the **plate bottom**),
  and the **M6 tap depth** — the **M6×16** head sits on a steel washer (Ø14 seat) and the shank spans
  `flange_mount_t = 8 mm` + ~1.6 mm washer, leaving **~6.4 mm** of bite, so verify the tap is **≥ 7 mm**
  (use **M6×18** for ~8 mm). **Print the flange plate alone first** — it is small and cheap, and it is the
  only part whose fit to the robot can fail.
- **Bores.** The base's central bore is **Ø30** (`cable_bore_d`) — the media-flange power/data bundle
  comes up through it. Tier 2's is **Ø20** (`tier2_bore_d`). The **cover** and stem have a separate
  **Ø16** bore (`apple_bore_d`) for the apple's own wiring: a Ø30 bore there would swallow the Ø24
  centring spigot.
- **Board fit:** all footprints are locked (see the table). After **any** size change run
  `check_layout.py` — big rectangles around a central bore collide easily. `part="electronics_mock"`
  shows the boards in place.
- **Stem strength:** the shaft is one piece with the flange and tapers from Ø24, so there is no joint to
  test there. Print the stem solid / high-perimeter. **If the apple still wobbles,** check the stem's
  3 screws are **M3 × 10** (longer ones bottom out in the 7 mm holes and never clamp the flange) and
  that the flange sits flat on the cover.
- **Feed bore vs wiring:** `sensor_bore_d = 9 mm` carries the ERM leads, the FSR tail, or a Ø2–3 mm
  pressure tube — the bulky parts load through the press-fit cap, not the bore. Widen it (watch the shaft
  wall) only if your *tail/tube bundle* is fat; tune `cap_lip_clear` for the cap's press-fit, and `tip_clear` / `ret_glue_clear` for the ball and
  retainer on the tip, on test prints.
- **Strain relief:** add a clamp / grommet at the central bore so cable load isn't on the connectors.

## Printing & finishing
- **Flange plate, base tiers, cover:** print flat, face down, in PETG — see
  [Print settings](#print-settings) for the per-part walls, shells and infill.
- **Cover:** opaque, same material as the base; print **top-face up** so the ring groove + screw
  counterbores are open on top (the ring drops in from above). The clear casing box shields it.
- **APPLE STEM (`apple_stem.stl`, PETG):** print it **lying on its side** — the strong axis for a
  100 mm cantilever — with tree supports under the flange and seat; or upright with the layer-adhesion
  settings in [Print settings](#the-apple-stem--lay-it-down). Print it **solid** (load path).
- **APPLE BALL (`apple_ball.stl`, TPU):** prints **upright on its flat base**, no supports (the
  underside is a 45° neck). Low infill keeps it soft and grippable. Keep it **rounded and smooth** — no
  sharp edges or pinch points.
- **RETAINER (`apple_retainer.stl`, PETG):** flat face down, solid. Tiny — put it on the stem's plate.
- **APPLE CAP (`apple_cap.stl`, TPU, separate):** the top dome; print **dome-up** (skirt on the bed). It
  **press-fits** onto the lower ball's rim rebate — tune `cap_lip_clear` on a test print. To hold it
  against pulls, run a thin bead of **neutral-cure silicone (RTV)** round the skirt: flexible, and it peels
  off cleanly when you need inside. Hot glue works too, but a standard gun (~190 °C) can soften the TPU —
  use a low-temperature gun, and know it is harder to undo. Round the seam. Load the ERM + sensor through the open ball,
  route wires / FSR tail / pressure tube down the shaft bore, then fit the cap.
- **Clear casing box:** built by hand from **clear acrylic sheet** (`case_box_*` params) — a 5-sided box
  **198 × 198** outer, walls reaching from the cover top down to the flange plate (~74 mm) plus a
  **3 mm** top, **open on the flange side**. The top has a **Ø44.8** centre hole (clears the apple stem
  boss) and 3 clamp holes on **Ø64**. It clears the Ø188 base by 2 mm a side. Set it over the stack so the
  top covers the ring, then screw the top down with **3× M3 into the cover**. Round all edges near the
  animal. (`part="casing"` exports the box as a dimensional reference; cut/bond it from sheet stock.)

## Assembly (after printing)
Everything is bolted (no glue). **Mount the empty flange plate and tier 1 to the robot first**, then
populate — the boards go in over the screw heads.

> **Two thread sizes only.** **M6×16** at the robot flange (8×, fixed by the flange) and **M3 everywhere
> else**. The single M3 spec lives in `m3_clear / m3_insert / m3_cbore / m3_cbore_h`; every joint derives
> from it. See **Fasteners — order list** above for exact lengths and quantities.

1. **Prepare the threads.** Every screw-into-plastic pilot is modelled at **Ø4.6 (`m3_insert`)** for an
   **M3 heat-set insert (Bambu M3×5×4)** — press one into each: **6× flange plate, 4× tier-1 pillars,
   4× tier-2 pillars, 3× cover boss, 3× cover top = 20**. No board inserts — the boards are velcro'd.
2. **Bolt the flange plate to the robot — bare.** **8× M6×16** through the plate into the flange's tapped
   holes, each on a steel washer in its Ø14 seat; the heads bury flush in the plate's top face. The Ø34
   recess clears the electric connector. Pull the power/data bundle up the **Ø30** bore.
3. **Bolt TIER 1 onto the plate, THEN populate it.** **6× M3×14** down the Ø100 circle into the plate's
   inserts; the heads recess into the floor. **Do this before any board goes in** — those screws sit
   under where boards land. Then dress the tool-connector bundle: bring it up the Ø30 bore, split it,
   and run each half out through its **bore port and feed channel** into a **perimeter trench**; tie it
   down at the slots. All of it sits below the floor line, so boards go on top of it. Now **velcro**
   the Metro, RS-422 and MPRLS down wherever they suit — clear of the four pillars — and land the CTR
   pairs on the RS-422 screw terminals.
4. **Stack TIER 2 and populate it.** Drop tier 2 on and drive **4× M3×14** down at 45/135/225/315 into
   tier 1's pillars (heads recess into tier 2's floor). Pass the 24 V pair up the central riser first.
   Then velcro the **Tobsun, level shifter and DRV2605L** down — anywhere clear of tier 2's four
   pillars — and fan **5 V out from the converter** to every board, back **down through a drop to
   tier 1**, and up to the ring. Leave a service loop at the riser; run the ERM + sensor leads and the
   pressure tube up the centre.
5. **Seat the ring + close the cover.** Butt the **4 quarter-arcs** into the full ring and drop it into
   the cover's **top groove** (LEDs up). The groove is 8.5 mm wide against the ring's 6 mm — **seat the
   arcs pushed OUTWARD, against the outer wall**, which is where the extra circumference is and what
   lets the four segments close. Solder the arc-to-arc joints and feed each arc's power/data leads down
   its **quarter pass-through**. Then lower the cover and drive the **4× M3×14** at **Ø120** (inside the
   ring) down into tier 2's pillars.
6. **Fit the casing box.** Lower the clear box over the stack (open side down to the flange) so its top
   covers the ring, line up the 3 top holes with the **Ø64** pilots, and drive **3× M3×6** (with washers)
   down into the cover. Do this before the apple — the box's Ø44.8 top hole won't pass over an
   assembled apple.
7. **Attach the apple stem, then the ball.** The stem pokes up through the box's top centre hole; set its
   flange on the cover boss (the **spigot** centres it) and drive the **3× M3×10** down into the cover boss.
   Pass the apple wiring up through its bore. Then:
   - **Dry-fit first.** Slide the **ball** down the tip until its flat base sits on the **seat**, and the
     **retainer** onto the tip through the open top: it should sit flat in the floor recess, flush with
     the tip top. Ease a tight fit by sanding, not by forcing the TPU.
   - **Glue the retainer.** Lift it off, put **thin CA (superglue) or 5-minute epoxy** round the top
     ~6 mm of the tip, push the retainer down hard so it clamps the floor, and hold it 30 s. Keep glue
     off the TPU and **out of the Ø9 bore** (run the wiring first, or plug the bore with a wire). Let it
     cure fully (CA: 1 h; epoxy: per the pack) before any pull.
8. **Load the electronics + close the ball.** With the **cap off**, seat the **ERM (Ø10)** against the inner
   TPU wall and the **force sensor** in the open cavity (FSR head against the wall behind a backing; or, for
   the MPRLS route, just the pressure tube — sensor stays in the base). Route the leads / FSR tail / tube
   **down the stem bore**, then **press the cap on** (a bead of silicone if pulls unseat it — see
   [Printing & finishing](#printing--finishing)). Confirm the **cap + ball can't pull off** by hand.
9. **Connect + commission.** Connect the media-flange power/data, **re-calibrate the tool load** in
   Sunrise (electronics + apple add mass — see main repo README §2 "Tool Load Data"), and first
   power-up / move in **T1** with a hand on the E-stop.

Disassembly is the reverse: pull the apple cap → withdraw the apple electronics via the bore → unbolt the
stem (3× M3) → unscrew the casing box → cover → tier 2 (4× M3, boards can stay velcro'd) → tier 1. The only
glue in the stack is the retainer on the stem tip: the ball, retainer and stem come off as one piece
(reprint the stem to change the apple height).

## Print settings

A flat-printed disc under a bending moment fails **along its layer lines**, and every number below is
aimed at that. Every part from the cover down is in the pull path
([Strength](#strength--the-pull-path)).

### The three things that matter most

1. **Dry the filament.** PETG is hygroscopic and wet PETG loses a large fraction of its layer
   adhesion — it prints with a rough, foamy texture and snaps cleanly between layers. Dry at 65 °C
   for 6–8 h and print from a dry box.
2. **Solid layers beat infill.** A plate in bending works like an I-beam: the top and bottom solid
   skins carry almost all the stress and the infill mostly just holds them apart. Adding top/bottom
   layers buys far more than adding infill.
3. **Run hot, cool little.** Layer adhesion rises with nozzle temperature and falls with part
   cooling. For the load-bearing parts go to the top of the PETG range and keep the fan low.

### Per part

| Part | Material | Layer | Walls | Top / bottom | Infill | Notes |
|---|---|---|---|---|---|---|
| **flange_plate** | PETG (**PAHT-CF / PET-CF if you have it**) | 0.20 mm | **8** | **8 / 8** | **60 %** gyroid | Highest-stressed part. Mating face down. |
| **base_tier1** | PETG (**PAHT-CF / PET-CF if you have it**) | 0.20 mm | **8** | **8 / 8** | **50 %** gyroid | Carries the base↔flange joint. Floor down on the plate. |
| **base_tier2** | PETG | 0.20 mm | **6** | **6 / 6** | **35 %** gyroid | In the load path, with large margins. |
| **cover** | PETG | 0.20 mm | **6** | **8 / 8** | **40 %** gyroid | **The apple bolts to this.** Opaque filament — not translucent. |
| **apple_stem** | PETG | **0.15 mm** | 6 | 6 / 6 | **100 %** | See below — this one is orientation-critical. |
| **apple_retainer** | PETG | 0.15 mm | 4 | 4 / 4 | **100 %** | Flat face down. |
| **apple_ball / apple_cap** | TPU 95A | 0.20 mm | 3 | 4 / 4 | 15 % gyroid | Slow (≤ 30 mm/s), retraction near zero. |

**Temperatures.** PETG: nozzle **250 °C**, bed 80 °C, **fan 20–30 %** (not 100 %). TPU: nozzle
230 °C, bed 45 °C, fan 50 %. Enclosure closed for both, no draught.

### The apple stem — lay it down

The stem is a 100 mm cantilever. Printed upright its layers lie **perpendicular to the bending
stress** — the worst possible orientation. Printed on its own (the separate build), it can lie down:

- **Lying on its side (recommended):** layers run along the shaft, its strong axis. Tree supports
  under the Ø44 flange and the Ø20 seat; brim on. Check the root fillet came out clean.
- **Upright, if you must:** lean hard on layer adhesion — **0.15 mm layers, 255 °C, fan off for the
  first 20 mm above the flange**, 100 % infill, alone on the plate, **brim 8 mm** (the Ø24 spigot is a
  small footprint for a 120 mm part).

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
| `apple_stem` | Ø44 | 116.5 | PETG | — (lie it down) |
| `apple_ball` | Ø45 | 27.8 | TPU 95A | — |
| `apple_retainer` | Ø22 | 6 | PETG | the stem's plate |
| `apple_cap` | Ø43 | 22.5 | TPU 95A | — |
| `casing` | 198 × 198 | 77 | *not printed* | acrylic, by hand |

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

**`base_tier1`** — carries the base↔flange joint.
1. Import, **Place on bed** (floor down).
2. Process: `0.20 Structural PETG` as-is (8 walls, 8/8 shells, 50 %).
3. No supports needed: the trenches, channels and bore ports are all open-topped or short bridges.
4. Print it **alone on the plate**. It is the one part where a failed layer matters most.

**`base_tier2`** — same shape, lower stress but still in the pull path.
1. Import, **Place on bed**.
2. Process: `0.20 Structural PETG`, then set **Wall loops → 6**, **Top/Bottom shell layers → 6**,
   **Sparse infill density → 35 %**.

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

#### 4. The apple — stem, ball, retainer

**`apple_stem` + `apple_retainer`** — PETG, one plate.
1. Import both. Rotate the **stem onto its side** (90° about X), **Place on bed**; the retainer
   **flat face down**.
2. **Support → Enable, type `tree(auto)`**, on build plate only — under the flange and the seat.
   **Others → Brim `8 mm`.**
3. **Layer height `0.15`**, **Sparse infill `100 %`**, **Wall loops 6**. Dry PETG, 250–255 °C, fan low.
4. Clean the supports off the flange underside and seat; the seat's top face is what the ball sits on,
   so keep it flat.

**`apple_ball`** — TPU, prints alone.
1. Import, **Place on bed**: it stands on its **flat base** (the neck pointing down, the open top up).
2. **No supports** — the underside is a 45° cone. **Brim `5 mm`**: the base is a small ring under a
   45 mm ball.
3. TPU settings as the cap: **Wall loops 3**, **Top/Bottom 4**, **infill 15 %**, **≤ 30 mm/s**,
   retraction near zero.

#### 5. Check on the first layer

- **Brim stuck down all the way round** on the stem and the ball — both stand on small footprints.
- **No gaps at the Ø30 bore wall** on `base_tier1`. It is a 4 mm ring and the first layer is where
  under-extrusion shows.
- If the first layer looks glassy and translucent rather than matte, the bed is too hot or the
  filament is wet.

## Safety (animal subject + electronics)
- Lightweight keeps tool inertia low (better impedance behaviour; gentler on contact) — the electronics
  add mass, so re-check the FRI load data / tool calibration after fitting (main repo README §2).
- The apple sits on a **PETG shaft tapering Ø24 → Ø14** (with a Ø22 retainer anchoring the TPU ball),
  integral with the cover flange. Re-check the margin after any change to
  `stem_shaft_len`; print the stem solid, and confirm the **TPU ball can't pull off the stem by hand**
  before use.
- Keep **5 V/24 V wiring** sealed under the cover and inside the clear casing box, strain-relieved;
  nothing the animal can reach or pull. The box shrouds the sides down to the flange plate, but it is
  clamped only at the centre — bond or edge-screw it to the plate if the animal could lever it.
- The cabinet's Cartesian impedance + Sunrise safety limits are the real safety layer — this tool just
  needs to be smooth, light, robustly attached (all flange bolts torqued), and electrically safe.
- First mount/move in **T1** with a hand on the E-stop.

## Reference
Barra et al., *A versatile robotic platform for … reaching and grasping tasks in monkeys*,
J. Neural Eng. 17(1):016004 (2019/2020) — same iiwa + macaque platform; silicone-over-3D-print objects
with integrated grip sensing. (Their Zenodo deposit is software-only — no object CAD — so this tool is
drawn from scratch.)
