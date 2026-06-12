# Apple-Pluck End-Effector (parametric, DRAFT v0.6)

A 3D-printable tool that bolts to the **KUKA LBR iiwa7 media flange (electric)** and presents a
compliant **"apple"** for the monkey to pull. It is also an **electronics hub**: it routes the
media-flange power/data, drives a **NeoPixel ring** visual cue, and carries the control board, the
DC-DC converter, and two small breakouts, with wiring extending to an **ERM/LRA actuator** and a
**pressure sensor** in the apple. Authored in **OpenSCAD** so it's text-based, parametric, and
version-controlled.

> **v0.6 — what changed**
> - **No more extension.** The apple **core base bolts straight to the cover**; the height is set by a
>   separate **adjustable shaft** (a detent pin in one of several holes → discrete height steps).
> - **Ring moved to the cover *top*.** It seats LEDs-up in a groove on the cover's top face. The cover
>   is now **opaque (same material as the base)** — no translucent skin.
> - **Plexiglass shroud.** A user-cut clear casing seats in a groove on the base rim and shields the
>   electronics + ring; the apple shaft exits the top. (The .scad models the seat + a preview tube.)
> - **Five boards in the base** (was two): Metro M4, PSM-B05 converter, an **optocoupler** (your
>   board, ≤ 70 × 34 mm), Pixel Shifter, DRV2605L. Three big boards sit at 0°/90°/180°; the two small
>   breakouts tuck into the 270° quadrant.
> - **Power distribution.** The 24 V media-flange bundle goes to the converter; **5 V is then fanned
>   out from the converter** to every board and up the centre riser (ring + apple), with data fanned
>   out from the Metro — real channels through the pocket dividers, not just the central bore.

## Design at a glance
The apple is three printed parts that stack on the cover; the **base** is the electronics hub:

```
[ iiwa7 media flange (electric) ]   8× mounting screws · 24 V + data bundle
        │  cable bundle THROUGH the base centre
  (1) BASE HUB        (Ø170) 8-hole flange mount + central cable bore; FIVE board pockets:
        ├─ Metro M4 AirLift (4000)         ┐ three big boards at 0°/90°/180°,
        ├─ PSM-B05 24 V→5 V converter      │ long side tangential, tucked to centre
        ├─ Optocoupler (your board ≤70×34) ┘ (12 o'clock, pushed out past the others)
        ├─ Pixel Shifter (6066)            ┐ two small breakouts in the 270° quadrant
        ├─ DRV2605L ERM/LRA driver (2305)  ┘
        ├─ POWER DISTRIBUTION channels: 24 V bore → converter, then 5 V fanned OUT to every
        │   board + the centre riser; data fanned out from the Metro board
        └─ seat groove on the rim for the plexiglass shroud
  (2) COVER           seats the NeoPixel ring (1768) in a groove on its TOP (LEDs up, opaque cover) ·
                      closes the board pockets · 4× M3 down to the base · lead pass-through to the base
        ▼  BOLTED FLANGE: apple CORE BASE screws down into the cover (3× M3 + centring spigot)
  (3a) APPLE CORE BASE  flange + a sleeve BOSS with a transverse HEIGHT-SET pin hole; wiring bore
        ╪  HEIGHT PIN: choose which shaft detent hole the boss pin engages → apple height
  (3b) ADJ SHAFT        rod with a column of DETENT holes (height steps) + a side window + ball lock hole
        ╪  CROSS-PIN / M3: the ball slides over the shaft top and locks transversely
  (3c) APPLE BALL       hollow grippable sphere (soft); socket sleeves the shaft top; interior CAVITY
                        holds the ERM/LRA actuator + pressure sensor
  ( + ) PLEXIGLASS CASING  user-cut clear shroud, seated in the base-rim groove (shaft exits the top)
```

> **Joints.** The cover↔core-base joint is a **bolted flange** (Ø44, 3×M3 on a bolt circle, centred by
> a spigot, cable bore down the middle). The flange fits **inside** the ring ID (Ø52), so the ring and
> the apple mount share the cover top.
>
> **Adjustable height.** The core base has a sleeve **boss** with one transverse pin hole; the shaft
> has a **column of detent holes** (`adj_n` holes, `adj_step` apart → ~32 mm of range in 8 mm steps).
> Slide the shaft to the wanted height, line up a hole, drop the **Ø3 pin** (or M3). The pin carries
> the pull-out load in **double shear** — positive, won't slip.
>
> **Core↔ball lock.** The ball's socket sleeves the shaft top; a second transverse **Ø3 dowel / M3**
> through ball + shaft locks it (low, in the ball's solid lower shell, below the cavity floor).

## Power / data flow
```
24 V (media flange) ─► PSM-B05 24 V→5 V ─► 5 V ─┬─► Metro M4 AirLift board
                                                 ├─► NeoPixel ring   (via Pixel Shifter: 3.3 V data → 5 V)
                                                 ├─► DRV2605L driver ─► ERM/LRA motor (actuator, apple cavity)
                                                 └─► pressure sensor (apple cavity)
   data: Metro 3.3 V ─► Pixel Shifter ─► ring DIN ;  Metro I²C ─► DRV2605L ;  sensor ─► Metro ADC/I²C
```
The **Metro M4 Express AirLift** runs everything: it drives the NeoPixel ring through the **Pixel
Shifter** (its 3.3 V data needs shifting to 5 V), commands the **DRV2605L** over I²C to run the
haptic **actuator** (an ERM/LRA motor in the apple), and reads the **pressure sensor**. The
**PSM-B05** steps the arm's 24 V down to 5 V; that **5 V rail is distributed from the converter** out
to every board and up the centre riser (ring + apple) through channels in the base. All five boards
live in the base; only the ERM motor + pressure sensor sit up in the apple cavity.

## Components (dimensions locked from datasheets)
| Item | Part / link | Size (mm) | Drives parameter |
|------|-------------|-----------|------------------|
| NeoPixel ring, 24×5050 RGB | [Adafruit 1768](https://www.adafruit.com/product/1768) | Ø66 / Ø52 | `ring_od`, `ring_id`, `ring_groove_h` |
| Control board | [Metro M4 Express AirLift Lite (4000)](https://www.adafruit.com/product/4000) | 72 × 54 × 15 | `board_l/w`, `board_clear_h` |
| DC-DC converter | [PSM-B05-1224-05](https://abra-electronics.com/power-supplies-transformers-adapters/dc-dc-step-down-converters/psm-b05-1224-05-12v-24v-to-5v-5a-dc-dc-converter-step-down-regulator.html) (12/24 V→5 V 5 A) | 63 × 53 × 20 | `conv_l/w`, `conv_clear_h` |
| NeoPixel level shifter | [Pixel Shifter (6066)](https://www.adafruit.com/product/6066) (3.3→5 V data) | 25.5 × 15 × 10.2 | `shifter_*` |
| Actuator driver | [DRV2605L (2305)](https://www.adafruit.com/product/2305) (ERM/LRA driver, I²C) | 25.8 × 17.8 × 4.6 | `haptic_*` |
| **Optocoupler** | **your board — buy to fit** | **≤ 70 × 34** | `opto_l/w` |
| Actuator | **ERM/LRA vibration motor** (your part) | small coin/cyl | apple cavity |
| Pressure sensor | (your part) | — | apple cavity |
| Ball↔shaft lock + height pin | Ø3 dowel pin **or** M3 screw ×2 | — | `lock_pin_d`, `hpin_d` |

> **Fit notes (read before printing):**
> 1. **Base is Ø170.** Three big boards in tangential pockets at 0°/90°/180° (Metro, opto, converter)
>    and the two small breakouts in the 270° quadrant. Run `part="electronics_mock"` after any size
>    change to re-check overlaps.
> 2. **Optocoupler — buy to fit ≤ 70 × 34 mm.** It sits at 12 o'clock, pushed out past the Metro +
>    converter (their ±36 mm tangential reach leaves a long-shallow slot: ~70 mm side-to-side × ~34 mm
>    in-out). A full M4-size opto does **not** fit flat on the compact base — that needs ~Ø200 or
>    stacking. If you get a bigger board, tell me and I'll grow/stack it.
> 3. **Metro mounting holes are not symmetric.** The standoffs in `base()` are placeholders; set their
>    XY to the real Arduino-Metro footprint before printing.
> 4. **Mount the empty base to the robot first.** The 8 media-flange bolts (r≈31) fall under the
>    boards; their heads counterbore on the robot side, so install electronics *after* bolting on.
> 5. **Thin shaft + close apple (by design).** The shaft is **Ø14** with a **Ø8 bore** and the boss is
>    **Ø20** — both sit well inside the ring ID (r≈10 vs ring r25), so they barely occlude the ring at
>    shallow viewing angles. The apple sits **~17–38 mm above the core base** (default ~24 mm, set by
>    the detent hole) — just enough to wrap a hand around the Ø45 ball without hitting the base.
> 6. **Shaft strength — OK for the 15–20 N pull.** Even thinned to Ø14/Ø8-bore, the shorter moment arm
>    (~60 mm at default height) gives ≈5–6 MPa bending vs ~30–50 MPa for a tough printed filament →
>    **safety factor ~6–8**, ~1 mm deflection; the detent pin sees ~1 MPa shear. Print it **solid in
>    nylon/PETG**. Swap in a **metal Ø14 rod** (same bore + detent holes) for near-zero flex.
> 7. **Apple cavity** is the annulus around the Ø14 shaft inside the Ø45 ball (~12 mm radial gap) —
>    room for a small ERM/LRA motor + leads. A flat sensor may want a bigger `apple_d`.

## Files
| File | What |
|------|------|
| `apple_pluck_end_effector.scad` | the parametric model (base + cover + apple core base + adj shaft + ball + casing + assembly) |
| `README.md` | this file |

## Render / export
Open `apple_pluck_end_effector.scad` in the **OpenSCAD GUI** (Customizer exposes every parameter), or
export each printed part headless:

```bash
openscad -D 'part="base"'            -o base.stl            apple_pluck_end_effector.scad
openscad -D 'part="cover"'           -o cover.stl           apple_pluck_end_effector.scad
openscad -D 'part="apple_core_base"' -o apple_core_base.stl apple_pluck_end_effector.scad   # rigid material
openscad -D 'part="adj_shaft"'       -o adj_shaft.stl       apple_pluck_end_effector.scad   # rigid/tough material
openscad -D 'part="apple_ball"'      -o apple_ball.stl      apple_pluck_end_effector.scad   # soft material
# previews only: 'part="assembly"' (default), 'part="apple"' (core base + shaft + ball),
# 'part="apple_section"' / 'part="section"' (half-cuts), 'part="electronics_mock"' (board/ring fit),
# 'part="casing"' (plexiglass shroud — reference only; you'll cut this from sheet/tube)
```

> The **apple_ball** has a sphere, so on the CGAL backend (OpenSCAD 2021.x) F6/STL render is slow
> (minutes); the **Manifold** backend (2023.06+, *Preferences → Features → Manifold*) renders it in
> seconds. Use F5 preview / `part="apple_section"` to check geometry without the wait.

## ⚠️ Verify before printing
- **Flange interface (`flange_*`):** the 8-hole pattern, **PCD**, **screw size**, counterbore, the
  **first-hole angle**, and the **centering-boss recess** are *nominal*. Confirm against the **iiwa7
  media-flange-electric** datasheet, then print a **thin fit-test ring** (reduce `base_h`) to check the
  bolt pattern + centering before printing the full hub.
- **Board fit:** the real Metro M4 (72 × 54) + PSM-B05 (63 × 53) drive the Ø165 base; use
  `part="electronics_mock"` to check overlaps and adjust `*_radius` / `*_angle`.
- **Shaft fit / strength:** the boss bore is `adj_shaft_d + adj_fit_clear`; print a short fit-test and
  confirm the shaft slides snugly and the detent holes align with the boss pin. See fit-note 4 on load.
- **Strain relief:** add a clamp / grommet at the central bore so cable load isn't on the connectors.

## Printing & finishing
- **Base:** print flat-face down; PETG/PLA+/nylon, ≥40 % infill (structural).
- **Cover:** opaque, same material as the base; print **top-face up** so the ring groove + screw
  counterbores are open on top (the ring drops in from above). The plexiglass shields it.
- **Apple CORE BASE (rigid):** print **flange-down**; high perimeter/infill so the boss + height-pin
  hole are strong.
- **ADJ SHAFT (rigid/tough):** print **upright** (no support); this is the load path — use nylon/PETG,
  solid or near-solid, or substitute a metal rod (see fit-note 4).
- **Apple BALL (soft):** print in a **flexible/grippable filament (TPU)** or as a thin shell
  over-moulded with **food-safe silicone**. Print **socket-up** (stem on the bed) so the socket and
  grooves come out clean. Keep it **rounded and smooth** — no sharp edges or pinch points.
- **Fit:** the ball socket is `adj_shaft_d + 2·ball_fit_clear`; TPU/silicone shrink differently than
  rigid prints, so print a short socket fit-test first and adjust `ball_fit_clear` so the ball slides
  on snugly and the **lock holes line up**.
- **Plexiglass casing:** user-fabricated from clear acrylic tube/sheet to seat in the base-rim groove
  (`case_*` params); the apple shaft exits the top. Smooth/round all edges near the animal.

## Assembly (after printing)
Everything is bolted (no glue). **Mount the empty base to the robot first**, then populate it — the
boards tuck in over the flange bolts.

1. **Prepare the threads.** Screw pilots are sized for self-tapping M3 — run an M3 in once, or
   (recommended) press an **M3 heat-set insert** into each: 4× base↔cover, 3× cover↔core-base. For
   inserts, open the relevant `*_pilot` first. (The height pin + ball lock are separate cross-pins.)
2. **Mount the empty base to the robot.** Bolt the base to the iiwa7 media flange (**8 screws**,
   engaging the centering recess) *before* fitting electronics. Pull the power/data bundle up the bore.
3. **Populate the base.** Seat the **PSM-B05** and **DRV2605L** in their pockets; mount the **Metro M4**,
   the **optocoupler**, and the **Pixel Shifter** on standoffs (M3). Route the harness through the
   distribution channels: **24 V → PSM-B05**, then **5 V from the converter → every board + the centre
   riser** (ring + apple); **data from the Metro → opto / shifter / DRV / centre riser**. Leave a
   service loop at the centre riser; run the ERM-motor + sensor leads up the centre.
4. **Seat the ring + close the cover.** Drop the **NeoPixel ring** into the cover's **top groove**
   (LEDs up), feed its leads down the pass-through to the board, then lower the cover and drive the
   **4× M3** through the cover rim into the base.
5. **Attach the apple core base.** Set its flange on the cover boss (the **spigot** centres it) and
   drive the **3× M3** down into the cover. Pass the apple wiring up through its bore.
6. **Set the shaft height.** Feed the apple leads up the shaft bore, slide the **adjustable shaft**
   into the core-base boss to the wanted height, line up a **detent hole** with the boss pin hole, and
   drop in the **Ø3 height pin** (or M3). Confirm it can't slide.
7. **Fit the ball.** Seat the **ERM motor + pressure sensor** in the ball cavity (leads out the shaft
   side window). Slide the soft **ball** over the shaft top until its **lock hole** lines up, then pin
   it (**Ø3 dowel** press-fit or **M3**). Confirm the ball can't pull off by hand.
8. **Fit the plexiglass shroud.** Seat the clear casing in the base-rim groove (shaft through the top).
9. **Connect + commission.** Connect the media-flange power/data, **re-calibrate the tool load** in
   Sunrise (electronics + apple add mass — see main repo README §2 "Tool Load Data"), and first
   power-up / move in **T1** with a hand on the E-stop.

Disassembly is the reverse: lift off the shroud → pull the ball lock pin + ball → pull the height pin
+ shaft → unbolt the core base → cover. No glue anywhere; the soft ball and shaft stay replaceable.

## Safety (animal subject + electronics)
- Lightweight keeps tool inertia low (better impedance behavior; gentler on contact) — the electronics
  add mass, so re-check the FRI load data / tool calibration after fitting (main repo README §2).
- The apple sits on a **thin Ø14 shaft**; it's sized with ~6–8× margin for the 15–20 N pull (fit-note
  6), but re-check after any height/length change and print it solid in a tough material.
- Keep **5 V/24 V wiring** enclosed and strain-relieved behind the plexiglass; nothing the animal can
  reach or pull.
- The cabinet's Cartesian impedance + Sunrise safety limits are the real safety layer — this tool just
  needs to be smooth, light, robustly attached (all flange bolts torqued), and electrically safe.
- First mount/move in **T1** with a hand on the E-stop.

## Reference
Barra et al., *A versatile robotic platform for … reaching and grasping tasks in monkeys*,
J. Neural Eng. 17(1):016004 (2019/2020) — same iiwa + macaque platform; silicone-over-3D-print objects
with integrated grip sensing. (Their Zenodo deposit is software-only — no object CAD — so this tool is
drawn from scratch.)
