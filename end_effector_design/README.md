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
> - **Big ring on the cover *top*.** The NeoPixel ring is now the large **Ø157 (6.2″) 60-LED RGBW**
>   ring (Adafruit 2874 — buy **4× quarter-arcs**). It seats LEDs-up in a groove near the cover rim.
>   The cover is **opaque (same material as the base)** — no translucent skin.
> - **Clear casing box.** A hand-built **5-sided clear box, open on the arm-flange side**, shrouds the
>   whole electronics stack. Its top lies over the ring and clamps by **3× M3 into the cover just outside
>   the apple-core boss**; the 4 walls drop to the flange face. The apple pokes up through its centre.
>   (The .scad models the box + a preview; the STL is a build reference — you make it by hand from acrylic.)
> - **Five boards in the base** (was two): Metro M4, PSM-B05 converter, an **optocoupler** (your
>   board, ≤ 70 × 34 mm), Pixel Shifter, DRV2605L. Three big boards sit at 0°/90°/180°; the two small
>   breakouts tuck into the 270° quadrant.
> - **Power distribution.** The 24 V media-flange bundle goes to the converter; **5 V is then fanned
>   out from the converter** to every board and up the centre riser (ring + apple), with data fanned
>   out from the Metro — real channels through the pocket dividers, not just the central bore.

## Design at a glance
The apple is three printed parts that stack on the cover; the **base** is the electronics hub:

```
[ iiwa7 media flange (electric) ]   8× M6 on Ø62 · Ø34 electric bore · 24 V + data bundle
        │  cable bundle THROUGH the base centre
  (1) BASE HUB        (Ø170) 8×M6 flange mount (Ø62) + Ø34 electric bore + central cable bore; FIVE board pockets:
        ├─ Metro M4 AirLift (4000)         ┐ three big boards at 0°/90°/180°,
        ├─ PSM-B05 24 V→5 V converter      │ long side tangential, tucked to centre
        ├─ Optocoupler (your board ≤70×34) ┘ (12 o'clock, pushed out past the others)
        ├─ Pixel Shifter (6066)            ┐ two small breakouts in the 270° quadrant
        ├─ DRV2605L ERM/LRA driver (2305)  ┘
        ├─ POWER DISTRIBUTION channels: 24 V bore → converter, then 5 V fanned OUT to every
        │   board + the centre riser; data fanned out from the Metro board
  (2) COVER           seats the big NeoPixel ring (2874, Ø157) in a groove near its TOP rim (LEDs up,
                      opaque cover) · closes the board pockets · 4× M3 down to the base ·
                      4 ring-lead pass-throughs (one per quarter) · 3× M3 pilots for the casing box
        ▼  BOLTED FLANGE: apple CORE BASE screws down into the cover (3× M3 + centring spigot)
  (3a) APPLE CORE BASE  flange + a sleeve BOSS with a transverse HEIGHT-SET pin hole; wiring bore
        ╪  HEIGHT PIN: choose which shaft detent hole the boss pin engages → apple height
  (3b) PETG SHAFT  ┐    rod with DETENT holes (height steps); runs up INTO the ball, ending in a flange;
                   │    feed bore up the middle into the cavity
  (3c) TPU BALL    ┘    LOWER cup FUSED to the shaft (dual-material print); solid cap embeds the low
                        flange; OPEN top loads the ERM + force sensor (all bigger than the bore)
  (3d) TPU CAP          press-fit dome closes the ball; wires / FSR tail / pressure tube exit the bore
  ( + ) CLEAR CASING BOX  5-sided clear box (open on flange side) over the whole stack; top clamps 3× M3 at Ø56
```

> **Joints.** The cover↔core-base joint is a **bolted flange** (Ø44, 3×M3 on a bolt circle, centred by
> a spigot, cable bore down the middle). The flange sits at the centre of the big ring (ID Ø145), with
> loads of clear space between the apple mount and the ring.
>
> **Adjustable height.** The core base has a sleeve **boss** with one transverse pin hole; the shaft
> has a **column of detent holes** (`adj_n` holes, `adj_step` apart → ~21 mm of range in 7 mm steps).
> Slide the fused apple to the wanted height, line up a hole, run an **M3×25 screw through and lock it with
> an M3 nyloc nut**. It carries the pull-out load in **double shear** — positive, and can't shake loose.
>
> **Fused apple (PETG + TPU).** The shaft + LOWER ball print as **one dual-material object** on the H2D.
> The PETG shaft runs up into the ball and ends in a low **flange**; a solid TPU **cap** embeds it, so the
> pull load is carried **mechanically**, not by the PETG↔TPU bond. The ball's **top is a separate press-fit
> TPU cap**: pop it, drop in the ERM + force sensor (all bigger than the bore), route their wires / FSR tail
> / pressure tube down the **Ø9 shaft bore**, press the cap back on (silicone it if the pull unseats it).

## Power / data flow
```
24 V (media flange) ─► PSM-B05 24 V→5 V ─► 5 V ─┬─► Metro M4 AirLift board
                                                 ├─► NeoPixel ring   (via Pixel Shifter: 3.3 V data → 5 V)
                                                 ├─► DRV2605L driver ─► ERM/LRA motor (actuator, apple cavity)
                                                 └─► pressure sensor (apple cavity)
   data: Metro 3.3 V ─► Pixel Shifter ─► ring DIN ;  Metro I²C ─► DRV2605L ;  sensor ─► Metro ADC/I²C
```
> **Ring current.** The 60-LED RGBW ring can pull **~3.5 A at 5 V** at full-white — inject 5 V at each
> of the 4 quarter pass-throughs (don't feed 60 LEDs through one arc). The PSM-B05 (5 A) covers it, but
> keep brightness capped in firmware and size the 5 V wiring for the load.
The **Metro M4 Express AirLift** runs everything: it drives the NeoPixel ring through the **Pixel
Shifter** (its 3.3 V data needs shifting to 5 V), commands the **DRV2605L** over I²C to run the
haptic **actuator** (an ERM/LRA motor in the apple), and reads the **pressure sensor**. The
**PSM-B05** steps the arm's 24 V down to 5 V; that **5 V rail is distributed from the converter** out
to every board and up the centre riser (ring + apple) through channels in the base. All five boards
live in the base; only the ERM motor + pressure sensor sit up in the apple cavity.

## Components (dimensions locked from datasheets)
| Item | Part / link | Size (mm) | Drives parameter |
|------|-------------|-----------|------------------|
| NeoPixel ring, 60×5050 RGBW (**buy 4× quarter-rings**) | [Adafruit 2874](https://www.adafruit.com/product/2874) | Ø157 / Ø145 × 3.25 (6.2″) | `ring_od`, `ring_id`, `ring_groove_h` |
| Control board | [Metro M4 Express AirLift Lite (4000)](https://www.adafruit.com/product/4000) | 72 × 54 × 15 | `board_l/w`, `board_clear_h` |
| DC-DC converter | [PSM-B05-1224-05](https://abra-electronics.com/power-supplies-transformers-adapters/dc-dc-step-down-converters/psm-b05-1224-05-12v-24v-to-5v-5a-dc-dc-converter-step-down-regulator.html) (12/24 V→5 V 5 A) | 63 × 53 × 20 | `conv_l/w`, `conv_clear_h` |
| NeoPixel level shifter | [Pixel Shifter (6066)](https://www.adafruit.com/product/6066) (3.3→5 V data) | 25.5 × 15 × 10.2 | `shifter_*` |
| Actuator driver | [DRV2605L (2305)](https://www.adafruit.com/product/2305) (ERM/LRA driver, I²C) | 25.8 × 17.8 × 4.6 | `haptic_*` |
| **Optocoupler** | **your board — buy to fit** | **≤ 70 × 34** | `opto_l/w` |
| Haptic actuator | [Vibrating Mini Motor Disc (1201)](https://www.adafruit.com/product/1201) (coin ERM) | **Ø10 × 2.7** | apple cavity |
| Force sensor **A** | [FSR — Alpha MF01A round, high-force 1–98 N (5475)](https://www.adafruit.com/product/5475) | **Ø18 head** (~Ø15 active) · 60 long · 0.56 thin | apple cavity (head) + tail down bore |
| Force sensor **B** | [MPRLS ported pressure sensor (3965)](https://www.adafruit.com/product/3965), I²C 0–25 PSI | board **17.8 × 16.7 × 7.5** · port Ø2.5 | **base** + Ø2–3 tube up the bore to a *sealed* cavity |
| Height-set pin | **M3×25 screw + M3 nyloc nut** ×1 | — | `hpin_d` |

> **Fit notes (read before printing):**
> 1. **Base is Ø170.** Three big boards in tangential pockets at 0°/90°/180° (Metro, opto, converter)
>    and the two small breakouts in the 270° quadrant. Run `part="electronics_mock"` after any size
>    change to re-check overlaps.
> 2. **Optocoupler — buy to fit ≤ 70 × 34 mm.** It sits at 12 o'clock, pushed out past the Metro +
>    converter (their ±36 mm tangential reach leaves a long-shallow slot: ~70 mm side-to-side × ~34 mm
>    in-out). A full M4-size opto does **not** fit flat on the compact base — that needs ~Ø200 or
>    stacking. If you get a bigger board, tell me and I'll grow/stack it.
> 3. **Metro mounting holes are not symmetric.** Only the **Metro + optocoupler** get standoffs (Ø9 posts
>    sized for an M3 insert); the potted converter + the two tiny breakouts sit in their pockets (tape /
>    headers). The post XY are placeholders — set them to each board's real footprint before printing.
> 4. **Mount the empty base to the robot first.** The 8 media-flange bolts (r≈31) fall under the
>    boards; the **M6 cap heads recess into the top** (Ø11 access wells) and drive from *inside* the
>    hub, so bolt the base to the flange *before* fitting electronics. Keep board undersides clear of the
>    heads (head + washer top ≈ z 14 mm) — or use countersunk M6 at the four bolts that sit under boards.
> 5. **Thin shaft + close apple (by design).** The shaft is **Ø14** with a **Ø9 feed bore** and the boss
>    is **Ø20** — both sit at the very centre, far inside the big ring (ID r≈72.5), so they don't occlude
>    the ring. The apple height is set over **~21 mm of range in 7 mm steps** (the detent hole) — enough
>    to wrap a hand around the Ø45 ball clear of the base.
> 6. **Shaft strength — OK for the 15–20 N pull.** At Ø14/Ø9-bore the shorter moment arm (~60 mm at
>    default height) gives ≈6 MPa bending vs ~30–50 MPa for PETG → **safety factor ~5–8**, ~1 mm
>    deflection; the detent pin sees ~1 MPa shear. Print the **PETG shaft solid / high-perimeter**. The
>    ball↔shaft **flange anchor (Ø22)** carries the pull in bearing on the solid TPU cap, not on the bond.
> 7. **Apple cavity + access.** The cavity (dome ≈ **Ø39 × 31 mm** above the flange) fits the **ERM
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
| M6 cap-head screw | **M6 × 16** | 8 | Robot media flange → into the **flange's own tapped holes**. Head sits on a **steel M6 washer** in the Ø13 seat and drives from the **top** through the Ø11 well. Spans the 8 mm wall + washer → **~6.4 mm thread bite**; **verify the flange tap ≥ 7 mm** (or use M6×18 for ~8 mm). |
| M3 cap-head screw | **M3 × 10** | 7 | 4× cover→base, 3× apple-core→cover (both thread into heat-set inserts). |
| M3 cap-head screw | **M3 × 6** | 3 | Casing-box top → cover. **Short on purpose** — the cover pilot is only 5 mm deep in the 6 mm cover, so M3×10 would bottom out and never clamp. |
| M3 cap-head screw | **M3 × 6** | 8 | Metro + optocoupler standoffs (4 each) — thread into **heat-set inserts** in the Ø9 posts (no self-tapping). Or use each board's own screws. |
| M3 brass heat-set insert | **Bambu M3×5×4** (M3, 5.0 mm OD, 4 mm long) | 18 | 4× base + 3× cover boss + 3× cover top + 4× Metro + 4× opto. Printed hole is **Ø4.6 (`m3_insert`)** (for the 5.0 mm OD); the 4 mm length seats in every pilot (shallowest = 5 mm cover-top). |
| Height-set pin | **M3×25 screw + M3 nyloc nut** | 1 | Detent pin through the Ø3.4 boss/shaft holes (double shear). Nyloc so the arm's motion can't shake it loose; back the screw out to re-set the apple height. |
| Steel washer, M6 | flat, OD ~12 (DIN 125) | 8 | **Under each M6 flange head** — spreads bolt torque so the printed 8 mm wall can't crush. Seats in the Ø13 recess (`flange_washer_*`). |
| Washer, M3 | small | 3 | Under the casing-top screws — spread the clamp load on the clear sheet. |

> **Shopping summary:** **M3 cap-head** — M3×10 (×7), M3×6 (×11: casing + boards), and one **M3×25**;
> **M6×16** (×8) + **8 steel M6 washers**; **M3 heat-set inserts** (**Bambu M3×5×4**, ~18–20); **M3 nyloc
> nut** (×1, for the height pin) + **3 M3 washers** (casing). **Every screw-into-plastic joint — including
> the PCB posts — takes the same M3 insert; there's no self-tapping anywhere.**

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
# FUSED APPLE — export shaft + lower ball, import at the SAME origin in Bambu Studio, assign filaments:
openscad -D 'part="adj_shaft"'       -o adj_shaft.stl       apple_pluck_end_effector.scad   # PETG (shaft + armature)
openscad -D 'part="apple_ball"'      -o apple_ball.stl      apple_pluck_end_effector.scad   # TPU  (lower ball, moulds onto it)
openscad -D 'part="apple_cap"'       -o apple_cap.stl       apple_pluck_end_effector.scad   # TPU  (press-fit cap, separate print)
# previews only: 'part="assembly"' (default), 'part="apple"' (core base + fused ball + cap),
# 'part="apple_section"' / 'part="section"' (half-cuts), 'part="electronics_mock"' (board/ring fit),
# 'part="casing"' (clear casing box — reference only; you build it by hand from clear acrylic sheet)
```

> The **apple_ball** has a sphere, so on the CGAL backend (OpenSCAD 2021.x) F6/STL render is slow
> (minutes); the **Manifold** backend (2023.06+, *Preferences → Features → Manifold*) renders it in
> seconds. Use F5 preview / `part="apple_section"` to check geometry without the wait.

## ⚠️ Verify before printing
- **Flange interface (`flange_*`):** set to the **measured** flange pattern — **Ø62 bolt circle,
  8 × M6, Ø34 central electric bore** (the 14 mm mounting ring between them = (62−34)/2). Still confirm:
  the **exact hole clocking** (the 8 holes are placed evenly at 45°; verify `flange_first_angle`), the
  **connector protrusion depth** (drives `flange_center_h`, currently a 6 mm recess kept *below* the
  board pockets — if the connector sticks up more than ~6 mm, deepen it and push the innermost boards
  out), and the **M6 tap depth** — the **M6×16** head sits on a **steel washer** (Ø13 seat) and the shank
  spans the `flange_mount_t = 8 mm` wall + ~1.6 mm washer, leaving **~6.4 mm** of thread bite, so verify
  the flange tap is **≥ 7 mm** (use **M6×18** for ~8 mm bite). The head recesses into and drives from the
  TOP down the Ø11 access well. Print a **thin fit-test ring** (reduce `base_h`) to check the bolt pattern
  + Ø34 bore before the full hub.
- **Board fit:** the real Metro M4 (72 × 54) + PSM-B05 (63 × 53) drive the Ø165 base; use
  `part="electronics_mock"` to check overlaps and adjust `*_radius` / `*_angle`.
- **Shaft fit / strength:** the boss bore is `adj_shaft_d + adj_fit_clear`; print a short fit-test and
  confirm the shaft slides snugly and the detent holes align with the boss pin. See fit-notes 5–7 on load.
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
- **FUSED APPLE (PETG shaft + TPU lower ball, one print):** import **adj_shaft.stl** and **apple_ball.stl**
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
  **178 × 178** outer, **~36 mm tall**, **open on the flange side**, walls/top **3 mm**. The top has a
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
   **M3 heat-set insert (Bambu M3×5×4: 5.0 mm OD, 4 mm long → Ø4.6 hole)** — press one into each: **4× base↔cover, 3× cover boss, 3× cover
   top (casing)** = 10, **plus 4× Metro + 4× opto standoffs** (Ø9 posts) = 18. No self-tapping anywhere.
   (The height pin is a separate **M3×25 screw + nyloc nut**.)
2. **Mount the empty base to the robot.** Bolt the base to the iiwa7 media flange (**8× M6**,
   the Ø34 bore clearing the electric connector) *before* fitting electronics. Pull the power/data
   bundle up the bore.
3. **Populate the base.** Seat the **PSM-B05** and **DRV2605L** in their pockets; mount the **Metro M4**,
   the **optocoupler**, and the **Pixel Shifter** on standoffs (M3). Route the harness through the
   distribution channels: **24 V → PSM-B05**, then **5 V from the converter → every board + the centre
   riser** (ring + apple); **data from the Metro → opto / shifter / DRV / centre riser**. Leave a
   service loop at the centre riser; run the ERM-motor + sensor leads up the centre.
4. **Seat the ring + close the cover.** Butt the **4 quarter-arcs** into the full ring and drop it into
   the cover's **top groove** (LEDs up); solder the arc-to-arc joints and feed each arc's power/data
   leads down its **quarter pass-through** to the boards. Then lower the cover and drive the **4× M3**
   through the cover rim into the base.
5. **Fit the casing box.** Lower the clear box over the stack (open side down to the flange) so its top
   covers the ring, line up the 3 top holes with the **Ø56** pilots, and drive **3× M3** down into the
   cover. (Do this before the apple parts — the box's Ø45 top hole won't pass over an assembled apple.)
6. **Attach the apple core base.** Its boss pokes up through the box's top centre hole; set its flange on
   the cover boss (the **spigot** centres it) and drive the **3× M3** down into the cover boss. Pass the
   apple wiring up through its bore.
7. **Load the electronics + fit the apple.** With the **cap off**, seat the **ERM (Ø10)** against the inner
   TPU wall and the **force sensor** in the open cavity (FSR head against the wall behind a backing; or, for
   the MPRLS route, just the pressure tube — sensor stays in the base). Route the leads / FSR tail / tube
   **down the shaft bore**, then **press the cap on** (silicone it if needed). Slide the **fused apple** into
   the core-base boss to the wanted height, line up a **detent hole** with the boss pin hole, and run the
   **M3×25 screw through + an M3 nyloc nut**. Confirm it can't slide, and that the **cap + ball can't pull off** by hand.
8. **Connect + commission.** Connect the media-flange power/data, **re-calibrate the tool load** in
   Sunrise (electronics + apple add mass — see main repo README §2 "Tool Load Data"), and first
   power-up / move in **T1** with a hand on the E-stop.

Disassembly is the reverse: undo the height-pin screw/nut → withdraw the fused apple (and its electronics
via the bore) → unbolt the core base → unscrew the casing box → cover. No glue in the stack; the fused
apple is one printed part (reprint to replace).

## Safety (animal subject + electronics)
- Lightweight keeps tool inertia low (better impedance behavior; gentler on contact) — the electronics
  add mass, so re-check the FRI load data / tool calibration after fitting (main repo README §2).
- The apple sits on a **thin Ø14 PETG shaft** (with a Ø22 flange anchoring the TPU ball); sized with
  ~5–8× margin for the 15–20 N pull (fit-note 6). Re-check after any height/length change; print the
  shaft solid, and confirm the **TPU ball can't pull off the flange by hand** before use.
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
