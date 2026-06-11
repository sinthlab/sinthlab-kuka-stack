# Apple-Pluck End-Effector (parametric, DRAFT v0.4)

A 3D-printable tool that bolts to the **KUKA LBR iiwa7 media flange (electric)** and presents a
compliant **"apple"** for the monkey to pull. It is also an **electronics hub**: it
routes the media-flange power/data, drives a **NeoPixel ring** visual cue, and carries the control
**board** + **DC-DC converter**, with wiring extending to a **pressure sensor** and **actuator**.
Authored in **OpenSCAD** so it's text-based, parametric, and version-controlled.

> The apple is split into a rigid **CORE** (flange + a full-length support shaft
> that runs the whole inner height of the ball) and a separate hollow **BALL** that slides over the
> shaft and locks with a **transverse pin / M3 screw**. This lets you print the two in **different
> materials** (rigid core, soft/grippable ball) and the pull load goes into the shaft + pin in double
> shear, not a slender neck. The **pressure sensor + actuator live in the ball cavity** around the
> shaft; wiring reaches them through the shaft bore and a side window.


## Design at a glance
Five printed parts couple in a line (the apple is two of them); the **base** is the electronics hub:

```
[ iiwa7 media flange (electric) ]   8× mounting screws · 24 V + data bundle
        │  cable bundle THROUGH the base centre
  (1) BASE HUB        8-hole flange mount + central cable bore
        ├─ NeoPixel ring (Adafruit 1768, 24×5050) in a concentric top groove
        ├─ compartment: Metro AirLift control board
        ├─ compartment: Tobsun 24 V→5 V converter
        └─ wire channels (the "bridges") linking bore ↔ converter ↔ board ↔ ring
  (2) COVER           closes the compartments · diffuses the ring · 4× M3 down to the base
        ▼  BOLTED FLANGE: extension screws down into the cover (4× M3 on a bolt circle + centring spigot; cable through the centre)
  (3) EXTENSION       shaft; central cable bore + a SIDE EXIT hole for the bundle; a flange at each end
        ▼  BOLTED FLANGE: apple CORE screws down into the extension top (4× M3 + spigot)
  (4a) APPLE CORE     flange + a FULL-LENGTH support shaft (rigid); central wiring bore;
                      a SIDE WINDOW releases wires into the cavity; a transverse LOCK hole
        ╪  CROSS-PIN / M3: the ball slides over the shaft and locks transversely to the core
  (4b) APPLE BALL     hollow grippable sphere (soft/separate material); a central socket
                      sleeves the core shaft; interior CAVITY holds the pressure sensor + actuator
```

> **Joints.** The three **bolted flanges** (base↔cover, cover↔extension, extension↔core) are N×M3 on a
> bolt circle, centred by a spigot, cable bore down the middle. Two flange sizes — the
> **cover↔extension** joint is slim (Ø44, 3×M3, sized to the Ø26 shaft), while the **extension↔core**
> joint is wider (Ø60, 4×M3, bolt circle r=25) so a driver clears the apple sphere.
>
> **Core↔ball lock (new in v0.4).** The ball does *not* bolt on — its central socket (Ø ≈ core +
> `ball_fit_clear`) slides down over the core's full-length **support shaft**, and a single transverse
> **Ø3 dowel pin** (snap-in) or **M3 screw** (bolted/serviceable) through both ball and shaft takes the
> pull-out load in **double shear**. The pin sits **low** (`lock_pin_z`, below the grip zone) and below
> the cavity floor, so it passes through solid material, not the hollow.

## Some Highlights
1. **8 screw holes** — the flange mount is `flange_bolt_n = 8` on a configurable bolt circle, to
   match the iiwa7 **media flange electric** pattern. *(Confirm the exact PCD, screw size, and the
   first-hole angle — see VERIFY.)*
2. **Media-flange cabling** — a **central cable bore** runs from the flange face up through the hub and
   the shaft (through the hollow centre of each bolted-flange joint); the **EXTENSION has a side-exit
   hole** (`shaft_exit_*`) to pull the whole bundle out.
3. **NeoPixel ring + electronics** — a **concentric groove** seats the Adafruit 1768 ring on the top
   rim (with a diffuser window in the cover); two **open-top compartments** ("cabinets") hold the
   **Metro AirLift** board (PCB standoffs) and the **Tobsun converter**; **wire channels** through the
   structure act as the bridges between them.
4. **Sensor + actuator wiring** — the central bore continues up through the apple **core shaft** and
   exits a **side window** (`core_window_*`) into the **ball cavity**, where the pressure sensor /
   actuator sit (wired from the same board).

5. **The Apple**
- **(5a) Apple CORE** (rigid) — the extension↔apple flange plus a **full-length support shaft**
  (`core_d`/`core_len`) that runs the whole inner height of the ball. The wiring bore goes up its
  centre and a **side window** (`core_window_*`) releases the wires into the cavity. A transverse
  **lock hole** (`lock_pin_*`) crosses the shaft low down.
- **(5b) Apple BALL** (soft) — a hollow sphere whose **central socket** slides over the core shaft.
  The interior is hollow **only above the lock height**, so the lower shell stays solid for the pin.
  The **cavity** around the shaft houses the pressure sensor + actuator. Grip grooves on the outside.
- **Lock:** one **Ø3 dowel** (snap-in) or **M3 screw** (serviceable) through ball + shaft. Load goes
  into the shaft + pin in double shear and the deep socket sleeve — not a slender neck.
- **Different materials:** print the **core rigid** (load path) and the **ball soft/grippable**
  (animal contact); see *Printing*.

## Power / data flow
```
24 V (media flange) ──► Tobsun 24 V→5 V ──► 5 V ──┬──► Metro AirLift board
                                                   ├──► NeoPixel ring (cue)
                                                   └──► pressure sensor + actuator (up the shaft)
   data lines (media flange) ───────────────────────► Metro AirLift board
```
The **Metro AirLift** drives the NeoPixel ring (visual cue) and reads/controls the pressure sensor and
actuator. The **Tobsun** steps the arm's 24 V down to 5 V for both the board and the ring.

## Components (verify dimensions in the .scad before printing)
| Item | Part / link | Drives parameter |
|------|-------------|------------------|
| NeoPixel ring, 24×5050 | [Adafruit 1768](https://www.adafruit.com/product/1768) | `ring_od`, `ring_id`, `ring_groove_h` |
| Control board | Metro AirLift (Arduino Uno footprint) | `board_l/w`, `board_*` compartment |
| DC-DC converter | Tobsun 24 V→5 V | `conv_l/w/conv_clear_h`, `conv_*` |
| Pressure sensor + actuator | (your parts — sit in the ball cavity) | `apple_d`, `apple_wall`, `core_window_d` |
| Core↔ball lock | Ø3 dowel pin **or** M3 screw | `lock_pin_d`, `lock_pin_z` |

## Files
| File | What |
|------|------|
| `apple_pluck_end_effector.scad` | the parametric model (base + cover + extension + apple core + apple ball + assembly) |
| `README.md` | this file |

## Render / export
Open `apple_pluck_end_effector.scad` in the **OpenSCAD GUI** (Customizer exposes every parameter), or
export each part headless:

```bash
openscad -D 'part="base"'       -o base.stl        apple_pluck_end_effector.scad
openscad -D 'part="cover"'      -o cover.stl       apple_pluck_end_effector.scad
openscad -D 'part="extension"'  -o extension.stl   apple_pluck_end_effector.scad
openscad -D 'part="apple_core"' -o apple_core.stl  apple_pluck_end_effector.scad   # rigid material
openscad -D 'part="apple_ball"' -o apple_ball.stl  apple_pluck_end_effector.scad   # soft material
# previews only: 'part="assembly"' (default), 'part="apple"' (core+ball together),
# 'part="apple_section"' / 'part="section"' (half-cuts), 'part="electronics_mock"' (board/ring/converter fit)
```

> The **apple_ball** has a sphere, so on the CGAL backend (OpenSCAD 2021.x) F6/STL render is slow
> (minutes); the **Manifold** backend (2023.06+, *Preferences → Features → Manifold*) renders it in
> seconds. Use F5 preview / `part="apple_section"` to check geometry without the wait.

## ⚠️ Verify before printing
- **Flange interface (`flange_*`):** the 8-hole pattern, **PCD**, **screw size**, counterbore, the
  **first-hole angle**, and the **centering-boss recess** are *nominal*. Confirm against the **iiwa7
  media-flange-electric** datasheet, then print a **thin fit-test ring** (reduce `base_h`) to check the
  bolt pattern + centering before printing the full hub.
- **Component fit:** the **Metro AirLift footprint drives `base_d`** — at the default ~122 mm the board
  compartment, the central hub, and the ring are geometrically tight; in OpenSCAD use
  `part="electronics_mock"` to check overlaps and adjust `base_d`, `board_radius/angle`,
  `conv_radius/angle`. If the board is too big to lie flat, mount it **on edge** (a future variant) to
  shrink the base.
- **Strain relief:** add a clamp / grommet at the central bore and the shaft side-exit so cable load
  isn't on the connectors.

## Printing & finishing
- **Base / cover:** print flat-face down; PETG/PLA+/nylon, ≥40 % infill (structural). Print the
  **cover in clear/white** so the ring diffuses through the window.
- **Extension:** print **flange-down** (a joint flange on the bed); rigid, 3–4 perimeters.
- **Apple CORE (rigid):** this is the load path — print in **PETG / PLA+ / nylon**, **flange-down**, with
  high perimeter/infill so the support shaft and the lock-pin hole are strong. The shaft prints
  upward with no support.
- **Apple BALL (soft):** print in a **flexible/grippable filament (TPU)** or print it as a thin shell
  and over-mould **food-safe silicone** — whichever you use for the animal contact surface. Print
  **socket-up** (stem on the bed) so the socket and grooves come out clean; the cavity may need a
  little support over the socket. Keep it **rounded and smooth** — no sharp edges or pinch points.
- **Fit:** the socket is sized at `core_d + 2·ball_fit_clear` (default +0.4 mm dia). TPU/silicone
  shrink and squeeze differently than rigid prints — print a short **fit-test ring** of the socket
  first and adjust `ball_fit_clear` so the ball slides on snugly and the **lock holes line up**.

## Assembly (after printing)
Do the electronics + wiring **inside the base before closing the cover**. Everything is bolted (no
glue), so any part stays swappable/serviceable.

1. **Prepare the threads.** The screw pilots are sized for self-tapping M3 — either run an M3 screw in
   once to form threads, or (recommended) press an **M3 heat-set insert** into each pilot:
   4× base↔cover, 3× cover↔extension, 4× extension↔core. For inserts, open the relevant `*_pilot`
   diameter to the insert OD first. (The core↔ball lock is a separate transverse pin/screw — step 5.)
2. **Populate the base.** Seat the **Tobsun** converter in its compartment, mount the **Metro AirLift**
   on the four standoffs (M3), and drop the **NeoPixel ring** into the top groove. Route the harness
   through the wire channels: media-flange **24 V → Tobsun → 5 V → board + ring**; **data → board**.
   Leave a small service loop at the central bore.
3. **Close the cover.** Lower the cover (diffuser window over the ring), feed the cable bundle up
   through the central bore, and drive the **4× M3** through the cover rim into the base.
4. **Attach the extension.** Set its bottom flange on the cover boss — the **spigot** centres it — and
   drive the **3× M3** down through the flange into the cover. Pass the wiring up the shaft; pull the
   bundle out the **side-exit hole** for service slack as needed.
5. **Attach the apple (core, then ball).**
   - **Core:** set the core's bottom flange on the extension top (spigot centres it) and drive the
     **4× M3**. The support shaft now stands proud, with the wiring bore up its centre.
   - **Wire + populate the cavity:** feed the sensor/actuator leads up through the bore and out the
     **side window** into the cavity; seat the **pressure sensor + actuator** in the cavity around the
     shaft (leave a small service loop, strain-relieve at the window).
   - **Ball:** slide the soft **ball** down over the shaft until its **lock hole lines up** with the
     shaft hole, then insert the **Ø3 dowel pin** (press/snap fit) or thread an **M3 screw** through
     ball + shaft to lock it. Confirm the ball can't pull off by hand.
6. **Mount to the robot.** Bolt the base to the iiwa7 media flange (**8 screws**, engaging the
   centering recess) and connect the media-flange power/data.
7. **Re-calibrate the tool load** in Sunrise (the electronics + apple add mass — see main repo README
   §2 "Tool Load Data"), then first power-up / move in **T1** with a hand on the E-stop.

Disassembly is the reverse: pull the lock pin → lift the ball off the shaft → unbolt core → extension
→ cover. No glue anywhere; the soft ball stays separately replaceable.

## Safety (animal subject + electronics)
- Lightweight + hollow keeps tool inertia low (better impedance behavior; gentler on contact) — note
  the electronics now add mass, so re-check the FRI load data / tool calibration after fitting (see the
  main repo README §2 "Tool Load Data").
- Keep **5 V/24 V wiring** enclosed and strain-relieved; nothing the animal can reach or pull.
- The cabinet's Cartesian impedance + Sunrise safety limits are the real safety layer — this tool just
  needs to be smooth, light, robustly attached (all flange bolts torqued), and electrically safe.
- First mount/move in **T1** with a hand on the E-stop.

## Reference
Barra et al., *A versatile robotic platform for … reaching and grasping tasks in monkeys*,
J. Neural Eng. 17(1):016004 (2019/2020) — same iiwa + macaque platform; silicone-over-3D-print objects
with integrated grip sensing. (Their Zenodo deposit is software-only — no object CAD — so this tool is
drawn from scratch.)
