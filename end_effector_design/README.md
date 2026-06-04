# Apple-Pluck End-Effector (parametric, DRAFT v0.1)

A 3D-printable tool that bolts to the **KUKA LBR iiwa7** tool flange and presents a compliant
**"apple"** for the monkey to pull. Authored in **OpenSCAD** so it's text-based, parametric, and
version-controlled.

> **Status:** first draft, **not yet rendered/printed** (no OpenSCAD on the dev box). Open it in
> OpenSCAD, check the geometry, and iterate before printing. Treat all dimensions as starting points.

## Design at a glance
Three parts couple in a line; everything is swappable:

```
[ iiwa7 tool flange ]
        |  M6 × N bolts on the flange PCD (+ centering recess + locating pin)
  (1) ADAPTER       — female coupler socket (tool side)
        |  slip stud Ø + radial M4 set screw
  (2) EXTENSION     — sets the standoff length (flange → apple)
        |  slip stud Ø + radial M4 set screw
  (3) APPLE         — hollow sphere, top stem, grip grooves; silicone-coatable
```

- **Why modular:** swap the **apple** (size/texture) and the **extension** (standoff) without
  remaking the flange adapter — mirrors the swappable-object approach in Barra et al. 2019.
- **No detach mechanism:** the "pluck" is the cabinet's **Cartesian-impedance displacement** (the arm
  yields and a displacement threshold fires the trial), so the apple is *rigidly* attached. See the
  control stack / `MoveToPositionAction` + `CartesianImpedanceDisplacementMonitor`.
- **Compliant skin:** print hollow, then brush/dip **two layers of food-safe silicone** for a soft,
  grippable, cleanable surface (cf. Barra 2019). The hollow shell also keeps payload low — good for
  impedance control.

## Files
| File | What |
|------|------|
| `apple_pluck_end_effector.scad` | the parametric model (adapter + extension + apple + assembly) |
| `README.md` | this file |

## Render / export
Open `apple_pluck_end_effector.scad` in the **OpenSCAD GUI** (the parameter panel exposes everything
via the Customizer), or export each part headless:

```bash
openscad -D 'part="adapter"'   -o adapter.stl    apple_pluck_end_effector.scad
openscad -D 'part="extension"' -o extension.stl  apple_pluck_end_effector.scad
openscad -D 'part="apple"'     -o apple.stl       apple_pluck_end_effector.scad
# 'part="assembly"' (default) is for preview only — don't print the assembly
```

## Key parameters (top of the .scad)
- **Robot flange** (`flange_*`): plate Ø/thickness, bolt PCD/count, M6 clearance + counterbore,
  centering recess, locating pin. **These are nominal — verify (see warning below).**
- **Coupler** (`coupler_*`, `setscrew_d`): the shared stud/socket fit used between all parts.
  `coupler_clear` tunes the slip fit; an M4 set screw locks each joint.
- **`standoff_len`**: flange-face → apple-base distance (keeps the monkey clear of the arm).
- **Apple** (`apple_d`, `apple_wall`, `apple_stem_*`, `apple_grooves`): size, shell thickness, stem,
  grip grooves.

## ⚠️ Verify the flange interface before printing the adapter
The `Robot flange` parameters default to **ISO 9409-1-50-4-M6** nominal values (Ø50 PCD, 4 × M6,
~Ø31.5 centering, locating pin). **Confirm against your specific iiwa7 R800 flange / media-flange
datasheet** — bolt count, PCD, the centering boss/recess diameter and which side has it, and the pin
position. Print a **thin “fit-test” adapter first** (reduce `flange_plate_t`) to confirm the bolt
pattern, centering, and pin before printing the full tool.

## Printing & finishing
- **Adapter:** print plate-face down; PETG/PLA+ or nylon. Solid-ish (≥40% infill) — it's structural.
- **Extension / apple:** print **stud-down**; the apple is hollow with a vent hole down the stem
  (lets trapped air/resin/silicone escape when coating). 3–4 perimeters.
- **Silicone:** 2 thin layers, fully cured, no loose edges the animal could pick at.
- Keep everything **rounded and smooth** — no sharp edges or pinch points near the animal.

## Safety (animal subject)
- Lightweight + hollow keeps tool inertia low (better impedance behavior, gentler on contact).
- The cabinet's Cartesian impedance + Sunrise safety limits are the real safety layer — this tool
  just needs to be smooth, light, and robustly attached (set screws seated, bolts torqued).
- First mount/move in **T1** with a hand on the E-stop.

## Reference
Barra et al., *A versatile robotic platform for … reaching and grasping tasks in monkeys*,
J. Neural Eng. 17(1):016004 (2019/2020) — same iiwa + macaque platform; silicone-over-3D-print
objects with integrated grip sensing. (Their Zenodo deposit is software-only — no object CAD — so
this tool is drawn from scratch.)
