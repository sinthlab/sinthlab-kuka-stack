# Reward and Sync Box

One 2U rack box with two independent jobs:

- **Reward**: opens the juice valve on a pedal, a hand switch or an NSP pulse.
- **Sync**: holds the Beckhoff EtherCAT stack that turns the Sunrise cabinet's trial
  events into 5 V TTL pulses for the Blackrock NSP —
  [Sync: TTL pulses to the NSP](#sync-ttl-pulses-to-the-nsp).

Both halves run from **one 24 V adapter**. At the **24 V IN** jack the supply splits into two
branches, each with its own 1 A breaker: **BREAKER VALVE** feeds the reward side, **BREAKER SYNC** the
Beckhoff stack. The two branches share 0 V at the jack.

## Contents

- [Reward](#reward): [How it works](#how-it-works) · [Circuit](#circuit)
- [Parts list](#parts-list)
- [Panels and the CAD files](#panels-and-the-cad-files) · [3D-printed stand-in](#3d-printed-stand-in)
- [Building it](#building-it)
- [Cables for the pedal, hand switch and NSP](#cables-for-the-pedal-hand-switch-and-nsp)
- [First test](#first-test)
- [Safety](#safety)
- [Check before ordering or drilling](#check-before-ordering-or-drilling)
- [Sync: TTL pulses to the NSP](#sync-ttl-pulses-to-the-nsp)
  - [Components](#components) · [Assembly](#assembly) · [The NSP side](#the-nsp-side) ·
    [Configuring the cabinet (once)](#configuring-the-cabinet-once) · [The code](#the-code) ·
    [Testing, step by step](#testing-step-by-step) · [Troubleshooting](#troubleshooting)

---

## Reward

The reward half opens the juice valve (a **Honeywell Skinner V52LB1100**, 24 V DC, 10 W) from any of
three inputs:

| Input | Connects with | Opens the valve |
|---|---|---|
| **NSP** | BNC cable from the Blackrock NSP digital out | for as long as the TTL pulse is high |
| **PEDAL** | foot pedal, plugged into its BNC jack | while the pedal is held down |
| **BUTTON** | hand switch, plugged into its BNC jack | while the button is held |

Plus two panel switches, each with an LED: **POWER**, and **FLUSH**, which holds the valve open for
filling and cleaning the line regardless of anything else.

**Plugging a device in is what arms it.** Nothing unplugged can trigger the valve.

The reward size is set by **how long** the valve is open: the NSP pulse width, or how long the pedal
or button is held.

### How it works

A solenoid valve has no separate control line: **power on the coil opens it, power off closes it**
(the V52LB1100 is normally closed). The box switches 24 V onto the coil.

```
              ┌──────────────────────── REWARD SIDE ───────────────────────┐
 24 V ═► BRK VALVE ─ POWER────────┬─ node P (+24 V)                          │
              │                   ├─ SSR output ──────────────┐              │
              │                   └─ FLUSH ───────────────────┼─► valve coil ═══► valve
              │                                               │              │
 NSP     ═══════► BNC ─┐                                      │              │
 PEDAL   ═══════► BNC ─┼─ diodes ─► SSR input ── (turns the SSR output on)   │
 BUTTON  ═══════► BNC ─┘                                                     │
              └─────────────────────────────────────────────────────────────┘
```

- **One switch drives the valve: a DC solid-state relay (SSR).** Every trigger — NSP pulse, pedal,
  button — only turns on the SSR's *input*, which needs ≥ 3.5 V and draws ~2–25 mA. The SSR output
  switches the coil's 0.42 A, so the pedal, the hand switch and the NSP never carry valve current.
- **FLUSH bypasses the SSR** and powers the coil directly, so cleaning does not depend on the
  electronics.
- **Diodes keep the inputs apart.** Each trigger reaches the SSR input through its own 1N4007, so the
  pedal's 24 V can never back-feed the NSP output, and vice versa.
- **A flyback diode across the coil** absorbs the coil's voltage spike at switch-off. It is mandatory:
  without it the spike destroys the SSR.
- **The NSP's ground is tied to the box's 0 V** (through the NSP jack and the SYNC jacks). That is
  safe only because the 24 V adapter is **2-prong (floating)**. An earthed 3-prong adapter would
  create a ground loop with the NSP.

### Circuit

```
 24 V adapter ══► [24 V IN jack] centre ──┬── [BREAKER VALVE] ── [POWER] ──┬── node P (+24 V) ── LED1 ── 0 V
                                          │                                │
                                          └── [BREAKER SYNC] ──► EK1100 +24 V (see Assembly)
                                 sleeve ── 0 V (star point: reward 0 V, EK1100 0 V, chassis)
                                                                         │
   VALVE   node P ── SSR out (+)      SSR out (−) ── VALVE coil A ─┐
                                                                    coil (~58 Ω)
           0 V ──────────────────────────────────── VALVE coil B ─┘
           node P ── [FLUSH] ──┬── VALVE coil A          (direct: bypasses the SSR)
                               └── LED2 ── 0 V
           D1 1N4007 across coil A–B, stripe (cathode) on A
           VALVE 3rd contact (case ground) ── 0 V, or leave free

   TRIGGERS  (D2–D4 1N4007, stripe toward the SSR)
           [PEDAL BNC]   shell ── node P      centre ──►|── D2 ──┐
           [BUTTON BNC]  shell ── node P      centre ──►|── D3 ──┼──► SSR in (+)
           [NSP BNC]     shell ── 0 V         centre ──►|── D4 ──┘    SSR in (−) ── 0 V

   CHASSIS ── 0 V (one wire from a chassis screw)
```

#### Wire list

| Net | Connects |
|---|---|
| **24V_IN** | 24 V IN jack centre lug → BREAKER VALVE terminal 1 and BREAKER SYNC terminal 1 (two separate wires from the lug) |
| **24V_BRK** | BREAKER VALVE terminal 2 → POWER toggle common |
| **24V_SYNC** | BREAKER SYNC terminal 2 → EK1100 point 1 (see [Assembly](#assembly)) |
| **P** (+24 V switched) | POWER toggle "on" lug → LED1 (+), SSR output (+), FLUSH toggle common, PEDAL BNC shell, BUTTON BNC shell |
| **COIL_A** | SSR output (−), FLUSH toggle "on" lug, LED2 (+), VALVE socket coil contact A, D1 cathode |
| **0V** | 24 V IN jack sleeve lug (star point), EK1100 point 5 (separate wire), VALVE coil contact B, D1 anode, LED1 (−), LED2 (−), SSR input (−), NSP BNC shell, chassis screw, VALVE 3rd contact (if used) |
| **TRIG** | SSR input (+), cathodes of D2, D3, D4 |
| — | D2 anode → PEDAL BNC centre; D3 anode → BUTTON BNC centre; D4 anode → NSP BNC centre |

- **LED2 sits on COIL_A**, so it lights whenever the coil is powered — by FLUSH *or* by a trigger.
- **Toggles (Carling 2M1-SP1-T1-B1-M1QE, SPDT):** use the centre lug (common) and one outer lug; leave the other
  outer lug free. Find the "on" lug for lever-up with a multimeter.
- **SSR (Crydom DC60S3):** the output and input terminals are marked + and − on the body.
- **Valve coil contacts:** with the valve cable plugged into the valve and into the box's socket, the
  two contacts that read **~58 Ω** are the coil (A and B, either way round — only D1's direction
  depends on which you make A). The third reads open: the case ground.

#### Current in each input

| Input | When it fires | Current in its cable |
|---|---|---|
| PEDAL / BUTTON | the switch connects the jack shell (+24 V) to its centre → D2/D3 → SSR | ~16 mA (the SSR input, ~1.5 kΩ) |
| NSP | the NSP drives 5 V on the centre → D4 → SSR | ~2–3 mA |
| FLUSH | the toggle puts +24 V straight on the coil | 0.42 A, all inside the box |

Neither the pedal nor the hand-switch cable carries 0 V, so a damaged cable can at worst short its two
wires together — the same as pressing it. The only short that draws real current is a +24 V jack
shell touching something earthed outside the box; BREAKER VALVE trips on that.

---

## Parts list

### Enclosure

| # | Item | Part | Digi-Key # | Qty |
|---|---|---|---|---|
| 1 | Enclosure, 2U 19" rack, 8" deep, black aluminium (both sides) | [Hammond RM2U1908SBK](https://www.digikey.ca/en/products/detail/hammond-manufacturing/RM2U1908SBK/2094735) | HM998-ND | 1 |

### Power (both sides)

| # | Item | Part | Digi-Key # | Qty |
|---|---|---|---|---|
| P1 | 24 V 1.5 A adapter, **2-prong**, 5.5 × 2.1 mm, centre positive | [Triad Magnetics WSU240-1500](https://www.digikey.ca/en/products/detail/triad-magnetics/WSU240-1500/3094973) | 237-1447-ND | 1 |
| P2 | DC jack, panel, 2.1 × 5.5 mm, solder lugs: 24 V IN | [Switchcraft 721A](https://www.digikey.ca/en/products/detail/switchcraft-inc/721A/8571716) | SC3508-ND | 1 |
| P3 | Breaker, 1 A thermal, push-to-reset, snap-in: BREAKER VALVE, BREAKER SYNC | [TE W28-XQ1A-1](https://www.digikey.ca/en/products/detail/te-connectivity-potter-brumfield-relays/W28-XQ1A-1/45056) | PB182-ND | 2 |

### Reward side — Digi-Key Canada

| # | Item | Part | Digi-Key # | Qty |
|---|---|---|---|---|
| 2 | Valve socket: M16 DIN 3-pin female, front mount (C091 A, contact layout 03-a) | [Amphenol Tuchel T3263000U](https://www.digikey.ca/en/products/result?keywords=T3263000U) | 361-1062-ND | 1 |
| 3 | Toggle, SPDT, 5 A @ 28 V DC: POWER, FLUSH | [Carling 2M1-SP1-T1-B1-M1QE](https://www.digikey.ca/en/products/detail/carling-technologies/2M1-SP1-T1-B1-M1QE/668359) | 432-1143-ND | 2 |
| 4 | Panel LED, green, 24 V, wire leads | [VCC 1091M5-24V](https://www.digikey.ca/en/products/detail/visual-communications-company-vcc/1091M5-24V/60780) | L50165-ND | 2 |
| 5 | BNC jack, panel, **isolated**: NSP, PEDAL, BUTTON | [TE 5227726-1](https://www.digikey.ca/en/products/detail/te-connectivity-amp-connectors/5227726-1/813448) | A32339-ND | 3 |
| 6 | Solid-state relay, DC output, 3.5–32 V DC input, 3 A @ 3–60 V DC | [Crydom DC60S3](https://www.digikey.ca/en/products/detail/sensata-crydom/DC60S3/221844) | CC1126-ND | 1 |
| 7 | Diode 1N4007 (D1–D4) | [onsemi 1N4007G](https://www.digikey.ca/en/products/detail/onsemi/1N4007G/1485479) | 1N4007GOS-ND | 10 |
| 8 | Lever connectors, 3-way | [WAGO 221-413](https://www.digikey.ca/en/products/detail/wago-corporation/221-413/13549610) | — | 15 |
| 9 | Hand switch: momentary push button, SPST-NO, 0.5 A @ 30 V DC, wire leads | [Switchcraft 921KX](https://www.digikey.ca/en/products/detail/switchcraft-inc/921KX/1832326) | SC1905-ND | 1 |

### Reward side — Abra Electronics

| # | Item | Part | Qty |
|---|---|---|---|
| 10 | Foot pedal, momentary, 2 m cable (COM / NO / NC) | [Adafruit 423 (423-ADA)](https://abra-electronics.com/electromechanical/switches/miscellaneous-en-16/423-ada-foot-switch-423-ada.html) | 1 |
| 11 | BNC male → terminal adapter, for the pedal and hand-switch ends | BB-BNC-P (screw) or [BB-BNC-P-SP (spring)](https://abra-electronics.com/interconnects/connectors/bnc-connectors-and-adapters/bnc-male-connector-to-spring-loaded-terminal-block-adapter.html), in [BNC adapters](https://abra-electronics.com/interconnects/connectors/bnc-connectors-and-adapters/) | 2 |
| 12 | BNC cable, RG58, 6 ft: NSP digital out → NSP | [BNC-120](https://abra-electronics.com/wire-cable-accessories/wirecable/bnc-cables/bnc-120-bnc-male-to-bnc-male-cable-rg58-6-ft-bnc-120.html) | 1 |
| 13 | Extension cable, 2 × 22 AWG shielded, by the foot (pedal / hand switch) | [CC2-22](https://abra-electronics.com/wire-cable-accessories/wirecable/round-communication-cable/cc2-22-round-multi-conductor-shielded-communication-cable-22awg-2-conductors-csa-per-foot.html) | ~20 ft |
| 14 | Hook-up wire 22 AWG red + black (inside the box), heat-shrink | any | — |

### Sync side (NSP sync, Beckhoff)

| # | Item | Part | Source / # | Qty |
|---|---|---|---|---|
| S1 | Beckhoff **EK1100**, **EL9505**, **EL2124**, **EL9011** on DIN rail | see [Components](#components) | in hand | 1 set |
| S2 | EtherCAT feedthrough, panel, D-size (etherCON Cat5e): ETHERCAT X65 | [Neutrik NE8FDP](https://www.digikey.ca/en/products/detail/neutrik-americas-inc/NE8FDP/29371355) | Digi-Key | 1 |
| S3 | BNC jack, panel, **isolated**: SYNC 1–4 | [TE 5227726-1](https://www.digikey.ca/en/products/detail/te-connectivity-amp-connectors/5227726-1/813448) | Digi-Key A32339-ND | 4 |
| S4 | BNC cable, RG58, 6 ft: SYNC 1 and SYNC 2 → NSP analog in 3 and 4 | [BNC-120](https://abra-electronics.com/wire-cable-accessories/wirecable/bnc-cables/bnc-120-bnc-male-to-bnc-male-cable-rg58-6-ft-bnc-120.html) | Abra | 2 |
| S5 | 35 mm DIN rail, ~120 mm, + 2 end stops | | | 1 |
| S6 | Cat5e patch cable, 0.3 m: ETHERCAT X65 feedthrough → EK1100 X1 IN | | | 1 |
| S7 | Cat5e cable: cabinet X65 → the box's ETHERCAT X65 | | | 1 |
| S8 | Hook-up wire 18–20 AWG (the stack's 24 V), 22–24 AWG (signals) | | | — |

### The valve

Honeywell Skinner **V52LB1100**, 24 V DC, 10 W (~0.42 A), normally closed. It is mounted on a plate
with its own **female 3-pin M16 DIN socket**; a male–male 3-pin cable runs from that socket to the
box's VALVE socket (#2).

### Plumbing

| Item | Choose | Where |
|---|---|---|
| Juice tubing | **Tygon S3 B-44-3** (Saint-Gobain), food & beverage grade: phthalate-free, FDA / NSF-51 | Cole-Parmer Canada, Fisher Scientific Canada, McMaster-Carr, US Plastic |
| Size | **1/4" ID × 3/8" OD** (1/16" wall). The valve's hose barb measures 7 mm across its largest ridge: a 1/4" barb | 50 ft roll |
| Retention | a small zip tie or ear clamp over each barb | |
| Spout arm | **Loc-Line 1/2" system** (~1/2" bore, so the 3/8" OD tube slides straight through): segment pack **51801** (2 × 5½") or bulk **59860** (25 ft); **51805** 1/2" NPT connectors (4-pack) for the arm's base | TENAQUIP, Cobalt Industrial, KBC Tools; modularhose.com |

---

## Panels and the CAD files

[`reward_and_sync_box.scad`](reward_and_sync_box.scad) (OpenSCAD) is the single source of every hole:
the `FRONT` and `REAR` tables at its top, and the printed stand-in's `PB_*` tables.

| File | `part=` | What it is |
|---|---|---|
| [`front_panel.stl`](front_panel.stl), [`rear_panel.stl`](rear_panel.stl) | `front`, `rear` | The 1.6 mm panels with every cut-out and the labels engraved 0.3 mm |
| [`reward_and_sync_box.stl`](reward_and_sync_box.stl) | `assembly` | The whole 2U box — frame, rack ears, both panels, rough bodies of the panel parts, the SSR and the Beckhoff stack on its DIN rail |
| [`print_box.stl`](print_box.stl), [`print_lid.stl`](print_lid.stl) | `print_box`, `print_lid` | The [3D-printed stand-in](#3d-printed-stand-in) box and lid |

Regenerate after editing the tables (from this folder; on this dev box OpenSCAD is the Windows install,
`"/mnt/c/Program Files/OpenSCAD/openscad.com"`):

```bash
openscad -D 'part="front"'     -o front_panel.stl          reward_and_sync_box.scad
openscad -D 'part="rear"'      -o rear_panel.stl           reward_and_sync_box.scad
openscad -D 'part="assembly"'  -o reward_and_sync_box.stl  reward_and_sync_box.scad
openscad -D 'part="print_box"' -o print_box.stl            reward_and_sync_box.scad
openscad -D 'part="print_lid"' -o print_lid.stl            reward_and_sync_box.scad
```

The file stops with an error if a hole strays into a keep-out margin.

Both panels are given **seen from outside**, origin at the bottom-left corner, on the RM2U1908SBK's
**415.30 × 84.53 × 1.6 mm** aluminium panels (Hammond drawing RM2U1908SBK). The keep-out (15 mm at
the sides, 8 mm top and bottom) clears the frame lips and panel screws.

```
 FRONT  ●LED   ●LED
        POWER  FLUSH   (NSP) (PEDAL) (BUTTON)   (ETHERCAT X65)  (SYNC 1) (SYNC 2) (SYNC 3) (SYNC 4)
         40     80      125   155     185            240           280      310      340      370   ← x, mm

 REAR   (24 V IN) (BREAKER VALVE) (BREAKER SYNC) (VALVE)
           35          90             150          210
```

| Panel | Hole | x | y | Cut-out (mm) | Part |
|---|---|---|---|---|---|
| front | POWER | 40 | 38 | Ø6.5 | Carling 2M1-SP1-T1-B1-M1QE |
| front | LED | 40 | 64 | Ø12.7 | VCC 1091M5-24V |
| front | FLUSH | 80 | 38 | Ø6.5 | Carling 2M1-SP1-T1-B1-M1QE |
| front | LED | 80 | 64 | Ø12.7 | VCC 1091M5-24V |
| front | NSP, PEDAL, BUTTON | 125, 155, 185 | 42 | single-D: Ø9.9, 8.84 across the flat | TE 5227726-1 |
| front | ETHERCAT X65 | 240 | 42 | Ø24 + 2 × Ø3.2 at (−9.5, +12) and (+9.5, −12) | Neutrik NE8FDP |
| front | SYNC 1–4 | 280, 310, 340, 370 | 42 | single-D: Ø9.9, 8.84 across the flat | TE 5227726-1 |
| rear | 24 V IN | 35 | 42 | Ø8 | Switchcraft 721A |
| rear | BREAKER VALVE | 90 | 42 | Ø16 | TE W28-XQ1A-1 |
| rear | BREAKER SYNC | 150 | 42 | Ø16 | TE W28-XQ1A-1 |
| rear | VALVE | 210 | 42 | Ø18, key notch 2.8 wide down to 19.1 from the top | Amphenol T3263000U |

Sources: M16 socket cut-out from the Amphenol C 091 A catalog (front-mount female receptacle, panel
max 5 mm); breaker cut-out 0.625–0.635" from the W28 listing; LED and toggle cut-outs from Digi-Key;
BNC D-cut from the Protocase cut-out library; etherCON cut-out from Neutrik's D-size panel drawing.

The front carries the controls and every signal connection (NSP, PEDAL, BUTTON, ETHERCAT X65,
SYNC 1–4); the rear carries what stays fixed: the 24 V input, both breakers and the valve.

### 3D-printed stand-in

Until the 2U box is in hand, everything goes in a printed box, [`print_box.stl`](print_box.stl) and
[`print_lid.stl`](print_lid.stl) (`part="print_assembly"` shows them together with the SSR and the
Beckhoff stack): the same parts, the same cut-outs and the same wiring as the 2U box.

The box is 180 (wide) × 170 (deep) × 85 (tall) mm plus the lid, so it fits an 11 × 11 in bed.

| Face | Parts |
|---|---|
| Front wall, upper row | POWER and FLUSH toggles (each with its LED above it); NSP, PEDAL, BUTTON |
| Front wall, lower row | ETHERCAT X65, SYNC 1–4 |
| Rear wall | 24 V IN, BREAKER VALVE, BREAKER SYNC (wall thinned to 1.5 mm around each breaker, so the W28 latches), VALVE socket (key notch down) |
| Lid | plain cover, four M3 screws |
| Floor | SSR (two M4 holes, 47.6 mm apart) on the left; DIN rail (two M4 holes, 80 mm apart) on the right, running front to back, with the Beckhoff stack on it |

#### Printing

| | Box | Lid |
|---|---|---|
| File | `print_box.stl` | `print_lid.stl` |
| Orientation | open side up, as exported | flat, as exported |
| Footprint × height | 180 × 170 mm × 85 mm | 180 × 170 mm × 2.5 mm |
| Supports | none | none |

Settings for both: **PETG** (or PLA), 0.4 mm nozzle, 0.2 mm layers, **4 perimeters** (walls solid
around the connector holes), 30 % infill, 5 top/bottom layers.

After printing: clear the holes of strings, check the BNC flats and the VALVE key notch with the
parts, and run an M3 screw into each corner post once before fitting the lid. Lid: four M3 × 10
self-tapping screws.

- **No chassis ground** in a plastic box. The isolated BNC jacks still keep the +24 V jack shells
  apart.
- Every part, the wiring and the cables move to the 2U box unchanged.

---

## Building it

1. **Drill the panels.** Remove each panel from the frame. Mark every hole from the table above,
   centre-punch, drill with a **step drill** (4–20 mm) and deburr. File the BNC flats and the VALVE
   key notch; the etherCON's two Ø3.2 holes take its M3 screws. Scrape the powder coat off around one
   chassis screw for the ground wire.
2. **Mount the panel parts** with their nuts: toggles, LEDs, BNC jacks (insulating washers on —
   these are isolated jacks), VALVE socket (key notch down), the DC jack, both breakers (snap in)
   and the etherCON feedthrough (two M3 screws).
3. **Mount the SSR** to the chassis floor with two M4 screws and nuts (no heatsink: 0.42 A).
4. **Wire the reward side** per the [wire list](#wire-list): 22 AWG, **red for +24 V / P, black for
   0 V**. Solder to the jack, toggle, BNC and socket lugs and sleeve each joint in heat-shrink; join
   the rest in Wago connectors.
5. **Diodes:** D1 across the VALVE socket's coil contacts (stripe on A). D2–D4 in the trigger leads
   (stripe toward the SSR), sleeved in heat-shrink.
6. **Sync stack:** screw the DIN rail to the floor behind the SYNC jacks, snap the stack on, and wire
   it per [Assembly](#assembly) — BREAKER SYNC and the 24 V IN sleeve to the EK1100, EL2124 outputs
   to SYNC 1–4, the short Cat5e patch from the etherCON to the EK1100's X1 IN. The stack's + comes
   only from BREAKER SYNC; no sync wire may touch P or COIL_A.
7. **Label** the panels — especially the BNC jacks.

---

## Cables for the pedal, hand switch and NSP

```
 PEDAL        COM + NO ── own 2 m cable ──[Wago]── CC2-22 (if longer) ──► [BB-BNC-P] ═► PEDAL jack
 HAND SWITCH  its two leads ──────────────[Wago]── CC2-22 (if longer) ──► [BB-BNC-P] ═► BUTTON jack
 NSP          digital out ═══════════ BNC-120 RG58 ══════════════════════════════════► NSP jack
```

- **Pedal (Adafruit 423):** find **COM + NO** with a multimeter (the pair that closes when pressed).
  Insulate NC.
- **Hand switch (Switchcraft 921KX):** SPST-NO, so its two leads go to the two terminals.
- **At the BB-BNC-P:** one wire to the centre terminal, one to the shield terminal (a switch has no
  polarity). With CC2-22, connect its drain wire to the shield terminal here only. Cover the
  terminals with heat-shrink or tape and add a cable tie as strain relief.
- **NSP:** a BNC cable from the NSP digital output; a TTL pulse opens the valve for the pulse's
  duration.

---

## First test

With the **valve unplugged** and a multimeter across the VALVE socket's coil contacts A–B:

1. **POWER** on → LED1 lights.
2. **FLUSH** on → LED2 lights, A–B reads **+24 V**. FLUSH off → 0 V.
3. Plug in the **pedal**: pressed → +24 V and LED2; released → 0 V.
4. Same for the **hand switch** on BUTTON.
5. A **5 V pulse** on the NSP jack → +24 V for the pulse.
6. Plug in the **valve** and repeat each, listening for the click.

If FLUSH clicks the valve but no trigger does, the fault is on the SSR side.

---

## Safety

- **Everything outside the box is 24 V DC.** Mains exists only inside the wall adapter.
- **The PEDAL and BUTTON jack shells are at +24 V.** Use only the isolated BNC jacks (#5, S3), label all
  the jacks, and keep the NSP cable out of the PEDAL and BUTTON jacks (it would short the NSP ground
  to +24 V until BREAKER VALVE trips).
- **Keep the adapter 2-prong** (floating) — see [How it works](#how-it-works).
- **One 0 V star point** at the 24 V IN jack's sleeve lug: the reward 0 V, the EK1100's 0 V and the
  chassis each run to it on their own wire.
- **Never remove D1.** Without the flyback diode, every valve close sends a high-voltage spike into
  the SSR.
- **Unplug the valve before working on the box.**

---

## Check before ordering or drilling

- **Breaker panel thickness.** The W28 snaps into panels **0.81–1.57 mm** thick; the Hammond panel is
  0.064" (1.63 mm) plus powder coat. Try it in a test hole first.
- **BNC cut-out.** The single-D (Ø9.9 / 8.84 flat) is from the Protocase library, not a TE drawing:
  check it against TE's product drawing for 5227726-1.
- **LED resistor.** Confirm on the VCC 1091M5-24V datasheet that the current-limiting resistor is
  built in (it is sold as a 24 V part).
- **SSR holes.** The 47.6 mm spacing in the printed box is the Crydom panel-mount standard; check it
  against the DC60S3 when it arrives.
- **Valve socket.** The C091 A 3-contact layout (DIN 41524 / IEC 60130-9, "03-a") is the only 3-pin
  C091 A layout; check the valve cable's plug mates before mounting.

---

## Sync: TTL pulses to the NSP

Trial events reach the Blackrock NSP as **5 V TTL pulses on BNC lines**, sent by the Sunrise cabinet
through a small Beckhoff EtherCAT stack. Each event type has its own line, so nothing has to be
decoded: **output 1 = trial start, output 2 = trial end**, outputs 3 and 4 spare. The NSP records
each line on an analog input, and the rising edge is the time reference for aligning the trial data.

```
 Sunrise cabinet            BOX PANELS              INSIDE: DIN rail on the box floor, modules left → right
 X65 (EtherCAT) ═ Cat5e ═► [ETHERCAT X65] ═ patch ═╗  ┌────────┬────────┬────────┬────────┐
                                                   ╚═►│ EK1100 │ EL9505 │ EL2124 │ EL9011 │
 24 V IN ═► [BREAKER SYNC] ───────────────────────────►│power in│24 V in*│4 × 5 V │end cap │
                                                      └────────┴────────┴───┬────┴────────┘
                           [SYNC 1] ◄── Out 1 + 0 V ────────────────────────┤ ═► NSP analog in 3 (trial start)
                           [SYNC 2] ◄── Out 2 + 0 V ────────────────────────┤ ═► NSP analog in 4 (trial end)
                           [SYNC 3] [SYNC 4] ◄── Out 3, Out 4 (spare) ──────┘
```
\* The EL9505's 24 V input is on its front, fed from the EK1100 by two jumpers (Assembly, step 3).

### Components

| Part | Role |
|---|---|
| Beckhoff **EK1100** | EtherCAT coupler: joins the stack to the cabinet's EtherCAT and takes the 24 V supply |
| Beckhoff **EL9505** | turns 24 V, fed to its own front points, into 5 V on the power contacts for the terminals to its right |
| Beckhoff **EL2124** | 4 digital outputs, 5 V push-pull (±20 mA each, short-circuit proof): the TTL lines |
| Beckhoff **EL9011** | end cap: covers the last terminal's exposed contacts |
| The box's 24 V supply, through the rear-panel **BREAKER SYNC** (TE W28-XQ1A-1) | power (the stack draws < 0.5 A). The adapter is floating (2-prong), so the stack makes no ground loop with the NSP |
| Front-panel **ETHERCAT X65** feedthrough (Neutrik NE8FDP etherCON) + a 0.3 m Cat5e patch inside | the cabinet's EtherCAT into the box, then to the EK1100 |
| Cat5e cable | cabinet X65 → the box's ETHERCAT X65 jack |
| 4 front-panel **isolated** BNC jacks, **SYNC 1–4** (TE 5227726-1) + one BNC cable per line used | the TTL lines to the NSP |
| 35 mm DIN rail, ~120 mm, + 2 end stops | screwed to the box floor; the stack snaps on |
| Hook-up wire 18–20 AWG (power), 22–24 AWG (signals) | |

Part numbers and links: [Sync side](#sync-side-nsp-sync-beckhoff) in the parts list.

### Assembly

1. **Snap the modules onto the DIN rail in this order: EK1100, EL9505, EL2124, EL9011.** The rail lies
   flat on the box floor, so the modules stand 68 mm tall with their terminals and RJ45 facing up. The side
   contacts carry the bus from module to module. The order matters: the EL9505 puts 5 V on the
   power contacts for everything to its right, which is what the EL2124 runs on.
2. **Power, at the EK1100.** It has two 24 V inputs and both must be fed: **Us** runs the coupler,
   **Up** feeds its power contacts. Wire **BREAKER SYNC** terminal 2 (+) and the 24 V IN jack's
   sleeve lug (0 V, its own wire to the star point) and two short jumpers:

   | EK1100 point | Label | Connect |
   |---|---|---|
   | 1 | 24V (Us) | **+24 V** from BREAKER SYNC |
   | 2 | + (Up) | jumper from point 1 |
   | 5 | 0V (Us) | **0 V** from the 24 V IN sleeve lug |
   | 7 | − (Up) | jumper from point 5 |
   | 4, 8 | PE | **leave empty** (floating supply, no ground loop) |

   Points 2/6 are both Up +, and 3/7 both Up −, joined inside the coupler, so 6 or 3 work as well.
   The point numbers are Beckhoff's standard coupler layout: go by the labels printed on the module.
   Check the polarity with a multimeter first (red on +, black on −: +24 V).
3. **Power, at the EL9505.** It does **not** take its 24 V through the side contacts: it has its own
   24 V input on the front, and with nothing wired there it makes no 5 V. Feed it from the EK1100's
   spare Up points with two more jumpers:

   | From EK1100 | To EL9505 |
   |---|---|
   | point 6 (+) | point **1** (24V) |
   | point 3 (−) | point **5** (0V) |

   The EL9505's other points: 2/6 are + and 3/7 are − of its 5 V output, 4/8 PE (leave empty).
   Check: 5 V between the EL9505's points 2 and 3.
4. **EtherCAT.** Cat5e from the cabinet's **X65** ("KUKA Extension Bus", an RJ45 socket meant for
   EtherCAT devices outside the controller; cabinet specification §6.7) to the box's front
   **ETHERCAT X65** jack; inside, the short patch from that jack to the EK1100's **upper** RJ45
   (X1 IN). The lower one (X2 OUT) stays empty; it is only for chaining more EtherCAT devices.
5. **The TTL lines, at the EL2124.** Each line is one output point plus 0 V. There are two 0 V
   points, shared by all four outputs:

   | Line | Front jack | Centre | Shell |
   |---|---|---|---|
   | Output 1, trial start | **SYNC 1** | point **1** | point **3** (0 V) |
   | Output 2, trial end | **SYNC 2** | point **5** | point **7** (0 V) |
   | Output 3 (spare) | SYNC 3 | point 4 | point 3 |
   | Output 4 (spare) | SYNC 4 | point 8 | point 7 |

   **Never connect a BNC to points 2 or 6: they carry +5 V**, and a shield there shorts the supply.
   Check before connecting the NSP, with the stack powered and the multimeter's black probe on the
   EL2124's **own point 3**: 2 and 6 read 5 V, 7 reads 0 V, and outputs 1 and 5 read 0 V while idle.
   If your module reads differently, follow the labels and the readings, not this table.
6. **BNC.** Solder each pair to its front **SYNC** jack's lugs per the table, and run a BNC cable from
   each jack used to the NSP. The shells carry the EL2124's 0 V; the jacks are the same isolated type
   as the reward side's.

### The NSP side

- **Output 1 → analog input 3, output 2 → analog input 4.** Record both at 30 kHz with filtering off.
  Analog inputs land in the continuous data (`.ns5` / NSx), **not** the `.nev` event file; the
  `.nev` takes events only through the NSP's digital input port.
- **Level.** A line is 0 V idle and 5 V during a pulse.
- **Ground.** The box's 0 V meets the NSP through the BNC shields. Keep the 24 V adapter floating
  (2-prong) and PE empty.

### Configuring the cabinet (once)

The cabinet has to know the stack is there before it can drive it. Its I/O configuration is edited in
**KUKA WorkVisual**, which Sunrise Workbench starts for you; Workbench alone cannot do it. These steps
follow the *KUKA Sunrise.OS 1.17* manual, chapter 11 "Bus configuration" (§11.1–11.7). **The names in
bold must be exactly as written**: the code finds the outputs by them.

1. **WorkVisual and its Sunrise option package**, once per laptop. Install **WorkVisual 5.0.5 Build 600**
   (the build the Sunrise.OS 1.17 release notes name), then in
   WorkVisual *Extras → Option package management* install **`Sunrise.kop`**, from the *WorkVisual
   AddOn* directory of the Sunrise Workbench installation. It must be the one supplied with *your*
   Workbench (1.17); after a Workbench update, replace it too. Restart WorkVisual.
2. **Create the I/O configuration from Workbench**, not in WorkVisual: select the project in the
   Package Explorer, then *File → New → I/O configuration*. WorkVisual starts with the project's
   WorkVisual project, and `IOConfiguration.wvs` appears in the Sunrise project; later, double-click
   that file to reopen it. (Started on its own, WorkVisual shows an empty project.)
3. **Import the Beckhoff device descriptions.** WorkVisual imports them only with no project open:
   close the project (*File → Close*), then *File → Import / Export → Import device description file*,
   and pick the three ESI files in
   [`sunrise_controller_code/ethercat_device_description/ESI_for_WorkVisual_505/`](../sunrise_controller_code/ethercat_device_description/ESI_for_WorkVisual_505/):
   `Beckhoff EK11xx.xml`, `Beckhoff EL2xxx.xml`, `Beckhoff EL9xxx.xml` (they cover the EK1100, EL2124
   and EL9505, prepared for WorkVisual 5.0.5). Then reopen the project by double-clicking
   `IOConfiguration.wvs` in Workbench.
4. **Set the controller active.** On the *Hardware* tab of the *Project structure* window, right-click
   the controller (shown as *Steuerung 1 (KRC4 - …)*, the Sunrise cabinet) → *Set as active
   controller*. The *I/O Mapping* window opens.
5. **Configure the bus** (WorkVisual manual, *Inserting devices into an internal field bus*). X65 is the
   **KUKA Extension Bus**, which WorkVisual does not show until it is added. **Do not add anything under
   *KUKA Controller Bus* or *KUKA System Bus*** — those are the robot's own internal buses.
   - Right-click *Bus structure* → *Add* → **KUKA Extension Bus (SYS-X44)** → *OK* (SYS-X44 is KUKA's
     name for the bus; on the Sunrise cabinet it leaves at X65).
   - Right-click that node → *Add* → **EK1100**. The terminals plug into the coupler, so they go **under
     the EK1100**, not under the bus: right-click the EK1100 → *Add* → **EL9505**, then **EL2124**, in
     their physical order (or drag them from the *DTM Catalog* onto the EK1100). Pick the catalogue
     entries whose revision matches the modules' labels. If they are not offered at all, their device
     descriptions were not imported: Beckhoff keeps them in separate ESI files by family
     (`Beckhoff EL2xxx.xml`, `Beckhoff EL9xxx.xml`). **Not the EL9011**: it is a passive end cap that never appears on the bus, and configuring a device the
     real bus does not have makes the bus fail to start.
   - For each device: right-click → *Settings…* → *General*: *Check vendor ID* and *Check product code*
     on, *Check revision number* off.
   - Right-click the Extension Bus → *Settings…* → *Topology*: check the chain is bus → EK1100 →
     EL9505 → EL2124 and correct any connection that is not.
6. **Create the Sunrise I/Os** (§11.5). In the *I/O Mapping* window: select the **EL2124** on the
   *Field buses* tab (top right), the *Sunrise I/Os* tab (top left), then the *Creates signals at the
   provider* button (bottom left). In *Create I/O signals*:
   - *Edit I/O group* → *Create*, name **`TtlSyncPulse`** (description, e.g. "TTL sync to the NSP") → *Create*.
   - For each of **`Output1`**, **`Output2`**, **`Output3`**, **`Output4`**: enter the name, *Direction*
     **Output**, *Type* **Digital**, *Data type* **BOOL** (bit width 1), and click *Create*.
   - *OK*.
7. **Map them** (§11.6.3): with `TtlSyncPulse` selected on *Sunrise I/Os* and the EL2124 on *Field buses*,
   drag `Output1` onto the EL2124's channel 1 output, `Output2` onto channel 2, and so on. Mapped
   signals show green arrows.
8. **Export to the Sunrise project** (§11.7): *File → Import / Export → Export the I/O configuration to
   the Sunrise Workbench project* → *Next* → *Finish*, then *File → Exit*. Workbench's automatic change
   recognition must be on (manual §5.9 "User definitions"; in Workbench *Window → Preferences →
   General → Workspace* → tick *Refresh using native hooks or polling*). In Workbench, refresh the
   project (**F5**): the generated class **`TtlSyncPulseIOGroup.java`** appears in the Package
   Explorer under the project's `src` → `com.kuka.generated.ioAccess` (next to `MediaFlangeIOGroup.java`),
   with `setOutput1` … `setOutput4`. Do not edit it: every export regenerates it. A
   "file is out of sync" warning on `IOConfiguration.wvs` means only that WorkVisual changed it on
   disk; F5 clears it.
9. **Synchronize** the project to the cabinet and **reboot the robot controller** (a changed I/O
   configuration only takes effect after the reboot). The EK1100's RUN LED goes steady once the
   stack is in operation.

When WorkVisual opens the project it may report *"No robot was found in controller Steuerung 1. Tools
and bases can not be imported."* That concerns importing tool and base data, which the I/O
configuration does not use; continue with the steps above.

The apps compile and run before this is done: until the generated class exists, the outputs report
themselves unavailable and nothing else changes.

### The code

All in [`sunrise_controller_code/`](../sunrise_controller_code/); deploy it like the main app
([Install Application to Robot](../README.md#install-application-to-robot)).

- **[`TtlPulser.java`](../sunrise_controller_code/TtlPulser.java)**: the outputs. `pulse(output, widthMs)`
  sets output 1–4 high for `widthMs` (1–5000) and returns at once; a second pulse on an output that is
  still high extends it. `allLow()` sets every output low; `close()` does too, on app exit. It binds to
  the generated `TtlSyncPulseIOGroup` by name at run time, and if the group is named differently, change
  `GROUP_CLASS` / `OUTPUT_SETTER` there.
- **[`LbrImpedanceControlServer.java`](../sunrise_controller_code/LbrImpedanceControlServer.java)**, the
  experiment app: opens the outputs at start-up (logging whether they are available), offers
  `sendTtlPulse(output, widthMs)`, and serves pulse commands from the network on **UDP 30002**:

  | Command (one UDP datagram, ASCII) | Reply |
  |---|---|
  | `PULSE <output 1-4> <width ms>` | `OK PULSE 1 10`, or `ERR <why>` |
  | `ALLOFF` | `OK ALLOFF` (every output low) |
  | `STATUS` | `OK available=true …`, or `ERR <why unavailable>` |

  The cabinet releases UDP ports 30000–30010 to applications; 30001 is the settings report, 30200 FRI.
- **[`PulseTestServer.java`](../sunrise_controller_code/PulseTestServer.java)**: a separate SmartPad app to
  test the lines **without moving the arm or starting FRI**. Pick an output (or *All*), a width
  (10 / 50 / 100 / 500 ms) and *Once* or *10 times, 1 per second*, and it fires; repeat until *Cancel*.
  *All* fires output N as N pulses, so each line identifies itself on the NSP. It serves the same UDP
  commands, so the network path from the ROS computer can be tested at the same time.

**From ROS**, each experiment sends `PULSE <output> <width ms>` at trial start and trial end, with the
cabinet address, the two outputs and the width set in its `nsp_sync` parameters (defaults
`172.31.1.147`, outputs 1 and 2, 10 ms): see
[Sync to the Blackrock NSP](../README.md#sync-to-the-blackrock-nsp).

**Timing.** The edge leaves when the cabinet writes the output, a few ms after the command, with a few
ms of jitter; the width comes from a Java timer and is good to a few ms. The edge marks when the pulse
was *commanded*, so align to the edge, and keep widths at 10 ms or more: shorter ones can fall between
I/O cycles.

### Testing, step by step

1. **Power only.** The EK1100's **Us** and **Up** LEDs are green, and the EL9505 makes 5 V: measure
   between **its own** points 2 (+) and 3 (−).
2. **Bus.** With the Cat5e in and the project synchronized, the EK1100's link LED at X1 IN flickers
   and its **RUN** LED is steady.
3. **SmartPad test.** Run **PulseTestServer**, pick *Output 1*, *500 ms*, *10 times*: the EL2124's
   channel-1 LED blinks once a second, and a multimeter between point 1 and point 3 jumps to 5 V with it.
   Repeat for output 2 (points 5 and 7).
4. **Network path.** With PulseTestServer running, from the ROS computer:
   ```bash
   echo "STATUS" | nc -u -w1 172.31.1.147 30002        # OK available=true ...
   echo "PULSE 1 10" | nc -u -w1 172.31.1.147 30002    # OK PULSE 1 10, and the channel-1 LED flickers
   ```
   The dashboard's *Diagnostics → TTL sync* tab sends the same commands
   ([its section](../experiment_ctrl_gui/README.md#ttl-sync)).
5. **NSP.** Connect the BNCs, start a Central recording, and run *All*, *100 ms*: analog input 3 shows
   one pulse, analog input 4 two.

### Troubleshooting

| Symptom | Likely cause → fix |
|---|---|
| No LEDs on the EK1100 | No power or reversed polarity: measure +24 V between points 1 and 5 |
| **Us** green, **Up** dark | The jumpers 1→2 and 5→7 are missing or loose |
| 0 V between the EL9505's own points 2 and 3, Us and Up green | The EL9505's front 24 V input is not fed: jumper EK1100 6 → EL9505 1 and EK1100 3 → EL9505 5 (Assembly step 3) |
| EK1100 link LED dark | The Cat5e is in the lower RJ45 (X2 OUT) or in the wrong cabinet port; or the cable is bad |
| RUN LED off or blinking | The cabinet has not taken the stack into operation: the I/O configuration does not match the physical modules or their order, or the project was not synchronized after it |
| SmartPad / log: *TTL outputs unavailable: no I/O group …TtlSyncPulseIOGroup* | The I/O configuration is not in the project, or the group is not named `TtlSyncPulse`: configure the cabinet, synchronize |
| *… lacks setOutput1* | The outputs are not named `Output1`…`Output4` |
| `nc` gets no reply | No app serving UDP 30002 is running, or the ROS computer cannot reach `172.31.1.147` (`ping` it); `nc` needs `-u` |
| *UDP 30002 is busy* in the log | Another app already holds the port (e.g. PulseTestServer and the experiment app at once): end one |
| `OK PULSE …` but the channel LED stays dark | The mapping in the I/O group points at the wrong channels or module: check it in WorkVisual |
| Channel LED blinks, but the NSP shows nothing | BNC wiring (output to centre, 0 V to shield), the wrong NSP input, or that input not enabled in Central |
| 50/60 Hz hum or spikes on the NSP channels | A ground loop: the adapter is earthed (3-prong) or PE is connected; use a floating adapter, leave PE empty |
| The 5 V reads lower under load, or a channel stays dead | A shield wired to point 2 or 6 (+5 V): rewire to 3 or 7 |
