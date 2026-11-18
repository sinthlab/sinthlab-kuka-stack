# Data path — the RS-422 link from the ROS computer to the end effector

How the ROS computer talks to the end-effector board (Metro M4 AirLift): the cue commands go out, the
board's acknowledgements come back, and the same link will carry the pressure sensor and haptics.
This page is the complete build: parts, cable, the X76 plug, both ends, the software, the tests and
troubleshooting. Follow it top to bottom to rebuild the link from nothing.

Related pages: the [tool connector pinout](../end_effector_design/README.md#tool-connector-pinout), the board's
[serial protocol](../end_effector_metro_code/README.md#the-serial-link), and
[§6.7 of the top-level README](../README.md#67-end-effector-board--the-visual-cue) for how the
experiments use it.

## Contents
- [Architecture](#architecture)
- [Parts](#parts)
- [Documents and references](#documents-and-references)
- [Pin map](#pin-map)
- [1. Prepare the cable](#1-prepare-the-cable)
- [2. Assemble the X76 plug (Intercontec 915 bypack)](#2-assemble-the-x76-plug-intercontec-915-bypack)
- [3. Check the plug and the arm with a meter](#3-check-the-plug-and-the-arm-with-a-meter)
- [4. PC end: the StarTech adapter](#4-pc-end-the-startech-adapter)
- [5. Software on the ROS computer](#5-software-on-the-ros-computer)
  - [Re-attach after WSL restarts or crashes](#re-attach-after-wsl-restarts-or-crashes)
- [6. Wrist loopback test](#6-wrist-loopback-test)
- [7. Tool end: MikroE and Metro](#7-tool-end-mikroe-and-metro)
- [8. Board tests](#8-board-tests)
- [9. Use it in the experiments](#9-use-it-in-the-experiments)
- [Current state of this build](#current-state-of-this-build)
- [Troubleshooting](#troubleshooting)

---

## Architecture

```
 ROS computer (Windows + WSL2)
 ┌───────────────────────────────────────────┐
 │ orchestrator ─► VisualCue / RingStateCue  │   visual_cue.trigger: "serial"
 │                 └► SerialCueTrigger       │   actions/visual_cue_serial.py
 │                     /dev/ttyUSB0 (WSL)    │
 │   usbipd-win attaches the USB device ─────┼──► USB
 └───────────────────────────────────────────┘
                                                  StarTech ICUSB422IS  (USB ↔ RS-422, isolated)
                                                  DIP 1 ON, 2-4 OFF
                                                   TX± ─┐      ┌─ RX±   GND (+ drains)
                                                        │      │         │
             ~10 m cable, 3 shielded twisted pairs ═════╪══════╪═════════╪═══
                                                        │      │         │
 Robot base, interface A1                          X76 1/2   X76 4/5    X76 7
 (Intercontec 915 12+3 plug)                            │      │         │
             arm's internal harness + media flange ═════╪══════╪═════════╪═══
                                                        │      │         │
 Tool connector at the wrist                      pins 9/10  12/13     15
                                                        │      │         │
 End effector                                      MikroE RS485 3 Click (SN65HVD31, 3.3 V)
                                                    RX±  ◄──┘      └──► TX±    GND
                                                    UART: click RX ◄─ Metro D0 (TX)
                                                          click TX ─► Metro D8 (RX)
                                                    Metro M4 AirLift ─ D5 ─► Pixel Shifter ─► ring
```

- **Full duplex, one pair each way.** The PC → board pair is CTR1 (X76 1/2, tool 9/10); the board →
  PC pair is CTR2 (X76 4/5, tool 12/13). Both are shielded pairs inside the arm.
- **It is a crossover.** Each end's TX pair lands on the other end's RX pair.
- **Signal ground** runs on CTR3 (X76 7, tool 15) so the StarTech's isolated side shares a reference
  with the board.
- **Drains are bonded at the PC end only.**
- **No optocoupler.** The StarTech isolates the link from the PC (2500 Vrms); the tool side runs on
  the Metro's own 3.3 V.
- The board runs from the cabinet's 24 V on tool pins 1/2 (not part of this link). Pins 3–6 carry
  live EtherCAT and stay capped.

## Parts

| # | Part | What it is | Link | Qty |
|---|---|---|---|---|
| 1 | **StarTech ICUSB422IS** | USB ↔ RS-422/485 adapter, isolated, FTDI FT232RL, USB-powered | [StarTech](https://www.startech.com/en-us/cards-adapters/icusb422is) · [Digi-Key search](https://www.digikey.ca/en/products/result?keywords=ICUSB422IS) | 1 |
| 2 | **MikroE MIKROE-2821** RS485 3 Click | SN65HVD31 full-duplex transceiver, 3.3 V; terminals TX±, RX±, 2 × GND | [MikroE](https://www.mikroe.com/rs485-3-click) · [Digi-Key search](https://www.digikey.ca/en/products/result?keywords=MIKROE-2821) | 1 |
| 3 | **Cable**, ~10 m (30 ft used) | 3 individually foil-shielded twisted pairs with drain wires (blue, red and green bundles, each a colour + black) | [Encore Wire C6040A-41-10](https://www.digikey.ca/en/products/detail/encore-wire/C6040A-41-10/98578) | room route + ~1 m |
| 4 | **X76 connector bypack** | KUKA's 12+3 bypack for X76: Intercontec **Series 915** plug, 12 signal + 3 power, lamella clamp, crimp contacts | supplied by KUKA with the arm | 1 |
| 5 | **Crimp tool**, 4-indent | IWISS 4-indent / 8-impression crimper for turned contacts, selector 1–7 | [Amazon.ca B07TW4X7JP](https://www.amazon.ca/dp/B07TW4X7JP) | 1 |
| 6 | Metro M4 Express AirLift Lite | the end-effector board | [Adafruit 4000](https://www.adafruit.com/product/4000) | (in the effector) |
| 7 | Kapton or electrical tape | insulates the contacts inside the plug (see [step 2](#2-assemble-the-x76-plug-intercontec-915-bypack)) | — | — |
| 8 | Heat shrink, hookup wire | drain stubs, MikroE ↔ Metro wires | — | — |

## Documents and references

| Document | What it gives |
|---|---|
| [Intercontec assembly instructions (Schweiger catalogue PDF)](https://www.schweiger-group.de/sites/default/files/Montageanleitungen_Gesamtkatalog.pdf), **page 12**: *Series 915 Plug / Extension, 12+3-pin, lamella clamp, cable 4.5–10.5 mm* (page 13 is the 10.5–12 mm version) | the plug's parts in order and the 15 assembly pictures |
| KUKA *Media flange* manual (V10, 20 Oct 2021), Inside electric: §5 connection tables, Fig. 5-38/5-39 | X76 contacts → tool connector pins; not in the repo |
| [`docs/Spez_KUKA_Sunrise_Cabinet_en.pdf`](../docs/Spez_KUKA_Sunrise_Cabinet_en.pdf) | the cabinet's interfaces |
| [Tool connector pinout](../end_effector_design/README.md#tool-connector-pinout) | every tool pin, its colour and its X76 contact |
| [TI SN65HVD31](https://www.ti.com/product/SN65HVD31) | the MikroE's transceiver |
| [usbipd-win](https://github.com/dorssel/usbipd-win) | sharing the USB adapter with WSL2 |
| [Firmware serial protocol](../end_effector_metro_code/README.md#the-serial-link) | `CUE`, `CUE r g b w`, `SEG`, `OFF`, `PING` and their replies |

## Pin map

The whole link, end to end. Tool-side colours are KUKA's; cable colours are this cable's.

| Signal | StarTech | Cable wire | X76 | Tool pin (colour, bundle) | MikroE |
|---|---|---|---|---|---|
| PC → board + | TX+ | blue bundle, **green** | **1** | 9 (BK, BK/BU/WH) | **RX+** |
| PC → board − | TX− | blue bundle, **black** | **2** | 10 (BU, BK/BU/WH) | **RX−** |
| drain | GND | blue bundle, drain | **3** | 11 (WH, BK/BU/WH) | — insulated |
| board → PC + | RX+ | red bundle, **red** | **4** | 12 (GN, GN/YE/WH) | **TX+** |
| board → PC − | RX− | red bundle, **black** | **5** | 13 (YE, GN/YE/WH) | **TX−** |
| drain | GND | red bundle, drain | **6** | 14 (WH, GN/YE/WH) | — insulated |
| signal ground | GND | green bundle, **white** | **7** | 15 (RD thin, RD/BU) | **GND** |
| spare | — capped | green bundle, black | 8 (optional) | 16 (BU thin, RD/BU) | — insulated |
| — | GND | green bundle, drain | — cut back at X76 | — | — |

MikroE labels its terminals from its own side: **TX±** are its driver outputs, **RX±** its receiver
inputs.

At the wrist, the bundles tell repeated colours apart: the 3-wire **BK/BU/WH** bundle is CTR1, the
3-wire **GN/YE/WH** bundle is CTR2, the thin 2-wire **RD/BU** bundle is CTR3. The 2-wire **YE/OG** and
**WH/BU** bundles are **EtherCAT** (tool 3–6) and the thick RD/BK wires are power; leave all of those
alone.

---

## 1. Prepare the cable

1. **Tag the three black wires at both ends** (blue, red, green) before cutting or stripping
   anything; out of their bundles they look the same.
2. At the robot end, strip the jacket for the plug (the Intercontec drawing strips 22 mm), and trim
   each bundle's foil back to the jacket so it cannot reach a contact.
3. Strip each conductor to the depth of a contact's crimp barrel (about 4–5 mm; check against a
   contact), twist the strands, no nicks.
4. Cut the **green bundle's drain** back to the jacket and cover the stub with heat shrink. Bundle 3
   at the wrist has no drain to meet it.
5. At the PC end, keep all three drains; they go to the StarTech's GND.

## 2. Assemble the X76 plug (Intercontec 915 bypack)

Follow **page 12** of the Intercontec PDF; the notes below are what this build needed on top of it.

**Parts, in the order they sit on the cable:** union nut (back nut) → lamella clamp → thin ring →
contacts in the signal comb (+ the 3-pin power comb) → insulating insert ("sleeve") → housing. The
**hinged plastic clip** (drawn closed in the parts row) is fitted last, around the wires. The
bypack's **green rubber ring** fits neither the nut nor the clamp and is not used.

1. **Slide onto the cable first:** the back nut, then the lamella clamp, then the thin ring. Nothing
   can go on after the contacts are crimped.
2. **Crimp the contacts** with the 4-indent tool:
   - The selector (1–7) is crimp depth, not wire gauge; a higher number crimps deeper. Find the
     setting on a spare contact: start at 3, pull-test, go up one number until the wire holds and
     the barrel shows four even dents without cracking.
   - Put the contact in the tool's locator so the jaws close on the **middle of the barrel**, not on
     the inspection hole or the pin. Squeeze one or two clicks to hold it, insert the wire until
     strands show in the inspection hole, squeeze until the ratchet releases.
   - A thin drain wire can slip out of the barrel: fold it back on itself to double it before
     crimping.
3. **Contacts into the signal comb.** The comb carries the contact numbers (the insert does not).
   Put each contact in its numbered slot per the [pin map](#pin-map) and push until its collar
   clicks in. All tips must stand at the same height, straight and parallel. Clip the power comb
   onto the signal comb even though it stays empty.
4. **The insulating insert does not seat on this build** (see [current state](#current-state-of-this-build)).
   In its place, **wrap the comb and contacts in one layer of Kapton or electrical tape, up to the
   contact tips**, so no contact can touch the metal housing or a neighbour.
5. **Hinged clip:** close it around the wires behind the comb until it clicks.
6. **Into the housing**, lining up the coding key, then tighten the back nut by hand plus a little
   with a 17 mm spanner. It does not need to close flush: the cable must not move when tugged.

The plug mates by **pushing straight on** to the robot's X76 socket (there is nothing to screw).

## 3. Check the plug and the arm with a meter

All with the **cabinet off** (tool pins 1–6 are live whenever it is on).

**3a. The plug alone (unplugged), meter on beep, nothing shorted:**

- every wire against the **metal housing**: silent;
- every wire against every other wire: silent.

Then short the PC-end blue green ↔ blue black: the plug's contacts **1 ↔ 2** beep.

**3b. The arm alone (plug unplugged):** twist the wrist's BK + BU (BK/BU/WH bundle) together and beep
the robot's X76 socket: contacts **1 and 2** beep.

**3c. Through everything (plug mated):** short at the PC end, beep at the wrist, and wiggle the plug
and cable during each:

| Short at the PC end | Beeps at the wrist |
|---|---|
| blue green ↔ blue black | BK ↔ BU (BK/BU/WH) |
| blue green ↔ blue drain | BK ↔ WH (BK/BU/WH) — open on this build, see below |
| red ↔ red-bundle black | GN ↔ YE (GN/YE/WH) |
| red ↔ red drain | GN ↔ WH (GN/YE/WH) |
| green-bundle white ↔ blue green | thin RD ↔ BK |
| nothing | no beep between any of them, nor to tool pin 2 or the flange |

The whole loop is a few ohms (30 ft of cable plus the arm); a meter's beep threshold is far above
that, so length never stops a beep. **OL in Ω mode means open.**

## 4. PC end: the StarTech adapter

1. Wire the cable to the StarTech's terminals per the [pin map](#pin-map): blue green TX+, blue black
   TX−, red RX+, red-bundle black RX−, green-bundle white and **all three drains** to GND. Cap the
   green-bundle black.
2. **DIP switches:**

   | Switch | Setting |
   |---|---|
   | 1 — 422 | **ON** (RS-422, 4-wire full duplex) |
   | 2 — N-Ech | **OFF** |
   | 3 — Term | **OFF** |
   | 4 — RVD | **OFF** |

3. It is **USB-powered**; plug it straight into a PC port.

**Adapter self-test:** with the cable disconnected, jumper **TX+ ↔ RX+** and **TX− ↔ RX−** at its
terminals and run the loopback command in [step 6](#6-wrist-loopback-test): `b'hello kuka'` proves
the adapter, its switches and the software.

## 5. Software on the ROS computer

Once per computer:

1. **usbipd-win** (Windows PowerShell, then reopen it as administrator):
   ```powershell
   winget install --interactive --exact dorssel.usbipd-win
   ```
2. **pyserial** (WSL; `pip` is blocked on Ubuntu 24.04's system Python):
   ```bash
   sudo apt install python3-serial
   ```
3. **Serial-port permission** (WSL), then restart WSL from PowerShell with `wsl --shutdown`:
   ```bash
   sudo usermod -aG dialout $USER
   ```

After every replug, reboot or `wsl --shutdown`:

1. **Attach the adapter to WSL** (PowerShell as administrator; `bind` is needed only the first time):
   ```powershell
   usbipd list                          # the StarTech shows as an FTDI / USB Serial Converter; note its BUSID
   usbipd bind --busid <busid>
   usbipd attach --wsl --busid <busid>
   ```
2. **Check it in WSL:** `ls /dev/ttyUSB*` shows `/dev/ttyUSB0`.
3. **Latency timer to 1 ms** (the FTDI default of 16 ms delays every reply):
   ```bash
   echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
   ```
   The experiments' serial sender warns at startup when it is not 1.

### Re-attach after WSL restarts or crashes

The adapter leaves WSL whenever WSL stops: `wsl --shutdown`, a WSL crash, a Windows reboot, or the
adapter being unplugged. `/dev/ttyUSB0` is then gone, and experiments with `trigger: "serial"` log
`serial port … cannot be opened`. Re-attach it (PowerShell **as administrator**):

```powershell
usbipd list                              # find the StarTech (FTDI / USB Serial Converter) and its BUSID
usbipd attach --wsl --busid <busid>
```

If `attach` fails — the device shows as *Not shared*, *Attached* to a WSL that no longer exists, or
the error says it is in use — reset it, then attach:

```powershell
usbipd detach --busid <busid>
usbipd unbind --busid <busid>
usbipd bind --busid <busid>
usbipd attach --wsl --busid <busid>
```

Then in WSL: `ls /dev/ttyUSB*` shows the port, set the latency timer again (it resets on every
attach), and **Check board** from Diagnostics → Visual cue with the path set to **Serial**. A running experiment's serial sender
reopens the port by itself once it is back; no restart is needed.

```bash
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

## 6. Wrist loopback test

Proves the whole cable and arm path in both directions before any board is involved.

1. Cabinet off. At the tool connector join **BK ↔ GN** (tool 9 ↔ 12) and **BU ↔ YE** (tool 10 ↔ 13).
2. In WSL:
   ```bash
   python3 -c "import serial,time;s=serial.Serial('/dev/ttyUSB0',115200,timeout=0.5);s.write(b'hello kuka');time.sleep(0.1);print(s.read(64))"
   ```
3. `b'hello kuka'` passes. Remove the jumpers.

## 7. Tool end: MikroE and Metro

Cabinet off while wiring.

| From | To |
|---|---|
| tool 9 (BK) | MikroE **RX+** |
| tool 10 (BU) | MikroE **RX−** |
| tool 12 (GN) | MikroE **TX+** |
| tool 13 (YE) | MikroE **TX−** |
| tool 15 (thin RD) | MikroE **GND** (either GND terminal) |
| MikroE 3.3V | Metro **3.3V** |
| MikroE GND | Metro **GND** |
| MikroE RX (data **into** the click) | Metro **D0** (board TX) |
| MikroE TX (data **out of** the click) | Metro **D8** (board RX) |

Tool 11 and 14 (drains) and 16 stay insulated.

**Why D0/D8 and not the board's 0/1 serial pins:** pins 0 and 1 sit too close together to solder.
The SAMD51 only runs a UART on certain pin pairs; TX on D0 with RX on D8 is one of them. To list the
valid pairs on the board, paste into its REPL:

```python
import board, busio
names = ["D0","D1","D3","D4","D6","D7","D8","D9","D10","D11","D12","D13",
         "A0","A1","A2","A3","A4","A5","SDA","SCL"]
pins = [(n, getattr(board, n)) for n in names if hasattr(board, n)]
for tn, tp in pins:
    for rn, rp in pins:
        if tp is rp: continue
        try:
            u = busio.UART(tp, rp, baudrate=115200); u.deinit()
            print("OK  TX =", tn, " RX =", rn)
        except Exception:
            pass
```

D2 is the board's old contact-closure input and D5 drives the ring; neither is used by the link.

## 8. Board tests

1. **Deploy the firmware:** copy [`end_effector_metro_code/code.py`](../end_effector_metro_code/code.py)
   to the CIRCUITPY drive. The serial console shows
   `Serial link on TX board.D0 / RX board.D8 at 115200 baud`, and the ring gives two dim boot
   flashes.
2. **Commands** (WSL):
   ```bash
   python3 -c "import serial,time;s=serial.Serial('/dev/ttyUSB0',115200,timeout=0.5)
   for c in (b'PING\n',b'CUE\n',b'CUE 255 0 0 0\n',b'SEG 4 0,255,0,0\n',b'OFF\n'):
       s.write(c);time.sleep(0.3);print(c,s.read(64))"
   ```
   Expect `PONG`, `ACK CUE 1` (ring flashes), `ACK CUE 2` (red), `ACK SEG` (quarter 0 green),
   `ACK OFF` (dark).

   **Or from the dashboard:** Diagnostics → Visual cue, path **Serial**. Every control there is the same
   as on the Wi-Fi path — Check board, Apply, Fire cue, Solid colour, Quarter 0 green, Off, Save, the
   LED-by-LED test — and each one goes down the link as `GET /<route>` and runs the board's own route,
   so a test passes or fails the same way on both paths. If the board's `code.py` predates `GET`, the
   page falls back to `PING` / `CUE` / `SEG` / `OFF`, and the settings and the LED test need Wi-Fi.
   The experiments use only `CUE`, `SEG`, `OFF` and `PING`, so they never need `GET`.
3. **Soak test:** 1000 round trips; wiggle the X76 plug and the cable while it runs.
   ```bash
   python3 - <<'EOF'
   import serial, time, statistics
   s = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.2)
   bad, rtts = 0, []
   for i in range(1000):
       t0 = time.perf_counter(); s.write(b"PING\n"); got = s.readline()
       rtts.append((time.perf_counter() - t0) * 1000); bad += got != b"PONG\n"
   print(f"errors {bad}/1000   rtt median {statistics.median(rtts):.1f} ms   max {max(rtts):.1f} ms")
   EOF
   ```
   Pass: 0 errors and a median round trip of a few ms.

## 9. Use it in the experiments

In the experiment's YAML (`sinthlab_bringup/config/*.yaml`):

```yaml
visual_cue:
  enabled: true
  trigger: "serial"             # serial or wifi
  serial_port: "/dev/ttyUSB0"
```

`trigger` is a live parameter (dashboard or `ros2 param set`). Each cue logs
`Visual cue '<site>' fired over the serial link (… ms round trip)`, and the board's acknowledgement
is recorded as `cue_visual_ack`. Code:
[`visual_cue_serial.py`](../sinthlab_bringup/sinthlab_bringup/actions/visual_cue_serial.py).

---

## Current state of this build

- **The X76 plug's insulating insert is missing from the assembly.** With it fitted, the comb stops
  about 3 mm short of its latch window and the contacts sit too deep for the robot's ~6 mm pins to
  reach. Until that is solved, the insert is left out and **the comb and contacts are wrapped in
  tape** so no contact can touch the metal housing or another contact. Nothing holds the contacts
  forward, so **recheck the link after every unplug**
  ([3a](#3-check-the-plug-and-the-arm-with-a-meter) shorts check, then **Check board** on the Serial path).
- **The blue bundle's drain does not reach tool pin 11** (open in the ring-out). The cable's shield
  is still bonded at the PC end; only the arm's internal CTR1 shield floats.
- **The green-bundle black (X76 8)** may be left unpinned; it is a spare.

## Troubleshooting

Work from the PC outwards; each test splits the path in two.

| Symptom | Check | Fix |
|---|---|---|
| `usbipd` not recognized | usbipd-win installed? | [Step 5](#5-software-on-the-ros-computer); reopen PowerShell |
| No `/dev/ttyUSB0` in WSL (after a WSL restart, crash, reboot or replug) | `usbipd list`: attached? | [Re-attach](#re-attach-after-wsl-restarts-or-crashes); if `attach` fails, detach → unbind → bind → attach |
| `Permission denied: '/dev/ttyUSB0'` | `groups` lists `dialout`? | `sudo usermod -aG dialout $USER`, then `wsl --shutdown`; one-off: `sudo chmod 666 /dev/ttyUSB0` |
| `pip install pyserial`: externally-managed-environment | — | `sudo apt install python3-serial` |
| Loopback returns `b''` | adapter self-test ([step 4](#4-pc-end-the-startech-adapter)) | fails: DIP switches (1 ON, 2–4 OFF), terminal labels. Passes: meter the path — with the wrist jumpers in, PC-end blue green ↔ red and blue black ↔ red-bundle black must beep |
| Loopback returns garbled bytes | — | one pair has + and − swapped: swap the two wires of one pair at the PC end |
| Wrist BK ↔ BU silent with PC-end green ↔ black shorted | 3a (plug alone) and 3b (arm alone) | plug fails: crimp or slot (re-crimp, check the comb number). Arm fails: tool connector not flush on the flange. Both pass: the contacts are not reaching each other when mated — check how far the robot's pins enter (marker on robot pin 1, mate, unmate, look) and the plug's contact depth (a 1 mm probe in contact 1 must beep from about 3 mm in) |
| A wire beeps to the plug's metal housing | — | a contact is touching the shell: open the plug, re-tape the comb up to the tips, recheck 3a before mating |
| Board does not answer (`b''`), wrist loopback passes | serial console: `Serial link on TX board.D0 …`? | swap the wires on D0 and D8; check the MikroE has 3.3 V and its **GND to the Metro**; console shows `Serial link unavailable` → the pins are not a valid UART pair |
| Board does not answer, console shows nothing received | the MikroE RX± wires | tool 9/10 must land on RX+/RX−, not TX± |
| Board receives (console activity) but the PC gets nothing | the MikroE TX± wires | tool 12/13 must land on TX+/TX− |
| Garbled replies from the board | — | a pair swapped at the MikroE: swap RX+/RX− or TX+/TX− |
| `ERR …` replies | the command | see the [protocol](../end_effector_metro_code/README.md#the-serial-link) |
| Board seems dead after rewiring | 3.3V ↔ GND on the Metro (power off, Ω) | near 0 Ω: a solder bridge or a swapped MikroE 3.3V/GND; unplug the MikroE 3.3V wire and power up to confirm |
| Errors in the soak test, more when wiggling | the X76 plug | re-seat and re-tape the contacts; recheck 3c while wiggling |
| Experiment log: `serial port … cannot be opened` | steps 5 | attach the adapter, `dialout`, `python3-serial`; the sender reopens the port on its own once it exists |
| Experiment log: `latency_timer is 16 ms` | — | `echo 1 \| sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer` (resets on every attach) |
| Experiment log: `NOT confirmed: no reply` | Diagnostics → Visual cue, path **Serial**, **Check board** | board power, the MikroE GND, the X76 plug |
