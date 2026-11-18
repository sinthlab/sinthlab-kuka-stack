# End-Effector Board Firmware — NeoPixel Cue Ring (Metro M4 AirLift)

> # ⛔ DO NOT COPY THIS FILE TO THE BOARD
> **`README.md` and `test_code.py` are repository files only — they must never be written to the
> `CIRCUITPY` drive.** When flashing or refreshing a board, copy **only** `code.py`,
> `settings.toml` and `lib/` — that is the complete payload. See [Deployment](#deployment).

CircuitPython firmware for the **Adafruit Metro M4 Express AirLift Lite** that lives in the base hub
of the [apple-pluck end effector](../end_effector_design/README.md). It drives the **60-LED RGBW
NeoPixel ring** (Adafruit 2874, Ø157) seated in the cover as a **visual cue** for the monkey.

---

## How it works

**A cue is on and off with a timer.** The ROS computer sends a cue; the board runs its configured cue
for its configured duration, then stops, and answers once the ring is lit. A cue can carry a colour,
and the pre-training tasks also hold quarter-ring states. That is the entire behaviour.

**It arrives on either of two paths, and the experiment is the same on both.** `visual_cue.trigger`
in the experiment's YAML picks one:

| | **Serial** — the RS-422 link | **Wi-Fi** — the board's access point |
|---|---|---|
| Path | StarTech → X76 → flange → tool pins 9/10, 12/13 → MikroE → UART `D0`/`D8` | the ROS computer joined to `KUKA_NEOPIXEL` → `http://192.168.4.1` |
| A cue | `CUE`, `CUE r g b w` → `ACK CUE <n>` | `/cue`, after `/config?r=..` for a colour |
| A ring state | `SEG f colors`, `OFF` | `/segments?factor=f&colors=…`, `/off` |
| Delivery time | a few ms, every cue | varies from cue to cue |

**What the cue looks like is a setting, changed over the board's own Wi-Fi.** Join the board's
access point from a laptop or phone and call `/config`:

```bash
curl "http://192.168.4.1/config?r=0&g=255&b=0&w=0&pattern=segment&segments=6&duration=1.2&save=1"
```

So **each cue says *when* (and optionally which colour), and the board — already configured by hand
— decides the rest.**

The board hosts its **own** access point and never joins another network. Wi-Fi bring-up is wrapped
in `try`/`except`, so a dead ESP32 or a bad credential leaves the serial link working, running the
settings saved in NVM; a UART that cannot be opened leaves Wi-Fi working.

The board also keeps a contact-closure input on `D2` (pull-up, active low) from before the serial
link. Nothing is wired to it: tool pins 9/10 now carry the link.

> ### Status — serial link proven to the board, ROS side untested on the arm
> The board runs on real hardware from the cabinet's 24 V. The RS-422 link has passed an echo test
> through X76, the flange and the MikroE; the experiments' serial sender has only been tested against
> a simulated board.

---

## Contents
- [Hardware context](#hardware-context)
- [The cue trigger — wiring](#the-cue-trigger--wiring)
- [Architecture](#architecture)
- [Settings](#settings)
- [Power — read before raising brightness](#power--read-before-raising-brightness)
- [Files on the board](#files-on-the-board)
- [HTTP API](#http-api)
- [Deployment](#deployment)
- [Testing](#testing)
- [`settings.toml` — Wi-Fi credentials](#settingstoml--wi-fi-credentials)
- [Changing the firmware without USB](#changing-the-firmware-without-usb)
- [Driving the trigger from ROS 2](#driving-the-trigger-from-ros-2)
- [Known limitations](#known-limitations)

---

## Hardware context

| | |
|---|---|
| **Board** | Adafruit Metro M4 Express AirLift Lite ([4000](https://www.adafruit.com/product/4000)), `samd51j19` |
| **Firmware** | CircuitPython **10.1.4** |
| **Wi-Fi** | ESP32 **co-processor over SPI** — *not* native Wi-Fi (this is why the code uses `adafruit_esp32spi`, not `wifi`) |
| **LED ring** | 60 × 5050 **RGBW**, Adafruit [2874](https://www.adafruit.com/product/2874), bought as 4 quarter-arcs |
| **Ring data** | `board.D5` → **Pixel Shifter** ([6066](https://www.adafruit.com/product/6066), 3.3 V → 5 V) → ring `DIN` |
| **Cue input** | `board.D2`, internal pull-up, active low. Nothing is wired to it: tool pins 9/10 carry the serial link |
| **Serial link** | UART **TX `board.D0`** → MikroE RS485 3 Click RX, **RX `board.D8`** ← click TX, 115200 baud; click RX± ← tool pins 9/10, TX± → tool pins 12/13, GND ← tool pin 15 |
| **Power** | cabinet 24 V via the X650/X651 data cable → tool pins 1/2 → Tobsun 24 V→5 V → board **and** ring (5 V injected at all 4 quarter joints) |

```
   MEDIA FLANGE (Inside electric) — a pass-through, no electronics
   │
   ├─ tool pins 1/2  ── 24 V from the cabinet ──► Tobsun 24 V → 5 V ──┬──► Metro M4 AirLift
   │                                                                 │     ├ SAMD51 (code)
   │                                                                 │     └ ESP32 (Wi-Fi)
   │                                                                 │
   │                                                                 └──► NeoPixel ring
   │                                                                      (5 V at all 4 quarter joints)
   │
   ├─ tool pins 9/10  ── CTR1, RS-422 from the ROS computer ──► MikroE RX± ─┐
   ├─ tool pins 12/13 ── CTR2, RS-422 to the ROS computer ◄── MikroE TX± ─┤ UART: Metro D0 (TX)
   └─ tool pin 15     ── CTR3, signal ground ───────────────── MikroE GND ─┘       Metro D8 (RX)

   Metro D5 ── 3.3 V data ──► Pixel Shifter ── 5 V data ──► ring DIN
```

> **⚡ Current is not capped in firmware — see [Power](#power--read-before-raising-brightness).**
> `brightness` is settable to 1.0, and at 1.0 the ring can draw up to **~4.8 A at 5 V**. The
> default of `0.2` is a conservative starting point, not a measured limit for your build. Never
> feed the ring from the Metro's own 5 V pin.

---

## The cue trigger — wiring

The trigger is a **contact closure**. Shorting X76 contacts 1 and 2 at the robot base connects `D2`
to `GND` through the media flange's CTR1 pair; the Metro's internal pull-up holds `D2` high the rest
of the time.

```
   robot base                  media flange (pass-through)             end effector

   X76 contact 1 ──┐                                                ┌── tool pin 9  (BK) ──► D2
                   ├─ switch ═══════ CTR1 shielded pair ════════════┤
   X76 contact 2 ──┘                                                └── tool pin 10 (BU) ──► GND

   X76 contact 3 ── CTR1 drain — bond here only; tool pin 11 (WH) stays insulated
```

| Tool connector | Colour | Connect to |
|---|---|---|
| pin **9** (CTR1_1) | BK | Metro **`D2`** |
| pin **10** (CTR1_2) | BU | Metro **`GND`** |
| pin 11 (CTR1_3, drain) | WH | insulated — the drain is bonded at the base |

**Polarity — `active_low` (default `1`):** the internal pull-up holds `D2` high, so **idle = HIGH**
and **cue asserted = LOW**. This is the safe polarity: an unplugged connector, an open switch, or a
broken wire all read as *no cue* rather than a stuck-on cue.

**No optocoupler is needed** as long as the switch is a floating contact — a jumper, or a relay's
dry contacts. The only voltage in the loop is the Metro's own 3.3 V. Anything that *outputs* a
voltage onto X76 1/2 is different: 24 V on `D2` destroys the SAMD51, so that must go through an
optocoupler — and since optocoupler output stages often invert, flip polarity over Wi-Fi rather than
opening the effector:

```bash
curl "http://192.168.4.1/config?active_low=0"   # switches the pin to an internal pull-down
```

The firmware resynchronises the debouncer as part of the change, so flipping polarity does not
itself look like an edge and fire a spurious cue.

For a production build, add an external **4.7 kΩ pull-up** from `D2` to 3.3 V. The internal pull-up
is weak, and the line runs several metres past seven joint drives.

### Bench-testing without the robot

You do not need the robot to verify the firmware. With the board on USB:

```
jumper D2 ──► GND     = cue asserted  (ring flashes)
remove jumper         = idle
```

Then confirm what the board actually sees over Wi-Fi:

```bash
curl http://192.168.4.1/status | grep trigger
# trigger_pin_raw=False        <- raw pin level
# trigger_asserted=True        <- after applying the active_low setting
# trigger_debounced=True       <- what the edge detector acted on
```

Those three lines together tell you whether a "the cue does not fire" problem is in the wiring, the
polarity setting, or the firmware.

---

## Architecture

Everything is one file, [`code.py`](code.py), which CircuitPython runs automatically at boot and
re-runs on every save. Bring-up is ordered so the primary path never depends on the secondary one:

```
 1. PIXEL SETUP      neopixel.NeoPixel(D5, 60, brightness=0.2,
                                       auto_write=False, pixel_order=GRBW)
        │            auto_write=False ⇒ writes are buffered until pixels.show()
        ▼
 2. BOOT INDICATOR   two short dim-white flashes — proves the board booted and the ring
        │            is wired, visible from outside a closed effector
        ▼
 3. LOAD SETTINGS    microcontroller.nvm → cfg, if a valid record is stored;
        │            otherwise DEFAULTS. Corrupt or absent NVM is not an error.
        ▼
 4. CUE INPUT        D2 as input, internal pull-up (or pull-down if active-high)
        │            ◄── the primary path is live from here on
        ▼
 5. WIFI + HTTP      ESP32 over SPI → create_AP() → Server(...).start(192.168.4.1, 80)
        │            ENTIRELY inside try/except: on failure, log and continue with
        │            server = None. The cue path is unaffected.
        ▼
 6. MAIN LOOP        while True: poll_trigger() ; cue_service() ; server.poll()
```

### The main loop

```python
while True:
    poll_trigger()          # debounced edge detect on D2  -> may start/stop a cue
    cue_service()           # advance the flash animation   -> non-blocking
    led_test_service()      # step /led_test, if running    -> non-blocking
    if server is not None:
        try:
            server.poll()   # handle at most one HTTP request -> non-blocking
        finally:
            close_accepted()  # close that request's connection if the server left it open
```

**Nothing in this loop sleeps and no route handler sleeps.** That is the property that makes the
whole design work: a cue animates while HTTP requests are being served, and HTTP traffic cannot
delay a cue beyond one short loop pass. Trigger latency is therefore ≈ the `debounce_ms` setting.

**No socket leaks.** The ESP32 has only a few sockets. `adafruit_httpserver` leaves a connection open
when a request ends in a reset or "try again" error, for example when the client gave up and hung up
first. Each such request would hold a socket for good, and after a few trials the board stops
answering HTTP while still answering `ping`. So the firmware tracks every connection the server
accepts and closes it after that `poll()` if the server has not. `/status` reports how many it had
to close (`connections_reclaimed`).

### The cue engine

A cue is state, not a blocking sequence:

```
cue_start(color, duration, period, pattern)
    ├─ snapshot the current ring contents      (restored when the cue ends)
    ├─ _cue_until_ns = now + duration          (or -1 for "until stopped")
    └─ render the frame at phase 0

cue_service()   ← every loop pass
    ├─ past _cue_until_ns? ──────► cue_stop(): restore the snapshot
    ├─ pattern "solid"     ──────► nothing to animate, return
    ├─ phase = ((now - start) % period) / period
    ├─ "flash" → redraw only when the half-period boundary is crossed
    └─ others  → redraw at a fixed ~30 fps, timed from *now* rather than by
                 accumulation, so a slow loop pass cannot queue up a burst
                 of catch-up frames
```

Snapshot/restore makes a cue **non-destructive**: if the ring was showing something before the cue —
a solid colour, a segment pattern — it comes back afterwards. A manual HTTP command arriving
mid-cue cancels it (`cue_stop(restore=False)`) and wins, because an operator at a browser is
deliberately overriding.

### Trigger modes

`mode` decides who owns the cue's length:

| Mode | Behaviour | Use when |
|---|---|---|
| **`pulse`** *(default)* | An asserted edge starts the cue, which runs for its configured **`duration`** and stops. The line may drop immediately. | Normal use — on and off with the board's timer. |
| `follow` | The cue runs for **exactly as long as the line is held**; `duration` is ignored. | You want the cabinet to own cue length, the way it owns `AudioCue` length. |

`retrigger=0` (the default) makes a second edge arriving mid-cue a no-op, so contact bounce or a
double-pulse cannot restart the cue.

## Settings

**This is the whole configurable surface of the board**, and all of it lives behind one endpoint,
`/config`. Every field is independent: send any subset and the rest is unchanged.

```bash
curl http://192.168.4.1/config                                  # read everything
curl "http://192.168.4.1/config?r=0&g=255&b=0&w=0"              # colour
curl "http://192.168.4.1/config?brightness=0.3&duration=1.2"    # brighter, shorter
curl "http://192.168.4.1/config?pattern=segment&segments=6"      # look
curl "http://192.168.4.1/config?save=1"                          # keep across reboots
curl http://192.168.4.1/cue                                      # try it without the wire
```

### What the cue looks like

| Field | Values | What it does |
|---|---|---|
| `r` `g` `b` `w` | 0–255 | colour (`w` is the dedicated white channel — cleaner than `r=g=b`) |
| `pattern` | `flash` `solid` `breathe` `chase` `segment` | the cue modality |
| `segments` | 1–30 | lit blocks, for the `segment` pattern |
| **`duration`** | 0.05–300 s | **the timer** — how long the ring stays on after a trigger |
| `period` | 0.02–10 s | one full cycle of the pattern |

| Pattern | Behaviour | Update rate |
|---|---|---|
| `flash` | square on/off at `period` | boundary only |
| `solid` | steady for `duration` | drawn once |
| `breathe` | smooth raised-cosine fade in/out | ~30 fps |
| `chase` | a lit arc of 8 LEDs rotating once per `period` | ~30 fps |
| `segment` | `segments` lit blocks evenly spaced, flashing together | boundary only |

### How the board behaves

| Field | Values | What it does |
|---|---|---|
| `brightness` | 0.0–1.0 | scales the whole ring. **Not capped — [read this](#power--read-before-raising-brightness).** |
| `mode` | `pulse` `follow` | who owns the cue's length — `duration`, or the wire |
| `active_low` | 0 / 1 | trigger polarity — **flip this if the trigger reads inverted, without opening the box** |
| `debounce_ms` | 1–200 | how long the input must hold a level |
| `retrigger` | 0 / 1 | may a new edge restart an in-flight cue |
| `enabled` | 0 / 1 | arm/disarm the wire entirely |

### Power — read before raising `brightness`

**`brightness` is the only scale, and it is deliberately not capped in firmware.** It multiplies
the ring's current draw very nearly linearly, and keeping that inside what the hardware can deliver
is the operator's call, not something the firmware should quietly decide for you.

At `brightness = 1.0`, 60 SK6812 RGBW LEDs draw roughly:

| Colour | Per LED | 60 LEDs |
|---|---|---|
| `w=255` only | ~20 mA | **~1.2 A** ← the white channel is the cheap one |
| `r=g=b=255, w=0` | ~60 mA | **~3.6 A** |
| `r=g=b=w=255` | ~80 mA | **~4.8 A** ← absolute worst case |

Multiply by `brightness` for the actual draw. The default cue (`w=255`, `brightness=0.2`) is about
**0.24 A**.

Before raising it, check **all** of:

- **The Tobsun 24 V→5 V converter's rating**, and its derating when warm. This is the usual limit.
- **The 5 V wiring gauge** down the flange bore. Several amps through thin wire means voltage drop,
  heat, and visible colour shift at the far end of the ring.
- **That 5 V is injected at all four quarter-ring joints.** Feeding 60 LEDs through one arc's
  traces browns out the far end and overheats the near end — the ring is bought as 4 arcs for
  exactly this reason.
- **Temperature inside the closed casing box.** The effector is handled by an animal, so surface
  temperature is a subject-safety limit, not just an electronics one.
- **How bright the cue actually needs to be.** A cue is a *signal*, not illumination, and the ring
  sits centimetres from the subject's face. The lowest brightness that reads reliably is the right
  one — for the subject as much as for the wiring.

**Never feed the ring from the Metro's own 5 V pin**: that comes off the board regulator and cannot
source anything close to these currents.

### Persistence — NVM, not a file

`/config?save=1` writes every setting above to `microcontroller.nvm`, restored at the next boot.
Without it, the board comes back with the defaults in `code.py`.

> **Why NVM and not a config file?** Writing any file from firmware requires `storage.remount()`
> in `boot.py`, which makes **CIRCUITPY read-only to your computer** and puts a boot-time failure
> between you and a working board. NVM has neither cost. See
> [Changing the firmware without USB](#changing-the-firmware-without-usb).

The record is magic-tagged, versioned and checksummed; a corrupt record falls back to the defaults
rather than failing to boot.

NVM **is** present on this board — `/status` reports `nvm=available`. The firmware still falls back
to a RAM buffer if a build ever lacks it, and that is about the *failure mode*, not the
probability: `nvm_load()` runs at module scope, **before the trigger is set up**, so an unguarded
access on a missing `nvm` would stop `code.py` before `poll_trigger()` exists and the ring would
fire nothing at all. The fallback costs two lines and lets `nvm_save`/`nvm_load`/`nvm_clear`
contain no presence checks at all — they always work, and the only difference is whether values
outlive a reboot. In that case `/status` reads `nvm=RAM ONLY (not persistent)` and `save=1` says
`saved to RAM ONLY`, so it never pretends.

#### Is repeated saving bad for the board?

`microcontroller.nvm` is a reserved region of the SAMD51's internal flash, and a write is an
erase-and-program of a flash row. That has a finite endurance — **~25,000 cycles** per the SAMD51
datasheet (confirm for your exact part). Three things keep it a non-issue:

- **Saving is never automatic.** `nvm_save()` has exactly one caller: `/config?save=1`. There is
  no timer, no save-on-change, no save-on-shutdown. Tuning over `/config` is free; only the
  explicit save writes.
- **An identical save costs nothing.** The firmware packs the record, compares it to what is
  already stored, and returns without touching flash if they match. Re-saving the same settings,
  or a script that loops on `save=1`, does **zero** writes. `reset=1` on an already-cleared record
  is likewise free.
- **The counter is visible.** `/status` reports `nvm_writes` — actual flash writes since power-up.
  If a script is saving more than you expect, it shows up there instead of quietly eating cycles.

So the cycle count advances only when the stored settings genuinely change. At a handful of real
changes per commissioning session, 25,000 cycles is thousands of sessions — not a limit you will
reach by hand.

**If it ever did wear out**, the failure is benign: the row stops holding the value, the checksum
fails at boot, and the board falls back to the defaults in `code.py`. Settings stop persisting;
nothing bricks, and the trigger keeps working. NVM is a separate region from the CIRCUITPY
filesystem, so wear there cannot touch `code.py`.

**Power loss mid-write** is the other risk worth naming — the arm's 24 V can vanish at any moment.
A half-written record fails its checksum on the next boot and is rejected wholesale, so the board
comes up on defaults rather than on a corrupted config. Save while the arm is idle, not mid-trial.

`/config?reset=1` restores the defaults and invalidates the record.

> **Values are quantised by the save.** `brightness` is stored as one byte (~0.4% steps) and
> `duration`/`period` as centiseconds. Read back after a save, `brightness=0.9` returns `0.898`.
> Colour components are bytes already, so those round-trip exactly.

## Files on the board

Only `code.py` and `settings.toml` run on the board as our code; `lib/` is the Adafruit library
bundle, and `README.md` never leaves the repo.

| Path | Origin | Deploy to board? | Notes |
|---|---|---|---|
| **`code.py`** | **ours** | **yes** | The entire application. Auto-run and auto-reloaded by CircuitPython. |
| **`settings.toml`** | **ours** | **yes** | AP SSID + password. See [below](#settingstoml--wi-fi-credentials). |
| **`lib/`** | Adafruit bundle | **yes** | `.mpy` libraries, below. |
| `README.md` | **ours** | **no** | This document. Repository documentation only. |
| **`test_code.py`** | **ours** | **no** | Offline test suite — see [Testing](#testing). Never copy it to the drive. |

The board's own drive also carries `boot_out.txt` (written *by* CircuitPython at boot — firmware
version, board ID, UID) and the zero-byte macOS indexing suppressors `.fseventsd/no_log`,
`.metadata_never_index` and `.Trashes`. **Leave all of those on the board**; none are mirrored here,
because none are ever deployed *from* the repo. Nothing else belongs on the drive: `code.py` needs
only the files listed here.

### `lib/` — what is actually required

| Library | Why |
|---|---|
| `neopixel.mpy` | Drives the ring. (`adafruit_pixelbuf` is built into the CircuitPython core on this board.) |
| `adafruit_esp32spi/` | ESP32 co-processor driver + the `socketpool` shim the HTTP server sits on. |
| `adafruit_httpserver/` | The routing / request / response layer. |
| `adafruit_bus_device/` | SPI transaction helper — a dependency of `adafruit_esp32spi`. |

`lib/` holds exactly these four. Leave everything *inside* `adafruit_esp32spi/` as it
is — the package imports its own submodules internally, so its apparent duplicates (`socketpool` vs
`adafruit_esp32spi_socketpool`, `wifimanager` vs `adafruit_esp32spi_wifimanager`) are not spare
copies to prune.

---

## HTTP API

**Secondary path.** All routes are **GET**, served at `http://192.168.4.1/` once you have joined the
`KUKA_NEOPIXEL` network. Colour components are `r`, `g`, `b`, `w`, each `0–255`.

> **Colour ordering.** `pixel_order=GRBW` is the **wire** order the LEDs expect; the library handles
> that reordering internally. In the API and in code you always pass values in **R, G, B, W** order.

> **Every route returns immediately.** No handler blocks — flashes are handed to the cue engine and
> animated by the main loop.

| Route | Parameters | Effect |
|---|---|---|
| **`/config`** | any setting, plus `save=1` / `reset=1` | **The whole configurable surface.** No parameters = read-only. |
| **`/cue`** | — | Run the configured cue **now**, without the wire. |
| **`/status`** | — | Everything `/config` shows, plus the **live trigger-pin state**, fire counts (`wire_fired`, `serial_fired`), the serial link's state (`serial`), uptime and `connections_reclaimed` (leaked HTTP connections it closed). First stop when debugging. |
| `/off` | — | Clear the ring and cancel a running cue. |
| `/set_color` | `r` `g` `b` `w` | Hold a solid colour (not a cue — it does not time out). |
| `/segments` | `factor`, `colors` | Static arcs. For a *timed* segmented cue use `pattern=segment`. |
| `/led_test` | `from`, `to` (pixel 0–59), `step` (seconds per colour, 0.05–5, default 0.25), `loop=1` | **Diagnostics.** Lights one LED at a time in red, green, blue, then white, walking the ring. `/status` shows `led_test_pixel` while it runs; `/off` stops it. |
| `/fs` | — | List what is on the board, with sizes. |
| `/fs/get` | `path` | Read a text file back (`code.py`, `*.txt`, `*.json`). **Read-only** — see [below](#changing-the-firmware-without-usb). |

### Examples

Join the board's access point (`KUKA_NEOPIXEL`) first — it is always at **192.168.4.1**.

```bash
# what is the board doing right now?  (settings, trigger pin, fire count, uptime)
curl http://192.168.4.1/status
```

**Set what the cue looks like — this is the part that replaces editing `code.py`:**

```bash
# dim green, six segments, 0.8 s
curl "http://192.168.4.1/config?r=0&g=255&b=0&w=0&pattern=segment&segments=6&duration=0.8"

# change ONE field; everything else carries over
curl "http://192.168.4.1/config?brightness=0.35"
curl "http://192.168.4.1/config?pattern=breathe&period=1.2"
curl "http://192.168.4.1/config?duration=2.5"

# try it without the robot, then keep it across reboots
curl http://192.168.4.1/cue
curl "http://192.168.4.1/config?save=1"
```

**How the board behaves:**

```bash
curl "http://192.168.4.1/config?mode=follow"        # let the wire own cue length
curl "http://192.168.4.1/config?brightness=0.25"    # READ THE POWER SECTION FIRST
curl "http://192.168.4.1/config?active_low=0"       # trigger reads inverted
curl "http://192.168.4.1/config?enabled=0"          # disarm the wire while you work on it
curl "http://192.168.4.1/config?reset=1"            # back to the defaults in code.py
```

**Manual control and read-back:**

```bash
curl "http://192.168.4.1/set_color?w=255"
curl http://192.168.4.1/off
curl http://192.168.4.1/fs                          # what is on the drive
curl "http://192.168.4.1/fs/get?path=code.py"       # confirm what is actually running
```

**Finding where a partly-dark ring stops:**

```bash
curl http://192.168.4.1/led_test                            # every LED, 4 colours each (~1 minute)
curl "http://192.168.4.1/led_test?from=43&to=48&step=1"     # slowly, across the joint into the last arc
curl "http://192.168.4.1/led_test?from=46&to=46&loop=1"     # hold one LED cycling while you probe it
curl http://192.168.4.1/status | grep led_test              # which pixel is lit right now
curl http://192.168.4.1/off                                 # stop
```

The ring is four arcs of 15 LEDs: pixels **0–14, 15–29, 30–44, 45–59**, in data order (the arc the data
wire lands on is pixel 0). Reading the result:

- **One LED lit at a time draws almost nothing**, so a pixel that stays dark in this test is *not* the
  supply sagging under load. It is a data joint, a dead LED, or an arc with no 5 V of its own.
- **An arc with data but no 5 V/GND can light its *first* LED dimly** — powered through the data
  line — while the rest of it stays dark. If the dark section starts one LED into an arc, measure
  5 V across that arc's pads before suspecting the LEDs.
- **5 V present, but the walk stops at the same pixel every time:** that LED passes no data on. The
  first LED next to a soldered joint is the usual casualty of soldering heat.

### `/segments` format

`colors` is a **`.`-separated list of segments**, each a **`,`-separated** `R,G,B[,W]` group:

```
colors = 255,0,0,0 . 0,255,0,0 . 0,0,255,0 . 0,0,0,255
          └ seg 0 ┘  └ seg 1 ┘  └ seg 2 ┘  └ seg 3 ┘
```

The `.` separator works only because every component is an integer — never introduce decimals here.
Segments beyond `factor` are ignored; a 3-value segment is auto-padded with `W=0`; a malformed
segment is skipped with a message on the serial console rather than failing the request.

> **Segment size truncates.** `seg_size = 60 // factor`. Any factor that does not divide 60 leaves
> the remaining pixels **dark** — e.g. `factor=7` lights 7 × 8 = 56 LEDs and leaves 4 off. Clean
> divisors of 60: **2, 3, 4, 5, 6, 10, 12, 15, 20, 30**.

---

## Deployment

CircuitPython presents the board as a USB mass-storage drive named **`CIRCUITPY`**. Deployment is a
plain file copy; **saving `code.py` makes the board reload and re-run it immediately** — there is no
build, flash, or reset step.

```bash
# Linux / WSL2 — the drive mounts through Windows, so it appears under /mnt/<letter>
ls /mnt/d/                                      # confirm you see code.py — i.e. the right drive
cp end_effector_metro_code/code.py /mnt/d/code.py
```

On Windows just copy to the `CIRCUITPY:` drive in Explorer; on macOS to `/Volumes/CIRCUITPY`.

**For a fresh board**, copy exactly three things — and nothing else:

```bash
cp end_effector_metro_code/code.py       /mnt/d/
cp end_effector_metro_code/settings.toml /mnt/d/
mkdir -p /mnt/d/lib
cp -r end_effector_metro_code/lib/. /mnt/d/lib/    # note the /. — `cp -r lib /mnt/d/lib` would
                                                   # nest it as /mnt/d/lib/lib on an existing board
```

A board straight from Adafruit already has a populated `lib/`; this copy adds ours **alongside**
those files rather than replacing them. Delete the factory-demo libraries off the drive
(`simpleio`, `adafruit_dotstar`, `adafruit_hid/`, `adafruit_waveform/`) to match this folder.

**Never copy `README.md` or `test_code.py` to the drive**, and never copy the board's
`boot_out.txt` back over itself —
it is the board's own record of the installed firmware. Leave the board's `.fseventsd/`,
`.metadata_never_index` and `.Trashes` in place.

**To pull the board's state back into this repo**, copy `code.py` (and `lib/` if you changed it) in
the other direction, and **mask the Wi-Fi password before committing** (see below).

### Serial console (REPL)

`code.py` prints the cue-pin configuration, the ESP32 firmware version and MAC, the server URL, any
Wi-Fi bring-up failure, every HTTP request (`debug=True`), and any exception from the poll loop.
Watch it while debugging:

```bash
# WSL2 cannot see USB serial directly — use PuTTY / Tera Term on the Windows side at 115200 baud,
# or attach the device with usbipd-win first.
screen /dev/ttyACM0 115200      # native Linux
```

`Ctrl-C` in the REPL stops `code.py`; `Ctrl-D` reloads it.

---

## Testing

[`test_code.py`](test_code.py) runs the **real firmware** on a desktop — no board, no
dependencies, nothing to install:

```bash
python3 end_effector_metro_code/test_code.py     # exits non-zero on failure
```

It stubs CircuitPython's modules in-process (`board`, `neopixel`, `digitalio`, the ESP32 driver,
the HTTP server, `microcontroller.nvm`) and replaces `time.monotonic_ns` with a virtual clock, so
a 2.5 s cue costs no real time and the suite cannot flake on a slow machine. `code.py` is then
executed up to — but not including — its `while True:` loop, which leaves every function and route
handler callable directly.

**121 checks**, covering the things that are painful to discover on hardware:

| | |
|---|---|
| The trigger | on at the edge, off when the timer expires, stays on after the line drops, several durations |
| Polarity & debounce | `active_low` flip does not fire a spurious cue, sub-debounce pulses are ignored, `retrigger`, `follow`, `enabled=0` |
| Settings | fields compose, bad input keeps the old value, brightness clamps to 0–1, colour is unrounded |
| Persistence | full round trip through NVM, and a **corrupt record is rejected wholesale** rather than half-loaded |
| Degradation | **Wi-Fi down → the wire still fires the cue** |
| Sockets | a request the server leaves open (reset, error) is closed after its poll; none stays open, and `/status` counts them |
| Patterns | each renders distinctly, and **every one is lit at phase 0** so a cue has a crisp onset |
| Cue hygiene | a cue restores what the ring was showing; a manual command cancels it and wins |
| LED test | exactly one LED lit per step, every LED through all four colours in order, `from`/`to`/`loop`, and a cue or manual command stops it without leaving a test pixel behind |
| File access | read-only, refuses `settings.toml`, refuses traversal, no write route |
| Surface | the HTTP endpoints are exactly the documented set, and `/status` reports every documented field |

**What it does not cover:** anything physical — the flange wiring, the real ESP32, LED timing,
current draw. It checks logic, not electrons. Still bench-test with a jumper on `D2`
([above](#bench-testing-without-the-robot)) before trusting a build on the arm.

Run it after any edit to `code.py`, before copying to the board.

---

## `settings.toml` — Wi-Fi credentials

CircuitPython parses `settings.toml` at boot and exposes its keys through `os.getenv()`:

```toml
CIRCUITPY_WIFI_SSID = "KUKA_NEOPIXEL"
CIRCUITPY_WIFI_PASSWORD = "your-ap-password"
```

Two things worth knowing:

1. **These key names are borrowed, not magic here.** On boards with *native* Wi-Fi, CircuitPython
   reads them to auto-join a network as a *client*. This board has no native Wi-Fi, so nothing
   automatic happens — `code.py` reads them itself and uses them to **create an AP**.
2. **⚠️ The committed copy has the password masked out (`####`), and that masked form is not valid
   TOML** — `#` starts a comment, so the value is lost and `os.getenv()` returns `None`. This is
   deliberate (the real credential is not in the repo), but it means **you must write a real, quoted
   password onto the board's own `settings.toml`** before the Wi-Fi path will come up. WPA2 requires
   **8–63 characters**.

   The firmware detects this case explicitly and prints
   `WiFi/HTTP unavailable: CIRCUITPY_WIFI_SSID / CIRCUITPY_WIFI_PASSWORD missing …` rather than
   dying — **the hardwired cue still works on a board with no valid credentials.**

---

## Changing the firmware without USB

The board ends up bolted inside a closed effector, so "do I have to open the box?" is a real
operational question. The answer depends on *what* you are changing.

| You want to change | Needs USB? | How |
|---|---|---|
| Cue colour, length, rate, **pattern**, brightness | **No** | `/config?...` |
| Trigger polarity, debounce, mode, arm/disarm | **No** | `/config?...` |
| Any of the above, permanently | **No** | `/config?...&save=1` (NVM) |
| Read back what is on the board | **No** | `/fs`, `/fs/get?path=code.py` |
| **Write** new code to the board | **Yes** | copy `code.py` to the drive |
| Debug a crash, read a traceback | **Yes** | USB serial — see [Serial console](#serial-console-repl) |

That split is deliberate: **tuning is a request, new behaviour is a code change.** The runtime
settings cover the cases that would otherwise send you to the USB port for a one-line edit.

### Why there is no upload endpoint

CircuitPython's built-in **Web Workflow** — browser file manager + REPL over Wi-Fi — is exactly the
feature this calls for, and it is **not available on this board**. Web Workflow requires *native*
Wi-Fi (the core `wifi` module); here the Wi-Fi is an **ESP32 co-processor driven by a Python
library**, so the core has no networking to serve it from. Confirm in the REPL — this fails:

```python
import wifi        # ImportError on metro_m4_airlift_lite
```

*(This is also why `CIRCUITPY_WIFI_SSID` / `CIRCUITPY_WIFI_PASSWORD` do nothing automatically:
they are Web Workflow's settings keys, and `code.py` has to read them by hand.)*

A custom upload endpoint is possible — the bundled `adafruit_httpserver` is **4.8.1**, which has
`POST` and `request.body` — but it is not shipped, because the write side is where the costs are.
**Reading files needs nothing; writing them changes how the board boots:**

1. Writing any file from firmware requires `storage.remount("/", readonly=False)` in `boot.py`,
   and from then on **CIRCUITPY is read-only to your computer.** You trade USB editing for Wi-Fi
   editing rather than gaining both. A jumper read in `boot.py` can switch between them — but you
   have to open the box to move it.
2. **`boot.py` runs before `code.py`.** If it raises, CircuitPython enters **safe mode** and never
   runs `code.py` at all: no Wi-Fi, no cue, USB-only recovery.
3. **One bad `code.py` upload takes the Wi-Fi server with it** — the script dies before
   `server.start()`, so the only way back in is the USB port you were trying to avoid.
4. Flash wear, and filesystem corruption if the arm's 24 V drops mid-write.

### If you decide you want it anyway

Do not OTA `code.py` directly — use a **loader pattern**, which makes bricking essentially
impossible:

- `code.py` stays small, stable and **never uploaded**. It brings up Wi-Fi and the upload endpoint
  **first**, then `try: import app` for the real logic.
- A broken `app.py` logs the traceback and leaves the network up, so you can push a fix.
- **Never** put `settings.toml` or `lib/` in the upload path: a bad `settings.toml` kills Wi-Fi at
  boot with no recovery but USB, and `lib/` is 57 KB of binaries that change approximately never.
- `adafruit_httpserver` buffers the whole request in `request_buffer_size` (default **1024 B**) and
  `code.py` is ~26 KB, so chunk the upload with an offset parameter rather than raising the buffer
  on a 192 KB-RAM part.
- Internal writes do not trigger auto-reload; call `supervisor.reload()` yourself.

### The alternative worth considering first

A **panel-mount USB extension** from the Metro out to the casing wall. It solves the actual problem
— physical access — with no software risk, keeps the drive read-write, and gives you the **serial
console**, which no amount of OTA can: `/status` reports the trigger pin, but boot failures and
exception tracebacks only ever appear on serial.

---

## Driving the trigger from ROS 2

### How the short reaches the board

This arm's **Media flange Inside electric** is a pass-through: whatever is connected at the robot
base appears at the tool connector. The trigger uses the shielded pair **CTR1**:

```
   X76 contact 1 ── CTR1_1 ══ flange ══ tool connector pin 9  (BK) ──► Metro D2
   X76 contact 2 ── CTR1_2 ══ flange ══ tool connector pin 10 (BU) ──► Metro GND
```

Shorting contacts 1 and 2 at the base pulls D2 to ground through the whole harness. The firmware's
`active_low` default and D2's internal pull-up mean **a bare contact is a valid trigger** — no power
and no optocoupler. Full pinout in
[`end_effector_design/README.md`](../end_effector_design/README.md#tool-connector-pinout).

### What closes the switch

**Not the cabinet.** The flange has no cabinet-driven I/O and the Sunrise project has no generated
I/O groups, so Sunrise has no output that could drive X76.

**Driven by the ROS computer, over an RS‑422 serial link.** The pressure sensor needs a data channel
off the tool anyway, and one differential link carries both; a dry-contact relay on X76 1/2 would be
two orders of magnitude slower (5–15 ms mechanical against ~80 µs of wire time). StarTech ICUSB422IS
on the ROS box, MikroE MIKROE‑2821 at the tool; wiring in
[`end_effector_design/README.md`](../end_effector_design/README.md) step 4.

### The serial link

Building the link itself — cable, X76 plug, StarTech, MikroE, WSL setup, tests and
troubleshooting — is in [`in_electric_flange_data_path/`](../in_electric_flange_data_path/README.md).

`busio.UART` on **TX `D0` / RX `D8`**, 115200 baud, 8N1. One ASCII command per line, ending in `\n`
(a trailing `\r` is ignored, and commands are not case-sensitive). Every command gets one reply line:

| Command | Effect | Reply |
|---|---|---|
| `CUE` | run the configured cue, exactly as `/cue` does | `ACK CUE <n>` |
| `CUE r g b w` | run it in this colour (each 0–255); the saved settings are not changed | `ACK CUE <n>` |
| `OFF` | stop any cue or LED test, ring dark | `ACK OFF` |
| `SEG f colors` | static arcs, exactly as `/segments?factor=f&colors=colors` (the pre-training ring states) | `ACK SEG` |
| `PING` | link check | `PONG` |
| `GET /route?query` | any [HTTP route](#http-api) except `/fs`, running the very same function: `GET /status`, `GET /config?r=0&g=255&b=0&w=0`, `GET /led_test?from=0&to=59`. The query is URL-encoded (`,` may be sent as `%2C`) | `OK <the route's text, lines joined by tabs>` |
| anything else | nothing | `ERR <reason>` |

`GET` is what makes the two paths interchangeable for testing: the dashboard's Diagnostics page
sends every one of its controls either as an HTTP request or as a `GET` line, and gets the same text
back. `/fs` stays Wi-Fi only, because sending a whole file down the link would stall the main loop.

`<n>` counts serial cues since boot, and `/status` reports it as `serial_fired`, next to
`serial=up|unavailable`. `CUE` always fires, restarting a cue already running.

**The ack marks the light.** `ACK CUE` is written only after `cue_start()` has called
`pixels.show()`, so its arrival on the ROS computer is a per-trial cue-delivery timestamp: the ring
lit about 1 ms (wire time at 115200 baud) before the ack arrived.

**A failed UART is not fatal.** If the port cannot be opened, the board boots without it, prints
why on the serial console, and reports `serial=unavailable`; the access point keeps working.

**Adding a function** (haptics, the pressure sensor) is one more entry in `SERIAL_COMMANDS`: a
handler that takes the command's arguments and returns its reply line.

Check the link from the ROS computer (after `usbipd attach` on Windows):

```bash
python3 -c "import serial,time;s=serial.Serial('/dev/ttyUSB0',115200,timeout=0.5);s.write(b'PING\n');time.sleep(0.1);print(s.read(64))"
```

`b'PONG\n'` means the whole path works: PC → StarTech → X76 → flange → MikroE → Metro, and back.

> **Optocoupler or not?** Not on the inbound trigger: the only voltage in that loop is the Metro's
> own 3.3 V. The *outbound* direction is different — anything the Metro drives toward a 24 V cabinet
> input needs one, as does anything that puts a voltage onto X76 1/2. 24 V on `D2` destroys the
> SAMD51.

For a production build, add an external **4.7 kΩ pull-up** from D2 to 3.3 V at the board. The
internal pull-up is weak, and the line runs several metres past seven joint drives.

### Commissioning it

Short X76 1–2 by hand and watch what the firmware saw:

```bash
python3 sinthlab_bringup/diagnostics/check_cue_wiring.py      # from a laptop on KUKA_NEOPIXEL
```

It pre-flights the board — catching `enabled=0`, a cue too dim or short to see, and **inverted
polarity** — then announces every edge. The full sequence, including the continuity checks that must
come first, is the
[commissioning checklist](../end_effector_design/README.md#commissioning-the-trigger--do-it-in-this-order).

### The ROS side

[`sinthlab_bringup/actions/visual_cue.py`](../sinthlab_bringup/sinthlab_bringup/actions/visual_cue.py)
mirrors `AudioCue` and is called beside it at every cue site — all four experiments, eight sites —
and [`ring_state_cue.py`](../sinthlab_bringup/sinthlab_bringup/actions/ring_state_cue.py) holds the
pre-training quarter states. Both hand each request to the sender `visual_cue.trigger` selects:

```yaml
visual_cue:
  enabled: true                 # master switch
  trigger: "serial"             # serial or wifi
  serial_port: "/dev/ttyUSB0"   # serial only
  remote_board: "192.168.4.1"   # wifi only
  colours:                      # optional: [r, g, b, w] per cue site
    play: [0, 255, 0, 0]        # green
    snap: [255, 0, 0, 0]        # red
```

The cue sites are `play` and `snap` in apple pluck and perturb, and `play`, `reward`, `goal` and
`timeout` in the maze. The trigger is a live parameter, read at every cue.

- **[`visual_cue_serial.py`](../sinthlab_bringup/sinthlab_bringup/actions/visual_cue_serial.py)**
  writes the [serial commands](#the-serial-link) and waits for each reply. It opens the port on first
  use and reopens it after an error, so re-attaching the adapter recovers without a restart, and it
  warns at startup if the FTDI `latency_timer` is not 1 ms.
- **[`visual_cue_remote.py`](../sinthlab_bringup/sinthlab_bringup/actions/visual_cue_remote.py)**
  sends the HTTP requests. A cue with a colour first sets it with `/config?r=0&g=255&b=0&w=0` (in RAM;
  no `save=1`), then sends `/cue`; the board keeps that colour until the next one. Join the ROS
  computer to `KUKA_NEOPIXEL` first — its Ethernet link to the robot is separate.

Both behave the same way:

- **They never stall a trial.** Commands go out one at a time from a background thread, and
  `start()` returns at once. Nothing raises into the orchestrator.
- **Everything is logged.** At startup, whether the board answered; then each cue with its round
  trip, or why it failed.
- **A late cue is dropped, not shown.** A cue still waiting behind a slow command after 0.5 s is
  skipped and logged. Ring states are never dropped: a late state is still the right one.
- **The ack is recorded.** The board answers only once the ring is lit, and that answer is recorded
  as `cue_visual_ack` with its round trip.

### On timestamping the cue against motion

The ack puts the light on the ROS clock. Over the serial link it arrives about 1 ms after the
ring lit (wire time at 115200 baud) plus the adapter's USB latency (1 ms with `latency_timer` at 1).
Over Wi-Fi the round trip varies from cue to cue, so it bounds when the ring lit without pinning
it down. The LED refresh itself (~2.4 ms for 60 RGBW pixels) happens before the ack on both paths.

---

## Known limitations

1. **The serial path is untested in an experiment.** The link passed an echo test to the board, and
   the ROS sender passed tests against a simulated board; no experiment has run on it yet.
2. **Only colour comes from ROS.** Each cue can carry a colour (`CUE r g b w`, or `/config` before
   `/cue` over Wi-Fi, in RAM, never saved). Pattern, duration and brightness are always the board's
   settings, set from any machine joined to its access point.
3. **The X76 plug's contacts are held by tape, not by its insulating insert,** which would not seat.
   Recheck the link (Diagnostics → Visual cue, path **Serial**, **Check board**) after the plug is unplugged and refitted.
4. **The unused `D2` input stays in the firmware.** Its pull-up keeps it idle, so it cannot fire a cue.
5. **Settings persistence depends on `microcontroller.nvm` being present in the build.** If it is
   absent, `/config?save=1` says so and settings revert to `DEFAULTS` on reset.
6. **No code upload over Wi-Fi.** Deliberate — see
   [Changing the firmware without USB](#changing-the-firmware-without-usb).
7. **No authentication or TLS on the HTTP path.** Acceptable only because the AP is isolated and
   WPA2-protected. Anyone on that AP can change the cue and read `code.py` (but not
   `settings.toml`). Do not put this board on a shared network as-is.
8. **`debug=True` on the HTTP server** prints every request to the serial console. Harmless, but
   noisy when you are watching for cue diagnostics.

---

## Provenance

`code.py` and `settings.toml` are original work for this project. Everything under `lib/` is from the
[Adafruit CircuitPython Bundle](https://github.com/adafruit/Adafruit_CircuitPython_Bundle) (MIT).
