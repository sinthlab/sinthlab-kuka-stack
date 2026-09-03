# End-Effector Board Firmware — NeoPixel Cue Ring (Metro M4 AirLift)

> # ⛔ DO NOT COPY THIS FILE TO THE BOARD
> **`README.md` is repository documentation only — it must never be written to the `CIRCUITPY`
> drive.** When flashing or refreshing a board, copy **only** `code.py`, `settings.toml` and
> `lib/` — that is the complete payload. See [Deployment](#deployment).

CircuitPython firmware for the **Adafruit Metro M4 Express AirLift Lite** that lives in the base hub
of the [apple-pluck end effector](../end_effector_design/README.md). It drives the **60-LED RGBW
NeoPixel ring** (Adafruit 2874, Ø157) seated in the cover as a **visual cue** for the monkey.

---

## The two control paths

The firmware is built around one decision: **the experiment cue is hardwired, and Wi-Fi is an
optional side door.**

| | **1. Hardwired cue** *(primary)* | **2. Wi-Fi + HTTP** *(secondary)* |
|---|---|---|
| Carries | **Timing** — one bit, "cue now" | **Content + config** — colour, duration, rate, manual control |
| Route | Sunrise app → 24 V media-flange output → optocoupler → `D2` | Laptop joins the board's own AP → `http://192.168.4.1/` |
| Used by | The experiment, every trial | Setup, debugging, ad-hoc control |
| Latency | ≈ the debounce time (5 ms), deterministic | Whatever the network does |
| If it fails | No cue | Experiment unaffected |

The wire carries **no colour and no duration** — a digital line is one bit. What the flash *looks
like* is configuration — live over HTTP, and persistable. Keep that split in
mind: **the cabinet decides *when*, the board decides *what*.**

Because the wired path is the one the experiment depends on, the firmware brings it up **first** and
**wraps the entire Wi-Fi stack in a `try`/`except`**. A dead ESP32, a malformed `settings.toml`, or a
WPA2-rejected password prints a message and drops the board to hardwired-only operation — it cannot
stop the ring cueing.

> ### Status — the board half is done, the cabinet half is not
> The firmware responds to the trigger line today. What does **not** exist yet is the link that makes
> the cabinet assert that line on command from a ROS orchestrator — see
> [Driving the trigger from ROS 2](#driving-the-trigger-from-ros-2). Experiment cues in the ROS stack
> are still audio-only (`AudioCue`).

---

## Contents
- [Hardware context](#hardware-context)
- [The cue trigger — wiring](#the-cue-trigger--wiring)
- [Architecture](#architecture)
- [Configuration, not code edits](#configuration-not-code-edits)
- [Files on the board](#files-on-the-board)
- [HTTP API](#http-api)
- [Deployment](#deployment)
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
| **Cue input** | `board.D2` ← **optocoupler** output, isolated from the 24 V media-flange line |
| **Power** | 24 V media flange → Tobsun 24 V→5 V → board **and** ring (5 V injected at all 4 quarter joints) |

```
                        ┌─────────── MEDIA FLANGE (electric) ───────────┐
                        │   24 V power pair        24 V digital output  │
                        └────────┬──────────────────────────┬───────────┘
                                 │                          │
                  ┌──────────────▼───────────────┐   ┌──────▼───────┐
                  │ Tobsun 24 V → 5 V converter  │   │ Optocoupler  │  galvanic
                  └───┬──────────────────────┬───┘   └──────┬───────┘  isolation
                  5 V │                  5 V │              │ logic level
                      │                      │              │
             ┌────────▼─────────┐            │              │
             │ Metro M4 AirLift │◄───────────┼──────────────┘  into D2
             │  ├ SAMD51 (code) │            │
             │  └ ESP32 (Wi-Fi) │            │  (5 V injected at each of
             └────────┬─────────┘            │   the ring's 4 quarter joints)
                 D5   │ 3.3 V data           │
             ┌────────▼─────────┐            │
             │  Pixel Shifter   │            │
             └────────┬─────────┘            │
                 5 V data                    │
             ┌────────▼─────────────────────▼──┐
             │ NeoPixel ring — 60 × RGBW, GRBW │
             └─────────────────────────────────┘
```

> **Current.** The ring can pull **~3.5 A at 5 V** at full white; brightness scales that roughly
> linearly. The default is `0.2` (**≈0.7 A**), and the firmware enforces a hard ceiling of
> `MAX_BRIGHTNESS = 0.35` (**≈1.2 A**) that **no HTTP request can exceed** — raising it is a
> deliberate code edit, to be made only after re-checking the converter rating and the 5 V wiring.
> Never try to feed the ring from the Metro's own 5 V pin.

---

## The cue trigger — wiring

The cabinet asserts a **24 V digital output on the media flange**; the optocoupler isolates it and
presents a logic level to `D2`. Isolation is the point: the 24 V industrial side and the 3.3 V
microcontroller side share no ground, so cabinet switching noise cannot reach the SAMD51.

```
  cabinet / Sunrise app                    │ galvanic isolation │        Metro M4
                                           │                    │
  media-flange DO ──24 V──►┤LED    photo-transistor├──────────► D2   (Pull.UP, active LOW)
  media-flange 0 V ────────┤                       ├────────────GND
```

**Polarity — `active_low` (default `1`) expects:**

| Opto side | Connect to |
|---|---|
| Input LED anode (via its series resistor) | media-flange 24 V digital **output** |
| Input LED cathode | media-flange **0 V** |
| Output transistor collector | Metro **`D2`** |
| Output transistor emitter | Metro **`GND`** |

The Metro's **internal pull-up** holds `D2` high when the opto is dark, so **idle = HIGH** and
**cue asserted = LOW**. This is the safe polarity: an unplugged connector, an unpowered cabinet, or a
broken wire all read as *no cue* rather than a stuck-on cue. If your optocoupler board inverts (many
industrial modules have their own output driver), **flip it over Wi-Fi without opening the
effector**:

```bash
curl "http://192.168.4.1/config?active_low=0"     # switches the pin to an internal pull-down
```

The firmware resynchronises the debouncer as part of the change, so flipping polarity does not
itself look like an edge and fire a spurious cue.

### Bench-testing without the robot

You do not need the cabinet to verify the firmware. With the board on USB:

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
    if server is not None:
        server.poll()       # handle at most one HTTP request -> non-blocking
```

**Nothing in this loop sleeps and no route handler sleeps.** That is the property that makes the
whole design work: a cue animates while HTTP requests are being served, and HTTP traffic cannot
delay a cue beyond one short loop pass. Trigger latency is therefore ≈ the `debounce_ms` setting.

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

`mode` selects who owns the cue's length — **settable over HTTP, no code edit**:

| Mode | Behaviour | Use when |
|---|---|---|
| **`pulse`** *(default)* | An asserted edge starts a cue of `duration`. The line can drop immediately — the board runs the full length. | The cabinet can only emit a short strobe, or you want one fixed cue length everywhere. |
| `follow` | The ring runs for **exactly as long as the line is held**. `duration` is ignored. | You want the orchestrator to own cue length directly, the way it owns `AudioCue` length. |

`retrigger=0` (the default) makes a second edge arriving mid-cue a no-op, so contact bounce or a
double-pulse cannot restart the cue. Set `retrigger=1` if re-triggering is what you want.

---

## Configuration, not code edits

The board is bolted inside a closed effector, so reaching its USB port is expensive. The firmware
is therefore built so that **everything an experiment might reasonably want to change is a runtime
setting**, reachable over HTTP and persistable across reboots. Editing `code.py` should be reserved
for genuinely new behaviour — not for tuning.

Only two classes of thing stay hard-coded:

| Stays in `code.py` | Why |
|---|---|
| `LED_PIN`, `CUE_PIN`, `PHYSICAL_LEDS`, `ORDER` | physical facts — changing them means re-wiring anyway |
| `MAX_BRIGHTNESS` | a **safety limit**: it caps the ring's current draw, so no HTTP request can ask the converter and the 5 V wiring for more than they are sized to deliver |

Everything else is live:

| Setting | Values | What it does |
|---|---|---|
| `r` `g` `b` `w` | 0–255 | cue colour |
| `duration` | 0.05–300 s | cue length (ignored in `follow` mode) |
| `period` | 0.02–10 s | one full cycle of the pattern |
| `pattern` | `flash` `solid` `breathe` `chase` | the cue **modality** |
| `brightness` | 0 – `MAX_BRIGHTNESS` (0.35) | ring brightness; the cap is enforced, not advisory |
| `mode` | `pulse` `follow` | who owns cue length — board or cabinet |
| `active_low` | 0 / 1 | trigger polarity — **flip this to match your optocoupler without opening the box** |
| `debounce_ms` | 1–200 | how long the input must hold a level |
| `retrigger` | 0 / 1 | may a new edge restart an in-flight cue |
| `enabled` | 0 / 1 | arm/disarm the wire entirely |

### Patterns

`pattern` is what makes a new cue *modality* a configuration change rather than a code change:

| Pattern | Behaviour | Update rate |
|---|---|---|
| `flash` | square on/off at `period` | redrawn only on the half-period boundary |
| `solid` | steady for `duration` | drawn once |
| `breathe` | smooth raised-cosine fade in/out | ~30 fps |
| `chase` | a lit arc of 8 LEDs rotating once per `period` | ~30 fps |

### Persistence — NVM, not a file

`/config?save=1` writes the settings to **`microcontroller.nvm`**, a small non-volatile byte area,
and they are restored at the next boot.

> **Why NVM and not a config file?** Writing any file from firmware requires
> `storage.remount()` in `boot.py`, which makes **CIRCUITPY read-only to your computer** and puts a
> boot-time failure between you and a working board. NVM has neither cost. See
> [Changing the firmware without USB](#changing-the-firmware-without-usb).

The record is magic-tagged, versioned and checksummed; a corrupt or absent record silently falls
back to the `DEFAULTS` in `code.py` rather than failing to boot. **NVM is not present in every
CircuitPython build** — if it is missing, everything still works, settings just revert to
`DEFAULTS` on reset, and `/config?save=1` tells you so instead of pretending. Check with:

```python
import microcontroller; print(microcontroller.nvm)      # None = not in this build
```

Saving writes to flash, so it is **only** done on an explicit `save=1` — never automatically on
every config change — to avoid wearing the cell out.

`/config?reset=1` restores the code defaults and invalidates the saved record.

---

## Files on the board

Only `code.py` and `settings.toml` run on the board as our code; `lib/` is the Adafruit library
bundle, and `README.md` never leaves the repo.

| Path | Origin | Deploy to board? | Notes |
|---|---|---|---|
| **`code.py`** | **ours** | **yes** | The entire application. Auto-run and auto-reloaded by CircuitPython. |
| **`settings.toml`** | **ours** | **yes** | AP SSID + password. See [below](#settingstoml--wi-fi-credentials). |
| **`lib/`** | Adafruit bundle | **yes** | `.mpy` libraries, below. |
| `README.md` | **ours** | **no** | This document. Repository documentation only. |

The board's own drive also carries `boot_out.txt` (written *by* CircuitPython at boot — firmware
version, board ID, UID) and the zero-byte macOS indexing suppressors `.fseventsd/no_log`,
`.metadata_never_index` and `.Trashes`. **Leave all of those on the board**; none are mirrored here,
because none are ever deployed *from* the repo. The Adafruit factory-demo leftovers that shipped on
the drive (`simpleio`, `adafruit_dotstar`, `adafruit_hid/`, `adafruit_waveform/`, and the stock
`README.txt`) have been **removed** — `code.py` imports none of them.

### `lib/` — what is actually required

| Library | Why |
|---|---|
| `neopixel.mpy` | Drives the ring. (`adafruit_pixelbuf` is built into the CircuitPython core on this board.) |
| `adafruit_esp32spi/` | ESP32 co-processor driver + the `socketpool` shim the HTTP server sits on. |
| `adafruit_httpserver/` | The routing / request / response layer. |
| `adafruit_bus_device/` | SPI transaction helper — a dependency of `adafruit_esp32spi`. |

`lib/` has been trimmed to exactly these four. Leave everything *inside* `adafruit_esp32spi/` as it
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
| **`/config`** | any setting above, plus `save=1` / `reset=1` | **Read or change every runtime setting.** No parameters = read-only. |
| `/cue` | `r` `g` `b` `w`, `duration`, `period`, `pattern` | Fire a cue **now** — the software equivalent of the trigger wire. Omitted parameters fall back to the configured cue. |
| `/status` | — | Everything `/config` reports, plus the **live trigger-pin state**, the active pattern and uptime. First stop when debugging. |
| `/fs` | — | List what is actually on the board, with sizes. |
| `/fs/get` | `path` | Read a text file back (`code.py`, `*.txt`, `*.json`). **Read-only** — see [below](#changing-the-firmware-without-usb). |
| `/set_color` | `r` `g` `b` `w` | Solid colour on the whole ring. Cancels a running cue. |
| `/segments` | `factor`, `colors` | Split the ring into `factor` equal arcs, colour each. Cancels a running cue. |
| `/blink` | `r` `g` `b` `w`, `delay` (s), `count` | Flash without touching the configuration. |
| `/off` | — | Clear the ring and cancel any running cue. |

### Examples

```bash
# what is the board doing right now?  (config + trigger pin + uptime)
curl http://192.168.4.1/status

# fire the configured cue — exactly what the trigger wire does
curl http://192.168.4.1/cue

# one-off, without changing the configuration: 3 s of amber chase
curl "http://192.168.4.1/cue?r=255&g=140&b=0&w=0&duration=3&period=0.6&pattern=chase"
```

**Re-configure what the *wire* does — this is the part that replaces editing `code.py`:**

```bash
# 1.5 s of green, breathing
curl "http://192.168.4.1/config?r=0&g=255&b=0&w=0&duration=1.5&period=1.0&pattern=breathe"

# the optocoupler turned out to be active-high — fix it without opening the effector
curl "http://192.168.4.1/config?active_low=0"

# noisy line: lengthen the debounce
curl "http://192.168.4.1/config?debounce_ms=25"

# let the cabinet own cue length instead of the board
curl "http://192.168.4.1/config?mode=follow"

# disarm the wire while you work on the wiring, then re-arm
curl "http://192.168.4.1/config?enabled=0"
curl "http://192.168.4.1/config?enabled=1"

# keep it across power cycles / start over
curl "http://192.168.4.1/config?save=1"
curl "http://192.168.4.1/config?reset=1"
```

**Read the board back:**

```bash
curl http://192.168.4.1/fs                          # what is on the drive
curl "http://192.168.4.1/fs/get?path=code.py"       # confirm what is actually running
```

**Manual control:**

```bash
curl "http://192.168.4.1/set_color?w=255"
curl "http://192.168.4.1/segments?factor=4&colors=255,0,0,0.0,255,0,0.0,0,255,0.0,0,0,255"
curl http://192.168.4.1/off
```

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

**Never copy `README.md` to the drive**, and never copy the board's `boot_out.txt` back over itself —
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

The board now does its half. The remaining work is upstream, and it is worth being precise about
where the gap actually is.

```
  ┌─ EXISTS ───────────────────────────────────────────────────────────┐
  │  Metro:  D2 asserted  →  ring flashes for 2 s                      │
  └────────────────────────────────────────────────────────────────────┘
                                   ▲
                        24 V media-flange digital output
                                   ▲
  ┌─ EXISTS (hardware) ────────────────────────────────────────────────┐
  │  Cabinet can drive media-flange I/O from a Sunrise application     │
  │  (MediaFlangeIOGroup.setOutputX(...))                              │
  └────────────────────────────────────────────────────────────────────┘
                                   ▲
  ┌─ ✗ MISSING ────────────────────────────────────────────────────────┐
  │  a channel by which a ROS orchestrator tells the Sunrise app       │
  │  "cue now"                                                          │
  └────────────────────────────────────────────────────────────────────┘
```

**The missing piece is not the wire — it is ROS → cabinet.** Our ROS↔cabinet link is FRI, and FRI
carries joint commands, not arbitrary application calls. Three ways to close it:

| Option | What it needs | Assessment |
|---|---|---|
| **A. Side channel into the Sunrise app** — a small TCP/UDP listener thread in `LbrImpedanceControlServer.java` that accepts a "cue" message from the ROS box and calls `setOutputX(true)`, then clears it | Java changes in [`sunrise_controller_code/`](../sunrise_controller_code/) + a ROS node that opens a socket | **Recommended.** Lives entirely in code we already own, no upstream edits, no FRI changes. Sub-millisecond on a wired LAN — far below the cue timing that matters. |
| **B. FRI boolean I/O** — declare a boolean I/O in the FRI configuration and set it from the ROS side each cycle | FRI I/O declarations in the Sunrise project **and** command-interface support in `lbr_ros2_control` (upstream, which we do not edit) | Cleanest in principle — the cue rides the existing 100 Hz channel and is timestamped with the motion. But it needs upstream work we have ruled out before. |
| **C. Cabinet-side timing** — the Sunrise app decides when to cue | nothing new | Rejected: the orchestrator owns trial timing, and splitting that across two codebases is how cue/trial desynchronisation bugs happen. |

### ROS-side shape

Whichever transport wins, the node shape is the same and should mirror the existing audio cue:

- a **`VisualCue` action** in `sinthlab_bringup/actions/`, alongside `AudioCue`, so an orchestrator
  fires a light at a maze checkpoint exactly the way it fires a tone today;
- **non-blocking** — an experiment orchestrator must never stall on a cue transport. Fire and
  forget; log a failure, never wait on one.

### Wi-Fi is not on this path

The board's AP exists for setup and debugging. Do **not** put the experiment cue on it: a single
Wi-Fi interface cannot be joined to `KUKA_NEOPIXEL` and the KUKA network at once, and an
experiment cue should not depend on a radio link when a wire is already running to the flange.

### If you later need more than one cue type

One line is one bit. To distinguish, say, "go" from "reward":

- **a second trigger line** into another free pin — 2 lines give 4 states, and the firmware change
  is a second `poll_trigger()` instance;
- **pulse-width encoding** on the existing line — the Sunrise app holds it 50 ms vs 200 ms, and the
  firmware measures the assert duration before choosing a pattern. One wire, but it costs the
  distinction between "cue" and "line stuck".

The first is simpler and unambiguous; prefer it if a pin and a conductor are available.

---

## Known limitations

1. **No ROS 2 integration yet.** The cabinet→board wire works; ROS→cabinet does not exist. See
   [above](#driving-the-trigger-from-ros-2).
2. **The wire carries one bit.** No colour, no duration, no cue type — only "now". See
   [above](#if-you-later-need-more-than-one-cue-type) for how to extend that.
3. **`CUE_PIN` and the optocoupler polarity are unverified against the real flange.** `D2` and the
   opto pinout in [The cue trigger](#the-cue-trigger--wiring) are assumptions. Confirm them against
   your optocoupler board and your media-flange variant's datasheet, and bench-test with a jumper
   first. Polarity itself is recoverable over Wi-Fi (`/config?active_low=0`); the pin choice is not.
4. **Settings persistence depends on `microcontroller.nvm` being present in the build.** If it is
   absent, `/config?save=1` says so and settings revert to `DEFAULTS` on reset.
5. **No code upload over Wi-Fi.** Deliberate — see
   [Changing the firmware without USB](#changing-the-firmware-without-usb).
6. **No authentication or TLS on the HTTP path.** Acceptable only because the AP is isolated and
   WPA2-protected. Anyone on that AP can change the cue and read `code.py` (but not
   `settings.toml`). Do not put this board on a shared network as-is.
7. **`debug=True` on the HTTP server** prints every request to the serial console. Harmless, but
   noisy when you are watching for cue diagnostics.

---

## Provenance

`code.py` and `settings.toml` are original work for this project. Everything under `lib/` is from the
[Adafruit CircuitPython Bundle](https://github.com/adafruit/Adafruit_CircuitPython_Bundle) (MIT).
