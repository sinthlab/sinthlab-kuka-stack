# Experiment Control Dashboard: architecture and software

**How to use the dashboard is in the main README:
[Quick start: the experiment dashboard](../README.md#quick-start-the-experiment-dashboard).** This file
covers how it is built: its components, the robot-side interfaces it relies on, how parameters are
classified, where the parameter help comes from, and how to extend it.

---

## Contents
- [Architecture](#architecture)
- [Components](#components)
- [Robot side: ExperimentControl](#robot-side-experimentcontrol)
- [Parameter model](#parameter-model)
- [Parameter documentation](#parameter-documentation)
- [Diagnostics page](#diagnostics-page)
- [HTTP API](#http-api)
- [Process control](#process-control)
- [Demo mode](#demo-mode)
- [Security](#security)
- [Extending](#extending)
- [Developing and testing](#developing-and-testing)

---

## Architecture

```mermaid
flowchart LR
    B["Browser<br/>static/index.html · app.js"]
    subgraph S["server.py (this folder)"]
        API["HTTP API + SSE<br/>log · state"]
        RUN["runner.py<br/>one ros2 launch"]
        BR["ros_bridge.py<br/>rclpy node"]
        PAR["params.py · experiments.py<br/>YAML → typed, tiered params"]
    end
    subgraph R["ROS 2 (namespace /lbr)"]
        L["ros2 launch sinthlab_bringup<br/>iiwa7_*.launch.py<br/>[params_file:=runs/…yaml]<br/>[clik_nullspace_cfg:=runs/…yaml]"]
        O["orchestrator<br/>+ ExperimentControl"]
        HW["lbr_state_broadcaster"]
    end
    Y[("sinthlab_bringup/config/*.yaml<br/>+ live_params.py")]

    B <-- "JSON · Server-Sent Events" --> API
    API --> RUN
    API --> BR
    API --> PAR
    PAR -. "reads" .-> Y
    RUN -- "subprocess (own process group)<br/>stdout → log" --> L
    L --> O
    O -- "experiment_status (JSON, latched)" --> BR
    BR -- "set_parameters_atomically (live only)" --> O
    BR -- "pause (SetBool)" --> O
    HW -- "lbr_state (FRI session, rate)" --> BR
```

- **The dashboard never talks to the arm.** It starts and stops the same `ros2 launch` a user would
  type, and uses the orchestrator's public ROS interfaces. The same interfaces serve the terminal
  (`ros2 param set`, `ros2 service call`), which is also why an experiment launched in a terminal
  is picked up automatically.
- **The YAMLs are the source of truth** for values, defaults, descriptions and notes. The dashboard
  never writes them: a per-run edit becomes a copy in `runs/`, passed as `params_file:=`.

---

## Components

| File | Role |
|---|---|
| `server.py` | `ThreadingHTTPServer`: static files, the JSON API, and a Server-Sent Events stream of log lines and state (state is re-sent every second). Owns start / stop / restart / stop-after-trial |
| `runner.py` | runs one `ros2 launch` in its own process group, pumps its output into the log ring buffer (5000 lines) and `logs/`, and stops it: SIGINT → 20 s → SIGTERM → 5 s → SIGKILL |
| `ros_bridge.py` | `RosBridge`: an rclpy node on a background executor that subscribes to `experiment_status` and `lbr_state` and calls `set_parameters_atomically` / `pause`. `UnavailableBridge` when ROS is not sourced. `DemoBridge` for `--demo` |
| `experiments.py` | the experiments (launch file, YAML, orchestrator node, SmartPad profile, controllers, run name, `group`): three experiments and three pre-training tasks, shown under the **Experiments** and **Pre-training** tabs (`GROUPS`); an experiment's `paradigms` (apple pluck: pluck / perturb / perturb bump / random start) appear as nested tabs on its card and set its `paradigm` parameter for the next Start, and parameters of some paradigms only (`live_params.PARADIGM_ONLY`) are shown only under those; cautions and linked parameters |
| `params.py` | YAML → flat dotted names; descriptions and notes; type coercion; per-run edits; the edited YAML and CLIK posture files; the Fixed tab's sections |
| `static/` | the page: `index.html`, `style.css`, `app.js`. No external libraries or fonts |
| `demo_launch.py` | stands in for `ros2 launch` in demo mode |
| `check_param_docs.py` | fails if any key in `sinthlab_bringup/config/*.yaml` has no one-line description |
| `run_gui.sh` | sources ROS 2 and the workspace, then runs `server.py` |

Dependencies: the Python standard library and PyYAML (installed with ROS); `rclpy` and the message
packages when ROS is sourced.

---

## Robot side: ExperimentControl

[`helpers/experiment_control.py`](../sinthlab_bringup/sinthlab_bringup/helpers/experiment_control.py)
makes an orchestrator controllable from outside. Every orchestrator creates one:

| Interface | Type | Behaviour |
|---|---|---|
| `<ns>/experiment_status` | `std_msgs/String`, JSON, transient-local | published on every trial event and once a second: experiment, trial, phase, paused / held, live values **in effect**, and changes **pending** for the next trial |
| `<ns>/<orchestrator>/pause` | `std_srvs/SetBool` | `true`: finish this trial, recover, hold at the start. `false`: apply pending changes and start the next trial. A trial with no natural end (Free Move) registers `on_pause_request()` and ends its session at once on `true` |
| `set_parameters` | parameter callback | names in `live_params.py` are range-checked, accepted and **applied at the next trial boundary**; anything else is rejected with the reason |
| `on_event(token, arg)` | `TrialRecorder` hook | every `mark()` updates the status, and with `nsp_sync.enabled` sends the event's NSP code (`_pulse()`, a stub until the DIO arrives) |

The orchestrators end each trial with `control.begin_trial(self.start_trial)`. That call is where a
pause holds and where accepted changes are applied: `_reload_live()` calls `reload()` on the cue,
monitor and perturbation actions and resets the quiet-window and timeout durations. So a change
never lands mid-trial, and the sidecar written at the next `TrialRecorder.start()` records the
values that trial used.

---

## Parameter model

| Tier | Decided by | Enforced by |
|---|---|---|
| **Live** | `LIVE_PARAMS` in [`live_params.py`](../sinthlab_bringup/sinthlab_bringup/helpers/live_params.py): fnmatch globs per experiment | the orchestrator's parameter gate, and the server |
| **Per-run** | every other key in the experiment YAML | the server refuses edits while that experiment runs; the orchestrator rejects runtime sets |
| **Fixed** | not in the experiment YAML: SmartPad selections, launch arguments, `iiwa7_hardware_controllers.yaml`, the CLIK posture | shown read-only (`read_fixed_config()`) |

`live_params.py` has no ROS import, so the dashboard loads it from the source tree whether or not
ROS is sourced; the orchestrators import the same module, so the two cannot disagree. It also holds
the checks, which `check_value()` applies on both sides:
- `LIMITS` (ranges), `CHOICES` (enums) and `STEPS` (a step makes the dashboard draw a **slider**
  next to the text box — e.g. the resistance, 10 N/m; the text box still takes any value in range).
  `limits_for(name, context)` narrows a range that depends on another value — the resistance slider
  runs from 50 N/m up to the SmartPad base entered, so the dashboard passes the current values when it
  describes the parameters;
- **poses** (`check_pose`): 7 values, within `IIWA7_LIMITS_DEG`, and not a straight arm
  (max(|A2|, |A4|, |A6|) ≥ `STRAIGHT_BELOW_DEG`, 12°: below that the arm is singular);
- **pairs**, checked against the other values in force (`context`): the perturbation's r against its
  plane's cap (`PERTURB_R_MAX`), and the start pose against the recover pose, which must be equal.

Live motion parameters: every move's `speed_scale` is live in every experiment, and IMMEDIATE
(`live_params.IMMEDIATE`): `ExperimentControl.apply_immediately` hands it straight to the move's
`set_speed_scale`, used from that move's next start, and logs it with `TrialRecorder.note_change`. The
dashboard marks such rows *applies now*. The base limits `move_to_pos_*` are per-run. The start /
recover pose is live in apple pluck only (`_POSE`), re-read at the trial boundary.

**Dashboard-only checks and conveniences** (`experiments.py`, `params.py`):
- `CAUTION`: amber notes on parameters other things depend on. They never lock anything.
- `LINKED`: `move_to_start.target_joint_position` and `move_to_start_recover.target_joint_position`
  are set together — per-run edits change both, and a live change sends both in one
  `set_parameters_atomically` call (all or nothing), so the node never sees them differ.
- **Parallel arrays** (`corridor_*`, `checkpoint_*`) must keep their length.
- **Coercion:** values from the browser are coerced to the YAML default's exact type, because ROS
  refuses a set that changes a parameter's type.

**Launch overrides** (`ExperimentParams.launch_overrides()`):
- With any edit, the YAML is written to `runs/<exp>_<time>.yaml` and passed as `params_file:=`.
- If a CLIK experiment's start pose was edited, a matching `runs/<exp>_<time>_clik_nullspace.yaml` is
  passed as `clik_nullspace_cfg:=`. `iiwa7_hardware.launch.py` joins that argument onto the package
  directory, and an absolute path replaces the package part, so the generated file is used as is.

---

## Parameter documentation

The help text comes from the YAML files (`params.yaml_docs()`):

```yaml
    # Notes: the reasoning, measurements, knobs. Comment lines directly above the key,
    # with no blank line between, belong to that key (or to that block, above a block).
    polar_r_m: 0.05  # perturbation distance from the start [m]      <- the description
```

- **Description:** the comment on the key's own line. It is shown under the parameter and at the top
  of its popup. Every key, blocks included, must have one.
- **Notes:** the comment block above the key. Prose lines are reflowed into paragraphs; indented
  lines (tables, diagrams) keep their columns.
- **Borrowed notes:** a key with no notes of its own borrows a sibling's notes when they mention it
  by name. The perturbation's notes sit above `polar_r_m` but explain θ and plane too.
- **Enforcement:** `check_param_docs.py` walks every YAML in `sinthlab_bringup/config/` and exits 1
  listing any undocumented key.

The Fixed tab's SmartPad and launch-argument rows are not YAML keys; their help is in
`SMARTPAD_HELP` and `LAUNCH_HELP` in `params.py`.

---

## Diagnostics page

`/diagnostics.html` — the **Diagnostics** button at the top right of the dashboard. Hardware checks
that never move the arm, one tab per device. **Visual cue** checks the end-effector NeoPixel ring
through its Metro board, over the board's own Wi-Fi (join this computer to `KUKA_NEOPIXEL`):

- **Cue board:** is the board reachable, and what it reports — enabled, brightness, colour, pattern,
  LEDs, whether a cue is running, how many wire cues it has fired, uptime.
- **Path: Wi-Fi.** Set what a cue looks like (colour, pattern, segments, duration, period,
  brightness, enabled) and apply it; fire a cue; solid colour; off; save on the board; reset to the
  firmware defaults; the **LED-by-LED test**, showing which pixel is lit so a dead stretch of the ring
  can be located. If the ring lights here, the board, the Pixel Shifter and the ring are fine.
- **Path: Wire** (X76 1–2 → tool pin 9 → Metro D2). Nothing on the computer can close the wire yet,
  so it is tested by hand: short X76 1–2 at the robot base while **Start watching** polls the board's
  D2 input (4×/s) and logs every edge and every cue the wire fires. It flags a line that reads
  asserted with nothing shorted (inverted polarity: untick *Active low*, no rewiring) and a board
  that is not enabled. The trigger settings (active low, debounce, retrigger, mode, enabled) are
  set here too. `diagnostics.html#wire` opens on this path.
- **Board replies:** the raw text of the last reply.

The browser cannot read the board's replies itself (another origin), so the server relays each
command (`board.py`). In `--demo` a simulated board answers, so the page can be tried without
hardware. The command-line equivalent for the wire is `sinthlab_bringup/diagnostics/check_cue_wiring.py`.

## HTTP API

POSTs need the header `X-Experiment-Ctrl: 1`; errors return 409 with `{"ok": false, "error": …}`.

| Method | Path | Does |
|---|---|---|
| GET | `/api/experiments` | every experiment, its tab (`group`) and its SmartPad selections |
| GET | `/api/groups` | the experiment-list tabs, in order: `experiments`, `pretraining` |
| GET | `/api/params/<exp>` | `params` (typed rows with tier, value, default, description, notes, caution, links, limits, choices, step), `groups` (block docs), `fixed` (Fixed tab sections) |
| GET | `/api/state` | runner state, ROS availability, status, robot, data folder, edit counts |
| GET | `/api/logs` | the log ring buffer |
| GET | `/api/events` | Server-Sent Events: `{"type": "log" \| "state", "data": …}` |
| POST | `/api/params/<exp>` `{name, value}` | per-run edit; refused while that experiment runs |
| POST | `/api/params/<exp>/reset` | drop all edits |
| POST | `/api/start` `{experiment}` | launch; refused if anything is running, here or in a terminal |
| POST | `/api/stop` `{mode: "now" \| "after_trial"}` | now: the orchestrator ends the trial and parks, then Ctrl-C (again = skip parking); after_trial: pause, then Ctrl-C once it holds at park |
| POST | `/api/restart` | stop now, then start the same experiment with the current edits |
| POST | `/api/pause` `{paused}` | the orchestrator's pause service |
| POST | `/api/live` `{name, value}` | `set_parameters_atomically` on the running orchestrator, with any linked parameter in the same call |
| POST | `/api/validate` | `analysis/validate_recording.py --folder <this run's folder>`, output to the log |
| POST | `/api/diag/board` `{addr, cmd, params}` | Diagnostics page: relays one command to the end-effector cue board (`board.py`): only its own commands (`status`, `config`, `cue`, `set_color`, `segments`, `off`, `led_test`), only to a private address |

---

## Process control

- **Start:** `ros2 launch sinthlab_bringup <launch> [overrides]` runs with `start_new_session=True`,
  so one signal reaches the launch and every node it spawned. `RCUTILS_COLORIZED_OUTPUT=0` and
  `PYTHONUNBUFFERED=1` keep the log clean and live.
- **Stop now:** SIGINT to the process group, exactly like Ctrl-C. After 20 s it sends SIGTERM, and
  after 5 s more SIGKILL.
- **Stop after trial:** pause, then Stop now once the status reports `held`. For Free Move the pause
  itself ends the session, so the arm returns to the start first.
- **Restart:** Stop now; when the launch has exited, start again after 1 s.
- **Shutting down the server** stops a launch it started, the same way.
- **The data folder** is the newest `analysis/expt_<run_name>_*` modified since the launch started.

---

## Demo mode

`--demo` replaces the ROS side:
- `demo_launch.py` stands in for `ros2 launch`. It prints a banner and exits cleanly on SIGINT, so
  Start / Stop / Restart run for real.
- `DemoBridge` simulates an orchestrator on a fast clock. It uses the same trial phases, the same
  pause / held behaviour, the same live-parameter rules (`live_params.py`), the same apply-at-next-trial
  behaviour, and heartbeats once a second.

Nothing connects to the arm.

---

## Security

The server binds to `127.0.0.1` unless `--host` says otherwise, since whoever reaches it can start the
robot. POSTs require a custom header that a page on another site cannot send without CORS, which
the server never grants. So a malicious web page cannot drive the robot through a local browser.
Static files are served from `static/` only, with no path traversal.

---

## Extending

**A new live parameter:**
1. Add its name or glob to `LIVE_PARAMS` in `live_params.py`, with a range in `LIMITS` (and
   `CHOICES` if it is an enum, or a `STEPS` entry for a slider).
2. Make the action that uses it re-readable: give it a `reload()` (see `AudioCue.reload()`,
   `CartesianImpedanceDisplacementMonitor.reload()`) and call that from the orchestrator's
   `_reload_live()`.
3. Declare it in the experiment YAML, with a one-line description on its line. ROS will not set a
   parameter the node never declared, and `check_param_docs.py` fails without the description.

It then appears on the Live tab automatically. Do step 2 before step 1: a listed name that is never
re-read would be accepted and silently ignored, which is the failure the gate exists to prevent.

**A new experiment:** add an `Experiment` to `EXPERIMENTS` in `experiments.py` (its `group` picks the
tab), an entry in
`LIVE_PARAMS`, and create an `ExperimentControl` in its orchestrator. The orchestrator must end each
trial with `control.begin_trial(...)` and pass `on_event=self.control.on_event` to its `TrialRecorder`.
Add its step list to `STEPS` in `static/app.js` for the status card.

---

## Developing and testing

```bash
python3 experiment_ctrl_gui/server.py --demo --port 8765      # the whole page against a simulated robot
python3 experiment_ctrl_gui/check_param_docs.py               # YAML documentation complete?
```

URL options, for testing and for linking:

| Option | Effect |
|---|---|
| `?exp=maze` | select an experiment |
| `?tab=live` \| `run` \| `fixed` | open a tab |
| `?hover=<parameter>` | open that parameter's help popup |
| `?nostream` | poll `/api/state` every second instead of using the event stream. Use it behind a proxy that breaks SSE, and for headless screenshots, since an open stream never lets a headless browser go idle |
