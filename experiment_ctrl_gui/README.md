# Experiment Control Dashboard

A local web page for running the four sinthlab KUKA experiments: pick an experiment, check or change
its parameters, start and stop it, change cues and other live settings between trials, and follow
its status and log.

It does not replace the terminal. Everything it does has a one-line CLI equivalent (see
[Without the dashboard](#without-the-dashboard)). It also works alongside an experiment started with
`ros2 launch` in a terminal.

> ⚠️ **Stop is not an emergency stop.** It sends Ctrl-C to the launch. **The SmartPad E-stop is the
> E-stop.** Keep a hand on it on any first run, in **T1**.

---

## Contents
- [Start the dashboard](#start-the-dashboard)
- [Running an experiment](#running-an-experiment)
- [Live, Per-run and Fixed parameters](#live-per-run-and-fixed-parameters)
- [Live parameters per experiment](#live-parameters-per-experiment)
- [How it works](#how-it-works)
- [Without the dashboard](#without-the-dashboard)
- [Files it writes](#files-it-writes)
- [Parameter help and the YAML's documentation](#parameter-help-and-the-yamls-documentation)
- [Adding a live parameter](#adding-a-live-parameter)
- [Troubleshooting](#troubleshooting)

---

## Start the dashboard

On the ROS computer wired to the arm, with the workspace built
([main README §3](../README.md#3-building-the-stack)):

```bash
~/lbr-stack/src/sinthlab-kuka-stack/experiment_ctrl_gui/run_gui.sh
```

Then open **http://localhost:8080** in a browser. Under WSL2 a Windows browser works, because
Windows forwards `localhost` to WSL.

`run_gui.sh` sources ROS 2 Jazzy and `~/lbr-stack/install/setup.bash`, then starts `server.py`. The
dashboard needs nothing beyond what the workspace already has: Python's standard library and PyYAML,
which ROS installs. The page loads nothing from the internet, so it works on an offline lab
computer.

| Option | Effect |
|---|---|
| `--port 8081` | serve on another port |
| `--robot-name lbr` | the arm's ROS namespace (default `lbr`) |
| `--demo` | **simulated robot**: nothing is launched and nothing connects to the arm. Use it to learn the page or develop it on any computer |
| `--host 0.0.0.0` | serve beyond this computer. **Anyone who can reach the port can start the robot.** Prefer an SSH tunnel: `ssh -L 8080:localhost:8080 <ros-box>` |
| `LBR_WS=/path` | workspace root, if it is not `~/lbr-stack` |

Try it first without the robot:

```bash
python3 experiment_ctrl_gui/server.py --demo      # then open http://localhost:8080
```

---

## Running an experiment

**Page layout.** The left column holds the experiment list, the **Run** controls and a **Before you
start** checklist. The middle column shows **status** (trial number, phase, the trial's steps, a
list of every event) above the **parameters**. The **log** sits on the right on a wide screen and
below on a narrow one. Pills in the top bar show the ROS connection, the robot (FRI session state
and `lbr_state` rate), and the run state.

1. **Pick the experiment**: Apple Pluck, Apple Pluck Perturb, Restricted Plane or Maze.
2. **Prepare the SmartPad.** *Before you start* lists what to select in the `LbrImpedanceControlServer`
   dialogs for this experiment: FRI send period, remote IP, stiffness profile, damping. The app then
   waits about 60 s for ROS, so press Start within that window.
3. **Check the parameters.** Change anything on the **Live** or **Per-run** tabs for this run (see
   the next section). Edited rows get an `edited` badge, and ↺ puts one back to the YAML value.
4. **▶ Start** runs `ros2 launch sinthlab_bringup <experiment>.launch.py`, plus the edited
   parameter file if you changed anything. The page switches to the running experiment. The log
   streams, and the status card follows each trial:
   `start → at start → quiet → go cue → armed → threshold → recover → end`.
5. **Change live settings** at any time on the **Live** tab: cues, NSP sync, the threshold, the
   perturbation, and so on. Each change is accepted by the orchestrator and applies **from the next
   trial**; the trial in progress is never changed. Until then the row shows a purple
   `next trial → …` badge, and the status card lists everything waiting. The log records when each
   change was accepted and when it was applied.
6. **Pause / resume.** *⏸ Pause after trial* lets the current trial finish. The arm recovers to the
   start and **holds there, still under control and compliant**. *▶ Resume* starts the next trial,
   with any live changes made while paused. Pressing the button again before the trial ends
   cancels the pause.
7. **Stop:**
   - **■ Stop after trial**: finish this trial, return to the start, then stop cleanly. Every trial
     on disk is complete. **Use this one.**
   - **■ Stop now**: Ctrl-C at once. The trial in progress is saved with `"partial": true` in its
     sidecar, which the validator reports.
   - **↻ Restart**: Stop now, then Start again with the current edits, for example after changing a
     per-run parameter.
8. **Validate recording** runs `analysis/validate_recording.py --folder <this run's folder>` and prints
   the result in the log.

The **log** shows the launch's own output: every node, colour-coded by level. Filter it by level
(*Warn+*, *Error*, *Dashboard*) or by text, or turn off *Follow* to scroll back. The full log is
also saved to `logs/`; the Run card gives the path.

**An experiment started in a terminal** is picked up automatically, because the dashboard listens
for its status topic. Live changes and pause work from the page. Stop does not: stop it with Ctrl-C
in its own terminal.

---

## Live, Per-run and Fixed parameters

Every parameter in the experiment's YAML is on the **Live** or **Per-run** tab. **Fixed** holds the
configuration that is not in the experiment YAML at all:

| Tab | When it can change | What is in it |
|---|---|---|
| **Live** | Any time, **including while running**. Applies from the next trial. | Cue switches, colours, tones and timing; NSP sync; the pull threshold and dwell; the perturbation (r, θ, plane, delay); the maze timeout. [Full list below.](#live-parameters-per-experiment) |
| **Per-run** | Before **Start**, then **locked for the whole run**. | **Everything else in the experiment YAML**: the start and recover poses (`move_to_start`, `move_to_start_recover`), the maze pre-start waypoint, maze rails, checkpoints and goal, move speeds and tolerances, safety limits, the fixture profile and its sine-rail shape, the displacement axis, frames and topics, `update_rate`, debug logging. |
| **Fixed** | Not from the dashboard. Change it at the SmartPad or in the file named. | What every experiment shares: the SmartPad (FRI) selections, launch arguments, `iiwa7_hardware_controllers.yaml`, and the CLIK redundancy posture. The posture follows an edited start pose automatically; see below. |

**Poses.** `target_joint_position` is 7 joint angles in degrees. Before it is accepted it is checked
against the iiwa7 joint limits: A1/A3/A5 ±170°, A2/A4/A6 ±120°, A7 ±175°. It must also not be a
nearly straight arm: max(|A2|, |A4|, |A6|) must be at least 12°, the same singularity test the maze's
pre-start guard uses. Mechanical zero is exactly singular, and moves from there do not reliably
arrive.
- **Start and recover are linked.** Editing either one sets both, because the next trial starts where
  the last one recovered to. The YAMLs say "change it in BOTH blocks", and the dashboard does it for
  you.
- **Restricted Plane and Maze:** the CLIK's redundancy posture must equal the start pose, or the arm
  holds the right tool pose in the wrong arm shape. When the start pose is edited, the dashboard
  writes a matching `runs/<exp>_<time>_clik_nullspace.yaml` and passes it as `clik_nullspace_cfg:=`.
  The Fixed tab then shows the posture in use.
- **Not checked here:** reachability of anything placed relative to the start. The maze is anchored
  on the start pose, so a new start moves the whole maze; run `check_maze.py` on the generated YAML
  before trusting it.

**Cautions.** Parameters that other things depend on are editable, but carry an amber ⚠ note that
says what to watch: the poses, frame and link names, `update_rate`, rails, checkpoints and goal.
Linked parameters carry a blue 🔗 note naming what else changes. Rails and checkpoints are parallel
arrays, one entry per rail or checkpoint across several parameters, so an edit must keep each
array's length. Adding or removing a rail is a YAML edit.

**The package YAML is never modified.** Per-run edits are written to a copy in `runs/`, which the
launch loads through its `params_file` argument. Each recorded trial's sidecar keeps the complete
parameter set it ran with (`params`), including any live change applied at that trial. So the data
always says what was used, however it was set.

**Why live changes wait for the next trial.** The orchestrator reads its parameters at start-up and
re-reads the live ones only at a trial boundary. That way each trial runs under one configuration,
and its sidecar is true for the whole trial.

**Why per-run parameters lock at Start.** Every other parameter is read once, at start-up. The orchestrator
**rejects** a runtime change to one of them, with a reason, instead of accepting a change that would
silently do nothing. That rule holds for the dashboard and for `ros2 param set` alike.

Values are checked before they are sent: types, ranges (an RGBW colour is 0–255 per channel,
`polar_r_m` ≤ 0.10 m) and choices (`polar_plane` is frontal / horizontal / sagittal). The same
checks run inside the orchestrator.

---

## Parameter help and the YAML's documentation

Every parameter shows a one-line description under its name. **Hover over a name, or tab to it with
the keyboard**, for the full help:

- the full dotted name, the description, the type, the YAML default, the allowed range or choices,
  and whether it is Live or per-run;
- the **notes from the YAML**: the longer comment above the key, with the reasoning, measurements
  and knobs. A key with no notes of its own shows a sibling's notes when they discuss it by name,
  as the perturbation's notes explain r, θ and plane together;
- any caution, and which parameters are linked to it.

Group headings show their block's description, and hovering one shows its notes. Rows on the Fixed
tab (the controller YAML, the CLIK posture, the SmartPad and launch settings) have the same help.
Escape closes a popup. To link someone to one parameter's help:
`http://localhost:8080/?exp=perturb&hover=perturb_start.polar_theta_deg`.

**All of it is read from the YAML files**, so the files and the page cannot disagree. The
convention, which every key in `sinthlab_bringup/config/*.yaml` follows:

```yaml
    # Longer notes: why this value, what was measured, what to change together.
    # These lines sit directly above the key, with no blank line between.
    polar_r_m: 0.05  # perturbation distance from the start [m]      <- the one-line description
```

The description is the comment on the key's own line: what it is, with units in brackets. Blocks
get one too (`perturb_start:  # the perturbation: ...`). Check that nothing lacks one:

```bash
python3 experiment_ctrl_gui/check_param_docs.py     # exit 1 lists every undocumented key
```

---

## Live parameters per experiment

The one list is [`sinthlab_bringup/helpers/live_params.py`](../sinthlab_bringup/sinthlab_bringup/helpers/live_params.py).
The dashboard and the orchestrators both read it, so they cannot disagree.

**Every experiment:**

| Parameter | Meaning |
|---|---|
| `quiet_window_sec` | pause at the start pose before the go cue, in seconds (default 2.0) |
| `audio_cue.enabled` | master switch for every beep. Off, the trial sequence is unchanged, only silent |
| `audio_cue_*.frequency_hz`, `audio_cue_*.duration_ms` | each tone (37–32767 Hz; 10–10000 ms) |
| `visual_cue.enabled` | master switch for the NeoPixel ring |
| `visual_cue.remote_test_trigger` | fire the ring over its Wi-Fi (demos) or over the X76 wire |
| `visual_cue.colours.<site>` | `[r, g, b, w]` per cue site (Wi-Fi trigger only; the wire is one bit) |
| `nsp_sync.enabled` | send event codes to the Blackrock NSP. **The DIO is not wired yet**: when on, it warns once and sends nothing. See [main README §7](../README.md#sync-to-the-blackrock-nsp) |

**Plus, per experiment:**

| Experiment | Extra live parameters |
|---|---|
| Apple Pluck | `…displacement.cartesian_displacement_threshold_m`, `…displacement.force_release_shutdown_delay_sec` |
| Apple Pluck Perturb | the two above, `…displacement.baseline_settle_sec`, `perturb_start.polar_r_m`, `perturb_start.polar_theta_deg`, `perturb_start.polar_plane`, `perturb_start.start_delay_sec` |
| Restricted Plane | `…displacement.cartesian_displacement_threshold_m`, `…displacement.force_release_shutdown_delay_sec` |
| Maze | `timeout_sec` |

(`…displacement` is `apple_pluck_impedance_control_displacement`.)

These are deliberately **not** live: `cartesian_axis`, because it changes what the threshold means,
which makes it a different experiment rather than a different trial; the **safety limits**, because
a limit must not be loosened in the middle of a session; and **poses and geometry**, which are
verified offline.

---

## How it works

```
 browser ──HTTP/SSE──► server.py ──subprocess──► ros2 launch sinthlab_bringup <exp>.launch.py [params_file:=runs/…yaml]
   ▲                      │                                    │
   │ status, log          │ rclpy                              ▼
   └──────────────────────┤◄── <ns>/experiment_status ◄── orchestrator + ExperimentControl
                          ├──► <ns>/<orchestrator>/set_parameters     (live parameters only)
                          ├──► <ns>/<orchestrator>/pause               (std_srvs/SetBool)
                          └◄── <ns>/lbr_state                          (FRI session, rate)
```

| File | Role |
|---|---|
| `server.py` | HTTP server and JSON API; streams log and state to the page (Server-Sent Events) |
| `runner.py` | starts one `ros2 launch` in its own process group. Stop = SIGINT to the group; after 20 s, SIGTERM; after 5 s more, SIGKILL |
| `ros_bridge.py` | the ROS side: status, `lbr_state`, parameter sets, pause. `DemoBridge` simulates it for `--demo` |
| `experiments.py` | the four experiments (launch file, YAML, node, SmartPad profile), the caution notes, linked parameters and joint limits |
| `params.py` | YAML ⇄ flat dotted parameter names, type coercion, the per-run copy |
| `static/` | the page: `index.html`, `style.css`, `app.js`. No external libraries |
| `demo_launch.py` | stands in for `ros2 launch` in demo mode |
| `check_param_docs.py` | fails if any key in `sinthlab_bringup/config/*.yaml` has no one-line description |

On the robot side, [`helpers/experiment_control.py`](../sinthlab_bringup/sinthlab_bringup/helpers/experiment_control.py)
is what makes an orchestrator controllable. Each of the four creates one, and it:

- publishes **`<ns>/experiment_status`** (`std_msgs/String`, JSON, latched) on every trial event and
  once a second: experiment, trial, phase, paused/held, the live values in effect, and changes
  pending for the next trial;
- serves **`<ns>/<orchestrator>/pause`** (`std_srvs/SetBool`);
- **gates `set_parameters`**: live names are accepted and applied at the next trial boundary, and
  everything else is rejected with the reason;
- is the recorder's **`on_event`** hook, so every `TrialRecorder.mark()` reaches the status topic and,
  with `nsp_sync.enabled`, the NSP.

**Security.** The server binds to `127.0.0.1` by default. POSTs must carry a custom header, which a
page on another site cannot send, so a malicious web page cannot start the robot through your
browser. Static files are served from `static/` only.

---

## Without the dashboard

Everything the page does, from a terminal (namespace `lbr`, apple pluck shown):

```bash
# start / stop
ros2 launch sinthlab_bringup iiwa7_apple_pluck_impedance_control.launch.py
ros2 launch sinthlab_bringup iiwa7_apple_pluck_impedance_control.launch.py params_file:=/path/edited.yaml
# Ctrl-C to stop

# status (latched JSON)
ros2 topic echo /lbr/experiment_status

# live change -- applies from the next trial; a non-live name is rejected with the reason
ros2 param set /lbr/apple_pluck_orchestrator visual_cue.enabled false
ros2 param set /lbr/apple_pluck_orchestrator apple_pluck_impedance_control_displacement.cartesian_displacement_threshold_m 0.08

# pause after this trial / resume
ros2 service call /lbr/apple_pluck_orchestrator/pause std_srvs/srv/SetBool "{data: true}"
ros2 service call /lbr/apple_pluck_orchestrator/pause std_srvs/srv/SetBool "{data: false}"
```

The orchestrator nodes are `apple_pluck_orchestrator`, `perturb_orchestrator`,
`restricted_plane_orchestrator` and `maze_orchestrator`.

---

## Files it writes

| Where | What | In git? |
|---|---|---|
| `experiment_ctrl_gui/runs/<exp>_<time>.yaml` | the edited parameter copy each Start used (only when something was edited) | no |
| `experiment_ctrl_gui/logs/<exp>_<time>.log` | the full launch output of each run | no |
| `analysis/expt_<run_name>_<time>/` | the trial recordings, written by the orchestrator as always ([main README §7](../README.md#7-data-collected)) | no |

---

## Adding a live parameter

1. Add its name (or a glob) to `LIVE_PARAMS` in
   [`live_params.py`](../sinthlab_bringup/sinthlab_bringup/helpers/live_params.py), with bounds in `LIMITS`
   (and `CHOICES` if it is an enum).
2. Make sure the action that uses it can **re-read** it: give the action a `reload()` (see
   `AudioCue.reload()`, `CartesianImpedanceDisplacementMonitor.reload()`), and call it from the
   orchestrator's `_reload_live()`.
3. The parameter must be **declared in the YAML**. ROS will not set a parameter the node never
   declared. Give it a one-line description on its line (see
   [above](#parameter-help-and-the-yamls-documentation)); `check_param_docs.py` fails without one.

The dashboard picks it up automatically: it appears on the Live tab. Skipping step 2 is the mistake
the gate exists to catch. The change would be accepted and then ignored, so do step 2 before
listing the name.

---

## Troubleshooting

- **The ROS pill is red: "ROS ✗".** The dashboard was started without ROS sourced. Use `run_gui.sh`,
  or source `/opt/ros/jazzy/setup.bash` and `~/lbr-stack/install/setup.bash` first. Start, Stop and
  the log still work, but live changes, pause and robot state need ROS.
- **"Robot: no state".** Nothing is publishing `/lbr/lbr_state`: the launch has not connected to
  FRI yet, or the SmartPad app is not running or timed out. Restart the app, then Start.
- **The run pill stays on "Starting…".** The launch is up but the orchestrator has not reported.
  Look in the log for the orchestrator's first lines or an exception. The node needs the controllers
  active first.
- **A live change is refused.** The toast and the log give the reason: not a live parameter, out of
  range, or wrong type. Per-run parameters need a Stop, an edit, and a Start.
- **Stop takes a while.** Ctrl-C shuts the nodes down in order. After 20 s the dashboard escalates to
  SIGTERM, and after 5 s more to SIGKILL; the log says so if it does.
- **The page says "Dashboard disconnected".** `server.py` stopped. Restart it; a running launch it
  started is stopped along with it, cleanly, on Ctrl-C.
