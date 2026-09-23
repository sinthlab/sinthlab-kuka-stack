# Trial data recording — specification

Status: **implemented** (§8). §1 is the audit of what existed before, kept because it is the reason
for most of the decisions that follow. Verified on hardware for the recording path, the 100 Hz rate
and the cabinet clock; the full event sequence and `stop_and_save()` are still to be exercised on a
completed trial.

---

## 0. At a glance

Every experiment writes **one CSV + one JSON sidecar per trial**, at 100 Hz, sharing a 41-column
core. Per-experiment columns are appended, never substituted, so one analysis path reads all three.

| | Apple pluck | Perturb | Maze |
|---|---|---|---|
| **Today** | nothing | nothing | 9 cols, 100 Hz |
| **Proposed** | **42 cols** | **42 cols** | **47 cols** |
| Extra columns | `disp_m` | `disp_m` | `rel_a` `rel_b` `corridor` `off_rail` `rail_dist` `rail_nearest` |
| Events | 10 | 12 | 13 |
| Sidecar extras | `baseline` | `baseline`, `perturbation` | `maze_geometry` |
| Size | 2.9 MB/min | 2.9 MB/min | 3.2 MB/min |
| Recording window | `start_trial` → `on_recover_complete` | same | `start_trial` → recover (**was** go cue) |

**The 41-column core:** time ×5 (`t`, `t_wall`, `t_ros`, `fri_s`, `fri_ns`) · pose ×7 (xyz +
quaternion) · joints ×21 (`meas`/`cmd`/`ext` × A1–A7) · FRI health ×6 · events ×2.

## 1. What existed before this

| Experiment | File written | Rate | Columns |
|---|---|---|---|
| Maze | `analysis/robot_trajectory_<date>_<time>.csv`, from the **go cue** | 100 Hz | 9 |
| Apple pluck | **nothing** | — | — |
| Perturb | **nothing** | — | — |

`TrajectoryRecorder` is instantiated in exactly one place — `MoveRestrictedOnAPlaneAction.__init__`,
which `MoveInMazeAction` inherits. The apple-pluck and perturb orchestrators never construct one, and
there is no rosbag in any launch file. `diagnostics/record_fri_session.py` writes a rich 29-column
file but is **manual only** and not wired to a trial.

Everything else — the snap, checkpoint rewards, goal/timeout, cue round-trips, safety trips — exists
only as text in `~/.ros/log/`.

### The gaps this spec closes

1. Two of three experiments record nothing at all.
2. Timestamps are `time.time()` sampled inside a Python callback. The cabinet's own clock
   (`LBRState.time_stamp_sec` / `time_stamp_nano_sec`) is already in the message and is discarded.
3. No events in the data file — trial structure is only recoverable by parsing log text.
4. No joint positions, external torques or tracking performance, though all arrive at 100 Hz.
5. No orientation: `record_pose` keeps `transform[0..2, 3]` and drops the rotation block.
6. No per-trial metadata — nothing records which config produced a file.
7. `plot_trajectory.py` reads maze rails from the **current** `maze_params.yaml`, not the geometry
   that was live when the run was recorded. Old runs silently plot against the wrong maze.

---

## 2. Design principles

- **One schema for all three experiments**, so one analysis path works everywhere. Per-experiment
  columns are appended, never substituted.
- **The cabinet clock is the anchor.** Every row carries the FRI timestamp. This is what the
  Blackrock NSP sync will align to; the wall clock is for correlating with video.
- **Pose and joints share one timestamp.** The recorder does its own FK from
  `LBRState.measured_joint_position` rather than reading TF, so position and joints come from the
  same sample rather than two differently-stamped pipelines.
- **Events live in the data file**, not only in the log.
- **Constants go in a sidecar**, not repeated per row.
- **Never block the control path.** Recording stays fire-and-forget, as it is today.

---

## 3. Architecture

A dedicated **`trial_recorder` node** rather than bolting a recorder onto each orchestrator:

```
  /lbr/lbr_state  (LBRState, 100 Hz) ─────┐
                                      ├──►  trial_recorder  ──►  robot_trajectory_<ts>.csv
  /sinthlab/trial_event (String) ─────┘                      └►  robot_trajectory_<ts>.meta.json
       ▲
       └── published by whichever orchestrator is running
```

Why a node and not a mixin:

- one implementation serves all three experiments, including the two that currently record nothing;
- it never touches the fixture's real-time path;
- it can be run standalone for ad-hoc capture, like `record_fri_session.py`;
- events are stamped by the **publisher**, so a late delivery still carries the right time.

**Event message:** `std_msgs/String` carrying JSON — `{"event": "snap", "arg": 0.1043, "t_ros": ...}`.
A custom IDL would be tidier but adds a message package and a rebuild; String keeps this to one new
file. Revisit if the event rate ever justifies it.

---

## 4. Common schema (41 columns, every experiment)

Written at the `/lbr/lbr_state` rate — 100 Hz, set by `controller_manager update_rate`.
**Every column is defined in §11.**

### Time (5)

| Column | Source | Notes |
|---|---|---|
| `t` | derived | seconds since recording start. Keeps existing plots working. |
| `t_wall` | `time.time()` | absolute Unix epoch — correlate with video file timestamps |
| `t_ros` | `node.get_clock().now()` | ROS clock, matches TF stamps |
| `fri_s` | `LBRState.time_stamp_sec` | **cabinet clock — the NSP alignment anchor** |
| `fri_ns` | `LBRState.time_stamp_nano_sec` | |

### Pose (7)

`x`, `y`, `z` (m, base frame) · `qx`, `qy`, `qz`, `qw`

FK of `measured_joint_position`, so it shares the FRI timestamp. Orientation is currently discarded
entirely and is needed for the pluck: the apple can be pulled off-axis.

### Joints (21)

`meas_A1..A7` (rad) · `cmd_A1..A7` (rad) · `ext_A1..A7` (Nm)

**Radians, SI throughout** — convert at plot time. Note `record_fri_session.py` writes degrees; the
two files are not directly comparable column-for-column and the spec deliberately does not follow it.

`cmd − meas` is the impedance droop that cost us the A2 sag diagnosis, and `ext_*` is how you see a
pull without a force sensor. Both are free — they are already in the message.

### FRI health (6)

`tracking` · `session` · `quality` · `safety` · `drive` · `control`

Mostly constant, cheap, and makes a bad trial self-diagnosing without a second file.

### Events (2)

| Column | Notes |
|---|---|
| `event` | empty on most rows; a token on the first row at or after the event |
| `event_arg` | one numeric payload — displacement at snap, checkpoint index, safety reason code |

---

## 5. Per experiment

### 5.1 Apple pluck

**Records from** `start_trial()` **to** `on_recover_complete()` — the whole trial including the
return to start, not just the pull.

**Extra column (1):** `disp_m` — displacement from the locked baseline, the value
`CartesianImpedanceDisplacementMonitor` already computes every tick and currently only prints at
`debug_log_rate_hz`. **This is the dependent variable of the experiment and it is not saved
anywhere.**

**Events** (callback → token):

| Orchestrator callback | `event` | `event_arg` |
|---|---|---|
| `start_trial()` | `trial_start` | trial index |
| `on_move_complete()` | `at_start` | — |
| `on_quiet_window_complete()` | `quiet_end` | — |
| ″ (cue fires) | `cue_go` | — |
| monitor `on_armed` | `armed` | — |
| `on_monitor_snap()` | `snap` | **displacement, m** |
| ″ | `cue_snap` | — |
| ″ (`freeze_hold.start()`) | `freeze` | — |
| `on_monitor_complete()` | `recover_start` | — |
| `on_recover_complete()` | `trial_end` | trial index |

**Cue delivery events** (all three experiments, fired from the cue classes rather than the state
machine, so they can appear anywhere and more than once):

| Event | Fired when | `event_arg` | Observed or inferred? |
|---|---|---|---|
| `cue_visual_ack` | the board answers a Wi-Fi cue | round trip, ms | **observed** — the firmware calls `pixels.show()` *inside* the `/cue` handler and answers afterwards, so an ack means the ring is already lit |
| `cue_audio_end` | the beep subprocess exits | process lifetime, s | **observed** — `[console]::Beep` blocks for exactly `duration_ms`, so the sound's START is `end − duration_ms` |

These exist because `AudioCue.on_complete` fires when `Popen` *returns* — about 49 ms in on WSL2,
and roughly 290 ms before any sound (the Windows process takes ~340 ms to spawn). Nothing else in
the system observes either cue, so without these the record cannot say when the animal was cued.

`armed` is the one that matters behaviourally — it is the moment the baseline locks and the animal
may pull. Reaction time is `snap − armed`.

### 5.2 Perturb

Identical to apple pluck, plus:

| Orchestrator callback | `event` | `event_arg` |
|---|---|---|
| `on_audio_complete()` | `perturb_delay_start` | delay, s |
| `on_perturb_complete()` | `perturb_applied` | magnitude, m |
| `on_monitor_armed()` | `armed` | — |

The applied perturbation **vector** is constant within a trial and goes in the sidecar, not in a
column.

### 5.3 Maze

**Records from** `on_go_complete()` — when the fixture and monitors start. This is why the existing
maze CSVs begin at the go cue, and it is the offset that bit us when matching GIF length to video.

> **Change this.** Start recording at `start_trial()` so the approach and the settle are captured,
> and mark the go cue as an event. Analysis can trim; it cannot un-discard.

**Extra columns (6):** `rel_a`, `rel_b`, `corridor`, `off_rail`, `rail_dist`, `rail_nearest`

**Events:**

| Orchestrator callback | `event` | `event_arg` |
|---|---|---|
| `start_trial()` | `trial_start` | trial index |
| `on_prestart_complete()` | `prestart_done` | — |
| `on_move_complete()` | `at_start` | — |
| `on_switched_to_fixture()` | `fixture_active` | — |
| `on_quiet_window_complete()` | `cue_go` | — |
| `on_go_complete()` | `maze_armed` | — |
| `on_checkpoint_reward(i)` | `checkpoint` | **index** |
| `on_goal_reached()` | `goal` | — |
| `on_timeout()` | `timeout` | — |
| `on_safety_trip(reason)` | `safety_trip` | reason code |
| `force_release.start()` | `release_wait` | — |
| force release complete | `released` | — |
| recover complete | `trial_end` | trial index |

`off_rail` transitions stay a column rather than events — it toggles too often to be useful as one.

---

## 6. Sidecar metadata — `robot_trajectory_<ts>.meta.json`

Everything constant within a trial. Written at `stop_and_save()`.

```jsonc
{
  "schema_version": 1,
  "experiment": "apple_pluck",          // apple_pluck | perturb | maze
  "trial_index": 3,
  "session_id": "20260922_143000",      // shared by every trial in one launch
  "subject": null,                      // from a ROS param; null until set

  "clock_sync": {                       // all four sampled together, start AND end
    "start": {"t_wall": 1790000000.123, "t_ros": 1790000000.118, "fri_s": 12345, "fri_ns": 678000000},
    "end":   {"t_wall": 1790000042.456, "t_ros": 1790000042.451, "fri_s": 12387, "fri_ns": 11000000}
  },

  "fri": {"sample_time_s": 0.01, "session_state": 4, "control_mode": 2},
  "controllers_active": ["lbr_state_broadcaster", "kuka_clik_controller"],
  "git": {"sha": "9bf745f", "dirty": true},

  "start_pose": {"commanded_joint_rad": [...], "measured_joint_rad": [...]},
  "baseline":   {"x": 0.0, "y": 0.0, "z": 0.0},     // apple pluck / perturb only
  "perturbation": {"dx": 0.0, "dy": 0.0, "dz": 0.0}, // perturb only

  "maze_geometry": { "corridors": [...], "checkpoints": [...], "goal": {...} },  // maze only

  "params": { /* full resolved ROS param dump for the orchestrator node */ }
}
```

Two of these earn their place immediately:

- **`maze_geometry`** fixes gap 7. Embedding the geometry that was *live at record time* means a run
  from six months ago still plots against the right maze. `plot_trajectory.py` should prefer the
  sidecar and fall back to `maze_params.yaml` only when it is absent.
- **`params`** is free: the orchestrators already run with
  `automatically_declare_parameters_from_overrides=True`, so the whole resolved set is available from
  the node with no extra plumbing.

---

## 7. Clock alignment (the reason this matters now)

Three clocks are in play and the current recorder uses the worst one:

| Clock | Where | Good for |
|---|---|---|
| Wall (`time.time()`) | ROS box | correlating with video files |
| ROS | ROS box | TF stamps, ROS-side reasoning |
| **FRI** | **cabinet, 1 kHz** | **aligning to neural data** |

Sampling all three together at trial start and end gives both the offsets and the drift over the
trial.

**Measured 2026-09-22 (see §10.1):** cabinet-vs-ROS-box drift is **< 0.1 ppm** with no
discontinuities — so a linear map fitted from two samples is more than adequate, and drift is not
the thing to worry about. Two things are:

- **Sub-sample event timing.** The cabinet stamp is quantised to the 10 ms sample period, so an
  event falling between samples is known only to ±5 ms *from the stamp alone*. Recover sub-ms
  timing by **interpolating the signal**, not by reading the clock: for the pluck, the threshold
  crossing is
  ```
  t_cross = t[i-1] + dt · (threshold − disp_m[i-1]) / (disp_m[i] − disp_m[i-1])
  ```
  This is why `disp_m` must be a per-sample column (§5.1) and not merely an `event_arg` — recording
  it makes the crossing recoverable offline with no extra code in the monitor.
- **The ROS box wall clock is not trustworthy either.** The measured cabinet offset was 14.8 min
  against the ROS box, but ~11.7 min against an independent reference a few hours earlier. Since the
  cabinet provably did not move (< 0.1 ppm would need ~6 years to accumulate 3 minutes), **the ROS
  box clock is itself ~3 min off**. Check it with `timedatectl` before relying on `t_wall` to
  correlate with camera files, which carry the camera's own clock.

**The sync hook is already in place.** `TrialRecorder(..., on_event=fn)` calls `fn(token, arg)`
synchronously inside `mark()`, before anything else, so a hardware pulse leaves at the same instant
the event is logged. It lives in the recorder rather than beside each `mark()` call in the
orchestrators precisely so the two cannot drift apart in a later edit:

```python
CODES = {"trial_start": 1, "at_start": 2, "armed": 3, "snap": 4,
         "checkpoint": 5, "goal": 6, "timeout": 7, "safety_trip": 8, "trial_end": 9}
recorder = TrialRecorder(..., on_event=lambda tok, arg: dio.pulse(CODES[tok]) if tok in CODES else None)
```

Two pulses per trial — one at each end — are enough on their own: they give offset *and* local rate,
and every other event is already in the CSV on the cabinet clock, so the fitted map carries them
along for free. Sending a code rather than a bare pulse means each one self-identifies, so a dropped
pulse is obvious instead of silently mispairing every subsequent trial.

When the Beckhoff lands:

- the NSP sync pulse should be emitted **at a named event**, and the sidecar records which one;
- a periodic pulse (1 Hz) corrects for drift between the cabinet and the NSP oscillator;
- the photodiode on the cue ring remains the ground truth for when light actually appeared — none of
  this replaces it.

---

## 8. Implementation order — **DONE**

All seven steps are implemented. What actually shipped, and where it differs from the plan above:

| # | Step | Status |
|---|---|---|
| 1 | `trial_recorder.py` | ✅ `actions/trial_recorder.py` |
| 2 | ~~`trial_event.py`~~ | **dropped** — see below |
| 3 | Wire the maze | ✅ starts at `start_trial()`, 13 events |
| 4 | Wire apple pluck | ✅ 10 events, `disp_m` |
| 5 | Wire perturb | ✅ 12 events, perturbation in the sidecar |
| 6 | `plot_trajectory.py` | ✅ new schema, sidecar rails, pull view, event markers |
| 7 | `validate_recording.py` | ✅ |

**The recorder is a class the orchestrator owns, not a standalone node.** §3 proposed a node with
events over a topic. In practice the orchestrators already host every action as a class in their own
node, so a class makes events direct calls with no topic hop or serialisation to mis-time, and
`robot_description` does not have to be plumbed to a second process. The trade — recording shares
the orchestrator's executor — is nothing against a 10 ms budget. **Step 2 disappeared with it:**
`recorder.mark(token, arg)` *is* the event API.

**Both recorders coexist.** `MoveRestrictedOnAPlaneAction` gained `own_recorder=True`; the maze
passes `False` so only the orchestrator's `TrialRecorder` writes. The fourth orchestrator
(`restricted_plane`) still uses the old `TrajectoryRecorder` unchanged.

### Tooling

```bash
python3 analysis/plot_trajectory.py            # newest trial; maze view or pull view as appropriate
python3 analysis/validate_recording.py --all   # exit 1 if any trial is unsound
```

`plot_trajectory.py` reads **both** schemas, so older 9-column recordings still plot. For pluck and
perturb — which previously had no first panel at all — it draws `disp_m` against time with the
threshold and the armed/snap markers on it; reaction time is the gap between the two.

`validate_recording.py` checks schema, rate and gaps, cabinet-clock monotonicity, FRI health across
the whole trial, event presence and ordering, and sidecar completeness.

## 9. Sizing

```
46 columns × ~12 B × 100 Hz  ≈  55 KB/s  ≈  3.3 MB/min
```

A 10-minute maze run is ~33 MB. Buffering in memory as today is ~20 MB of Python list — workable but
wasteful, and a crash loses the trial. **Write incrementally through a buffered writer** and flush on
event rows, so a crash costs at most the last fraction of a second.

`analysis/*.csv` is gitignored; sidecars should be too, or the repo fills with trial metadata.

## 10. Open questions

1. ~~Is the FRI timestamp actually populated at runtime?~~ **RESOLVED 2026-09-22 — measured, and
   the news is good.** `ros2 run sinthlab_bringup check_clock_drift.py`, 29996 samples over 299.9 s:

   | | Result |
   |---|---|
   | Populated? | **Yes** — real Unix epoch, decodes to the correct date |
   | Resolution | **quantised to 10.000 ms — exactly the FRI sample period** |
   | Sample spacing | median 10.000 ms, **p99 10.000 ms** |
   | Drift vs ROS box | **< 0.1 ppm** (3σ) over 5 minutes |
   | Discontinuities | **none** above 50 ms |
   | Absolute offset | 885.4 s (14.8 min) ahead of the ROS box |

   **The stamp is a sample grid, not a free-running clock.** `time_stamp_nano_sec` takes exactly 100
   distinct values (8 ms … 998 ms on a 10 ms grid). That is better than it sounds: the grid is
   *exact and jitter-free*, so the stamp identifies which sample with no ambiguity. What it does not
   give you is sub-sample resolution — see the note in §7.

   **Drift is a non-issue.** At < 0.1 ppm an hour accumulates under 0.4 ms. The §7 periodic sync
   pulse is now belt-and-braces rather than a requirement, and a single sync at trial start would be
   defensible on its own.

2. **One file per trial, or one per session with a trial column?** Per trial matches the current
   scheme and is easier to discard a bad run; per session is easier to analyse in bulk.
3. **Subject/session ID** — where does it come from? A launch argument is simplest.
4. **Is `event_arg` as a single number enough**, or does `safety_trip` need its reason string? A
   reason code plus a lookup table in the sidecar would keep the CSV numeric.

---

## 11. Data dictionary

Every column, what it holds and where it comes from. `LBRState` fields are copied straight from
`<ns>/lbr_state`; "derived" means the recorder computes it.

### 11.1 Time (all experiments)

| Column | Unit | Source | Meaning |
|---|---|---|---|
| `t` | s | derived | Seconds since the recorder started. Convenience axis for plotting; **not** an alignment clock. |
| `t_wall` | s (Unix epoch) | `time.time()` on the ROS box | Absolute wall time at the moment the callback ran. Use to correlate with video files — but see §7: the ROS box clock was measured ~3 min off. |
| `t_ros` | s | `node.get_clock().now()` | ROS clock. Matches TF stamps, so it is the right clock for reasoning about anything ROS-side. |
| `fri_s` | s (Unix epoch) | `LBRState.time_stamp_sec` | Cabinet clock, seconds part. |
| `fri_ns` | ns | `LBRState.time_stamp_nano_sec` | Cabinet clock, nanoseconds part. **Quantised to the 10 ms sample period** — see §10.1. Together with `fri_s` this is the exact, jitter-free sample grid and the anchor for NSP alignment. |

### 11.2 End-effector pose (all experiments)

Forward kinematics of `measured_joint_position`, expressed in `lbr_link_0` (the robot base).
Computed by the recorder rather than read from TF, so pose and joints share one timestamp.

| Column | Unit | Meaning |
|---|---|---|
| `x` `y` `z` | m | EE position in the base frame. |
| `qx` `qy` `qz` `qw` | — | EE orientation as a unit quaternion (scalar last). Currently discarded entirely; needed because the apple can be pulled off-axis. |

### 11.3 Joints (all experiments)

Seven values each, `A1`…`A7`, base to wrist. **Radians and newton-metres — SI throughout.**
(`record_fri_session.py` writes degrees; the two files are deliberately not the same convention.)

| Column | Unit | Source | Meaning |
|---|---|---|---|
| `meas_A1..A7` | rad | `measured_joint_position` | Where the arm actually is. |
| `cmd_A1..A7` | rad | `commanded_joint_position` | Where the cabinet is commanding it to be — under Cartesian impedance this is the *equilibrium*, not a position the arm will reach. |
| `ext_A1..A7` | Nm | `external_torque` | Torque the cabinet attributes to outside forces, gravity model removed. This is how a pull is measured without a force sensor. A steady non-zero value at rest means un-modelled tool mass — that is what diagnosed the 3.0 Nm A2 sag. |

**`cmd − meas` is the impedance droop.** Under Cartesian impedance the arm deliberately lags its
equilibrium by `F / k`; that difference is the signal, not an error.

### 11.4 FRI health (all experiments)

Mostly constant. Included per-sample so a bad trial is self-diagnosing without cross-referencing a
second file.

| Column | Source | Values |
|---|---|---|
| `tracking` | `tracking_performance` | 0…1. How well the cabinet is following the command; drops when the commanded pose is unreachable or the arm is being fought. |
| `session` | `session_state` | 0 IDLE · 1 MONITORING_WAIT · 2 MONITORING_READY · 3 COMMANDING_WAIT · **4 COMMANDING_ACTIVE** (the only one in which commands take effect) |
| `quality` | `connection_quality` | 0 POOR · 1 FAIR · 2 GOOD · 3 EXCELLENT |
| `safety` | `safety_state` | **0 NORMAL_OPERATION** · 1 SAFETY_STOP_LEVEL_0 · 2 LEVEL_1 · 3 LEVEL_2 |
| `drive` | `drive_state` | 0 OFF · 1 TRANSITIONING · **2 ACTIVE** |
| `control` | `control_mode` | 0 POSITION · **1 CART_IMP** · 2 JOINT_IMP · 3 NO_CONTROL |

Anything other than session 4 / safety 0 / drive 2 for any part of a trial means that trial is
suspect. `validate_recording.py` (§8.7) should refuse it.

### 11.5 Events (all experiments)

| Column | Meaning |
|---|---|
| `event` | Empty on most rows. On the first sample at or after an event, the token from §5. |
| `event_arg` | One number whose meaning depends on the token: trial index, displacement in m at `snap`, checkpoint index, perturbation magnitude, safety reason code. Empty where the token carries no payload. |

Events land on a **sample boundary**, so their time is known to ±5 ms from the row alone. Recover
finer timing by interpolating the underlying signal — see §7.

### 11.6 Apple pluck / perturb only

| Column | Unit | Source | Meaning |
|---|---|---|---|
| `disp_m` | m | `CartesianImpedanceDisplacementMonitor` | Distance of the EE from the baseline pose locked at `armed`, along `cartesian_axis` (`norm` = 3-D distance). **The dependent variable.** The trial ends when it crosses `cartesian_displacement_threshold_m`. Recording it per sample is what makes the crossing time recoverable to sub-millisecond by interpolation. |

### 11.7 Maze only

All maze coordinates are **relative to the anchor** — the EE pose captured after the fixture's
`anchor_settle_sec` — and lie in the plane left free by `restricted_axis` (the maze is in Y-Z, so
`a` = Y and `b` = Z).

| Column | Unit | Meaning |
|---|---|---|
| `rel_a` | m | In-plane offset from the anchor along the first free axis. |
| `rel_b` | m | Same, second free axis. Plot `rel_b` against `rel_a` to get the maze view. |
| `corridor` | index | Index of the nearest corridor **when on-rail**, else **−1**. Not a strict inside/outside test: the corridors are zero-width lines, so "on rail" means within `on_rail_tol` of one. |
| `off_rail` | 0 / 1 | 1 when the arm is further than `on_rail_tol` from every corridor — i.e. being actively pulled back by the virtual fixture. Mirrors `corridor == -1`. |
| `rail_dist` | m | Distance to the **nearest** corridor segment, always populated. On-rail it is ≤ `on_rail_tol`; off-rail it is how hard the fixture is working. |

> **Gap worth closing while implementing.** `_maze_coords` computes the nearest corridor index
> *before* the on-rail test and then throws it away when off-rail, so `corridor` is −1 and the CSV
> cannot say *which* corridor the arm was pushed off. The debug log prints it; the data does not.
> **Done** — `rail_nearest` (nearest corridor index, always populated regardless of on-rail
> state) is now the sixth maze column, so off-rail episodes can be attributed to a junction.

### 11.8 Sidecar fields

| Field | Meaning |
|---|---|
| `schema_version` | Bump when columns change, so loaders can branch. |
| `experiment` | `apple_pluck` · `perturb` · `maze` |
| `trial_index` | Counter within the session. |
| `session_id` | Shared by every trial from one launch. |
| `subject` | Animal identifier, from a ROS param. |
| `clock_sync.start/end` | All four clocks sampled together at both ends of the trial. Gives the offsets *and* the drift over that trial. |
| `fri` | `sample_time_s`, and the session/control mode at trial start. |
| `controllers_active` | Which ros2_control controllers were running — catches "wrong controller was loaded". |
| `git` | Commit SHA and dirty flag of the stack that produced the file. |
| `start_pose` | Commanded and measured joint vectors at trial start. |
| `baseline` | Apple pluck / perturb: the pose `disp_m` is measured from. |
| `perturbation` | Perturb: the applied offset vector. Constant per trial, hence metadata not a column. |
| `maze_geometry` | Maze: corridors, checkpoints and goal **as they were at record time**. Without this an old run silently plots against whatever `maze_params.yaml` says today. |
| `params` | Full resolved ROS parameter dump for the orchestrator node. |
| `events` | **Every `mark()` with the moment it actually happened** — `t`, `t_wall`, `t_ros`, `fri_s`, `fri_ns` and the CSV row it landed on. The CSV column is quantised to the 10 ms sample grid; this is not. Use the column to *find* an event, this to *time* it — and it is what a TTL pulse lines up against. |
