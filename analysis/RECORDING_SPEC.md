# Trial data recording — specification

Status: **proposed, not implemented.** Audit of the current state is in §1; everything from §2 on is
the design.

---

## 1. What exists today

| Experiment | File written | Rate | Columns |
|---|---|---|---|
| Maze | `analysis/robot_trajectory_<date>_<time>.csv` | 100 Hz | 9 |
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

**Extra columns (5, unchanged):** `rel_a`, `rel_b`, `corridor`, `off_rail`, `rail_dist`

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
trial. When the Beckhoff lands:

- the NSP sync pulse should be emitted **at a named event**, and the sidecar records which one;
- a periodic pulse (1 Hz) corrects for drift between the cabinet and the NSP oscillator;
- the photodiode on the cue ring remains the ground truth for when light actually appeared — none of
  this replaces it.

---

## 8. Implementation order

Each step is independently testable; nothing after step 1 needs hardware.

1. **`trial_recorder.py`** (new node) — subscribe `/lbr/lbr_state`, FK, buffer, write CSV + sidecar.
   Testable offline by replaying a recorded `LBRState` sequence.
2. **`trial_event.py`** (new helper) — a one-line `publish_event(name, arg)` for orchestrators.
3. **Wire the maze** — it already records; move the start to `start_trial()` and add events.
   Compare old and new output on the same run to confirm nothing regressed.
4. **Wire apple pluck**, including `disp_m` out of the displacement monitor.
5. **Wire perturb** — same, plus the perturbation events.
6. **`plot_trajectory.py`** — read the new schema, prefer sidecar geometry, keep reading old 9-column
   files so existing recordings still plot.
7. **`validate_recording.py`** — sanity-check a finished file the way `check_layout.py` does the CAD:
   rate within tolerance, no gaps, FRI session state healthy throughout, every expected event present
   and ordered.

## 9. Sizing

```
46 columns × ~12 B × 100 Hz  ≈  55 KB/s  ≈  3.3 MB/min
```

A 10-minute maze run is ~33 MB. Buffering in memory as today is ~20 MB of Python list — workable but
wasteful, and a crash loses the trial. **Write incrementally through a buffered writer** and flush on
event rows, so a crash costs at most the last fraction of a second.

`analysis/*.csv` is gitignored; sidecars should be too, or the repo fills with trial metadata.

## 10. Open questions

1. ~~Is the FRI timestamp actually populated at runtime?~~ **RESOLVED 2026-09-22 — yes.**
   ```
   $ ros2 topic echo /lbr/lbr_state --field time_stamp_sec --no-daemon
   1790102440   ->  2026-09-22 18:40:40 UTC
   ```
   It is a **real Unix epoch wall clock**, not an arbitrary counter, and it decodes to the correct
   date. The seconds field ticks once per second with run lengths consistent with 100 Hz.

   Two consequences:
   - The FRI timestamp is directly comparable in *units* to `time.time()`, which makes the alignment
     arithmetic trivial once the offset is known.
   - **The offset is not zero and must be measured.** The observed value ran ~12 minutes ahead of the
     development box's clock. The cabinet is not NTP-disciplined against the ROS box, so never assume
     the two agree — this is exactly why §7 samples all three clocks together at trial start and end.

   **Still to verify — this one gates the whole design:** does `time_stamp_nano_sec` actually vary?
   Only `time_stamp_sec` was checked. If the nanosecond field is always zero the timestamp has
   1-second resolution and is useless for aligning anything:
   ```
   ros2 topic echo /lbr/lbr_state --field time_stamp_nano_sec --no-daemon
   ```
   Expect values spread across 0..999999999, stepping by roughly 10 ms.
2. **One file per trial, or one per session with a trial column?** Per trial matches the current
   scheme and is easier to discard a bad run; per session is easier to analyse in bulk.
3. **Subject/session ID** — where does it come from? A launch argument is simplest.
4. **Is `event_arg` as a single number enough**, or does `safety_trip` need its reason string? A
   reason code plus a lookup table in the sidecar would keep the CSV numeric.
