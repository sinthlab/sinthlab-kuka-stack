"""Which experiment parameters may change WHILE an experiment runs -- the single source of truth.

Every orchestrator reads its parameters once, when it starts. Most of them must stay that way: a
start pose, a maze rail or a safety limit changed mid-session would silently split one session into
two experiments. The parameters listed here are the exception. They are re-read at the next TRIAL
BOUNDARY (never mid-trial), so every trial runs under one consistent configuration, and the trial's
sidecar records the values it actually ran with.

Two consumers, no ROS import here so both can load it:
  * helpers/experiment_control.py -- the orchestrator side. It REJECTS a `ros2 param set` on
    anything not listed here, so a change that would have no effect cannot look as if it worked.
  * experiment_ctrl_gui/ -- the dashboard, which shows these under "Live" and everything else as
    per-run (edit before Start) or fixed.

Patterns are fnmatch globs over the dotted ROS parameter name.
"""
from __future__ import annotations

import ipaddress
from fnmatch import fnmatch
from typing import Dict, List, Optional, Tuple

# Cue and sync switches every experiment has.
_COMMON = [
    "quiet_window_sec",            # pause at the start pose before the go cue
    "move_to_park.hold_sec",       # wait at park before each trial's move to the start
    # The park pose: only the joint controller drives the arm there, so it can change between trials
    # anywhere. Checked like any pose, and its elbow (A4) must bend the same way as the start's.
    "move_to_park.target_joint_position",
    "quiet_gate.*",                # the guarded quiet window: early touch -> no reward, park, time-out
    "audio_cue.enabled",           # master switch for every beep
    "audio_cue_*.frequency_hz",
    "audio_cue_*.duration_ms",
    "visual_cue.enabled",          # master switch for the NeoPixel ring
    "visual_cue.trigger",          # serial (the RS-422 link) or wifi (the board's access point)
    "visual_cue.colours.*",
    # TTL sync pulses to the Blackrock NSP: switch, cabinet address, the trial outputs, width. Not
    # nsp_sync.experiment_start_output: that pulse is sent once, as the run begins (per-run).
    "nsp_sync.enabled", "nsp_sync.cabinet_ip", "nsp_sync.trial_*_output", "nsp_sync.pulse_width_ms",
]

# The reward: a pulse on a Beckhoff output (SYNC 4) into the reward box's NSP jack on a successful trial.
# Every experiment has it; the success outcomes are experiment_control.REWARD_EVENTS.
_REWARD = ["reward_pulse.enabled", "reward_pulse.output", "reward_pulse.pulse_ms",
           "reward_pulse.count", "reward_pulse.gap_ms"]

# The pull threshold and its dwell. Deliberately NOT cartesian_axis: that changes what the
# threshold means, so it is a different experiment, not a different trial.
_PULL = [
    "apple_pluck_impedance_control_displacement.cartesian_displacement_threshold_m",
    "apple_pluck_impedance_control_displacement.force_release_shutdown_delay_sec",
]

# Move speed of every joint-space move (start, park, the perturbation): one scale per move that
# time-stretches it -- 0.5 = twice as long, 2 = half as long. It scales that move's v/a/j limits
# (move_to_pos_*, per-run) by s, s^2, s^3, so it changes every move, however short. The raw limits
# barely do: on these few-degree moves v never binds and a/j act through square / cube roots.
# IMMEDIATE (below): applied the moment it is set, from the next move on (a move under way keeps its speed).
_SPEED = ["*.speed_scale"]

# The start pose. Live ONLY where the joint controller alone drives the arm (apple pluck). Restricted
# plane and maze also hand the arm to the CLIK, whose redundancy posture must equal the start pose and
# is read once when the controller is configured -- and the maze's whole geometry hangs off the start.
# There the pose stays per-run. (The park pose is live everywhere: see _COMMON.)
_POSE = ["move_to_start.target_joint_position"]

# Pre-training: the quarter-ring colours.
_RING = ["ring_cue.quarters", "ring_cue.colours.*"]

# The felt stiffness of the pull (apple pluck, perturb). ROS cannot change the cabinet's stiffness; it
# softens it by letting the equilibrium follow part of the pull -- see actions/resistance_softener.py.
_RESIST = ["resistance.stiffness_n_per_m"]

# Haptic guidance (actions/haptic_guidance.py): the maze and the rails' centre-out reach.
_HAPTIC = ["haptic.*"]

# Pre-training: Free Move, and Free Move on a rail (Move Vertical / Horizontal).
_FREE = _COMMON + _REWARD + _SPEED + _RING + _HAPTIC + [
    # Move Vertical / Horizontal: the centre-out reach (actions/centre_out.py). Free Move has none.
    "centre_out.*",
    "rule.*",                      # the colour rule (helpers/rule_schedule.py; colour_rule.yaml only)
    # Free Move: the long-movement reward (actions/long_move.py). The rails have none.
    "long_move.*",
    "session_sec",
    "response_timeout_sec",        # not moved in time -> the session ends, park
    # The feel of the admittance -- see pretrain_free_move.yaml.
    "free_move.damping_ns_per_m",
    "free_move.deadband_n",
    "free_move.max_speed_mps",
    "free_move.accel_limit_mps2",
    "free_move.force_filter_tau_sec",
    "free_move.tare_sec",
    "free_move.trail_m",
    "free_move.low_force_n",
    "free_move.low_force_damping_ns_per_m",
]

LIVE_PARAMS: Dict[str, List[str]] = {
    # Every paradigm. perturb_start.* act only in perturb / perturb_bump, grasp.* only in perturb_bump.
    "apple_pluck": _COMMON + _REWARD + _PULL + _SPEED + _POSE + _RESIST + [
        "response_timeout_sec",    # no pluck in time -> the trial ends, no reward, park
        "perturb_start.baseline_settle_sec",
        # The perturbation itself -- the manipulated variable, so it is the one most worth varying
        # trial to trial. Recorded per trial in the sidecar's `perturbation` block.
        "perturb_start.polar_r_m",
        "perturb_start.polar_theta_deg",
        "perturb_start.polar_plane",
        "perturb_start.start_delay_mode",
        "perturb_start.start_delay_sec",
        "perturb_start.start_delay_min_sec",
        "perturb_start.start_delay_max_sec",
        # perturb_bump: detecting the grasp that triggers the perturbation, and the delay after it.
        "grasp.threshold_n",
        "grasp.hold_sec",
        "grasp.bump_speed_scale",
        "grasp.delay_mode",
        "grasp.delay_sec",
        "grasp.delay_min_sec",
        "grasp.delay_max_sec",
    ],
    "restricted_plane": _COMMON + _REWARD + _PULL + _SPEED + ["response_timeout_sec"],
    "maze": _COMMON + _REWARD + _SPEED + _HAPTIC + [
        "response_timeout_sec",    # the trial's time limit from the go cue
        # How easy moving along a rail feels -- see rail_lead_sec in maze_params.yaml. Walls unchanged.
        "virtual_fixtures.rail_lead_sec",
    ],
    # Pre-training. The rail geometry stays per-run, like the maze's rails.
    "free_move": _FREE,
    # Free Move on one rail: the same session and feel; the rail itself (free_move.rail_*) is per-run.
    "move_vertical": _FREE,
    "move_horizontal": _FREE,
    # Restricted Move: Free Move on one rail, a session per trial (no centre-out reach).
    "restricted_move_horizontal": _FREE,
    "restricted_move_vertical": _FREE,
    # The colour rule: the same node and feel; Rail / No rail (free_move.rail_axis) is per-run, and the
    # centre-out mode is fixed to the rule (so not centre_out.*: only the reach distances are live).
    "colour_rule": [x for x in _FREE if x != "centre_out.*"] + ["centre_out.end_tol_m", "centre_out.free_reach_m"],
    # N-choice: the same node and feel; the asterisk (free_move.rail_axis, ray_layout, ray_length_m) is per-run.
    "n_choice": [x for x in _FREE if x not in ("centre_out.*", "rule.*", "long_move.*")] + ["n_choice.*"],
    # Reach Map: Free Move in the frontal plane; the reward (long_move.reward: each_move / new_best) and the
    # New-best settings are live; the box is per-run.
    "reach_map": [x for x in _FREE if x not in ("centre_out.*", "rule.*")] + ["reach_map.*"],
}

# Bounds checked on every live change, (min, max) inclusive. A colour is checked per element.
LIMITS: Dict[str, Tuple[float, float]] = {
    "quiet_window_sec": (0.0, 60.0),
    "move_to_park.hold_sec": (0.0, 600.0),
    "quiet_gate.move_m": (0.005, 0.2),
    "quiet_gate.force_n": (1.0, 50.0),
    "quiet_gate.penalty_sec": (0.0, 120.0),
    "response_timeout_sec": (2.0, 600.0),
    "audio_cue_*.frequency_hz": (37, 32767),      # [console]::Beep refuses anything outside this
    "audio_cue_*.duration_ms": (10, 10000),
    "visual_cue.colours.*": (0, 255),
    "*.cartesian_displacement_threshold_m": (0.005, 0.5),
    "*.force_release_shutdown_delay_sec": (0.0, 10.0),
    "*.baseline_settle_sec": (0.0, 10.0),
    # Apple pluck (per-run): the elbow wall and the wait for the released arm (resistance_softener.py).
    "resistance.elbow_min_deg": (10.0, 60.0),
    "resistance.park_wait_timeout_sec": (0.5, 30.0),
    # The floor (per-run): height above the ground [m]; the arm cannot reach far above 1.3 m.
    "floor.height_m": (0.0, 1.3),
    "floor.trip_margin_m": (0.0, 0.2),
    # random_start offsets [m] from the start, each of a and b; the run refuses any it cannot reach.
    "random_start.positions.*": (-0.20, 0.20),
    "perturb_start.polar_r_m": (0.0, 0.175),      # overall; the real cap depends on the plane, below
    "perturb_start.polar_theta_deg": (-360.0, 360.0),
    "perturb_start.start_delay_sec": (0.0, 10.0),
    "perturb_start.start_delay_min_sec": (0.0, 10.0),
    "perturb_start.start_delay_max_sec": (0.0, 10.0),
    "grasp.threshold_n": (0.5, 30.0),
    "grasp.hold_sec": (0.0, 0.5),
    # The bump: perturb's base limits x 3 / 9 / 27 at most = a 18 rad/s^2, j 135 rad/s^3, within the
    # fastest perturbation validated (a 20, j 150).
    "grasp.bump_speed_scale": (0.25, 3.0),
    "grasp.delay_sec": (0.0, 10.0),
    "grasp.delay_min_sec": (0.0, 10.0),
    "grasp.delay_max_sec": (0.0, 10.0),
    # The cabinet's timer is good to a few ms, and shorter pulses can fall between its I/O cycles.
    "nsp_sync.pulse_width_ms": (10, 5000),
    # How long the juice valve stays open. The cabinet pulses 1-5000 ms.
    "reward_pulse.pulse_ms": (10, 5000),
    # A burst: pulses per reward, and the closed valve between them (the cabinet stretches a pulse that is
    # still high into one long opening, so they must not touch).
    "reward_pulse.count": (1, 10),
    "reward_pulse.gap_ms": (100, 5000),
    # Centre-out: how close to a rail end counts as reaching it.
    "centre_out.end_tol_m": (0.005, 0.03),
    # The colour rule, No rail: how far left or right is a choice.
    "centre_out.free_reach_m": (0.03, 0.2),
    # Cued: at most this many trials in a row to the same end (0 = no cap).
    "centre_out.max_repeats": (0, 20),
    # The colour rule (helpers/rule_schedule.py).
    "rule.window": (1, 100),
    "rule.criterion_pct": (50.0, 100.0),
    "rule.min_trials": (1, 200),
    "rule.max_repeats": (0, 20),
    # N-choice (actions/n_choice.py, helpers/bandit_schedule.py).
    "n_choice.end_tol_m": (0.005, 0.03),
    "n_choice.p_max": (0.0, 1.0),
    "n_choice.p_min": (0.0, 1.0),
    "n_choice.deplete": (0.0, 1.0),
    "n_choice.recover": (0.0, 1.0),
    "free_move.ray_length_m": (0.02, 0.15),
    # Long moves (actions/long_move.py).
    "long_move.min_m": (0.02, 0.40),
    "long_move.min_straightness": (0.0, 1.0),
    "long_move.min_gap_sec": (0.0, 30.0),
    "long_move.start_speed_mps": (0.01, 0.3),
    "long_move.stop_speed_mps": (0.005, 0.3),
    "long_move.stop_sec": (0.05, 2.0),
    "long_move.confirm_m": (0.005, 0.1),
    # Reach Map (helpers/reach_map.py).
    "reach_map.sectors": (4, 36),
    "reach_map.margin_m": (0.0, 0.05),
    "reach_map.min_m": (0.0, 0.10),
    "reach_map.percentile": (50.0, 100.0),
    "haptic.min_pct": (0, 100),
    "haptic.max_pct": (0, 100),
    "haptic.ramp_start_m": (0.005, 0.3),
    "haptic.ramp_full_m": (0.0, 0.1),
    "end_effector.fsr_rate_hz": (0, 200),
    # Assist is capped low: past the real lag the spring pushes the arm ALONG the rail by itself.
    # Drag may go further -- it only ever resists.
    "virtual_fixtures.rail_lead_sec": (-0.15, 0.05),
    # Speed limits. v_max in deg/s per joint, kept under the slowest joint's rating (98 deg/s, A1/A2).
    # a_max / j_max up to the fastest perturbation measured (20 rad/s^2, 150 rad/s^3: 0.23 s for 5 cm).
    "*.move_to_pos_v_max": (5.0, 90.0),
    "*.move_to_pos_a_max": (0.5, 20.0),
    "*.move_to_pos_j_max": (1.0, 150.0),
    "*.speed_scale": (0.25, 2.0),
    # Felt stiffness [N/m]: from MIN_FELT_STIFFNESS up to the SmartPad base the user enters -- checked
    # against resistance.base_stiffness_n_per_m (_check_pair), and the dashboard sizes the slider to
    # match (limits_for with a context). The static range here is just the widest possible base.
    "resistance.stiffness_n_per_m": (50.0, 5000.0),
    # The SmartPad profile's translational stiffness (per-run): Sunrise caps it at 5000 N/m.
    "resistance.base_stiffness_n_per_m": (50.0, 5000.0),
    # Pre-training. A rail's ends, from the start (the vertical one is limited upward by reach).
    "free_move.rail_min_m": (-0.30, 0.0),
    "free_move.rail_max_m": (0.0, 0.30),
    "ring_cue.colours.*": (0, 255),
    "ring_cue.quarters": (0, 3),
    # Admittance feel. The speed cap stays under the free-move safety monitor's 0.7 m/s.
    "session_sec": (0.0, 3600.0),          # 0 = until Stop
    "free_move.damping_ns_per_m": (3.0, 200.0),
    "free_move.deadband_n": (0.0, 15.0),
    "free_move.max_speed_mps": (0.02, 0.5),
    "free_move.accel_limit_mps2": (0.0, 10.0),
    "free_move.force_filter_tau_sec": (0.0, 0.5),
    "free_move.tare_sec": (0.2, 5.0),
    "free_move.trail_m": (0.0, 0.03),               # at most the leash
    "free_move.low_force_n": (0.0, 30.0),
    "free_move.low_force_damping_ns_per_m": (40.0, 300.0),
}

# Joint poses: 7 angles in degrees, inside the iiwa7 limits (lbr joint_limits.yaml, which is 1 deg
# stricter than the hardware), and not a nearly straight arm -- max(|A2|, |A4|, |A6|) below this is
# singular (checked against the Jacobian: the same as "smallest singular value < 0.05").
IIWA7_LIMITS_DEG = [169.0, 119.0, 169.0, 119.0, 169.0, 119.0, 174.0]
STRAIGHT_BELOW_DEG = 12.0
_START, _PARK = "move_to_start.target_joint_position", "move_to_park.target_joint_position"

# Paradigm-only parameters: in an experiment with a paradigm (apple pluck's `paradigm`, the rails'
# `centre_out.target`), these act -- and the dashboard shows them -- only in those paradigms. The first
# matching pattern wins.
PARADIGM_ONLY: Dict[str, Tuple[str, ...]] = {
    "perturb_start.baseline_settle_sec": ("perturb",),
    "perturb_start.speed_scale": ("perturb",),           # perturb_bump: grasp.bump_speed_scale
    "perturb_start.start_delay_*": ("perturb",),
    "perturb_start.*": ("perturb", "perturb_bump"),
    "grasp.*": ("perturb_bump",),
    "random_start.*": ("random_start",),
    # Move Vertical / Horizontal, whose paradigm is centre_out.target.
    "centre_out.max_repeats": ("cued",),
    "centre_out.block_other_end": ("cued", "left", "right", "down", "up"),
    # The colour rule, whose paradigm is Rail / No rail (free_move.rail_axis).
    "centre_out.free_reach_m": ("frontal",),
    "centre_out.end_tol_m": ("y", "x", "z", "either", "cued", "left", "right", "down", "up"),
    # Free Move, whose paradigm is long_move.reward.
    "long_move.*": ("each_move", "new_best"),
}


def paradigm_of(name: str) -> Optional[Tuple[str, ...]]:
    """The paradigms a parameter belongs to, or None if it applies to every paradigm."""
    for pattern, paradigms in PARADIGM_ONLY.items():
        if fnmatch(name, pattern):
            return paradigms
    return None


# Slider steps: a parameter listed here gets a slider (plus its text box) on the dashboard. The text box
# still takes any value within LIMITS; the slider moves in these steps.
STEPS: Dict[str, float] = {
    "resistance.stiffness_n_per_m": 10.0,
    "*.speed_scale": 0.05,
    "grasp.bump_speed_scale": 0.05,
    "centre_out.max_repeats": 1,
    "long_move.min_m": 0.01,
}

# Live parameters applied the moment they are set, instead of at the next trial boundary. Only for
# what cannot change the trial's outcome or its measurement: how fast the arm is driven -- from the
# next move on, even within the current trial. Each change is logged in the trial's sidecar
# (`live_changes`) with when it happened.
IMMEDIATE: List[str] = ["*.speed_scale", "grasp.bump_speed_scale"]


def is_immediate(name: str) -> bool:
    return any(fnmatch(name, p) for p in IMMEDIATE)

# Softening never goes below this felt stiffness [N/m]: whatever the base, at least this much spring
# always remains (following the arm completely would leave nothing holding it up).
MIN_FELT_STIFFNESS = 50.0
_K, _K_BASE = "resistance.stiffness_n_per_m", "resistance.base_stiffness_n_per_m"

# Parameters with a fixed set of values. The dashboard renders these as a drop-down.
CHOICES: Dict[str, List[str]] = {
    "perturb_start.polar_plane": ["frontal", "horizontal", "sagittal"],
    "*.cartesian_axis": ["norm", "x", "y", "z"],
    "*.restricted_axis": ["x", "y", "z"],
    "*.corridor_frame": ["relative", "absolute"],
    "*.polar_plane": ["frontal", "horizontal", "sagittal"],
    "random_start.plane": ["frontal", "horizontal", "sagittal"],
    "floor.reference": ["apple_bottom", "base_edge"],      # helpers/floor.REFERENCES
    "paradigm": ["pluck", "perturb", "perturb_bump", "random_start"],
    "perturb_start.start_delay_mode": ["fixed", "random"],
    "grasp.delay_mode": ["fixed", "random"],
    "free_move.rail_axis": ["none", "x", "y", "z", "frontal", "rays"],   # helpers/rail_shapes.SHAPES
    "nsp_sync.*_output": ["1", "2", "3", "4"],                # the EL2124's outputs, front jacks SYNC 1-4
    "reward_pulse.output": ["1", "2", "3", "4"],
    "free_move.box.preset": ["standard", "more_x", "most_x", "centred", "custom"],   # helpers/free_move_box.CHOICES
    # helpers/reach_targets.modes_for: either | cued | an end's name. The node refuses an end its rail
    # does not have (left / right on the vertical rail, down / up on the horizontal one).
    "centre_out.target": ["either", "cued", "left", "right", "down", "up", "rule"],
    "rule.red_end": ["left", "right", "down", "up"],          # the node refuses an end its rail lacks
    "centre_out.quarter_*": ["0", "1", "2", "3"],             # the ring's quarters
    "long_move.reward": ["session", "each_move", "new_best"],  # actions/long_move.MODES (new_best: the Reach Map only)
    "n_choice.start": ["full", "random"],                    # helpers/bandit_schedule.STARTS
    "free_move.ray_layout": ["2", "3", "4", "5", "6", "7", "8"],   # helpers/rail_shapes.LAYOUT_COUNTS
    "visual_cue.trigger": ["serial", "wifi"],                 # actions/visual_cue.TRIGGERS
}


# Largest perturbation per plane [m]. From the apple-pluck start pose, position-only IK (the same DLS
# PerturbInitialPosition uses) keeps EVERY direction in the plane at least 36 deg from every joint
# limit, and a further 15 cm pull toward the monkey still solves with >= 49 deg left; the smallest
# singular value of the 6D Jacobian stays >= 0.064 / 0.079 / 0.069 (frontal / horizontal /
# sagittal). One step further out (frontal 0.15, sagittal 0.125), that 15 cm pull runs out of reach.
PERTURB_R_MAX: Dict[str, float] = {"frontal": 0.125, "horizontal": 0.175, "sagittal": 0.10}
_R, _PLANE = "perturb_start.polar_r_m", "perturb_start.polar_plane"
# Delay ranges (min, max): perturb's after the go cue, perturb_bump's after the grasp.
_DELAY_RANGES = (("perturb_start.start_delay_min_sec", "perturb_start.start_delay_max_sec"),
                 ("grasp.delay_min_sec", "grasp.delay_max_sec"))
_SYNC_OUTPUTS = ("nsp_sync.trial_start_output", "nsp_sync.trial_end_output", "nsp_sync.experiment_start_output",
                 "reward_pulse.output")
CHOICE_GEOMETRY = ("free_move.rail_axis", "free_move.ray_length_m", "n_choice.end_tol_m",
                   "centre_out.free_reach_m", "free_move.box.preset") + tuple(
    f"free_move.box.{axis}_{end}_m" for axis in "xyz" for end in ("min", "max"))


def choice_geometry_problem(params):
    axis = params.get("free_move.rail_axis")
    if axis == "rays" and "n_choice.end_tol_m" in params:
        if not 0 < params["n_choice.end_tol_m"] < params.get("free_move.ray_length_m", 0):
            return "n_choice.end_tol_m must be positive and smaller than free_move.ray_length_m"
    if axis == "frontal" and "centre_out.free_reach_m" in params:
        from sinthlab_bringup.helpers.free_move_box import box_bounds
        try:
            lo, hi = box_bounds(params.__getitem__)
        except (KeyError, ValueError) as exc:
            return str(exc)
        if not 0 < params["centre_out.free_reach_m"] <= min(-lo[1], hi[1]):
            return "centre_out.free_reach_m must be positive and fit inside both lateral box bounds"
    return None


PAIRED = CHOICE_GEOMETRY + (_R, _PLANE, _START, _PARK, _K, _K_BASE) + sum(_DELAY_RANGES, ()) + _SYNC_OUTPUTS   # checked together: see check_value(context=...)


def check_perturbation(r, plane) -> Optional[str]:
    """None if a perturbation of r metres in `plane` is within that plane's cap, else the reason."""
    cap = PERTURB_R_MAX.get(str(plane))
    if cap is None or r is None or float(r) <= cap + 1e-9:
        return None
    return (f"polar_r_m {float(r):g} m is beyond the {plane}-plane limit of {cap} m (caps: "
            + ", ".join(f"{k} {v}" for k, v in PERTURB_R_MAX.items())
            + " m -- beyond them a further pull toward the monkey runs out of reach)")


def _lookup(table: dict, name: str):
    for pattern, value in table.items():
        if fnmatch(name, pattern):
            return value
    return None


def is_live(experiment: str, name: str) -> bool:
    return any(fnmatch(name, p) for p in LIVE_PARAMS.get(experiment, []))


def limits_for(name: str, context: Optional[dict] = None) -> Optional[Tuple[float, float]]:
    """(min, max) for `name`. With `context` (the other values in effect), ranges that depend on another
    parameter are narrowed to it: the felt stiffness to what the SmartPad base can be softened to."""
    lim = _lookup(LIMITS, name)
    if lim is not None and name == _K and context and context.get(_K_BASE) is not None:
        base = float(context[_K_BASE])
        lo, hi = MIN_FELT_STIFFNESS, min(lim[1], base)
        if lo <= hi:
            return (round(lo, 6), round(hi, 6))
    return lim


def choices_for(name: str) -> Optional[List[str]]:
    return _lookup(CHOICES, name)


def step_for(name: str) -> Optional[float]:
    return _lookup(STEPS, name)


def check_stiffness(k, base) -> Optional[str]:
    """None if a felt stiffness k is reachable from the SmartPad base stiffness, else the reason."""
    if k is None or base is None:
        return None
    k, base = float(k), float(base)
    if k > base + 1e-9:
        return (f"resistance.stiffness_n_per_m {k:g} N/m is stiffer than the SmartPad base profile "
                f"({base:g} N/m, resistance.base_stiffness_n_per_m): ROS can only soften the cabinet spring")
    if k < MIN_FELT_STIFFNESS - 1e-9:
        return (f"resistance.stiffness_n_per_m {k:g} N/m is below {MIN_FELT_STIFFNESS:g} N/m, the least "
                f"spring that must always remain")
    return None


def check_value(name: str, value, context: Optional[dict] = None) -> Optional[str]:
    """None if `value` is acceptable for `name`, else a one-line reason.

    `context` holds the other parameter values in effect (name -> value). With it, parameters that are
    only valid together are checked together -- the perturbation's r against its plane's cap."""
    if name in CHOICE_GEOMETRY and context:
        problem = choice_geometry_problem({**context, name: value})
        if problem:
            return problem
    choices = choices_for(name)
    if choices is not None and str(value) not in choices:
        return f"{name} must be one of {choices}, got {value!r}"
    lim = limits_for(name)
    if lim is None or isinstance(value, (bool, str)):
        return _check_pair(name, value, context)
    lo, hi = lim
    values = list(value) if isinstance(value, (list, tuple)) else [value]
    if name.startswith(("visual_cue.colours.", "ring_cue.colours.")) and len(values) not in (3, 4):
        return f"{name} must be [r, g, b] or [r, g, b, w], got {len(values)} values"
    for v in values:
        if not (lo <= float(v) <= hi):
            return f"{name} must be within [{lo}, {hi}], got {v}"
    return _check_pair(name, value, context)


def check_pose(name: str, value) -> Optional[str]:
    """None if `value` is a usable joint pose [deg] for `name`, else the reason."""
    try:
        q = [float(v) for v in value]
    except (TypeError, ValueError):
        return f"{name} needs 7 joint angles in degrees"
    if len(q) != 7:
        return f"{name} needs 7 joint angles in degrees"
    for i, (a, lim) in enumerate(zip(q, IIWA7_LIMITS_DEG), start=1):
        if abs(a) > lim:
            return f"{name}: A{i} = {a:g} deg is outside the iiwa7 limit of ±{lim:.0f} deg"
    bend = max(abs(q[1]), abs(q[3]), abs(q[5]))
    if bend < STRAIGHT_BELOW_DEG:
        return (f"{name}: max(|A2|, |A4|, |A6|) = {bend:g} deg < {STRAIGHT_BELOW_DEG:.0f} deg -- a nearly "
                f"straight, singular arm. Bend A2, A4 or A6 further.")
    return None


def check_same_elbow(start, park) -> Optional[str]:
    """None if the start and park poses bend the elbow (A4) the same way, else the reason. Moving
    between poses whose A4 have opposite signs passes A4 = 0 -- a straight, singular elbow, where the
    cabinet's Cartesian impedance cannot follow -- on every park <-> start move."""
    if start is None or park is None:
        return None
    a4s, a4p = float(start[3]), float(park[3])
    if (a4s > 0) != (a4p > 0):
        return (f"the park pose's elbow (A4 = {a4p:g} deg) bends the other way from the start pose's "
                f"(A4 = {a4s:g} deg): every park <-> start move would pass a straight, singular elbow. "
                f"Use a park pose with A4 on the same side as the start.")
    return None


def _check_pair(name, value, context) -> Optional[str]:
    if name == "nsp_sync.cabinet_ip":
        try:
            ipaddress.IPv4Address(str(value).strip())
        except ValueError:
            return f"{name} must be an IPv4 address such as 172.31.1.147, got {value!r}"
        return None
    if name in _SYNC_OUTPUTS:
        for other in _SYNC_OUTPUTS:
            if other != name and context is not None and context.get(other) is not None \
                    and int(context[other]) == int(value):
                return f"{name} and {other} must be different outputs (both {value}): each event has its own line"
        return None
    if name in ("haptic.min_pct", "haptic.max_pct", "haptic.ramp_start_m", "haptic.ramp_full_m") and context:
        ctx = {**context, name: value}
        if all(k in ctx for k in ("haptic.min_pct", "haptic.max_pct")) and \
                float(ctx["haptic.min_pct"]) > float(ctx["haptic.max_pct"]):
            return f"haptic.min_pct ({ctx['haptic.min_pct']}) must not exceed haptic.max_pct ({ctx['haptic.max_pct']})"
        if all(k in ctx for k in ("haptic.ramp_start_m", "haptic.ramp_full_m")) and \
                float(ctx["haptic.ramp_full_m"]) >= float(ctx["haptic.ramp_start_m"]):
            return (f"haptic.ramp_full_m ({ctx['haptic.ramp_full_m']}) must be closer to the goal than "
                    f"haptic.ramp_start_m ({ctx['haptic.ramp_start_m']})")
        return None
    if name.startswith("random_start.positions."):
        # [a, b] metres in random_start.plane; reach and joint margins are checked when the run solves them
        if not isinstance(value, (list, tuple)) or len(value) != 2:
            return f"{name} needs [a, b] in metres from the start pose"
        return None
    if name.endswith("target_joint_position"):
        problem = check_pose(name, value)
        if problem or context is None or name not in (_START, _PARK):
            return problem
        start, park = (value, context.get(_PARK)) if name == _START else (context.get(_START), value)
        return check_same_elbow(start, park)
    if name in (_K, _K_BASE):
        ctx = context or {}
        k = value if name == _K else ctx.get(_K)
        base = value if name == _K_BASE else ctx.get(_K_BASE)
        return check_stiffness(k, base)
    for dmin, dmax in _DELAY_RANGES:
        if name in (dmin, dmax):
            ctx = context or {}
            lo = value if name == dmin else ctx.get(dmin)
            hi = value if name == dmax else ctx.get(dmax)
            if lo is not None and hi is not None and float(lo) > float(hi):
                return f"{dmin} ({lo}) must not exceed {dmax} ({hi})"
            return None
    if context is None or name not in (_R, _PLANE):
        return None
    r = value if name == _R else context.get(_R)
    plane = value if name == _PLANE else context.get(_PLANE)
    return check_perturbation(r, plane)
