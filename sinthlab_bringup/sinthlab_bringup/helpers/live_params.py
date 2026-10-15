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

from fnmatch import fnmatch
from typing import Dict, List, Optional, Tuple

# Cue and sync switches every experiment has.
_COMMON = [
    "quiet_window_sec",            # pause at the start pose before the go cue
    "audio_cue.enabled",           # master switch for every beep
    "audio_cue_*.frequency_hz",
    "audio_cue_*.duration_ms",
    "visual_cue.enabled",          # master switch for the NeoPixel ring
    "visual_cue.remote_test_trigger",
    "visual_cue.colours.*",
    "nsp_sync.enabled",            # event codes to the Blackrock NSP (needs the DIO; see README §7)
]

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
# There the pose stays per-run. The park pose (move_to_park) is per-run everywhere.
_POSE = ["move_to_start.target_joint_position"]

# Pre-training: the out-and-back task and the quarter-ring state colours.
_TASK = ["travel_task.threshold_m", "travel_task.direction", "travel_task.require_return",
         "travel_task.return_tolerance_m"]
_RING = ["ring_cue.quarters", "ring_cue.colours.*"]
_RAIL = _COMMON + _SPEED + _TASK + _RING + ["timeout_sec", "virtual_fixtures.rail_lead_sec"]

# The felt stiffness of the pull (apple pluck, perturb). ROS cannot change the cabinet's stiffness; it
# softens it by letting the equilibrium follow part of the pull -- see actions/resistance_softener.py.
_RESIST = ["resistance.stiffness_n_per_m"]

LIVE_PARAMS: Dict[str, List[str]] = {
    # Every paradigm. perturb_start.* act only in perturb / perturb_bump, grasp.* only in perturb_bump.
    "apple_pluck": _COMMON + _PULL + _SPEED + _POSE + _RESIST + [
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
        "grasp.timeout_sec",
        "grasp.delay_mode",
        "grasp.delay_sec",
        "grasp.delay_min_sec",
        "grasp.delay_max_sec",
    ],
    "restricted_plane": _COMMON + _PULL + _SPEED,
    "maze": _COMMON + _SPEED + [
        "timeout_sec",
        # How easy moving along a rail feels -- see rail_lead_sec in maze_params.yaml. Walls unchanged.
        "virtual_fixtures.rail_lead_sec",
    ],
    # Pre-training. The rail geometry stays per-run, like the maze's rails.
    "free_move": _COMMON + _SPEED + _RING + [
        "session_sec",
        # The feel of the admittance -- see pretrain_free_move.yaml.
        "free_move.damping_ns_per_m",
        "free_move.deadband_n",
        "free_move.max_speed_mps",
        "free_move.accel_limit_mps2",
        "free_move.force_filter_tau_sec",
        "free_move.tare_sec",
    ],
    "move_vertical": _RAIL,
    "move_horizontal": _RAIL,
}

# Bounds checked on every live change, (min, max) inclusive. A colour is checked per element.
LIMITS: Dict[str, Tuple[float, float]] = {
    "quiet_window_sec": (0.0, 60.0),
    "audio_cue_*.frequency_hz": (37, 32767),      # [console]::Beep refuses anything outside this
    "audio_cue_*.duration_ms": (10, 10000),
    "visual_cue.colours.*": (0, 255),
    "*.cartesian_displacement_threshold_m": (0.005, 0.5),
    "*.force_release_shutdown_delay_sec": (0.0, 10.0),
    "*.baseline_settle_sec": (0.0, 10.0),
    "perturb_start.polar_r_m": (0.0, 0.175),      # overall; the real cap depends on the plane, below
    "perturb_start.polar_theta_deg": (-360.0, 360.0),
    "perturb_start.start_delay_sec": (0.0, 10.0),
    "perturb_start.start_delay_min_sec": (0.0, 10.0),
    "perturb_start.start_delay_max_sec": (0.0, 10.0),
    "grasp.threshold_n": (0.5, 30.0),
    "grasp.hold_sec": (0.0, 0.5),
    "grasp.timeout_sec": (1.0, 120.0),
    "grasp.delay_sec": (0.0, 10.0),
    "grasp.delay_min_sec": (0.0, 10.0),
    "grasp.delay_max_sec": (0.0, 10.0),
    "timeout_sec": (5.0, 600.0),
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
    # Pre-training. The threshold must also stay short of the rail end; the orchestrator warns if not.
    "travel_task.threshold_m": (0.01, 0.30),
    "travel_task.return_tolerance_m": (0.005, 0.10),
    "ring_cue.colours.*": (0, 255),
    "ring_cue.quarters": (0, 3),
    # Admittance feel. The speed cap stays under the free-move safety monitor's 0.7 m/s.
    "session_sec": (0.0, 3600.0),          # 0 = until Pause / Stop after trial
    "free_move.damping_ns_per_m": (3.0, 200.0),
    "free_move.deadband_n": (0.0, 15.0),
    "free_move.max_speed_mps": (0.02, 0.5),
    "free_move.accel_limit_mps2": (0.0, 10.0),
    "free_move.force_filter_tau_sec": (0.0, 0.5),
    "free_move.tare_sec": (0.2, 5.0),
}

# Joint poses: 7 angles in degrees, inside the iiwa7 limits (lbr joint_limits.yaml, which is 1 deg
# stricter than the hardware), and not a nearly straight arm -- max(|A2|, |A4|, |A6|) below this is
# singular (checked against the Jacobian: the same as "smallest singular value < 0.05").
IIWA7_LIMITS_DEG = [169.0, 119.0, 169.0, 119.0, 169.0, 119.0, 174.0]
STRAIGHT_BELOW_DEG = 12.0
_START, _PARK = "move_to_start.target_joint_position", "move_to_park.target_joint_position"

# Paradigm-only parameters: in an experiment with a `paradigm` (apple pluck), these act -- and the
# dashboard shows them -- only in those paradigms. The first matching pattern wins.
PARADIGM_ONLY: Dict[str, Tuple[str, ...]] = {
    "perturb_start.baseline_settle_sec": ("perturb",),
    "perturb_start.start_delay_*": ("perturb",),
    "perturb_start.*": ("perturb", "perturb_bump"),
    "grasp.*": ("perturb_bump",),
    "random_start.*": ("random_start",),
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
}

# Live parameters applied the moment they are set, instead of at the next trial boundary. Only for
# what cannot change the trial's outcome or its measurement: how fast the arm is driven -- from the
# next move on, even within the current trial. Each change is logged in the trial's sidecar
# (`live_changes`) with when it happened.
IMMEDIATE: List[str] = ["*.speed_scale"]


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
    "paradigm": ["pluck", "perturb", "perturb_bump", "random_start"],
    "perturb_start.start_delay_mode": ["fixed", "random"],
    "grasp.delay_mode": ["fixed", "random"],
    "travel_task.direction": ["both", "positive", "negative"],
    "travel_task.axis": ["x", "y", "z", "norm"],
}


# Largest perturbation per plane [m]. From the apple-pluck start pose, position-only IK (the same DLS
# PerturbInitialPosition uses) keeps EVERY direction in the plane at least 10 deg from every joint
# limit, with >= 19 deg left after a further 15 cm pull toward the monkey and the smallest singular
# value of the 6D Jacobian >= 0.10. The next step out breaks the 10 deg margin in each plane
# (frontal 0.175 -> 9.0, horizontal 0.20 -> 9.7, sagittal 0.125 -> 7.7; sagittal reaches the limit
# at 0.20). Position-only IK lets the flange tilt: up to ~22 / 19 / 14 deg at these caps.
PERTURB_R_MAX: Dict[str, float] = {"frontal": 0.15, "horizontal": 0.175, "sagittal": 0.10}
_R, _PLANE = "perturb_start.polar_r_m", "perturb_start.polar_plane"
# Delay ranges (min, max): perturb's after the go cue, perturb_bump's after the grasp.
_DELAY_RANGES = (("perturb_start.start_delay_min_sec", "perturb_start.start_delay_max_sec"),
                 ("grasp.delay_min_sec", "grasp.delay_max_sec"))
PAIRED = (_R, _PLANE, _START, _PARK, _K, _K_BASE) + sum(_DELAY_RANGES, ())   # checked together: see check_value(context=...)


def check_perturbation(r, plane) -> Optional[str]:
    """None if a perturbation of r metres in `plane` is within that plane's cap, else the reason."""
    cap = PERTURB_R_MAX.get(str(plane))
    if cap is None or r is None or float(r) <= cap + 1e-9:
        return None
    return (f"polar_r_m {float(r):g} m is beyond the {plane}-plane limit of {cap} m (caps: "
            + ", ".join(f"{k} {v}" for k, v in PERTURB_R_MAX.items())
            + " m -- beyond them some directions come within 10 deg of a joint limit)")


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
    if name.startswith("random_start.positions."):
        return check_pose(name, value)
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
