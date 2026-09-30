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

# Move speed limits of every joint-space move (start, recover, the perturbation). Ruckig is rebuilt
# from them at each move's start, so a change simply takes effect on the next move.
_SPEED = ["*.move_to_pos_v_max", "*.move_to_pos_a_max", "*.move_to_pos_j_max"]

# The start pose, and the recover pose that must equal it. Live ONLY where the joint controller alone
# drives the arm (apple pluck, perturb). Restricted plane and maze also hand the arm to the CLIK, whose
# redundancy posture must equal the start pose and is read once when the controller is configured --
# and the maze's whole geometry hangs off the start. There the pose stays per-run.
_POSE = ["move_to_start.target_joint_position", "move_to_start_recover.target_joint_position"]

LIVE_PARAMS: Dict[str, List[str]] = {
    "apple_pluck": _COMMON + _PULL + _SPEED + _POSE,
    "perturb": _COMMON + _PULL + _SPEED + _POSE + [
        "apple_pluck_impedance_control_displacement.baseline_settle_sec",
        # The perturbation itself -- the manipulated variable, so it is the one most worth varying
        # trial to trial. Recorded per trial in the sidecar's `perturbation` block.
        "perturb_start.polar_r_m",
        "perturb_start.polar_theta_deg",
        "perturb_start.polar_plane",
        "perturb_start.start_delay_sec",
    ],
    "restricted_plane": _COMMON + _PULL + _SPEED,
    "maze": _COMMON + _SPEED + [
        "timeout_sec",
    ],
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
    "timeout_sec": (5.0, 600.0),
    # Speed limits. v_max in deg/s per joint, kept under the slowest joint's rating (98 deg/s, A1/A2).
    # a_max / j_max up to the fastest perturbation measured (20 rad/s^2, 150 rad/s^3: 0.23 s for 5 cm).
    "*.move_to_pos_v_max": (5.0, 90.0),
    "*.move_to_pos_a_max": (0.5, 20.0),
    "*.move_to_pos_j_max": (1.0, 150.0),
}

# Joint poses: 7 angles in degrees, inside the iiwa7 limits (lbr joint_limits.yaml, which is 1 deg
# stricter than the hardware), and not a nearly straight arm -- max(|A2|, |A4|, |A6|) below this is
# singular (the same test, and value, as STRAIGHT_BELOW_DEG in LbrImpedanceControlServer.java).
IIWA7_LIMITS_DEG = [169.0, 119.0, 169.0, 119.0, 169.0, 119.0, 174.0]
STRAIGHT_BELOW_DEG = 12.0
_START, _RECOVER = _POSE

# Parameters with a fixed set of values. The dashboard renders these as a drop-down.
CHOICES: Dict[str, List[str]] = {
    "perturb_start.polar_plane": ["frontal", "horizontal", "sagittal"],
    "*.cartesian_axis": ["norm", "x", "y", "z"],
    "*.restricted_axis": ["x", "y", "z"],
    "*.corridor_frame": ["relative", "absolute"],
    "*.polar_plane": ["frontal", "horizontal", "sagittal"],
}


# Largest perturbation per plane [m]. From the apple-pluck start pose, position-only IK (the same DLS
# PerturbInitialPosition uses) keeps EVERY direction in the plane at least 10 deg from every joint
# limit, with >= 19 deg left after a further 15 cm pull toward the monkey and the smallest singular
# value of the 6D Jacobian >= 0.10. The next step out breaks the 10 deg margin in each plane
# (frontal 0.175 -> 9.0, horizontal 0.20 -> 9.7, sagittal 0.125 -> 7.7; sagittal reaches the limit
# at 0.20). Position-only IK lets the flange tilt: up to ~22 / 19 / 14 deg at these caps.
PERTURB_R_MAX: Dict[str, float] = {"frontal": 0.15, "horizontal": 0.175, "sagittal": 0.10}
_R, _PLANE = "perturb_start.polar_r_m", "perturb_start.polar_plane"
PAIRED = (_R, _PLANE, "move_to_start.target_joint_position",
          "move_to_start_recover.target_joint_position")   # checked together: see check_value(context=...)


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


def limits_for(name: str) -> Optional[Tuple[float, float]]:
    return _lookup(LIMITS, name)


def choices_for(name: str) -> Optional[List[str]]:
    return _lookup(CHOICES, name)


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
    if name.startswith("visual_cue.colours.") and len(values) not in (3, 4):
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


def _check_pair(name, value, context) -> Optional[str]:
    if name.endswith("target_joint_position"):
        problem = check_pose(name, value)
        if problem or context is None or name not in (_START, _RECOVER):
            return problem
        # The next trial starts where the last one recovered to, so the two must stay equal. Set them
        # in the same call (the dashboard does); one on its own is refused rather than half-applied.
        other = _RECOVER if name == _START else _START
        if other in context and [float(v) for v in context[other]] != [float(v) for v in value]:
            return (f"{name} must equal {other} -- set both together, in one call "
                    f"(the dashboard does this for you)")
        return None
    if context is None or name not in (_R, _PLANE):
        return None
    r = value if name == _R else context.get(_R)
    plane = value if name == _PLANE else context.get(_PLANE)
    return check_perturbation(r, plane)
