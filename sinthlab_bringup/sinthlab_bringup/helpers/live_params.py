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

LIVE_PARAMS: Dict[str, List[str]] = {
    "apple_pluck": _COMMON + _PULL,
    "perturb": _COMMON + _PULL + [
        "apple_pluck_impedance_control_displacement.baseline_settle_sec",
        # The perturbation itself -- the manipulated variable, so it is the one most worth varying
        # trial to trial. Recorded per trial in the sidecar's `perturbation` block.
        "perturb_start.polar_r_m",
        "perturb_start.polar_theta_deg",
        "perturb_start.polar_plane",
        "perturb_start.start_delay_sec",
    ],
    "restricted_plane": _COMMON + _PULL,
    "maze": _COMMON + [
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
    "perturb_start.polar_r_m": (0.0, 0.10),       # IK-checked out to 0.10 m (perturb YAML)
    "perturb_start.polar_theta_deg": (-360.0, 360.0),
    "perturb_start.start_delay_sec": (0.0, 10.0),
    "timeout_sec": (5.0, 600.0),
}

# Parameters with a fixed set of values. The dashboard renders these as a drop-down.
CHOICES: Dict[str, List[str]] = {
    "perturb_start.polar_plane": ["frontal", "horizontal", "sagittal"],
    "*.cartesian_axis": ["norm", "x", "y", "z"],
    "*.restricted_axis": ["x", "y", "z"],
    "*.corridor_frame": ["relative", "absolute"],
    "*.polar_plane": ["frontal", "horizontal", "sagittal"],
}


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


def check_value(name: str, value) -> Optional[str]:
    """None if `value` is acceptable for `name`, else a one-line reason."""
    choices = choices_for(name)
    if choices is not None and str(value) not in choices:
        return f"{name} must be one of {choices}, got {value!r}"
    lim = limits_for(name)
    if lim is None or isinstance(value, (bool, str)):
        return None
    lo, hi = lim
    values = list(value) if isinstance(value, (list, tuple)) else [value]
    if name.startswith("visual_cue.colours.") and len(values) not in (3, 4):
        return f"{name} must be [r, g, b] or [r, g, b, w], got {len(values)} values"
    for v in values:
        if not (lo <= float(v) <= hi):
            return f"{name} must be within [{lo}, {hi}], got {v}"
    return None
