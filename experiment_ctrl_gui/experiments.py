"""The four experiments the dashboard can run, and how their parameters are classified.

Three tiers, shown as three tabs in the dashboard:

  LIVE     changeable while the experiment runs; applied from the next trial. Decided by
           sinthlab_bringup/helpers/live_params.py -- the orchestrator enforces the same list, so the
           dashboard can never offer a live change that the robot side would ignore.
  PER-RUN  everything else in the experiment YAML -- poses and maze geometry included: edit before
           Start, then locked for the whole run (the orchestrator reads them once, at start-up).
           Sensitive ones carry a CAUTION note saying what else depends on them.
  FIXED    the configuration every experiment shares and that is not in the experiment YAML: SmartPad
           (FRI) selections, launch arguments, iiwa7_hardware_controllers.yaml, the CLIK posture.
"""
from __future__ import annotations

import sys
from dataclasses import dataclass, field
from fnmatch import fnmatch
from pathlib import Path
from typing import Dict, List, Optional

REPO = Path(__file__).resolve().parent.parent
BRINGUP = REPO / "sinthlab_bringup"
CONFIG = BRINGUP / "config"
ANALYSIS = REPO / "analysis"

# The live list is shared with the orchestrators. It is plain Python (no ROS import), so load it
# from the source tree whether or not ROS is sourced.
sys.path.insert(0, str(BRINGUP))
from sinthlab_bringup.helpers import live_params  # noqa: E402

_FRI_COMMON = {
    "SmartPad application": "LbrImpedanceControlServer",
    "FRI send period [ms]": "10",
    "Remote IP address": "172.31.1.148 (this ROS computer)",
    "Damping ratio (D0)": "0.7 (Standard)",
}


@dataclass
class Experiment:
    key: str                       # also the `experiment` name the orchestrator reports
    label: str
    summary: str
    launch_file: str
    params_yaml: str               # file name under sinthlab_bringup/config
    node: str                      # orchestrator node name (namespace /<robot_name>)
    stiffness_profile: str         # SmartPad "Cartesian stiffness (K diagonal)" selection
    ros_controller: str
    run_name: Optional[str]        # recording folder prefix: analysis/expt_<run_name>_<ts>/
    launch_args: Dict[str, str] = field(default_factory=dict)
    clik_nullspace: Optional[str] = None

    def smartpad(self) -> Dict[str, str]:
        out = dict(_FRI_COMMON)
        out["Cartesian stiffness (K diagonal)"] = self.stiffness_profile
        return out


EXPERIMENTS: List[Experiment] = [
    Experiment(
        key="apple_pluck", label="Apple Pluck",
        summary="Pull the apple 0.1 m in any direction from a fixed start; snap cue, hold, recover.",
        launch_file="iiwa7_apple_pluck_impedance_control.launch.py",
        params_yaml="apple_pluck_impedance.yaml", node="apple_pluck_orchestrator",
        stiffness_profile="Uniform Medium (Apple Pluck)",
        ros_controller="lbr_joint_position_command_controller",
        run_name="iiwa7_apple_pluck_impedance_control",
        launch_args={"ctrl": "lbr_joint_position_command_controller"},
    ),
    Experiment(
        key="perturb", label="Apple Pluck Perturb",
        summary="As Apple Pluck, but the apple is displaced by a polar (r, θ) offset after the go cue.",
        launch_file="iiwa7_apple_pluck_impedance_perturb.launch.py",
        params_yaml="apple_pluck_impedance_perturb.yaml", node="perturb_orchestrator",
        stiffness_profile="Uniform Medium (Apple Pluck)",
        ros_controller="lbr_joint_position_command_controller",
        run_name="iiwa7_apple_pluck_impedance_perturb",
        launch_args={"ctrl": "lbr_joint_position_command_controller"},
    ),
    Experiment(
        key="restricted_plane", label="Restricted Plane",
        summary="Virtual fixture (sine rail by default): free along the pull axis, walled elsewhere.",
        launch_file="iiwa7_move_restricted_plane.launch.py",
        params_yaml="virtual_fixtures_params.yaml", node="restricted_plane_orchestrator",
        stiffness_profile="Rail guide (uniform 1000)",
        ros_controller="kuka_clik_controller",
        run_name=None,   # no TrialRecorder: the fixture action writes its own trajectory CSV
        launch_args={"ctrl": "lbr_joint_position_command_controller",
                     "extra_inactive_ctrl": "kuka_clik_controller"},
        clik_nullspace="clik_nullspace_default.yaml",
    ),
    Experiment(
        key="maze", label="Maze",
        summary="Drive the arm along vertical-plane rails; reward at the forks, goal or timeout ends it.",
        launch_file="iiwa7_maze.launch.py",
        params_yaml="maze_params.yaml", node="maze_orchestrator",
        stiffness_profile="Maze walls + easy guiding (rot 120)",
        ros_controller="kuka_clik_controller",
        run_name="iiwa7_maze",
        launch_args={"ctrl": "lbr_joint_position_command_controller",
                     "extra_inactive_ctrl": "kuka_clik_controller",
                     "clik_nullspace_cfg": "config/clik_nullspace_maze.yaml"},
        clik_nullspace="clik_nullspace_maze.yaml",
    ),
]
BY_KEY = {e.key: e for e in EXPERIMENTS}

# Launch arguments every experiment shares (experiment_base.launch.py defaults).
COMMON_LAUNCH_ARGS = {"robot_type": "iiwa7", "robot_name": "lbr", "startup_delay": "0.0"}

# Per-run parameters that are editable but easy to get wrong: shown with this note, never locked.
CAUTION: Dict[str, str] = {
    "*update_rate": "Loop rate of the action, tied to the controller and FRI timing. Rarely a tuning knob.",
    "*.base_frame": "TF frame name. Must exist in the robot description.",
    "*.ee_frame": "TF frame name. Must exist in the robot description.",
    "state_topic": "Topic name. Must match what lbr_state_broadcaster publishes.",
    "base_link": "URDF link name. Must exist in the robot description.",
    "end_effector_link": "URDF link name. Must exist in the robot description.",
    "move_to_start.target_joint_position": (
        "Start pose, joint angles in degrees. Kept equal to move_to_start_recover (edited together), "
        "and for the CLIK experiments the CLIK redundancy posture follows it. Checked against the "
        "iiwa7 joint limits; reachability of anything relative to it (the maze) is not checked here."),
    "move_to_start_recover.target_joint_position": (
        "Recover pose. Kept equal to move_to_start (edited together) -- the next trial starts here."),
    "move_to_prestart.target_joint_position": "Maze pre-start waypoint, degrees. Only used from a near-straight arm.",
    "virtual_fixtures.maze.*": "Maze rail. Run check_maze.py on the edited YAML (runs/…) to confirm reachability.",
    "checkpoint_monitor.checkpoint_*": "Checkpoints must sit on the rails. Arrays stay the same length.",
    "checkpoint_monitor.goal_[xyz]": "The goal must sit on a rail.",
    "checkpoint_monitor.relative_to_start": "Must match the rails' corridor_frame (relative = true).",
    "virtual_fixtures.*.type": "Geometry class of this profile. Normally chosen with virtual_fixture_profile instead.",
    "visual_cue.remote_board": "The board's own access point always serves at 192.168.4.1.",
}

# Parameters kept equal to each other: an edit to one sets them all. The YAML notes say "change it in
# BOTH blocks"; the dashboard does it for you. (The CLIK posture follows too -- see params.py.)
LINKED: List[List[str]] = [
    ["move_to_start.target_joint_position", "move_to_start_recover.target_joint_position"],
]
START_POSE = "move_to_start.target_joint_position"

# iiwa7 joint limits [deg], A1..A7, from the KUKA LBR iiwa 7 R800 spec.
IIWA7_LIMITS_DEG = [170.0, 120.0, 170.0, 120.0, 170.0, 120.0, 175.0]
STRAIGHT_BELOW_DEG = 12.0


def tier(exp: Experiment, name: str) -> str:
    return "live" if live_params.is_live(exp.key, name) else "run"


def caution(name: str) -> Optional[str]:
    for pattern, note in CAUTION.items():
        if fnmatch(name, pattern):
            return note
    return None


def linked(name: str) -> List[str]:
    for group in LINKED:
        if name in group:
            return group
    return [name]


def check_extra(name: str, value) -> Optional[str]:
    """Checks beyond live_params.check_value that only the dashboard makes."""
    if name.endswith("target_joint_position"):
        if not isinstance(value, list) or len(value) != 7:
            return f"{name} needs 7 joint angles in degrees"
        for i, (q, lim) in enumerate(zip(value, IIWA7_LIMITS_DEG), start=1):
            if abs(q) > lim:
                return f"{name}: A{i} = {q} deg is outside the iiwa7 limit of ±{lim:.0f} deg"
        # A nearly straight arm is singular (mechanical zero: smallest singular value 0.0), and a
        # Cartesian-impedance move from or around it does not reliably arrive. Same test as the maze's
        # pre-start guard (maze_params.yaml, extended_if_bend_below_deg), validated against the Jacobian.
        bend = max(abs(value[1]), abs(value[3]), abs(value[5]))
        if name != "move_to_prestart.target_joint_position" and bend < STRAIGHT_BELOW_DEG:
            return (f"{name}: max(|A2|, |A4|, |A6|) = {bend} deg < {STRAIGHT_BELOW_DEG:.0f} deg -- a nearly "
                    f"straight, singular arm. Bend A2, A4 or A6 further.")
    return None


def choices(exp: Experiment, name: str, flat: Dict[str, object]) -> Optional[List[str]]:
    if name == "virtual_fixture_profile":
        # Every block under virtual_fixtures that declares a `type` is a selectable profile.
        return sorted({k.split(".")[1] for k in flat
                       if k.startswith("virtual_fixtures.") and k.endswith(".type")})
    return live_params.choices_for(name)
