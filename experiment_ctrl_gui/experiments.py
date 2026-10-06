"""The experiments the dashboard can run -- four experiments and three pre-training tasks, shown
under two tabs -- and how their parameters are classified.

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
    "Null-space (elbow) stiffness": "30 (Standard)",
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
    group: str = "experiments"     # dashboard tab: "experiments" or "pretraining"

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

# Pre-training: shorter tasks that get a subject used to the arm before the real experiments. All
# three start at the maze start (tool along +X) and run on the CLIK like the maze.
_MAZE_LAUNCH = {"ctrl": "lbr_joint_position_command_controller",
                "extra_inactive_ctrl": "kuka_clik_controller",
                "clik_nullspace_cfg": "config/clik_nullspace_maze.yaml"}
EXPERIMENTS += [
    Experiment(
        key="free_move", label="Free Move", group="pretraining",
        summary="Admittance: the arm goes wherever it is pushed and holds where it is let go. No goal, no boundary.",
        launch_file="iiwa7_pretrain_free_move.launch.py",
        params_yaml="pretrain_free_move.yaml", node="free_move_orchestrator",
        stiffness_profile="Rail guide (uniform 1000)",
        ros_controller="kuka_clik_controller",
        run_name="iiwa7_pretrain_free_move",
        launch_args=dict(_MAZE_LAUNCH), clik_nullspace="clik_nullspace_maze.yaml",
    ),
    Experiment(
        key="move_vertical", label="Move Vertical", group="pretraining",
        summary="One vertical rail: up or down to the threshold (ring green → red), then back to start.",
        launch_file="iiwa7_pretrain_move_vertical.launch.py",
        params_yaml="pretrain_move_vertical.yaml", node="rail_training_orchestrator",
        stiffness_profile="Maze walls + easy guiding (rot 120)",
        ros_controller="kuka_clik_controller",
        run_name="iiwa7_pretrain_move_vertical",
        launch_args=dict(_MAZE_LAUNCH), clik_nullspace="clik_nullspace_maze.yaml",
    ),
    Experiment(
        key="move_horizontal", label="Move Horizontal", group="pretraining",
        summary="One horizontal rail: left or right to the threshold (ring green → red), then back to start.",
        launch_file="iiwa7_pretrain_move_horizontal.launch.py",
        params_yaml="pretrain_move_horizontal.yaml", node="rail_training_orchestrator",
        stiffness_profile="Maze walls + easy guiding (rot 120)",
        ros_controller="kuka_clik_controller",
        run_name="iiwa7_pretrain_move_horizontal",
        launch_args=dict(_MAZE_LAUNCH), clik_nullspace="clik_nullspace_maze.yaml",
    ),
]
BY_KEY = {e.key: e for e in EXPERIMENTS}
GROUPS = [("experiments", "Experiments"), ("pretraining", "Pre-training")]

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
    "virtual_fixtures.maze.*": "Maze rail. Run check_maze.py on the edited YAML (runs/…) to confirm reachability.",
    "checkpoint_monitor.checkpoint_*": "Checkpoints must sit on the rails. Arrays stay the same length.",
    "checkpoint_monitor.goal_[xyz]": "The goal must sit on a rail.",
    "checkpoint_monitor.relative_to_start": "Must match the rails' corridor_frame (relative = true).",
    "virtual_fixtures.*.type": "Geometry class of this profile. Normally chosen with virtual_fixture_profile instead.",
    "visual_cue.remote_board": "The board's own access point always serves at 192.168.4.1.",
    "virtual_fixtures.*_rail.*": "The rail. Keep travel_task.threshold_m short of its ends; up is limited to ~+0.12 m by reach.",
    "travel_task.axis": "Must be one of the rail's own axes (vertical: z, horizontal: y).",
    "perturb_start.polar_r_m": ("The cap depends on the plane: frontal 0.15, horizontal 0.175, sagittal 0.10 m "
                                "(every direction >= 10 deg from a joint limit, from the default start pose). "
                                "The flange tilts more as r grows -- up to ~22 deg at the frontal cap."),
}

# Parameters kept equal to each other: an edit to one sets them all. The YAML notes say "change it in
# BOTH blocks"; the dashboard does it for you. (The CLIK posture follows too -- see params.py.)
LINKED: List[List[str]] = [
    ["move_to_start.target_joint_position", "move_to_start_recover.target_joint_position"],
]
START_POSE = "move_to_start.target_joint_position"

# Pose checks (joint limits, the straight-arm rule) live in live_params.check_pose, shared with the
# orchestrators; these are re-exported for anything that wants the numbers.
IIWA7_LIMITS_DEG = live_params.IIWA7_LIMITS_DEG
STRAIGHT_BELOW_DEG = live_params.STRAIGHT_BELOW_DEG


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


def choices(exp: Experiment, name: str, flat: Dict[str, object]) -> Optional[List[str]]:
    if name == "virtual_fixture_profile":
        # Every block under virtual_fixtures that declares a `type` is a selectable profile.
        return sorted({k.split(".")[1] for k in flat
                       if k.startswith("virtual_fixtures.") and k.endswith(".type")})
    return live_params.choices_for(name)
