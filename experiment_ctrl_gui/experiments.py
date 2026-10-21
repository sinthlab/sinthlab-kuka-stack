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
from typing import Dict, List, Optional, Tuple

REPO = Path(__file__).resolve().parent.parent
BRINGUP = REPO / "sinthlab_bringup"
CONFIG = BRINGUP / "config"
ANALYSIS = REPO / "analysis"

# The live list is shared with the orchestrators. It is plain Python (no ROS import), so load it
# from the source tree whether or not ROS is sourced.
sys.path.insert(0, str(BRINGUP))
from sinthlab_bringup.helpers import live_params  # noqa: E402



# The SmartPad app's stiffness profiles {X, Y, Z, A, B, C} -- a copy of stiffness_profiles_ /
# stiffness_vals_ in sunrise_controller_code/LbrImpedanceControlServer.java, kept in step with it. Every
# experiment's stiffness_profile must be one of these (checked at import).
SMARTPAD_PROFILES: Dict[str, List[float]] = {
    "Uniform 100": [100.0, 100.0, 100.0, 300.0, 300.0, 300.0],
    "Uniform 400": [400.0, 400.0, 400.0, 300.0, 300.0, 300.0],
    "Uniform 1000": [1000.0, 1000.0, 1000.0, 300.0, 300.0, 300.0],
    "Z lock 2500, X/Y 1000, rot 120": [1000.0, 1000.0, 2500.0, 120.0, 120.0, 120.0],
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
    run_name: str                  # recording folder prefix: analysis/expt_<run_name>_<ts>/
    launch_args: Dict[str, str] = field(default_factory=dict)
    clik_nullspace: Optional[str] = None
    group: str = "experiments"     # dashboard tab: "experiments" or "pretraining"
    # Variants of one experiment, chosen per run by its `paradigm` parameter and shown as nested tabs
    # on its card: [(value, label), ...]. Parameters that belong to one paradigm only
    # (live_params.PARADIGM_ONLY) are shown only under it.
    paradigms: List[Tuple[str, str]] = field(default_factory=list)

    def smartpad(self) -> Dict[str, str]:
        """The SmartPad selections, in the order LbrImpedanceControlServer asks for them."""
        return {
            "SmartPad application": "LbrImpedanceControlServer",
            "FRI send period [ms]": "10",
            "Remote IP address": "172.31.1.148 (this ROS computer)",
            "Cartesian stiffness (K diagonal)": self.stiffness_profile,
            "Damping ratio (D0)": "0.7 (Standard)",
            "Null-space (elbow) stiffness": "30 (Standard)",
            "Then, automatically": ("arm moves to mechanical zero, then the ready pose; "
                                    "keep the space above the arm clear"),
        }


EXPERIMENTS: List[Experiment] = [
    Experiment(
        key="apple_pluck", label="Apple Pluck",
        summary="Pull the apple 0.1 m in any direction; snap cue, hold, recover. Normal: from a fixed start. "
                "Perturb: the apple is first displaced by a polar (r, θ) offset after the go cue. "
                "Perturb bump: the same perturbation, triggered when the monkey grasps the apple. "
                "Random start: each trial starts at one of 9 positions (origin + 8 compass directions).",
        paradigms=[("pluck", "Pluck normal"), ("perturb", "Pluck perturb"),
                   ("perturb_bump", "Pluck perturb bump"), ("random_start", "Pluck random start")],
        launch_file="iiwa7_apple_pluck_impedance_control.launch.py",
        params_yaml="apple_pluck_impedance.yaml", node="apple_pluck_orchestrator",
        stiffness_profile="Uniform 100",
        ros_controller="lbr_joint_position_command_controller",
        run_name="iiwa7_apple_pluck_impedance_control",
        launch_args={"ctrl": "lbr_joint_position_command_controller"},
    ),
    Experiment(
        key="restricted_plane", label="Restricted Plane",
        summary="Virtual fixture (sine rail by default): free along the pull axis, walled elsewhere.",
        launch_file="iiwa7_move_restricted_plane.launch.py",
        params_yaml="virtual_fixtures_params.yaml", node="restricted_plane_orchestrator",
        stiffness_profile="Uniform 1000",
        ros_controller="kuka_clik_controller",
        run_name="iiwa7_move_restricted_plane",
        launch_args={"ctrl": "lbr_joint_position_command_controller",
                     "extra_inactive_ctrl": "kuka_clik_controller"},
        clik_nullspace="clik_nullspace_default.yaml",
    ),
    Experiment(
        key="maze", label="Maze",
        summary="Drive the arm along vertical-plane rails; reward at the forks, goal or timeout ends it.",
        launch_file="iiwa7_maze.launch.py",
        params_yaml="maze_params.yaml", node="maze_orchestrator",
        stiffness_profile="Z lock 2500, X/Y 1000, rot 120",
        ros_controller="kuka_clik_controller",
        run_name="iiwa7_maze",
        launch_args={"ctrl": "lbr_joint_position_command_controller",
                     "extra_inactive_ctrl": "kuka_clik_controller",
                     "clik_nullspace_cfg": "config/clik_nullspace_maze.yaml"},
        clik_nullspace="clik_nullspace_maze.yaml",
    ),
]

# Pre-training: shorter tasks that get a subject used to the arm before the real experiments. All
# three start 11 cm below and 5 cm in front of the maze start (tool along +X; the maze's lowest row is
# at their start height) and run on the CLIK like the maze.
_MAZE_LAUNCH = {"ctrl": "lbr_joint_position_command_controller",
                "extra_inactive_ctrl": "kuka_clik_controller",
                "clik_nullspace_cfg": "config/clik_nullspace_pretrain.yaml"}
EXPERIMENTS += [
    Experiment(
        key="free_move", label="Free Move", group="pretraining",
        summary="Admittance: the arm goes wherever it is pushed and holds where it is let go. No goal, no boundary.",
        launch_file="iiwa7_pretrain_free_move.launch.py",
        params_yaml="pretrain_free_move.yaml", node="free_move_orchestrator",
        stiffness_profile="Uniform 1000",
        ros_controller="kuka_clik_controller",
        run_name="iiwa7_pretrain_free_move",
        launch_args=dict(_MAZE_LAUNCH), clik_nullspace="clik_nullspace_pretrain.yaml",
    ),
    Experiment(
        key="move_vertical", label="Move Vertical", group="pretraining",
        summary="One vertical rail: up or down to the threshold (ring green → red), then back to start.",
        launch_file="iiwa7_pretrain_move_vertical.launch.py",
        params_yaml="pretrain_move_vertical.yaml", node="rail_training_orchestrator",
        stiffness_profile="Z lock 2500, X/Y 1000, rot 120",
        ros_controller="kuka_clik_controller",
        run_name="iiwa7_pretrain_move_vertical",
        launch_args=dict(_MAZE_LAUNCH), clik_nullspace="clik_nullspace_pretrain.yaml",
    ),
    Experiment(
        key="move_horizontal", label="Move Horizontal", group="pretraining",
        summary="One horizontal rail: left or right to the threshold (ring green → red), then back to start.",
        launch_file="iiwa7_pretrain_move_horizontal.launch.py",
        params_yaml="pretrain_move_horizontal.yaml", node="rail_training_orchestrator",
        stiffness_profile="Z lock 2500, X/Y 1000, rot 120",
        ros_controller="kuka_clik_controller",
        run_name="iiwa7_pretrain_move_horizontal",
        launch_args=dict(_MAZE_LAUNCH), clik_nullspace="clik_nullspace_pretrain.yaml",
    ),
]
EXPERIMENTS += [
    Experiment(
        key="reach_calib", label="Reach Calib",
        summary="Admittance free move inside a box around the start: goes where pushed, stops at the walls.",
        launch_file="iiwa7_reach_calib.launch.py",
        params_yaml="reach_calib.yaml", node="reach_calib_orchestrator",
        stiffness_profile="Uniform 1000",
        ros_controller="kuka_clik_controller",
        run_name="iiwa7_reach_calib",
        launch_args=dict(_MAZE_LAUNCH), clik_nullspace="clik_nullspace_pretrain.yaml",
    ),
]
BY_KEY = {e.key: e for e in EXPERIMENTS}
for _e in EXPERIMENTS:
    assert _e.stiffness_profile in SMARTPAD_PROFILES, f"{_e.key}: unknown SmartPad profile {_e.stiffness_profile!r}"
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
        "Start pose, joint angles in degrees. For the CLIK experiments the CLIK redundancy posture "
        "follows it. Checked against the iiwa7 joint limits, and its elbow (A4) must bend the same way "
        "as the park pose's; reachability of anything relative to it (the maze) is not checked here."),
    "move_to_park.target_joint_position": (
        "Park pose: out of the monkey's reach, between trials. Its elbow (A4) must bend the same way as "
        "the start pose's -- otherwise every park <-> start move passes a straight, singular elbow."),
    "virtual_fixtures.maze.*": "Maze rail. Run check_maze.py on the edited YAML (runs/…) to confirm reachability.",
    "checkpoint_monitor.checkpoint_*": "Checkpoints must sit on the rails. Arrays stay the same length.",
    "checkpoint_monitor.goal_[xyz]": "The goal must sit on a rail.",
    "checkpoint_monitor.relative_to_start": "Must match the rails' corridor_frame (relative = true).",
    "virtual_fixtures.*.type": "Geometry class of this profile. Normally chosen with virtual_fixture_profile instead.",
    "visual_cue.remote_board": "The board's own access point always serves at 192.168.4.1.",
    "virtual_fixtures.*_rail.*": "The rail. Keep travel_task.threshold_m short of its ends; up is limited to ~+0.12 m by reach.",
    "travel_task.axis": "Must be one of the rail's own axes (vertical: z, horizontal: y).",
    "resistance.base_stiffness_n_per_m": ("Must equal the SmartPad profile's translational stiffness (Uniform 100 = "
                                          "100, Uniform 400 = 400). If they differ, every pull feels softer or "
                                          "stiffer than the resistance slider says."),
    "perturb_start.polar_r_m": ("The cap depends on the plane: frontal 0.15, horizontal 0.175, sagittal 0.10 m "
                                "(every direction >= 10 deg from a joint limit, from the default start pose). "
                                "The flange tilts more as r grows -- up to ~22 deg at the frontal cap."),
}

# The start pose: for the CLIK experiments the CLIK redundancy posture follows it (see params.py).
START_POSE = "move_to_start.target_joint_position"


def tier(exp: Experiment, name: str) -> str:
    return "live" if live_params.is_live(exp.key, name) else "run"


def caution(name: str) -> Optional[str]:
    for pattern, note in CAUTION.items():
        if fnmatch(name, pattern):
            return note
    return None


def choices(exp: Experiment, name: str, flat: Dict[str, object]) -> Optional[List[str]]:
    if name == "virtual_fixture_profile":
        # Every block under virtual_fixtures that declares a `type` is a selectable profile.
        return sorted({k.split(".")[1] for k in flat
                       if k.startswith("virtual_fixtures.") and k.endswith(".type")})
    return live_params.choices_for(name)
