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
from sinthlab_bringup.helpers.stiffness_profiles import SMARTPAD_PROFILES, required_profile
from sinthlab_bringup.helpers import live_params  # noqa: E402
from sinthlab_bringup.helpers import outcome_tables  # noqa: E402

# Where the recordings are: the recorder's save folder (outcome_tables.data_dir: $SINTHLAB_DATA_DIR, else
# ~/lbr-stack/src/sinthlab-kuka-stack/analysis), whichever checkout this dashboard runs from. ANALYSIS
# above is this checkout's analysis CODE (validate_recording.py, nwb_export.py).
DATA = outcome_tables.data_dir()



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
    # Variants of one experiment, chosen per run by its `paradigm_param` parameter and shown as nested
    # tabs on its card: [(value, label), ...]. A live paradigm_param (the rails' centre_out.target) can
    # also be switched by those tabs during a run, from the next trial. Parameters that belong to one
    # paradigm only (live_params.PARADIGM_ONLY) are shown only under it.
    paradigms: List[Tuple[str, str]] = field(default_factory=list)
    paradigm_param: str = "paradigm"
    # Parameters fixed by the task itself, not shown in the dashboard (the colour rule: centre_out.target is
    # always "rule"). The YAML holds them; the node refuses another value.
    hidden: Tuple[str, ...] = ()
    # One CARD can hold several experiments, one per variant (Restricted Move and Cued Reaches: a horizontal
    # and a vertical rail, each its own YAML, launch file and start pose). The card shows a row of variant
    # tabs above its paradigm tabs. Empty: the experiment is a card of its own.
    card: str = ""
    variant: str = ""
    # TODAY'S settings: the few the task itself needs, shown on the dashboard's front page (Task tab). The
    # monkey's own settings (feel, reach, pacing, reward) come from its profile (profiles.py); everything
    # else is under All settings. Empty (Navin's experiments): the card opens on All settings.
    task_settings: Tuple[str, ...] = ()

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
                "Random start: each trial starts at one of a set of positions in a plane around the start (default: origin + 8 compass directions, 5 cm).",
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
# Move Vertical / Horizontal: the centre-out goal (centre_out.target). It is live, so these tabs also
# switch it during a run, from the next trial: one end only -> the other -> cued -> either.
_CENTRE_OUT = [("either", "Either end"), ("cued", "Cued end")]
# Cards holding several experiments (Experiment.card): key -> (label, summary).
CARDS = {
    "restricted_move": ("Restricted Move", "Free Move held to one rail, horizontal or vertical: the arm goes along "
                        "it wherever it is pushed. A session per trial, rewarded when it runs its full length. No target."),
    "cued_reach": ("Cued Reaches", "One reach per trial along a rail, horizontal or vertical, from the centre to an "
                   "end: either end, a cued end, or one end only (switchable mid-session). The goal end is rewarded."),
}
_CUED_TASK = ("centre_out.max_repeats", "centre_out.block_other_end", "centre_out.end_tol_m")
_CENTRE_OUT_VERTICAL = _CENTRE_OUT + [("down", "Down only"), ("up", "Up only")]
_CENTRE_OUT_HORIZONTAL = _CENTRE_OUT + [("left", "Left only"), ("right", "Right only")]
EXPERIMENTS += [
    Experiment(
        key="free_move", label="Free Move", group="pretraining",
        summary="Admittance: the arm goes wherever it is pushed within a box around the start, and holds where it is let go. Rewards the session's end, or each long movement.",
        launch_file="iiwa7_pretrain_free_move.launch.py",
        params_yaml="pretrain_free_move.yaml", node="free_move_orchestrator",
        stiffness_profile=required_profile("free_move"),
        ros_controller="kuka_clik_controller",
        run_name="iiwa7_pretrain_free_move",
        # The reward: the session's end, or each long movement (long_move.reward, live: switchable mid-run).
        paradigms=[("session", "Session"), ("each_move", "Long moves")], paradigm_param="long_move.reward",
        task_settings=("session_sec", "long_move.min_m", "long_move.min_straightness", "long_move.min_gap_sec"),
        launch_args=dict(_MAZE_LAUNCH), clik_nullspace="clik_nullspace_pretrain.yaml",
    ),
    # Restricted Move: Free Move held to one rail, a session per trial, rewarded at its end. No target.
    Experiment(
        key="restricted_move_horizontal", label="Restricted Move · Horizontal", group="pretraining",
        summary=CARDS["restricted_move"][1], card="restricted_move", variant="Horizontal",
        launch_file="iiwa7_pretrain_restricted_move_horizontal.launch.py",
        params_yaml="pretrain_restricted_move_horizontal.yaml", node="free_move_orchestrator",
        stiffness_profile=required_profile("restricted_move_horizontal"),
        ros_controller="kuka_clik_controller",
        run_name="iiwa7_pretrain_restricted_move_horizontal",
        task_settings=("session_sec",),
        launch_args=dict(_MAZE_LAUNCH), clik_nullspace="clik_nullspace_pretrain.yaml",
    ),
    Experiment(
        key="restricted_move_vertical", label="Restricted Move · Vertical", group="pretraining",
        summary=CARDS["restricted_move"][1], card="restricted_move", variant="Vertical",
        launch_file="iiwa7_pretrain_restricted_move_vertical.launch.py",
        params_yaml="pretrain_restricted_move_vertical.yaml", node="free_move_orchestrator",
        stiffness_profile=required_profile("restricted_move_vertical"),
        ros_controller="kuka_clik_controller",
        run_name="iiwa7_pretrain_restricted_move_vertical",
        task_settings=("session_sec",),
        # It starts 5 cm lower, in the middle of its rail, so it has its own CLIK posture.
        launch_args={**_MAZE_LAUNCH, "clik_nullspace_cfg": "config/clik_nullspace_pretrain_vertical.yaml"},
        clik_nullspace="clik_nullspace_pretrain_vertical.yaml",
    ),
    # Cued Reaches: one reach per trial from the centre of a rail to an end -- either end, a cued end, or one
    # end only. The experiment keys stay move_horizontal / move_vertical (the centre-out tasks on main), so
    # their recordings stay comparable with earlier ones.
    Experiment(
        key="move_horizontal", label="Cued Reaches · Horizontal", group="pretraining",
        summary=CARDS["cued_reach"][1], card="cued_reach", variant="Horizontal",
        launch_file="iiwa7_pretrain_move_horizontal.launch.py",
        params_yaml="pretrain_move_horizontal.yaml", node="free_move_orchestrator",
        stiffness_profile=required_profile("move_horizontal"),
        ros_controller="kuka_clik_controller",
        run_name="iiwa7_pretrain_move_horizontal",
        paradigms=list(_CENTRE_OUT_HORIZONTAL), paradigm_param="centre_out.target",
        task_settings=_CUED_TASK,
        launch_args=dict(_MAZE_LAUNCH), clik_nullspace="clik_nullspace_pretrain.yaml",
    ),
    Experiment(
        key="move_vertical", label="Cued Reaches · Vertical", group="pretraining",
        summary=CARDS["cued_reach"][1], card="cued_reach", variant="Vertical",
        launch_file="iiwa7_pretrain_move_vertical.launch.py",
        params_yaml="pretrain_move_vertical.yaml", node="free_move_orchestrator",
        stiffness_profile=required_profile("move_vertical"),
        ros_controller="kuka_clik_controller",
        run_name="iiwa7_pretrain_move_vertical",
        paradigms=list(_CENTRE_OUT_VERTICAL), paradigm_param="centre_out.target",
        task_settings=_CUED_TASK,
        # It starts 5 cm lower, in the middle of its rail, so it has its own CLIK posture.
        launch_args={**_MAZE_LAUNCH, "clik_nullspace_cfg": "config/clik_nullspace_pretrain_vertical.yaml"},
        clik_nullspace="clik_nullspace_pretrain_vertical.yaml",
    ),
]
# The colour rule with unsignaled reversals (CI2B Aim 3): a task of its own, on the free-move node. Rail
# or No rail is the run's option (free_move.rail_axis, per-run), with the same criterion.
EXPERIMENTS += [
    Experiment(
        key="colour_rule", label="Colour Rule", group="experiments",
        summary="Red or blue ring: each colour means left or right; the mapping reverses, unsignaled, at criterion. On a rail or free in the frontal plane.",
        launch_file="iiwa7_colour_rule.launch.py",
        params_yaml="colour_rule.yaml", node="free_move_orchestrator",
        stiffness_profile=required_profile("colour_rule"),
        ros_controller="kuka_clik_controller",
        run_name="iiwa7_colour_rule",
        paradigms=[("y", "Rail"), ("frontal", "No rail")], paradigm_param="free_move.rail_axis",
        task_settings=("rule.red_end", "rule.window", "rule.criterion_pct", "rule.min_trials", "rule.max_repeats",
                       "centre_out.end_tol_m", "centre_out.free_reach_m"),
        hidden=("centre_out.enabled", "centre_out.target", "centre_out.block_other_end",
                "centre_out.max_repeats", "centre_out.quarter_left", "centre_out.quarter_right"),
        launch_args=dict(_MAZE_LAUNCH), clik_nullspace="clik_nullspace_pretrain.yaml",
    ),
]
# N-choice with depleting rewards on an asterisk of rails: a task of its own, on the free-move node. It
# starts where Move Vertical does (the middle of the arm's vertical span), so rails can go up and down.
EXPERIMENTS += [
    Experiment(
        key="n_choice", label="N-Choice", group="experiments",
        summary="An asterisk of rails from the centre: each direction pays with a probability that falls while chosen and recovers while left alone.",
        launch_file="iiwa7_n_choice.launch.py",
        params_yaml="n_choice.yaml", node="free_move_orchestrator",
        stiffness_profile=required_profile("n_choice"),
        ros_controller="kuka_clik_controller",
        run_name="iiwa7_n_choice",
        # How many choices (free_move.ray_layout, per run), evenly spaced: work up from 2 over sessions.
        paradigms=[("2", "2 choices"), ("3", "3"), ("4", "4"), ("5", "5"), ("6", "6"), ("7", "7"), ("8", "8")],
        paradigm_param="free_move.ray_layout",
        task_settings=("n_choice.p_max", "n_choice.p_min", "n_choice.deplete", "n_choice.recover", "n_choice.start",
                       "n_choice.end_tol_m", "free_move.ray_length_m"),
        launch_args={**_MAZE_LAUNCH, "clik_nullspace_cfg": "config/clik_nullspace_pretrain_vertical.yaml"},
        clik_nullspace="clik_nullspace_pretrain_vertical.yaml",
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
    "free_move.rail_axis": "The rail through the start: y = horizontal, z = vertical; none = Free Move's box, frontal = the No-rail plane, rays = the N-Choice asterisk.",
    "free_move.rail_m??_m": "The rail's end, from the start. With a monkey chosen it comes from the profile's reach (left / right on a horizontal rail, down / up on a vertical one); up is the arm's tight direction.",
    "resistance.base_stiffness_n_per_m": ("Used only until the SmartPad app reports its profile; from then on the base "
                                          "is the reported stiffness (Uniform 100 = 100, Uniform 400 = 400), and the "
                                          "felt-stiffness slider runs up to it once the run has started."),
    "free_move.damping_ns_per_m": ("Below about 40 the arm rings (stop-go bursts) or trembles: the hand-arm loop has "
                                   "~170 ms of delay. 50 is smooth."),
    "floor.height_m": ("The lowest the end effector may go, height above the ground. The run refuses to start if the "
                       "start or park pose, or the move between them, goes below it."),
    "random_start.positions.*": ("[a, b] metres from the start pose in random_start.plane (frontal: a = +Y, b = up). "
                                 "The run solves them to joints at Start and stops with the reason if one is out of "
                                 "reach or near a limit; check first with solve_start_grid.py."),
    "perturb_start.polar_r_m": ("The cap depends on the plane: frontal 0.125, horizontal 0.175, sagittal 0.10 m "
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
