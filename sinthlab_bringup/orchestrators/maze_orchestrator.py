#!/usr/bin/env python3

import rclpy
from rclpy.node import Node as rclpyNode

from sinthlab_bringup.actions.move_to_position_joint_space import MoveToPositionJointSpace
from sinthlab_bringup.actions.park_move import ParkMove
from sinthlab_bringup.actions.switch_controller import SwitchControllerAction
from sinthlab_bringup.actions.move_in_maze import MoveInMazeAction
from sinthlab_bringup.actions.checkpoint_monitor import CheckpointMonitor
from sinthlab_bringup.actions.safety_stop_monitor import SafetyStopMonitor
from sinthlab_bringup.actions.force_release_waiter import ForceReleaseWaiter
from sinthlab_bringup.actions.audio_cue import AudioCue
from sinthlab_bringup.actions.visual_cue import VisualCue
from sinthlab_bringup.actions.wait_action import WaitAction
from sinthlab_bringup.actions.trial_recorder import TrialRecorder
from sinthlab_bringup.helpers.node_utils import get_required_param, build_node
from sinthlab_bringup.helpers.experiment_control import ExperimentControl

JOINT_CTRL = "lbr_joint_position_command_controller"
CLIK_CTRL = "kuka_clik_controller"


class MazeOrchestratorNode(rclpyNode):
    """Maze-exploration trial loop, composed entirely of actions:

    [first: move_park, slowly, from wherever the arm is]
    move_to_start (JOINT) -> switch to CLIK -> quiet_window -> go_cue
    -> (maze_fixtures + checkpoint_monitor + timeout) -> [reward_cue per checkpoint, any order]
    -> goal OR timeout -> stop fixtures -> force_release (wait until the operator lets go)
    -> switch to JOINT -> move_park (JOINT, out of reach) -> repeat.

    The start and park moves run on the joint-position controller so the arm reaches the EXACT
    configuration (deterministic posture). The corridor virtual fixtures need the CLIK, so we switch
    to it once the arm is at start, and switch back before parking. The operator drives the
    compliant arm through the corridors; the cabinet impedance provides the walls. Both the goal and
    the timeout converge on the same safe ending: stop the fixtures, wait for release, park.
    """

    def __init__(self) -> None:
        super().__init__("maze_orchestrator", automatically_declare_parameters_from_overrides=True)
        AudioCue.warmup(self)  # wake the WSL2 audio driver so the first cue isn't delayed
        VisualCue.warmup(self)  # logs which visual-cue trigger is active (Wi-Fi test or wire)

        self.trial_count = 0
        self._trial_ending = False

        # Actions that make up the trial.
        self.move_to_start = MoveToPositionJointSpace(
            self, param_prefix="move_to_start", on_complete=self.on_move_complete
        )
        self.switch_to_fixture = SwitchControllerAction(
            self, activate=[CLIK_CTRL], deactivate=[JOINT_CTRL],
            on_complete=self.on_switched_to_fixture, name="switch->clik",
        )
        self.quiet_window = WaitAction(
            self, duration_sec=float(get_required_param(self, "quiet_window_sec")),
            on_complete=self.on_quiet_window_complete, name="quiet_window"
        )
        # Audio and visual cues fire together at each site. The visual cue sends TIMING only --
        # what the ring shows is configured on the board itself. It is a no-op when
        # `visual_cue.enabled` is false, so the experiment runs unchanged before the ring is wired.
        self.go_cue = AudioCue(
            self, param_prefix="audio_cue_play", on_complete=self.on_go_complete,
            # self.recorder does not exist yet -- it needs maze_fixtures, built below. That is fine:
            # the lambda resolves it at CALL time, which is when a beep ends, long after __init__.
            on_finished=lambda secs: self.recorder.mark("cue_audio_end", round(secs, 4)),
        )
        self.go_cue_visual = VisualCue(self, label="play", on_complete=lambda: None)
        self.maze_fixtures = MoveInMazeAction(self, param_prefix="")

        # Trial data -- see README.md section 7, Data Collected. Records from trial START (approach and
        # settle included, not just the maze phase) and marks the go cue, so a run lines up with video
        # without a manual offset.
        # Dashboard / CLI control: status topic, pause between trials, live parameters, NSP codes.
        # See helpers/experiment_control.py; which parameters are live is in helpers/live_params.py.
        self.control = ExperimentControl(self, "maze")
        # A live start / park pose whose move would take the end effector below the floor is refused.
        self.control.add_validator(lambda n, v, c: self.move_park.check_live(n, v, c))
        self.control.on_reload(self._reload_live)
        self.control.on_stop_request(self._stop_now)   # Stop now: end the trial, park, hold
        self._stopping = False
        self.recorder = TrialRecorder(
            self, experiment="maze",
            extra_header=self.maze_fixtures.record_extra_header(),
            extra_fn=self.maze_fixtures.record_extra,
            on_event=self.control.on_event,
        )

        # --- cue delivery, as observed rather than assumed -------------------------------------
        # AudioCue.on_complete fires when Popen RETURNS (~49 ms on WSL2), which is ~290 ms before
        # any sound. The beep process blocks for exactly duration_ms, so its EXIT gives the real
        # end and the start follows by subtraction. The visual ack comes back after the firmware
        # has already called pixels.show(), so it brackets the light.
        VisualCue.set_result_sink(
            lambda lbl, ok, ms: self.recorder.mark("cue_visual_ack", round(ms, 1)))
        self.checkpoint_monitor = CheckpointMonitor(
            self, param_prefix="checkpoint_monitor",
            on_complete=self.on_goal_reached, on_reward=self.on_checkpoint_reward,
            # Place checkpoints + goal on the rails' own origin (anchored after the settle), not a
            # separate first-tick capture ~4 cm above it -- that offset made the goal unreachable.
            origin_provider=self.maze_fixtures.anchor_position,
        )
        self.safety = SafetyStopMonitor(self, param_prefix="maze_safety", on_trip=self.on_safety_trip)
        self.reward_cue = AudioCue(self, param_prefix="audio_cue_reward", on_complete=lambda: None)
        self.goal_cue = AudioCue(self, param_prefix="audio_cue_goal", on_complete=lambda: None)
        self.timeout_cue = AudioCue(self, param_prefix="audio_cue_timeout", on_complete=lambda: None)
        self.reward_cue_visual = VisualCue(self, label="reward", on_complete=lambda: None)
        self.goal_cue_visual = VisualCue(self, label="goal", on_complete=lambda: None)
        self.timeout_cue_visual = VisualCue(self, label="timeout", on_complete=lambda: None)
        self.timeout = WaitAction(
            self, duration_sec=float(get_required_param(self, "timeout_sec")),
            on_complete=self.on_timeout, name="experiment_timeout",
        )
        self.force_release = ForceReleaseWaiter(
            self, param_prefix="force_release", on_complete=self.on_force_released
        )
        self.switch_to_joint = SwitchControllerAction(
            self, activate=[JOINT_CTRL], deactivate=[CLIK_CTRL],
            on_complete=self.on_switched_to_joint, name="switch->joint",
        )
        self.move_park = ParkMove(self, on_complete=self.on_park_complete)

        # Move speed sliders act at once, from each move's next start (live_params.IMMEDIATE).
        self.control.apply_immediately({
            "move_to_start.speed_scale": self.move_to_start.set_speed_scale,
            "move_to_park.speed_scale": self.move_park.set_speed_scale,
        }, log=self.recorder.note_change)
        # Every move's actual speed (scaled limits, duration) goes into the trial sidecar's `moves`.
        for move in (self.move_to_start, self.move_park):
            move.report_moves_to(self.recorder.note_move)
        self.get_logger().info("=== MULTI-TRIAL MAZE EXPERIMENT INITIALIZED ===")
        # The run's first move: to park, out of reach, from wherever the arm is; then the trial loop.
        self.control.on_event("parking")
        self.move_park.start_first(lambda: self.control.begin_trial(self._next_trial))

    def start_trial(self):
        self._stopping = False
        self.recorder.start(trial_index=self.trial_count + 1,
                            maze_geometry=self.maze_fixtures.maze_geometry())
        self.recorder.mark("trial_start", self.trial_count + 1)
        self.trial_count += 1
        self._trial_ending = False
        self.get_logger().info(f"--- STARTING TRIAL {self.trial_count} ---")
        self.move_to_start.start()

    def on_move_complete(self):
        self.recorder.mark("at_start")
        self.get_logger().info("Arm at exact maze start. Switching to CLIK for the fixtures...")
        self.switch_to_fixture.start()

    def on_switched_to_fixture(self):
        if self._stopping:                 # Stop now came during the switch: straight back
            self.switch_to_joint.start()
            return
        self.recorder.mark("fixture_active")
        self.get_logger().info("On CLIK. Waiting for a quiet window...")
        self.quiet_window.start()

    def on_quiet_window_complete(self):
        self.recorder.mark("cue_go")
        self.get_logger().info("Quiet window complete. Sounding go cue.")
        self.go_cue.start()
        self.go_cue_visual.start()

    def on_go_complete(self):
        if self._stopping:
            return
        self.recorder.mark("maze_armed")
        self.get_logger().info("Go! Maze fixtures + checkpoint monitor active; timeout + safety armed.")
        self.maze_fixtures.start()
        self.checkpoint_monitor.start()
        self.timeout.start()
        self.safety.start()

    def on_checkpoint_reward(self, index):
        self.get_logger().info(f"Reward at checkpoint {index}.")
        self.recorder.mark("checkpoint", index)
        self.reward_cue.start()
        self.reward_cue_visual.start()

    def on_goal_reached(self):
        self._end_trial("goal")

    def on_timeout(self):
        self._end_trial("timeout")

    def on_safety_trip(self, reason: str):
        # Runaway (e.g. a gravity-driven fall): abort immediately, do NOT wait for the operator to let go.
        self._end_trial("safety")

    def _end_trial(self, reason: str):
        # Goal, timeout and safety can race; only the first ends the trial.
        if self._trial_ending:
            return
        self._trial_ending = True
        # goal | timeout | safety -- whichever won the race
        self.recorder.mark(reason if reason != "safety" else "safety_trip")
        self.timeout.stop()
        self.checkpoint_monitor.stop()
        self.safety.stop()
        self.maze_fixtures.stop()
        if reason == "safety":
            # Do not play a cue or wait for release — drive straight to park, which (in the cabinet's
            # impedance) moves the equilibrium there and pulls the arm out of the fault.
            self.get_logger().error("SAFETY ABORT: parking immediately.")
            self.switch_to_joint.start()
            return
        if reason == "goal":
            self.get_logger().info("Maze solved! Playing goal cue; waiting for release before reset.")
            self.goal_cue.start()
            self.goal_cue_visual.start()
        else:
            self.get_logger().info("Timeout reached. Playing timeout cue; waiting for release before reset.")
            self.timeout_cue.start()
            self.timeout_cue_visual.start()
        # Park only once the operator lets go (external force ~ 0).
        self.recorder.mark("release_wait")
        self.force_release.start()

    def on_force_released(self):
        self.recorder.mark("released")
        self.get_logger().info("Arm released. Switching to joint controller to park.")
        self.switch_to_joint.start()

    def on_switched_to_joint(self):
        self.move_park.start()

    def on_park_complete(self):
        self.get_logger().info(f"--- TRIAL {self.trial_count} COMPLETE ---")
        self.recorder.mark("trial_end", self.trial_count)
        self.recorder.stop_and_save()
        self.control.begin_trial(self._next_trial)   # holds at park instead if paused

    def _next_trial(self):
        self.move_park.wait_then(self.start_trial)    # wait move_to_park.hold_sec at park, then the next trial

    def _stop_now(self):
        """Stop now (the dashboard's Stop, or ~/stop): end the trial at once and park -- no cue, no
        release wait. The control is paused, so it then holds at park and the launch can end."""
        if self._stopping:
            return
        if self.move_park.cancel_hold():          # at park, waiting for the next trial: just hold
            self.control.begin_trial(self._next_trial)
            return
        if self.move_park.active() or self.switch_to_joint.active():
            return                                # already on the way to park
        self._stopping = True
        self.recorder.mark("stop_requested")
        self._trial_ending = True
        self.quiet_window.stop()
        self.timeout.stop()
        self.checkpoint_monitor.stop()
        self.safety.stop()
        self.maze_fixtures.stop()
        self.force_release.stop()
        if self.move_to_start.active():           # still on the joint controller
            self.move_to_start.cancel()
            self.recorder.mark("recover_start")
            self.move_park.start()
        elif not self.switch_to_fixture.active():  # on the CLIK: back to joint, then park
            self.switch_to_joint.start()
        # (mid-switch to the CLIK, on_switched_to_fixture turns back)

    def _reload_live(self):
        """Re-read the live parameters (helpers/live_params.py). Runs between trials only."""
        for cue in (self.go_cue, self.reward_cue, self.goal_cue, self.timeout_cue,
                    self.go_cue_visual, self.reward_cue_visual, self.goal_cue_visual,
                    self.timeout_cue_visual):
            cue.reload()
        self.quiet_window.set_duration(float(get_required_param(self, "quiet_window_sec")))
        self.move_to_start.reload()      # speed limits (and, for apple pluck / perturb, the pose)
        self.move_park.reload()
        self.timeout.set_duration(float(get_required_param(self, "timeout_sec")))
        self.maze_fixtures.reload()      # rail_lead_sec: how easy moving along a rail feels


def main(args=None) -> None:
    rclpy.init(args=args)
    node = build_node(MazeOrchestratorNode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
