#!/usr/bin/env python3

import rclpy
from rclpy.node import Node as rclpyNode

from sinthlab_bringup.actions.move_to_position_joint_space import MoveToPositionJointSpace
from sinthlab_bringup.actions.switch_controller import SwitchControllerAction
from sinthlab_bringup.actions.move_restricted_on_a_plane import MoveRestrictedOnAPlaneAction
from sinthlab_bringup.actions.checkpoint_monitor import CheckpointMonitor
from sinthlab_bringup.actions.force_release_waiter import ForceReleaseWaiter
from sinthlab_bringup.actions.audio_cue import AudioCue
from sinthlab_bringup.actions.wait_action import WaitAction
from sinthlab_bringup.helpers.common_threshold import get_required_param

JOINT_CTRL = "lbr_joint_position_command_controller"
CLIK_CTRL = "kuka_clik_controller"


class MazeOrchestratorNode(rclpyNode):
    """Maze-exploration trial loop, composed entirely of actions:

    move_to_start (JOINT) -> switch to CLIK -> quiet_window -> go_cue
    -> (maze_fixtures + checkpoint_monitor + timeout) -> [reward_cue per checkpoint, any order]
    -> goal OR timeout -> stop fixtures -> force_release (wait until the operator lets go)
    -> switch to JOINT -> move_recover (JOINT) -> repeat.

    The start/recover moves run on the joint-position controller so the arm reaches the EXACT
    configuration (deterministic posture). The corridor virtual fixtures need the CLIK, so we switch
    to it once the arm is at start, and switch back before recovering. The operator drives the
    compliant arm through the corridors; the cabinet impedance provides the walls. Both the goal and
    the timeout converge on the same safe ending: stop the fixtures, wait for release, return to start.
    """

    def __init__(self) -> None:
        super().__init__("maze_orchestrator", automatically_declare_parameters_from_overrides=True)
        AudioCue.warmup(self)  # wake the WSL2 audio driver so the first cue isn't delayed

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
            self, duration_sec=2.0, on_complete=self.on_quiet_window_complete, name="quiet_window"
        )
        self.go_cue = AudioCue(self, param_prefix="audio_cue_play", on_complete=self.on_go_complete)
        self.maze_fixtures = MoveRestrictedOnAPlaneAction(self, param_prefix="")
        self.checkpoint_monitor = CheckpointMonitor(
            self, param_prefix="checkpoint_monitor",
            on_complete=self.on_goal_reached, on_reward=self.on_checkpoint_reward,
        )
        self.reward_cue = AudioCue(self, param_prefix="audio_cue_reward", on_complete=lambda: None)
        self.goal_cue = AudioCue(self, param_prefix="audio_cue_goal", on_complete=lambda: None)
        self.timeout_cue = AudioCue(self, param_prefix="audio_cue_timeout", on_complete=lambda: None)
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
        self.move_recover = MoveToPositionJointSpace(
            self, param_prefix="move_to_start_recover", on_complete=self.on_recover_complete
        )

        self.get_logger().info("=== MULTI-TRIAL MAZE EXPERIMENT INITIALIZED ===")
        self.start_trial()

    def start_trial(self):
        self.trial_count += 1
        self._trial_ending = False
        self.get_logger().info(f"--- STARTING TRIAL {self.trial_count} ---")
        self.move_to_start.start()

    def on_move_complete(self):
        self.get_logger().info("Arm at exact maze start. Switching to CLIK for the fixtures...")
        self.switch_to_fixture.start()

    def on_switched_to_fixture(self):
        self.get_logger().info("On CLIK. Waiting for a quiet window...")
        self.quiet_window.start()

    def on_quiet_window_complete(self):
        self.get_logger().info("Quiet window complete. Sounding go cue.")
        self.go_cue.start()

    def on_go_complete(self):
        self.get_logger().info("Go! Maze fixtures + checkpoint monitor active; timeout armed.")
        self.maze_fixtures.start()
        self.checkpoint_monitor.start()
        self.timeout.start()

    def on_checkpoint_reward(self, index):
        self.get_logger().info(f"Reward at checkpoint {index}.")
        self.reward_cue.start()

    def on_goal_reached(self):
        self._end_trial("goal")

    def on_timeout(self):
        self._end_trial("timeout")

    def _end_trial(self, reason: str):
        # Goal and timeout can race; only the first ends the trial.
        if self._trial_ending:
            return
        self._trial_ending = True
        self.timeout.stop()
        self.checkpoint_monitor.stop()
        self.maze_fixtures.stop()
        if reason == "goal":
            self.get_logger().info("Maze solved! Playing goal cue; waiting for release before reset.")
            self.goal_cue.start()
        else:
            self.get_logger().info("Timeout reached. Playing timeout cue; waiting for release before reset.")
            self.timeout_cue.start()
        # Return to start only once the operator lets go (external force ~ 0).
        self.force_release.start()

    def on_force_released(self):
        self.get_logger().info("Arm released. Switching to joint controller to recover.")
        self.switch_to_joint.start()

    def on_switched_to_joint(self):
        self.move_recover.start()

    def on_recover_complete(self):
        self.get_logger().info(f"--- TRIAL {self.trial_count} COMPLETE ---")
        self.start_trial()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MazeOrchestratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
