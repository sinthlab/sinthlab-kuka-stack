#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node as rclpyNode

from sinthlab_bringup.actions.move_to_position_joint_space import MoveToPositionJointSpace
from sinthlab_bringup.actions.switch_controller import SwitchControllerAction
from sinthlab_bringup.actions.move_in_maze import MoveInMazeAction
from sinthlab_bringup.actions.checkpoint_monitor import CheckpointMonitor
from sinthlab_bringup.actions.safety_stop_monitor import SafetyStopMonitor
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
        # Pre-start waypoint, used ONLY when the arm is parked in a near-singular ("straight") posture
        # such as mechanical zero -- a Cartesian-impedance move commanded from there does not reliably
        # reach the maze start (see move_to_prestart in maze_params.yaml). It is NOT run otherwise:
        # dragging the arm out to a different posture and back is disruptive and was causing the arm to
        # detour to the restricted-plane start even when it was already sitting at the maze start.
        self.move_to_prestart = MoveToPositionJointSpace(
            self, param_prefix="move_to_prestart", on_complete=self.on_prestart_complete
        )
        self._did_prestart = False
        self._prestart_wait = None

        # A near-singular posture is one where the arm is nearly STRAIGHT, i.e. the bending joints
        # A2/A4/A6 are all close to zero. Verified against the Jacobian: this test agrees exactly with
        # "smallest singular value < 0.05" for mechanical zero, near-zero, and extended-but-rotated
        # configurations, while correctly clearing the maze start (79.5 deg) and the plane start (90).
        self._extended_deg = 12.0
        if self.has_parameter("move_to_prestart.extended_if_bend_below_deg"):
            self._extended_deg = float(self.get_parameter("move_to_prestart.extended_if_bend_below_deg").value)

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
        self.maze_fixtures = MoveInMazeAction(self, param_prefix="")
        self.checkpoint_monitor = CheckpointMonitor(
            self, param_prefix="checkpoint_monitor",
            on_complete=self.on_goal_reached, on_reward=self.on_checkpoint_reward,
        )
        self.safety = SafetyStopMonitor(self, param_prefix="maze_safety", on_trip=self.on_safety_trip)
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
        if self._did_prestart:
            self.move_to_start.start()
            return
        # Decide ONCE, from where the arm actually is, whether the waypoint is needed at all.
        self._await_state_then_start()

    def _await_state_then_start(self):
        """Wait for the first robot state, then go via the waypoint only if the arm is near-singular."""
        q = self.move_to_start.latest_measured_joints()
        if q is None:
            if self._prestart_wait is None:
                self.get_logger().info("Waiting for robot state before deciding the start route...")
                self._prestart_wait = self.create_timer(0.1, self._await_state_then_start)
            return
        if self._prestart_wait is not None:
            # cancel() only -- destroy_timer() from inside the timer's own callback is not safe.
            self._prestart_wait.cancel()
            self._prestart_wait = None

        bend_deg = float(np.max(np.abs(np.rad2deg(q[[1, 3, 5]]))))  # |A2|, |A4|, |A6|
        self._did_prestart = True  # decided; never re-evaluated
        if bend_deg < self._extended_deg:
            self.get_logger().warn(
                f"Arm is nearly straight (max |A2,A4,A6| = {bend_deg:.1f} deg < {self._extended_deg:.0f}) "
                f"-- that is a near-singular posture. Stepping via the pre-start waypoint first."
            )
            self.move_to_prestart.start()
            return
        self.get_logger().info(
            f"Arm is in a well-conditioned posture (max |A2,A4,A6| = {bend_deg:.1f} deg); "
            f"moving straight to the maze start."
        )
        self.move_to_start.start()

    def on_prestart_complete(self):
        # Waypoint reached; from here the maze start is reliably reachable. Never repeated.
        self.get_logger().info("At pre-start waypoint. Moving to the maze start...")
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
        self.get_logger().info("Go! Maze fixtures + checkpoint monitor active; timeout + safety armed.")
        self.maze_fixtures.start()
        self.checkpoint_monitor.start()
        self.timeout.start()
        self.safety.start()

    def on_checkpoint_reward(self, index):
        self.get_logger().info(f"Reward at checkpoint {index}.")
        self.reward_cue.start()

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
        self.timeout.stop()
        self.checkpoint_monitor.stop()
        self.safety.stop()
        self.maze_fixtures.stop()
        if reason == "safety":
            # Do not play a cue or wait for release — drive straight back to the safe start posture, which
            # (in the cabinet's impedance) sets the equilibrium at start and pulls the arm out of the fault.
            self.get_logger().error("SAFETY ABORT: recovering to the start posture immediately.")
            self.switch_to_joint.start()
            return
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
