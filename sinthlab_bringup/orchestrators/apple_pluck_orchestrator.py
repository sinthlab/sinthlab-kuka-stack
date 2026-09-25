#!/usr/bin/env python3

import rclpy
from rclpy.node import Node as rclpyNode

from sinthlab_bringup.actions.move_to_position_joint_space import MoveToPositionJointSpace
from sinthlab_bringup.actions.cartesian_impedance_displacement_monitor import CartesianImpedanceDisplacementMonitor
from sinthlab_bringup.actions.audio_cue import AudioCue
from sinthlab_bringup.actions.visual_cue import VisualCue
from sinthlab_bringup.actions.wait_action import WaitAction
from sinthlab_bringup.actions.freeze_at_pose import FreezeAtPoseAction
from sinthlab_bringup.actions.trial_recorder import TrialRecorder
from sinthlab_bringup.helpers.common_threshold import get_optional_param
from sinthlab_bringup.helpers.experiment_control import ExperimentControl


class ApplePluckOrchestratorNode(rclpyNode):
    """Apple-pluck trial loop, composed entirely of actions:

    move_to_start -> quiet_window -> audio_cue -> monitor -> (snap cue) -> move_recover -> repeat.
    """

    def __init__(self) -> None:
        super().__init__(
            "apple_pluck_orchestrator",
            automatically_declare_parameters_from_overrides=True,
        )
        AudioCue.warmup(self)  # wake the WSL2 audio driver so the first cue isn't delayed
        VisualCue.warmup(self)  # logs which visual-cue trigger is active (Wi-Fi test or wire)

        self.trial_count = 0

        # Actions that make up the trial.
        self.move_to_start = MoveToPositionJointSpace(
            self, param_prefix="move_to_start", on_complete=self.on_move_complete
        )
        self.quiet_window = WaitAction(
            self, duration_sec=float(get_optional_param(self, "quiet_window_sec", 2.0)),
            on_complete=self.on_quiet_window_complete, name="quiet_window"
        )
        self.audio_cue = AudioCue(
            self, param_prefix="audio_cue_play", on_complete=self.on_audio_complete,
            on_finished=lambda secs: self.recorder.mark("cue_audio_end", round(secs, 4)),
        )
        self.audio_cue_snap = AudioCue(
            self, param_prefix="audio_cue_snap", on_complete=lambda: None
        )
        # Audio and visual cues fire together. The visual cue sends TIMING only --
        # what the ring shows is configured on the board itself. It is a no-op when
        # `visual_cue.enabled` is false, so the experiment runs unchanged before the
        # ring is wired.
        self.visual_cue = VisualCue(
            self, label="play", on_complete=lambda: None
        )
        self.visual_cue_snap = VisualCue(
            self, label="snap", on_complete=lambda: None
        )
        self.monitor = CartesianImpedanceDisplacementMonitor(
            self, param_prefix="apple_pluck_impedance_control_displacement",
            on_complete=self.on_monitor_complete, on_snap=self.on_monitor_snap,
            on_armed=self.on_monitor_armed,   # baseline locked -> mark it; reaction time = snap - armed
        )
        # At threshold, freeze the equilibrium on the arm's current pose so it stops pulling back
        # (the "give") but stays supported — NOT limp. Held for the monitor's
        # force_release_shutdown_delay_sec, then recover.
        self.freeze_hold = FreezeAtPoseAction(self)
        self.move_recover = MoveToPositionJointSpace(
            self, param_prefix="move_to_start_recover", on_complete=self.on_recover_complete
        )

        # Trial data -- see README.md section 7, Data Collected.
        # disp_m is the dependent variable and is sampled here every state message, so the threshold
        # crossing can be interpolated offline to finer than the 10 ms cabinet stamp.
        # Dashboard / CLI control: status topic, pause between trials, live parameters, NSP codes.
        # See helpers/experiment_control.py; which parameters are live is in helpers/live_params.py.
        self.control = ExperimentControl(self, "apple_pluck")
        self.control.on_reload(self._reload_live)
        self.recorder = TrialRecorder(
            self, experiment="apple_pluck",
            extra_header=["disp_m"],
            extra_fn=lambda _T: [f"{self.monitor.current_disp():.6f}"],
            on_event=self.control.on_event,
        )

        # --- cue delivery, as observed rather than assumed -------------------------------------
        # AudioCue.on_complete fires when Popen RETURNS (~49 ms on WSL2), which is ~290 ms before
        # any sound. The beep process blocks for exactly duration_ms, so its EXIT gives the real
        # end and the start follows by subtraction. The visual ack comes back after the firmware
        # has already called pixels.show(), so it brackets the light.
        VisualCue.set_result_sink(
            lambda lbl, ok, ms: self.recorder.mark("cue_visual_ack", round(ms, 1)))

        self.get_logger().info("=== AUTOMATED MULTI-TRIAL EXPERIMENT INITIALIZED ===")
        self.start_trial()

    def start_trial(self):
        self.trial_count += 1
        self.get_logger().info(f"--- STARTING TRIAL {self.trial_count} ---")
        # Record the WHOLE trial, approach included -- analysis can trim, it cannot un-discard.
        self.recorder.start(trial_index=self.trial_count,
                            threshold_m=self.monitor.threshold_m())
        self.recorder.mark("trial_start", self.trial_count)
        self.move_to_start.start()

    def on_move_complete(self):
        self.get_logger().info("Arm returned to start. Waiting for a quiet window...")
        self.recorder.mark("at_start")
        self.quiet_window.start()

    def on_quiet_window_complete(self):
        self.get_logger().info("Quiet window complete. Sounding audio cue.")
        self.recorder.mark("quiet_end")
        self.audio_cue.start()
        self.visual_cue.start()
        self.recorder.mark("cue_go")

    def on_audio_complete(self):
        self.get_logger().info("Audio cue played. Initiating Displacement Monitor.")
        self.monitor.start()

    def on_monitor_armed(self):
        # Baseline locked: this is the moment the animal may pull. Reaction time is snap - armed.
        self.recorder.mark("armed")

    def on_monitor_snap(self):
        self.get_logger().info("Threshold reached — freezing the arm at its current pose (pull released).")
        self.recorder.mark("snap", round(self.monitor.current_disp(), 6))
        self.audio_cue_snap.start()
        self.visual_cue_snap.start()
        self.recorder.mark("cue_snap")
        self.freeze_hold.start()  # equilibrium moves onto the arm and holds there for the dwell
        self.recorder.mark("freeze")

    def on_monitor_complete(self):
        # Dwell elapsed: stop holding and return to start for the next trial.
        self.get_logger().info("Apple plucked! Returning to start.")
        self.recorder.mark("recover_start")
        self.freeze_hold.stop()
        self.move_recover.start()

    def on_recover_complete(self):
        self.get_logger().info(f"--- TRIAL {self.trial_count} COMPLETE ---")
        self.recorder.mark("trial_end", self.trial_count)
        self.recorder.stop_and_save()
        self.control.begin_trial(self.start_trial)   # holds here instead if paused

    def _reload_live(self):
        """Re-read the live parameters (helpers/live_params.py). Runs between trials only."""
        for cue in (self.audio_cue, self.audio_cue_snap, self.visual_cue, self.visual_cue_snap):
            cue.reload()
        self.monitor.reload()
        self.quiet_window.set_duration(float(get_optional_param(self, "quiet_window_sec", 2.0)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ApplePluckOrchestratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
