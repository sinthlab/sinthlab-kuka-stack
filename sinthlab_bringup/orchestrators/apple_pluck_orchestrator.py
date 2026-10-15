#!/usr/bin/env python3

import random

import numpy as np
import rclpy
from rclpy.node import Node as rclpyNode

from sinthlab_bringup.actions.move_to_position_joint_space import MoveToPositionJointSpace
from sinthlab_bringup.actions.perturb_initial_position import PerturbInitialPosition
from sinthlab_bringup.actions.grasp_detector import GraspDetector
from sinthlab_bringup.actions.cartesian_impedance_displacement_monitor import CartesianImpedanceDisplacementMonitor
from sinthlab_bringup.actions.audio_cue import AudioCue
from sinthlab_bringup.actions.visual_cue import VisualCue
from sinthlab_bringup.actions.wait_action import WaitAction
from sinthlab_bringup.actions.freeze_at_pose import FreezeAtPoseAction
from sinthlab_bringup.actions.resistance_softener import ResistanceSoftener
from sinthlab_bringup.actions.trial_recorder import TrialRecorder
from sinthlab_bringup.helpers.node_utils import get_required_param, get_launch_param
from sinthlab_bringup.helpers.experiment_control import ExperimentControl

PARADIGMS = ("pluck", "perturb", "perturb_bump", "random_start")
# random_start: the origin C and the 8 compass directions around it (random_start.positions).
START_POSITIONS = ("C", "N", "NE", "E", "SE", "S", "SW", "W", "NW")


class ApplePluckOrchestratorNode(rclpyNode):
    """Apple-pluck trial loop, in one of the PARADIGMS (`paradigm`, fixed for the run):

      pluck    move_to_start -> quiet_window -> go cue -> monitor -> (threshold) snap cue + freeze
               -> move_recover -> repeat.
      perturb  the same, but after the go cue the apple is displaced: perturb_delay -> perturbation
               (a polar (r, theta) offset in a plane, perturb_start.*) -> the monitor arms once the arm
               has settled there (perturb_start.baseline_settle_sec).
      perturb_bump  the same perturbation, triggered by the GRASP instead of the go cue: go cue -> wait
               for the hand to take hold (GraspDetector, grasp.*; none within grasp.timeout_sec -> the
               trial ends and resets) -> perturb_delay -> perturbation -> the pull is measured from the
               perturbed SPRING position (the perturbation's target), locked at once: a held apple
               never settles there.
      random_start  as pluck, but each trial starts (and recovers) at one of 9 positions
               (random_start.positions), in shuffled blocks: all 9 once before any repeats.

    Two cues only: the go cue at trial start and the snap cue at the threshold -- no cue when the
    monitor arms after a perturbation. Every trial is recorded as experiment "apple_pluck", with
    `paradigm` in its sidecar (and, for perturb, the perturbation it ran with; for random_start, the
    start position).
    """

    def __init__(self) -> None:
        super().__init__(
            "apple_pluck_orchestrator",
            automatically_declare_parameters_from_overrides=True,
        )
        AudioCue.warmup(self)  # wake the WSL2 audio driver so the first cue isn't delayed
        VisualCue.warmup(self)  # logs which visual-cue trigger is active (Wi-Fi test or wire)

        # The launch argument `paradigm:=` (paradigm_override) wins over the YAML, so the paradigm can
        # be chosen at launch without editing a file; the dashboard edits the YAML copy instead.
        override = str(get_launch_param(self, "paradigm_override", "") or "").strip().lower()
        self.paradigm = override or str(get_required_param(self, "paradigm")).strip().lower()
        if self.paradigm not in PARADIGMS:
            raise ValueError(f"paradigm must be one of {PARADIGMS}, got {self.paradigm!r}")
        self.perturbing = self.paradigm in ("perturb", "perturb_bump")
        self.bump = self.paradigm == "perturb_bump"
        self.trial_count = 0
        if self.paradigm == "random_start":
            self.start_positions = {
                name: [float(v) for v in get_required_param(self, f"random_start.positions.{name}")]
                for name in START_POSITIONS}
            self._start_queue = []

        # Actions that make up the trial.
        self.move_to_start = MoveToPositionJointSpace(
            self, param_prefix="move_to_start", on_complete=self.on_move_complete
        )
        self.quiet_window = WaitAction(
            self, duration_sec=float(get_required_param(self, "quiet_window_sec")),
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
        # Perturb paradigms only: the delay (after the go cue, or the grasp), then the displacement.
        if self.perturbing:
            self._read_delay()
            self.perturb_delay = WaitAction(
                self, duration_sec=0.0, on_complete=self.start_perturb, name="perturb_delay")
            self.perturb = PerturbInitialPosition(
                self, param_prefix="perturb_start", on_complete=self.on_perturb_complete
            )
        # Perturb bump only: wait for the grasp, for at most grasp.timeout_sec.
        if self.bump:
            self.grasp = GraspDetector(self, param_prefix="grasp", on_grasp=self.on_grasp)
            self.grasp_timeout = WaitAction(
                self, duration_sec=float(get_required_param(self, "grasp.timeout_sec")),
                on_complete=self.on_grasp_timeout, name="grasp_timeout")
        self.monitor = CartesianImpedanceDisplacementMonitor(
            self, param_prefix="apple_pluck_impedance_control_displacement",
            on_complete=self.on_monitor_complete, on_snap=self.on_monitor_snap,
            on_armed=self.on_monitor_armed,   # baseline locked -> mark it; reaction time = snap - armed
        )
        self._apply_settle()
        # At threshold, freeze the equilibrium on the arm's current pose so it stops pulling back
        # (the "give") but stays supported — NOT limp. Held for the monitor's
        # force_release_shutdown_delay_sec, then recover.
        self.freeze_hold = FreezeAtPoseAction(self)
        # Felt stiffness of the pull, below the SmartPad profile (live: resistance.stiffness_n_per_m).
        # Softens from the moment the monitor arms until the threshold, where the freeze takes over.
        self.resistance = ResistanceSoftener(self, param_prefix="resistance")
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

        # Move speed sliders act at once, from each move's next start (live_params.IMMEDIATE).
        speeds = {"move_to_start.speed_scale": self.move_to_start.set_speed_scale,
                  "move_to_start_recover.speed_scale": self.move_recover.set_speed_scale}
        if self.perturbing:
            speeds["perturb_start.speed_scale"] = self.perturb.set_speed_scale
        self.control.apply_immediately(speeds, log=self.recorder.note_change)
        # Every move's actual speed (scaled limits, duration) goes into the trial sidecar's `moves`.
        for move in (self.move_to_start, self.move_recover) + ((self.perturb,) if self.perturbing else ()):
            move.report_moves_to(self.recorder.note_move)
        if self.paradigm == "random_start":
            self._check_origin()
        self.get_logger().info(
            f"=== AUTOMATED MULTI-TRIAL APPLE PLUCK EXPERIMENT INITIALIZED (paradigm: {self.paradigm}) ===")
        self.start_trial()

    def _apply_settle(self):
        # perturb: the arm must settle where the perturbation left it before the pull baseline locks.
        # pluck / random_start: the quiet window has settled it. perturb_bump: no settling -- the
        # baseline is the perturbed spring position (on_perturb_complete).
        settle = (float(get_required_param(self, "perturb_start.baseline_settle_sec"))
                  if self.paradigm == "perturb" else 0.0)
        self.monitor.set_baseline_settle(settle)

    def _read_delay(self):
        """The delay before the perturbation: fixed (start_delay_sec) or drawn per trial, uniformly from
        [start_delay_min_sec, start_delay_max_sec]. Re-read between trials."""
        p = "perturb_start."
        self._delay_mode = str(get_required_param(self, p + "start_delay_mode")).strip().lower()
        if self._delay_mode not in ("fixed", "random"):
            raise ValueError(f"{p}start_delay_mode must be fixed or random, got {self._delay_mode!r}")
        self._delay_fixed = float(get_required_param(self, p + "start_delay_sec"))
        self._delay_range = (float(get_required_param(self, p + "start_delay_min_sec")),
                             float(get_required_param(self, p + "start_delay_max_sec")))

    def _start_delay(self):
        delay = (self._delay_fixed if self._delay_mode == "fixed"
                 else random.uniform(min(self._delay_range), max(self._delay_range)))
        self.perturb_delay.set_duration(delay)
        self.recorder.mark("perturb_delay_start", round(delay, 4))
        self.perturb_delay.start()

    def _check_origin(self):
        """The grid was solved around the normal start (C); warn if move_to_start has moved since."""
        start = np.array(get_required_param(self, "move_to_start.target_joint_position"), dtype=float)
        if not np.allclose(start, self.start_positions["C"], atol=0.05):
            self.get_logger().warn(
                "random_start.positions.C differs from move_to_start.target_joint_position: the grid was "
                "solved around another start pose. Re-solve it (solve_start_grid.py) if that is not intended.")

    def _next_start_position(self) -> str:
        # Shuffled blocks: every position once, in a random order, before any repeats.
        if not self._start_queue:
            self._start_queue = random.sample(START_POSITIONS, len(START_POSITIONS))
        return self._start_queue.pop()

    def start_trial(self):
        self.trial_count += 1
        # Record the WHOLE trial, approach included -- analysis can trim, it cannot un-discard.
        sidecar = {"paradigm": self.paradigm, "threshold_m": self.monitor.threshold_m()}
        where = ""
        if self.paradigm == "random_start":
            name = self._next_start_position()
            pose = self.start_positions[name]
            self.move_to_start.set_target_deg(pose)
            self.move_recover.set_target_deg(pose)     # recover to this trial's own start
            sidecar.update(start_position=name, start_joint_deg=pose)
            where = f", start {name}"
        self.get_logger().info(f"--- STARTING TRIAL {self.trial_count} ({self.paradigm}{where}) ---")
        if self.perturbing:
            sidecar["perturbation"] = {**self.perturb.perturbation_spec(),
                                       "trigger": "grasp" if self.bump else "go_cue",
                                       "delay_mode": self._delay_mode}
        self.recorder.start(trial_index=self.trial_count, **sidecar)
        self.recorder.annotate(resistance=self.resistance.describe())
        self.recorder.mark("trial_start", self.trial_count)
        self.move_to_start.start()

    def on_move_complete(self):
        self.get_logger().info("Arm returned to start. Waiting for a quiet window...")
        self.recorder.mark("at_start")
        if self.bump:
            self.grasp.start()        # the apple is untouched now: measure its resting force (tare)
        self.quiet_window.start()

    def on_quiet_window_complete(self):
        self.get_logger().info("Quiet window complete. Sounding audio cue.")
        self.recorder.mark("quiet_end")
        self.audio_cue.start()
        self.visual_cue.start()
        self.recorder.mark("cue_go")

    def on_audio_complete(self):
        if self.bump:
            self.get_logger().info("Audio cue played. Waiting for the grasp...")
            self.grasp.arm()
            self.grasp_timeout.start()
            self.recorder.mark("grasp_wait")
        elif self.perturbing:
            self.get_logger().info("Audio cue played. Waiting before perturbation...")
            self._start_delay()
        else:
            self.get_logger().info("Audio cue played. Initiating Displacement Monitor.")
            self.monitor.start()

    def on_grasp(self, force_n: float):
        self.grasp_timeout.stop()
        self.recorder.mark("grasp", round(force_n, 3))
        self.get_logger().info("Grasp detected. Waiting before perturbation...")
        self._start_delay()

    def on_grasp_timeout(self):
        # No grasp in time: end the trial and reset (the arm is still at the start).
        self.grasp.stop()
        self.get_logger().info("No grasp within grasp.timeout_sec. Resetting for the next trial.")
        self.recorder.mark("grasp_timeout")
        self.recorder.mark("recover_start")
        self.move_recover.start()

    def start_perturb(self):
        self.get_logger().info("Initiating Perturbation Move.")
        self.perturb.start()

    def on_perturb_complete(self):
        self.get_logger().info("Perturbation complete. Initiating Displacement Monitor.")
        self.recorder.mark("perturb_applied", self.perturb.perturbation_spec()["r_m"])
        # perturb_bump: the pull is measured from where the perturbation moved the spring, at once.
        self.monitor.start(baseline_xyz=self.perturb.target_xyz() if self.bump else None)

    def on_monitor_armed(self):
        # Baseline locked: this is the moment the animal may pull. Reaction time is snap - armed.
        # No cue here -- in the perturb paradigm the monkey is already reaching (cued at trial start).
        self.recorder.mark("armed")
        self.resistance.start()   # from here the pull is softened to the selected stiffness
        self.recorder.annotate(baseline=self.monitor.baseline_xyz())

    def on_monitor_snap(self):
        self.get_logger().info("Threshold reached — freezing the arm at its current pose (pull released).")
        self.recorder.mark("snap", round(self.monitor.current_disp(), 6))
        self.audio_cue_snap.start()
        self.visual_cue_snap.start()
        self.recorder.mark("cue_snap")
        self.resistance.stop()    # the freeze ramps from the equilibrium this left behind
        self.freeze_hold.start()  # equilibrium moves onto the arm and holds there for the dwell
        self.recorder.mark("freeze")

    def on_monitor_complete(self):
        # Dwell elapsed: stop holding and return to start for the next trial.
        self.get_logger().info("Apple plucked! Returning to start.")
        self.recorder.mark("recover_start")
        self.freeze_hold.stop()
        self.resistance.stop()
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
        self._apply_settle()
        self.resistance.reload()    # felt stiffness of the pull
        self.quiet_window.set_duration(float(get_required_param(self, "quiet_window_sec")))
        self.move_to_start.reload()      # speed limits and the start / recover pose
        self.move_recover.reload()
        if self.paradigm == "random_start":
            self._check_origin()         # start_trial sets this trial's start pose after this reload
        if self.perturbing:
            self.perturb.reload()        # the perturbation: r, theta, plane
            self._read_delay()           # fixed or random delay, and its values
        if self.bump:
            self.grasp.reload()          # threshold, hold time
            self.grasp_timeout.set_duration(float(get_required_param(self, "grasp.timeout_sec")))


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
