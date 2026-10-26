#!/usr/bin/env python3

import random
import time

import numpy as np
import rclpy
from rclpy.node import Node as rclpyNode

from sinthlab_bringup.actions.move_to_position_joint_space import MoveToPositionJointSpace
from sinthlab_bringup.actions.park_move import ParkMove
from sinthlab_bringup.actions.perturb_initial_position import PerturbInitialPosition
from sinthlab_bringup.actions.grasp_detector import GraspDetector
from sinthlab_bringup.actions.cartesian_impedance_displacement_monitor import CartesianImpedanceDisplacementMonitor
from sinthlab_bringup.actions.audio_cue import AudioCue
from sinthlab_bringup.actions.visual_cue import VisualCue
from sinthlab_bringup.actions.wait_action import WaitAction
from sinthlab_bringup.actions.resistance_softener import ResistanceSoftener
from sinthlab_bringup.actions.safety_stop_monitor import SafetyStopMonitor
from sinthlab_bringup.actions.trial_recorder import TrialRecorder
from sinthlab_bringup.helpers.node_utils import get_required_param, get_launch_param
from sinthlab_bringup.helpers.experiment_control import ExperimentControl
from sinthlab_bringup.helpers.smartpad_link import SmartPadLink
from sinthlab_bringup.helpers.start_grid import StartGrid

PARADIGMS = ("pluck", "perturb", "perturb_bump", "random_start")


class ApplePluckOrchestratorNode(rclpyNode):
    """Apple-pluck trial loop, in one of the PARADIGMS (`paradigm`, fixed for the run):

      pluck    [first: move_park, slowly, from wherever the arm is]
               move_to_start -> quiet_window -> go cue -> monitor -> (threshold) snap cue + release
               -> move_park (out of reach) -> repeat.
      perturb  the same, but after the go cue the apple is displaced: perturb_delay -> perturbation
               (a polar (r, theta) offset in a plane, perturb_start.*) -> the monitor arms once the arm
               has settled there (perturb_start.baseline_settle_sec).
      perturb_bump  the same perturbation, triggered by the GRASP instead of the go cue: go cue -> wait
               for the hand to take hold (GraspDetector, grasp.*; none within grasp.timeout_sec -> the
               trial ends and resets) -> perturb_delay -> perturbation -> the pull is measured from the
               perturbed SPRING position (the perturbation's target), locked at once: a held apple
               never settles there.
      random_start  as pluck, but each trial starts at one of random_start.positions -- [a, b] offsets
               from the start pose in random_start.plane, solved to joints by helpers/start_grid.py --
               in shuffled blocks: every position once before any repeats.

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
            self.start_positions = {}       # name -> joint angles [deg], solved around the start pose
            self._start_offsets = {}        # name -> its plane and [a, b] offset, for the sidecar
            self._grid_start = None         # the start pose they were solved for
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
            self.perturb.set_speed_scale(float(get_required_param(self, "grasp.bump_speed_scale")))
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
        # Felt stiffness of the pull, below the SmartPad profile (live: resistance.stiffness_n_per_m),
        # from the moment the monitor arms. At the threshold it RELEASES: the equilibrium follows the arm,
        # so the apple is "plucked" -- no pull-back -- until the arm comes to rest. Never toward a
        # straight elbow or below the floor (resistance_softener.py).
        self.resistance = ResistanceSoftener(self, param_prefix="resistance")
        self._park_wait_timeout = float(get_required_param(self, "resistance.park_wait_timeout_sec"))
        self._park_wait = None       # timer: after the dwell, park once the arm has settled
        self.move_park = ParkMove(self, on_complete=self.on_park_complete)
        # The floor (helpers/floor.py): abort the trial and park if the apple is pulled below it. Armed
        # from the arrival at the start to the park; distance and speed checks are off (pluck_safety).
        self.safety = SafetyStopMonitor(self, param_prefix="pluck_safety", on_trip=self.on_safety_trip)

        # Trial data -- see README.md section 7, Data Collected.
        # disp_m is the dependent variable and is sampled here every state message, so the threshold
        # crossing can be interpolated offline to finer than the 10 ms cabinet stamp.
        # Dashboard / CLI control: status topic, pause between trials, live parameters, NSP codes.
        # See helpers/experiment_control.py; which parameters are live is in helpers/live_params.py.
        self.control = ExperimentControl(self, "apple_pluck")
        # A live start / park pose whose move would take the end effector below the floor is refused.
        self.control.add_validator(lambda n, v, c: self.move_park.check_live(n, v, c))
        self.control.on_reload(self._reload_live)
        self.control.on_stop_request(self._stop_now)   # Stop now: end the trial, park, hold
        self._stopping = False
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
                  "move_to_park.speed_scale": self.move_park.set_speed_scale}
        if self.perturbing:
            # perturb_bump moves faster -- a bump -- at its own grasp.bump_speed_scale.
            speeds["grasp.bump_speed_scale" if self.bump else "perturb_start.speed_scale"] = self.perturb.set_speed_scale
        self.control.apply_immediately(speeds, log=self.recorder.note_change)
        # Every move's actual speed (scaled limits, duration) goes into the trial sidecar's `moves`.
        for move in (self.move_to_start, self.move_park) + ((self.perturb,) if self.perturbing else ()):
            move.report_moves_to(self.recorder.note_move)
        if self.paradigm == "random_start":
            self._grid = StartGrid(self.move_to_start.robot)
            self._solve_start_grid(strict=True)
        self.get_logger().info(
            f"=== AUTOMATED MULTI-TRIAL APPLE PLUCK EXPERIMENT INITIALIZED (paradigm: {self.paradigm}) ===")
        # The run's first move: to park, out of reach, from wherever the arm is; then the trial loop.
        self.control.on_event("parking")
        self.move_park.start_first(lambda: self.control.begin_trial(self._next_trial))

    def _apply_settle(self):
        # perturb: the arm must settle where the perturbation left it before the pull baseline locks.
        # pluck / random_start: the quiet window has settled it. perturb_bump: no settling -- the
        # baseline is the perturbed spring position (on_perturb_complete).
        settle = (float(get_required_param(self, "perturb_start.baseline_settle_sec"))
                  if self.paradigm == "perturb" else 0.0)
        self.monitor.set_baseline_settle(settle)

    def _read_delay(self):
        """The delay before the perturbation, fixed or drawn per trial (uniformly from [min, max]).
        perturb: after the go cue, perturb_start.start_delay_*; perturb_bump: after the grasp,
        grasp.delay_* (default 0 -- the bump follows the grasp at once). Re-read between trials."""
        p = "grasp.delay_" if self.bump else "perturb_start.start_delay_"
        self._delay_mode = str(get_required_param(self, p + "mode")).strip().lower()
        if self._delay_mode not in ("fixed", "random"):
            raise ValueError(f"{p}mode must be fixed or random, got {self._delay_mode!r}")
        self._delay_fixed = float(get_required_param(self, p + "sec"))
        self._delay_range = (float(get_required_param(self, p + "min_sec")),
                             float(get_required_param(self, p + "max_sec")))

    def _start_delay(self):
        delay = (self._delay_fixed if self._delay_mode == "fixed"
                 else random.uniform(min(self._delay_range), max(self._delay_range)))
        self.perturb_delay.set_duration(delay)
        self.recorder.mark("perturb_delay_start", round(delay, 4))
        self.perturb_delay.start()

    def _solve_start_grid(self, strict: bool):
        """Solve random_start.positions ([a, b] in random_start.plane) to joints around the start pose.
        Only when the start pose has changed (it is live). A position that cannot be used stops the run
        at start-up (strict); later, the previous positions are kept and the reason is logged."""
        start = [float(v) for v in get_required_param(self, "move_to_start.target_joint_position")]
        if start == self._grid_start:
            return
        plane = str(get_required_param(self, "random_start.plane")).strip().lower()
        prefix = "random_start.positions."
        names = sorted(n for n in self.list_parameters([prefix[:-1]], 0).names if n.startswith(prefix))
        offsets = {n[len(prefix):]: [float(v) for v in get_required_param(self, n)] for n in names}
        if not offsets:
            raise ValueError("random_start.positions is empty")
        solved = self._grid.solve(start, plane, offsets)
        for n, (q, why) in solved.items():      # and the move there from park stays above the floor
            if why is None:
                solved[n] = (q, self.move_park.floor_problem(q, name=f"the move from park to {n}"))
        bad = [f"{n} {offsets[n]}: {why}" for n, (_, why) in solved.items() if why]
        if bad:
            msg = (f"random_start ({plane} plane, around start {start}): " + "; ".join(bad)
                   + ". Move those positions closer to the start, or check with solve_start_grid.py.")
            if strict:
                raise ValueError(msg)
            self.get_logger().error(msg + " Keeping the previous start positions.")
            return
        self.start_positions = {n: q for n, (q, _) in solved.items()}
        self._start_offsets = {n: {"plane": plane, "offset_m": offsets[n]} for n in offsets}
        self._grid_start = start
        self._start_queue = []
        self.get_logger().info(
            f"random_start: {len(offsets)} positions in the {plane} plane around the start: "
            + ", ".join(f"{n} {offsets[n]} m -> {q} deg" for n, q in self.start_positions.items()))

    def _check_resistance_base(self):
        """The resistance base must equal the stiffness the SmartPad app reports running with; if they
        differ, every pull feels softer or stiffer than the slider says. Checked every trial."""
        k = SmartPadLink.get(self.get_logger()).translational_stiffness()
        base = self.resistance.describe()["base_stiffness_n_per_m"]
        if k is not None and abs(k - base) > 0.5:
            self.get_logger().warn(
                f"resistance.base_stiffness_n_per_m is {base:g} N/m but the SmartPad app reports {k:g} N/m: "
                f"every pull feels {k / base:.2g}x the slider's stiffness. Set the base to {k:g} "
                f"(Per-run) or change the SmartPad profile.")

    def _next_start_position(self) -> str:
        # Shuffled blocks: every position once, in a random order, before any repeats.
        if not self._start_queue:
            self._start_queue = random.sample(list(self.start_positions), len(self.start_positions))
        return self._start_queue.pop()

    def start_trial(self):
        self._stopping = False
        self.trial_count += 1
        # Record the WHOLE trial, approach included -- analysis can trim, it cannot un-discard.
        sidecar = {"paradigm": self.paradigm, "threshold_m": self.monitor.threshold_m()}
        where = ""
        if self.paradigm == "random_start":
            name = self._next_start_position()
            pose = self.start_positions[name]
            self.move_to_start.set_target_deg(pose)
            sidecar.update(start_position=name, start_joint_deg=pose, **self._start_offsets[name])
            where = f", start {name}"
        self.get_logger().info(f"--- STARTING TRIAL {self.trial_count} ({self.paradigm}{where}) ---")
        if self.perturbing:
            sidecar["perturbation"] = {**self.perturb.perturbation_spec(),
                                       "trigger": "grasp" if self.bump else "go_cue",
                                       "delay_mode": self._delay_mode}
        self.recorder.start(trial_index=self.trial_count, **sidecar)
        self.recorder.annotate(resistance=self.resistance.describe())
        self._check_resistance_base()
        self.recorder.mark("trial_start", self.trial_count)
        self.move_to_start.start()

    def on_move_complete(self):
        self.get_logger().info("Arm returned to start. Waiting for a quiet window...")
        self.recorder.mark("at_start")
        self._check_elbow()
        self.safety.start()
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
        if self._stopping:
            return
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
        # No grasp in time: end the trial -- park, then the next trial starts as usual.
        self.grasp.stop()
        self.get_logger().info("No grasp within grasp.timeout_sec. Resetting for the next trial.")
        self.recorder.mark("grasp_timeout")
        self.recorder.mark("recover_start")
        self.safety.stop()
        self.move_park.start()

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
        self.get_logger().info("Threshold reached — the apple is plucked: the pull-back is released.")
        self.recorder.mark("snap", round(self.monitor.current_disp(), 6))
        self.audio_cue_snap.start()
        self.visual_cue_snap.start()
        self.recorder.mark("cue_snap")
        self.resistance.release()   # the equilibrium follows the arm: no pull-back
        self.recorder.mark("freeze")

    def on_monitor_complete(self):
        # Dwell elapsed. Park only once the arm has come to rest with the elbow bent: a park move that
        # starts while the arm is pulled straight can fold the elbow the wrong way (resistance_softener.py).
        self.recorder.mark("release_wait")
        self._wait_t0 = time.monotonic()
        self._park_wait = self.create_timer(0.05, self._park_when_settled)

    def _park_when_settled(self):
        ready = self.resistance.settled() and self.resistance.elbow_bent()
        if not ready and time.monotonic() - self._wait_t0 < self._park_wait_timeout:
            return
        self._cancel_park_wait()
        if not ready:
            self.get_logger().warn(
                f"The arm has not come to rest with the elbow bent within {self._park_wait_timeout:g} s "
                f"(elbow A4 {self.resistance.elbow_deg():.0f} deg): parking anyway.")
        self.get_logger().info("Apple plucked! Parking.")
        self._check_elbow()
        self.recorder.mark("recover_start")
        self.resistance.stop()
        self.safety.stop()
        self.move_park.start()

    def _cancel_park_wait(self):
        if self._park_wait is not None:
            self.destroy_timer(self._park_wait)
            self._park_wait = None

    def _check_elbow(self):
        """Record it if the elbow has folded the wrong way. The run goes on; the operator decides."""
        if self.resistance.elbow_flipped():
            self.recorder.mark("elbow_flipped", round(self.resistance.elbow_deg(), 1))
            self.get_logger().error(
                f"The elbow is folded the WRONG WAY (A4 {self.resistance.elbow_deg():+.0f} deg, commanded "
                "the other side): moves will swing and park will end a few cm off. Restarting the SmartPad "
                "app re-homes the arm.")

    def on_park_complete(self):
        self.get_logger().info(f"--- TRIAL {self.trial_count} COMPLETE ---")
        self.recorder.mark("trial_end", self.trial_count)
        self.recorder.stop_and_save()
        self.control.begin_trial(self._next_trial)   # holds at park instead if paused

    def _next_trial(self):
        self.move_park.wait_then(self.start_trial)    # wait move_to_park.hold_sec at park, then the next trial

    def _stop_now(self):
        """Stop now (the dashboard's Stop, or ~/stop): end the trial at once and park. The control is
        paused, so it then holds at park and the launch can end."""
        if self._stopping:
            return
        if self.move_park.cancel_hold():          # at park, waiting for the next trial: just hold
            self.control.begin_trial(self._next_trial)
            return
        if self.move_park.active():
            return                                # already on the way to park
        self._abort_trial("stop_requested")

    def on_safety_trip(self, reason: str):
        """The floor: the apple went below it. End the trial at once and park; the next trial follows
        as usual (no pause)."""
        if self._stopping or self.move_park.active():
            return
        self.get_logger().error(f"SAFETY ABORT ({reason}): parking.")
        self._abort_trial("safety_trip")

    def _abort_trial(self, mark: str):
        """End the trial in progress at once -- every action stopped -- and park."""
        self._stopping = True
        self.recorder.mark(mark)
        self.safety.stop()
        self.quiet_window.stop()
        if self.perturbing:
            self.perturb_delay.stop()
            self.perturb.cancel()
        if self.bump:
            self.grasp_timeout.stop()
            self.grasp.stop()
        self.move_to_start.cancel()
        self.monitor.stop()
        self._cancel_park_wait()
        self.resistance.stop()
        self.recorder.mark("recover_start")
        self.move_park.start()

    def _reload_live(self):
        """Re-read the live parameters (helpers/live_params.py). Runs between trials only."""
        for cue in (self.audio_cue, self.audio_cue_snap, self.visual_cue, self.visual_cue_snap):
            cue.reload()
        self.monitor.reload()
        self._apply_settle()
        self.resistance.reload()    # felt stiffness of the pull
        self.quiet_window.set_duration(float(get_required_param(self, "quiet_window_sec")))
        self.move_to_start.reload()      # speed limits and the start pose
        self.move_park.reload()
        if self.paradigm == "random_start":
            self._solve_start_grid(strict=False)   # if the start pose changed; start_trial picks after this
        if self.perturbing:
            self.perturb.reload()        # the perturbation: r, theta, plane
            if self.bump:
                self.perturb.set_speed_scale(float(get_required_param(self, "grasp.bump_speed_scale")))
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
