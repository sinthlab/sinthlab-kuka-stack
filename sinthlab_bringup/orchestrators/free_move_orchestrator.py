#!/usr/bin/env python3

import rclpy
from rclpy.node import Node as rclpyNode

from sinthlab_bringup.actions.move_to_position_joint_space import MoveToPositionJointSpace
from sinthlab_bringup.actions.park_move import ParkMove
from sinthlab_bringup.actions.switch_controller import SwitchControllerAction
from sinthlab_bringup.actions.admittance_move import AdmittanceMoveAction
from sinthlab_bringup.actions.safety_stop_monitor import SafetyStopMonitor
from sinthlab_bringup.actions.force_release_waiter import ForceReleaseWaiter
from sinthlab_bringup.helpers.live_params import CHOICE_GEOMETRY, choice_geometry_problem
from sinthlab_bringup.helpers.stiffness_profiles import SMARTPAD_PROFILES, required_profile
from sinthlab_bringup.actions.audio_cue import AudioCue
from sinthlab_bringup.actions.go_cue import GoCue
from sinthlab_bringup.actions.visual_cue import VisualCue
from sinthlab_bringup.actions.ring_state_cue import RingStateCue
from sinthlab_bringup.actions.centre_out import CentreOutMonitor
from sinthlab_bringup.actions.long_move import LongMoveMonitor
from sinthlab_bringup.actions.n_choice import NChoiceMonitor
from sinthlab_bringup.actions.free_reach import FreeReachMonitor
from sinthlab_bringup.actions.haptic_guidance import HapticGuidance
from sinthlab_bringup.actions.wait_action import WaitAction
from sinthlab_bringup.actions.quiet_gate import QuietGate
from sinthlab_bringup.actions.trial_recorder import TrialRecorder
from sinthlab_bringup.helpers.node_utils import get_required_param, build_node
from sinthlab_bringup.helpers.experiment_control import ExperimentControl, REWARD_EVENTS

JOINT_CTRL = "lbr_joint_position_command_controller"
CLIK_CTRL = "kuka_clik_controller"
REWARD_FLASH_SEC = 1.0      # a reward mid-session (Long moves, New best): the reward cue shows this long


class FreeMoveOrchestratorNode(rclpyNode):
    """Pre-training: the arm in admittance -- it goes wherever it is pushed and stays where it is let go.
    No threshold. A trial is one free-movement SESSION that runs for session_sec, or, with session_sec
    0, until Stop.

    One node, several tasks, set by the YAML's `experiment`: free_move (in a box around the start);
    restricted_move_horizontal / _vertical (the same admittance on one rail through the start,
    free_move.rail_axis, a session per trial); move_horizontal / move_vertical (the Cued Reaches card: that
    rail with one centre-out reach per trial); colour_rule; n_choice; reach_map (Free Move in the frontal
    plane, rewarding long movements or each new best reach per direction: maps how far the monkey reaches);
    move_free (the Cued Reaches card's free variant: one reach per trial, free in 3-D, with jump and bump
    perturbations: actions/free_reach.py).

    REWARD. Free Move: a session that runs its full session_sec. Move Vertical / Horizontal with
    centre_out.enabled: ONE CENTRE-OUT REACH PER TRIAL (actions/centre_out.py) -- the go cue lights the
    whole ring (either end) or the cued end's quarter; the first end the arm reaches ends the trial: a goal
    end gives the reward (`reach`), the other end none (`reach_wrong`); then the release wait and park, as
    any trial. No reach within response_timeout_sec ends it too; session_sec does not apply. Restricted
    cued (centre_out.target restricted) closes the other half of the rail, so the arm only moves toward
    the cue; cued end leaves it open. Free Move with long_move.reward
    each_move (actions/long_move.py): every movement of at least long_move.min_m is rewarded as it ends,
    with the reward tone, and the session's end pays nothing.

    [first: move_park, slowly, from wherever the arm is]
    move_to_start (JOINT) -> switch to CLIK -> hold + measure the resting force (tare) during the
    quiet window -> go cue (ring GREEN) -> admittance + runaway (speed) monitor
    -> session ended or a reach / choice (end tone; with a reward, the reward cue: the whole ring WHITE, the same
       in every task): the arm holds where it is
    -> release wait (ring dark at release)
    -> switch to JOINT -> park (out of reach) -> the next session.
    """

    def __init__(self) -> None:
        super().__init__("free_move_orchestrator", automatically_declare_parameters_from_overrides=True)
        AudioCue.warmup()
        VisualCue.warmup(self)
        self.experiment = str(get_required_param(self, "experiment"))   # see the class doc

        geometry = {k: get_required_param(self, k) for k in CHOICE_GEOMETRY if self.has_parameter(k)}
        problem = choice_geometry_problem(geometry)
        if problem:
            raise ValueError(problem)

        self.trial_count = 0
        self._trial_ending = False
        self._end_tone_pending = False
        self._ended = False

        self.move_to_start = MoveToPositionJointSpace(
            self, param_prefix="move_to_start", on_complete=self.on_move_complete)
        self.switch_to_fixture = SwitchControllerAction(
            self, activate=[CLIK_CTRL], deactivate=[JOINT_CTRL],
            on_complete=self.on_switched_to_fixture, name="switch->clik")
        self.admittance = AdmittanceMoveAction(self, param_prefix="free_move")
        self.ring = RingStateCue(self)
        # The quiet window, guarded: moving or pushing the apple in it ends the trial (on_early_touch).
        self.quiet_window = QuietGate(
            self, on_complete=self.on_quiet_window_complete, on_broken=self.on_early_touch)
        # No response: the arm not moved more than quiet_gate.move_m within response_timeout_sec of the
        # go cue ends the session (no end cue, park). Once it has been moved, the session runs on.
        self.centre_out = None             # Move Vertical / Horizontal: the centre-out reach, created below
        self.long_move = None              # Free Move: the long-movement reward, created below
        self.n_choice = None               # N-Choice: one choice per trial on the asterisk, created below
        self.free_reach = None             # Cued Reaches, free: one reach per trial in 3-D, created below
        self.response_timeout = WaitAction(
            self, duration_sec=float(get_required_param(self, "response_timeout_sec")),
            on_complete=self.on_response_timeout, name="response_timeout")
        self.go_cue = AudioCue(
            self, param_prefix="audio_cue_play", on_complete=None,
            on_finished=lambda secs: self.recorder.mark("cue_audio_end", round(secs, 4)))

        self.control = ExperimentControl(self, self.experiment)
        # A live start / park pose whose move would take the end effector below the floor is refused.
        self.control.add_validator(lambda n, v, c: self.move_park.check_live(n, v, c))
        self.control.on_reload(self._reload_live)
        self.control.on_stop_request(self._stop_now)   # Stop: end the trial, park, hold
        self._stopping = False
        self._session_live = False
        self.recorder = TrialRecorder(
            self, experiment=self.experiment,
            extra_header=self.admittance.record_extra_header(), extra_fn=self.admittance.record_extra,
            on_event=self.control.on_event)
        self.go = GoCue(self, self.recorder, self.quiet_window, self.go_cue, self.on_go_complete)
        # The centre-out reach on the rail (Move Vertical / Horizontal only: Free Move has no centre_out block).
        self.haptic = None                 # Move Vertical / Horizontal: buzz harder nearing the goal end
        if self.has_parameter("centre_out.enabled"):
            self.centre_out = CentreOutMonitor(self, self.admittance)
            self.control.add_validator(self.centre_out.check_live)   # e.g. no "left" on the vertical rail
            if self.experiment == "colour_rule" and not self.centre_out.rule_on():   # its own task: the rule
                raise ValueError("colour_rule.yaml must have centre_out.target: rule (the Colour Rule task "
                                 "always runs the colour rule)")
            if self.has_parameter("haptic.enabled"):
                self.haptic = HapticGuidance(self, self.centre_out.distance_to_goal)
        # The long-movement reward (Free Move only: the rails have no long_move block).
        if self.has_parameter("long_move.reward"):
            self.long_move = LongMoveMonitor(self, self.admittance)
            self.control.add_validator(self.long_move.check_live)
            self.reward_cue = AudioCue(self, param_prefix="audio_cue_reward", on_complete=lambda: None)
            self.reward_flash = WaitAction(self, duration_sec=REWARD_FLASH_SEC, on_complete=self._end_reward_flash,
                                           name="reward_flash")
        # The N-choice task (actions/n_choice.py): the asterisk of rails, depleting rewards.
        if self.has_parameter("n_choice.p_max"):
            self.n_choice = NChoiceMonitor(self, self.admittance)
            self.control.add_validator(self.n_choice.check_live)
        # Cued Reaches, free (actions/free_reach.py): one reach per trial in 3-D, jumps and bumps.
        if self.has_parameter("free_reach.cue"):
            self.free_reach = FreeReachMonitor(self, self.admittance, self.recorder.mark)
            self.control.add_validator(self.free_reach.check_live)
        self._set_reward_events()

        self.safety = SafetyStopMonitor(self, param_prefix="free_move_safety", on_trip=self.on_safety_trip)
        self.end_cue = AudioCue(self, param_prefix="audio_cue_end", on_complete=lambda: None)
        self.session = WaitAction(          # only used when session_sec > 0
            self, duration_sec=max(float(get_required_param(self, "session_sec")), 1.0),
            on_complete=self.on_session_over, name="session")
        self._session_sec = float(get_required_param(self, "session_sec"))
        self.force_release = ForceReleaseWaiter(
            self, param_prefix="force_release", on_complete=self.on_force_released)
        self.switch_to_joint = SwitchControllerAction(
            self, activate=[JOINT_CTRL], deactivate=[CLIK_CTRL],
            on_complete=self.on_switched_to_joint, name="switch->joint")
        self.move_park = ParkMove(self, on_complete=self.on_park_complete)

        # Move speed sliders act at once, from each move's next start (live_params.IMMEDIATE).
        self.control.apply_immediately({
            "move_to_start.speed_scale": self.move_to_start.set_speed_scale,
            "move_to_park.speed_scale": self.move_park.set_speed_scale,
        }, log=self.recorder.note_change)
        # Every move's actual speed (scaled limits, duration) goes into the trial sidecar's `moves`.
        for move in (self.move_to_start, self.move_park):
            move.report_moves_to(self.recorder.note_move)
        self.get_logger().info(f"=== PRE-TRAINING: {self.experiment.replace('_', ' ').upper()} INITIALIZED ===")
        # The run's first move: to park, out of reach, from wherever the arm is; then the trial loop.
        self.control.on_event("parking")
        self.move_park.start_first(lambda: self.control.begin_trial(self._next_trial))

    def start_trial(self):
        self.ring.show("off")
        self.move_to_start.start()

    def on_move_complete(self):
        self.recorder.mark("at_start")
        self.switch_to_fixture.start()

    def on_switched_to_fixture(self):
        if self._stopping:                 # Stop came during the switch: straight back
            self.switch_to_joint.start()
            return
        # Hold still and measure the resting force while the quiet window runs; the arm only
        # starts following the hand at the go cue. Keep quiet_window_sec >= free_move.tare_sec.
        self.recorder.mark("fixture_active")
        self.admittance.start()
        self.quiet_window.start()

    def on_quiet_window_complete(self):
        if not self._smartpad_ok():        # checked before the go cue: no "go" for an arm that will not move
            return
        if self._centre_out_on():          # this trial's goal: drawn now, lit with the go cue
            target = self.centre_out.pick_target()
            # One-direction trial: close the other half of the rail before the arm may move. Marked
            # before `target`, so the dashboard's phase reads "the goal lit".
            self.admittance.set_open_span(self.centre_out.open_span())
            closed = self.centre_out.closed_end()
            if closed is not None:
                self.recorder.mark("closed_end", closed)
            if self.centre_out.rule_on():
                # The colour rule: the state the trial runs under (mapping, block, criterion) goes into its
                # record; a reversal is marked on the first trial after it. Nothing of it reaches the animal.
                self.recorder.annotate(rule=self.centre_out.rule_state())
                mapping = self.centre_out.take_reversal()
                if mapping is not None:
                    self.recorder.mark("reversal", mapping)
                self.recorder.mark("cue_colour", self.centre_out.cue_colour())
            self.recorder.mark("target", target)
            state, quarters = self.centre_out.ring_cue()
            self.go.start(lambda label: self.ring.show(state, quarters, label=label),
                          visual_enabled=self.ring.enabled())
        elif self._free_reach_on():        # this trial's plan: direction and any perturbation, lit with the go cue
            plan = self.free_reach.pick()
            self.recorder.annotate(free_reach={**plan.record(), "cue_delay_sec": self.free_reach.cue_delay()})
            self.recorder.mark("target", self.free_reach.target())
            quarters = self.free_reach.quarters()
            self.go.start(lambda label: self.ring.show("go", quarters, label=label),
                          visual_enabled=self.ring.enabled(), preview_sec=self.free_reach.cue_delay())
        else:
            if self._n_choice_on():        # every direction's reward probability, before the choice
                self.recorder.annotate(n_choice=self.n_choice.state())
            self.go.start(lambda label: self.ring.show("go", label=label),
                          visual_enabled=self.ring.enabled())

    def _centre_out_on(self) -> bool:
        return self.centre_out is not None and self.centre_out.enabled

    def _n_choice_on(self) -> bool:
        return self.n_choice is not None

    def _free_reach_on(self) -> bool:
        return self.free_reach is not None

    def _choice_on(self) -> bool:
        """One choice per trial (centre-out, the colour rule, N-choice, the free reach), rather than a free session."""
        return self._centre_out_on() or self._n_choice_on() or self._free_reach_on()

    def on_free_reach(self, correct: bool, angle_deg: float):
        """The free reach went its distance -- the trial's outcome, as a rail reach's: `reach` (the reward) or
        `reach_wrong`, with the direction it ended in [deg]."""
        self.recorder.annotate(free_reach_result=self.free_reach.result())
        self._end_trial("reach" if correct else "reach_wrong", angle_deg)

    def on_choice(self, direction: str, rewarded: bool):
        """N-choice: the arm reached the end of a ray -- the trial's outcome. `choice` sends the reward."""
        self._end_trial("choice" if rewarded else "choice_unrewarded", direction)

    def _long_move_on(self) -> bool:
        return self.long_move is not None and self.long_move.enabled

    def _set_reward_events(self):
        """Centre-out on: the reach to a goal end gives the reward, not the session's end. Long moves on:
        each long movement does."""
        self.control.set_reward_events(("reach",) if self._centre_out_on() or self._free_reach_on() else
                                       ("choice",) if self._n_choice_on() else
                                       ("long_move", "new_best") if self._long_move_on() else REWARD_EVENTS)

    def _on_long_move(self, token: str, arg=None):
        """Long moves: a movement started / ended / earned the reward (the pulse follows the mark). A reward
        mid-session flashes the reward cue (the whole ring white) for REWARD_FLASH_SEC."""
        self.recorder.mark(token, arg)
        if token in ("long_move", "new_best"):
            self.reward_cue.start()
            if self.control.pays(token):
                self.ring.reward()
                self.reward_flash.start()

    def _end_reward_flash(self):
        if self._session_live and self.ring.state() == "reward":
            self.ring.show("off")

    def on_reach(self, end: str, rewarded: bool):
        """Centre-out: the arm reached an end of the rail -- the trial's outcome."""
        self._end_trial("reach" if rewarded else "reach_wrong", end)

    def on_go_complete(self):
        if self._stopping:
            return
        self.recorder.mark("armed")
        self.admittance.enable_motion()
        self.safety.start()
        self.response_timeout.start()
        self._session_live = True
        if self._centre_out_on():
            self.centre_out.start(self.on_reach)
            if self.haptic is not None:
                self.haptic.start()
            self.get_logger().info("Centre-out: reach to the lit end of the rail.")
        elif self._n_choice_on():
            self.n_choice.start(self.on_choice)
            self.get_logger().info("N-choice: reach out along one of the rails.")
        elif self._free_reach_on():
            self.free_reach.start(self.on_free_reach)
            self.get_logger().info(f"Free reach: reach {self.free_reach.target()}.")
        elif self._session_sec > 0:
            self.session.start()
            self.get_logger().info(f"Free move: the arm follows the hand for {self._session_sec:.0f} s.")
        else:
            self.get_logger().info("Free move: the arm follows the hand until Stop.")
        if self._long_move_on():
            self.long_move.start(self._on_long_move)

    def _smartpad_ok(self) -> bool:
        """Check the task's six-axis stiffness diagonal before opening the response window."""
        need = float(get_required_param(self, "free_move.smartpad_stiffness_n_per_m"))
        if need <= 0.0:
            return True
        name = required_profile(self.experiment)
        expected = SMARTPAD_PROFILES[name]
        problem = (f"configured X stiffness {need:g} disagrees with {name}" if need != expected[0]
                   else self.control.smartpad().stiffness_problem(expected))
        if problem is None:
            return True
        self.get_logger().error(f"Cannot start trial: {problem}. Check the cabinet report and select {name}.")
        self.control.hold(problem)
        self._end_trial("smartpad_mismatch")
        return False

    def on_session_over(self):
        self._end_trial("session_end")

    def on_response_timeout(self):
        """response_timeout_sec after the go cue: if the arm has not been moved more than quiet_gate.move_m,
        end the session without the end cue and park. If it has, nothing happens -- except in centre-out,
        where the response is the reach itself: no reach by then ends the trial."""
        if self._trial_ending or self._stopping:
            return
        if self._choice_on():
            self.get_logger().warn(f"NO REACH within {self.response_timeout.duration_sec():g} s: waiting for release before parking.")
            self._end_trial("response_timeout")
            return
        moved = self.admittance.max_excursion_m()
        if moved > float(get_required_param(self, "quiet_gate.move_m")):
            return
        self.get_logger().warn(
            f"NO RESPONSE within {self.response_timeout.duration_sec():g} s (arm moved "
            f"{moved * 100:.1f} cm): parking.")
        self._end_trial("response_timeout")

    def on_early_touch(self, why: str):
        """The apple was moved or pushed during the quiet window (actions/quiet_gate.py): end the trial
        without reward, park, and add the penalty wait before the next trial."""
        self.go.cancel()
        self.go.warn_early_touch()
        self.get_logger().warn(f"EARLY TOUCH ({why}): no reward, parking, then a penalty wait.")
        self.move_park.penalize(float(get_required_param(self, "quiet_gate.penalty_sec")))
        self._end_trial("early_touch")

    def on_safety_trip(self, reason: str):
        self._end_trial("safety")

    def _end_trial(self, reason: str, arg=None):
        if self._trial_ending:
            return
        self._trial_ending = True
        self._session_live = False
        self._trial_over(reason if reason != "safety" else "safety_trip", arg)
        if self.centre_out is not None:
            self.centre_out.stop()
        if self.long_move is not None:
            self.long_move.stop()
        if self.n_choice is not None:
            self.n_choice.stop()
        if self.free_reach is not None:
            self.free_reach.stop()
        if self.haptic is not None:
            self.haptic.stop()
        self.session.stop()
        self.response_timeout.stop()
        self.safety.stop()
        self.admittance.stop()          # holds where it is
        # The reward cue, the same in every task: the whole ring white with a reward that is given, until the
        # hand lets go. No reward (a wrong reach, an unpaid choice, a time-out): the ring goes dark.
        if self.control.pays(reason):
            self.ring.reward()
        else:
            self.ring.show("off")
        if reason in ("safety", "early_touch", "smartpad_mismatch") or (
                reason == "response_timeout" and not self._choice_on()):
            if reason == "safety":
                self.get_logger().error("SAFETY ABORT: parking immediately.")
            self.switch_to_joint.start()
            return
        # A completed session or a reach: the arm holds where it is until the hand lets go, then parks.
        self.get_logger().info("Session over. The arm holds; waiting for release before reset."
                               if reason == "session_end" else "The arm holds; waiting for release before reset.")
        if reason in ("choice", "choice_unrewarded", "reach", "reach_wrong"):
            self.end_cue.start()
        self._end_tone_pending = reason == "session_end"
        self.recorder.mark("release_wait")
        self.force_release.start()

    def on_force_released(self):
        self.recorder.mark("released")
        self.ring.show("off")
        self.switch_to_joint.start()

    def on_switched_to_joint(self):
        if self._end_tone_pending:
            self.end_cue.start()
        self._end_tone_pending = False
        self.move_park.start()

    def on_park_complete(self):
        self.get_logger().info(f"--- {'TRIAL' if self._choice_on() else 'SESSION'} {self.trial_count} COMPLETE ---")
        self._mark_end()                # normally sent already, at the outcome
        self.recorder.stop_and_save()
        self.control.begin_trial(self._next_trial)   # holds at park instead once stopped

    def _next_trial(self):
        """The trial starts at park: its file opens and trial_start goes to the NSP as the wait before the
        move to the start pose begins (move_to_park.hold_sec, plus any early-touch penalty wait)."""
        self._stopping = False
        self._trial_ending = False
        self._end_tone_pending = False
        self._ended = False
        self.trial_count += 1
        self.recorder.start(trial_index=self.trial_count)
        self.recorder.mark("trial_start", self.trial_count)
        self.get_logger().info(f"--- STARTING {'TRIAL' if self._choice_on() else 'SESSION'} {self.trial_count} ---")
        self.move_park.wait_then(self.start_trial)    # wait move_to_park.hold_sec at park, then move to start

    def _trial_over(self, why: str, arg=None):
        """The trial's outcome -- success or failure: log it and send trial_end (NSP sync), once a trial.
        The park move that follows is still in the trial's file."""
        self.go.cancel()
        self.recorder.mark(why, arg)
        self._mark_end()

    def _mark_end(self):
        if not self._ended:
            self._ended = True
            self.recorder.mark("trial_end", self.trial_count)

    def _stop_now(self):
        """Stop (the dashboard's Stop, or ~/stop): end the trial at once and park -- no cue, no release
        wait. No further trial starts, so the arm then holds at park and the launch can end."""
        self._end_tone_pending = False
        if self._stopping:
            return
        if self.move_park.cancel_hold():          # at park, waiting for the next trial: end it there, hold
            self._trial_over(self.control.stop_token())
            self.recorder.stop_and_save()
            self.control.begin_trial(self._next_trial)
            return
        if self.move_park.active() or self.switch_to_joint.active():
            return                                # already on the way to park
        self._stopping = True
        self._trial_over(self.control.stop_token())
        self._trial_ending = True
        self._session_live = False
        self.quiet_window.stop()
        if self.centre_out is not None:
            self.centre_out.stop()
        if self.long_move is not None:
            self.long_move.stop()
        if self.n_choice is not None:
            self.n_choice.stop()
        if self.free_reach is not None:
            self.free_reach.stop()
        if self.haptic is not None:
            self.haptic.stop()
        self.session.stop()
        self.response_timeout.stop()
        self.safety.stop()
        self.admittance.stop()
        self.force_release.stop()
        self.ring.show("off")
        if self.move_to_start.active():           # still on the joint controller
            self.move_to_start.cancel()
            self.recorder.mark("recover_start")
            self.move_park.start()
        elif not self.switch_to_fixture.active():  # on the CLIK: back to joint, then park
            self.switch_to_joint.start()
        # (mid-switch to the CLIK, on_switched_to_fixture turns back)

    def _reload_live(self):
        for cue in (self.go_cue, self.end_cue):
            cue.reload()
        self.ring.reload()
        self.quiet_window.set_duration(float(get_required_param(self, "quiet_window_sec")))
        self.quiet_window.reload()
        self.move_to_start.reload()
        self.move_park.reload()
        self._session_sec = float(get_required_param(self, "session_sec"))
        self.session.set_duration(max(self._session_sec, 1.0))
        self.response_timeout.set_duration(float(get_required_param(self, "response_timeout_sec")))
        self.admittance.reload()
        if self.centre_out is not None:
            self.centre_out.reload()
        if self.long_move is not None:
            self.long_move.reload()
            self.reward_cue.reload()
        if self.n_choice is not None:
            self.n_choice.reload()
        if self.free_reach is not None:
            self.free_reach.reload()
        if self.haptic is not None:
            self.haptic.reload()
        self._set_reward_events()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = build_node(FreeMoveOrchestratorNode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
