#!/usr/bin/env python3

import rclpy
from rclpy.node import Node as rclpyNode

from sinthlab_bringup.actions.move_to_position_joint_space import MoveToPositionJointSpace
from sinthlab_bringup.actions.audio_cue import AudioCue
from sinthlab_bringup.actions.wait_action import WaitAction

class DummyPTPNode(rclpyNode):
	"""Dummy PTP moves task
trial_step 0
.		move_to_start -> audio_cue_ready -> 
.				 quiet_window_ready ->  ... 
.
trial_step 1	move_step1 -> audio_cue -> 				
.			      quiet_window (1s) -> ...	
trial_step 2	move_step2 -> audio_cue -> 				
.			      quiet_window (1s) -> ...
trial_step 3	move_step3 -> audio_cue -> 
.			      quiet_window (1s) -> ...
trial_step 4	move_step4 -> audio_cue -> 
.			      quiet_window (1s) -> ...
trial_step 5	
		move_recover -> repeat
	"""

	def __init__(self):
		super().__init__(
			"dummy_ptp_orchestrator",
			automatically_declare_parameters_from_overrides=True,
		)

		AudioCue.warmup(self) # wake the WSL2 audio driver so the first cue isn't delayed

		self.trial_count = 0
		self.trial_step  = 0
		# ====== Actions
		# -------------------------------------------------------------------------------
		self.move_to_start = MoveToPositionJointSpace(
			self, param_prefix="move_to_start", on_complete=self.on_move2start_complete
			)
		self.quiet_window_ready = WaitAction(
			self, duration_sec=2.0, on_complete=self.on_quiet_window_ready_complete, name="quiet_window_ready"
			)
		self.audio_cue_ready = AudioCue(
			self, param_prefix="audio_cue_ready", on_complete=lambda: None
			)
		# -------------------------------------------------------------------------------
		self.quiet_window_mid = WaitAction(
			self, duration_sec=1.0, on_complete=self.on_quiet_window_mid_complete, name="quiet_window_mid"
			)
		self.audio_cue_mid = AudioCue(
			self, param_prefix="audio_cue_mid", on_complete=lambda: None
			)
		# -------------------------------------------------------------------------------
		self.move_step1 = MoveToPositionJointSpace(
			self, param_prefix="move_step1", on_complete=self.on_move_step1_complete
			)
		self.move_step2 = MoveToPositionJointSpace(
			self, param_prefix="move_step2", on_complete=self.on_move_step2_complete
			)
		self.move_step3 = MoveToPositionJointSpace(
			self, param_prefix="move_step3", on_complete=self.on_move_step3_complete
			)
		self.move_step4 = MoveToPositionJointSpace(
			self, param_prefix="move_step4", on_complete=self.on_move_step4_complete
			)
		# -------------------------------------------------------------------------------
		self.move_recover = MoveToPositionJointSpace(
			self, param_prefix="move_to_start_recover", on_complete=self.on_recover_complete
			)
		self.get_logger().info("=== Experiment Started ===")
		self.start_trial()

	# ======= Defs
	def start_trial(self):
		self.trial_count += 1
		self.get_logger().info(f" === Trial Number {self.trial_count:03d} started! ===")
		self.move_to_start.start()
	def on_move2start_complete(self):
		self.get_logger().info("Arm moved to start position!")
		self.quiet_window_ready.start()
		self.audio_cue_ready.start()
	def on_quiet_window_ready_complete(self):
		self.get_logger().info("Trial ready! (Quiet window completed)")
		self.trial_step = 1
		self.move_step1.start()
	# -------------------------------------------------------------------------------
	def on_move_step1_complete(self):
		self.trial_step = 2
		self.get_logger().info("Arm moved to FIRST position!")
		self.quiet_window_mid.start()
		self.audio_cue_mid.start()
	def on_move_step2_complete(self):
		self.trial_step = 3
		self.get_logger().info("Arm moved to SECOND position!")
		self.quiet_window_mid.start()
		self.audio_cue_mid.start()
	def on_move_step3_complete(self):
		self.trial_step = 4
		self.get_logger().info("Arm moved to THIRD position!")
		self.quiet_window_mid.start()
		self.audio_cue_mid.start()
	def on_move_step4_complete(self):
		self.trial_step = 5
		self.get_logger().info("Arm moved to FOURTH position!")
		self.quiet_window_mid.start()
		self.audio_cue_mid.start()
	def on_quiet_window_mid_complete(self):
		if self.trial_step == 2:
			self.move_step2.start()
		elif self.trial_step == 3:
			self.move_step3.start()
		elif self.trial_step == 4:
			self.move_step4.start()
		elif self.trial_step == 5:
			self.move_recover.start()
	# -------------------------------------------------------------------------------
	def on_recover_complete(self):
		self.get_logger().info(f"--- TRIAL {self.trial_count} COMPLETE ---")
		self.start_trial()

def main(args=None) -> None:
    rclpy.init(args=args)
    node = DummyPTPNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()