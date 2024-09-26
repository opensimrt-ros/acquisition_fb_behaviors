#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from acquisition_fb_flexbe_behaviors.imu_startup_sequence_sm import imu_startup_sequenceSM
from acquisition_fb_flexbe_states.check_if_alive import HostAliveState
from acquisition_fb_flexbe_states.multi_service_call_state import MultiServiceCallState
from acquisition_fb_flexbe_states.play_sound_state import PlaySoundState
from acquisition_fb_flexbe_states.set_name_and_path_state import MultiSetNameAndPathState
from acquisition_fb_flexbe_states.tmux_setup_from_yaml_state import TmuxSetupFromYamlState
from flexbe_states.calculation_state import CalculationState
from flexbe_states.check_condition_state import CheckConditionState
from flexbe_states.log_state import LogState
from flexbe_states.operator_decision_state import OperatorDecisionState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Apr 22 2024
@author: frekle
'''
class acquire_imu_and_ikSM(Behavior):
	'''
	acquire using embeddable IMU behavior
- with tmux 
- IMUs with upright start
- IK using old heading 
- playing sounds
	'''


	def __init__(self):
		super(acquire_imu_and_ikSM, self).__init__()
		self.name = 'acquire_imu_and_ik'

		# parameters of this behavior
		self.add_parameter('activity_name', 'walking')
		self.add_parameter('activity_duration', 10)
		self.add_parameter('subject_num', 0)
		self.add_parameter('num_reps', 1)
		self.add_parameter('run_nodes', True)

		# references to used behaviors
		self.add_behavior(imu_startup_sequenceSM, 'imu_startup_sequence')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:

		# ! 41 513 
		# !!!! here we need to also clear all the started nodes so we are back in the beginning, |n|nwe can kill nodes by name with rosnode kill |n|nand we can do a cleanup with they are dangling with rosnode cleanuo

		# ! 657 14 /Recording_trial
		# Missing sending the start time to everyone so that we have a very similar setup to the playback

		# O 265 11 /Calibration_and_Heading
		# Right now we are not using these results to calibrate the IK node just yet. |n|nThe only guys that use this are the resolve headings service to show the imus and the external heading calibrator which uses the pelvis avg quaternion

		# O 335 93 /Check_If_Devices_Are_On
		# TODO: This should be a part of the device monitoring bit, so don't have to run this as a state and also since they may fail at any point

		# ! 273 7 
		# TODO:Here we also need to make sure we are loading the correct models every time!

		# O 519 273 /Calibration_and_Heading
		# This published the tfs for showing the IMUs on rViz|n|nNot really necessary, since we are not using this inside the node just yet

		# O 500 109 /Calibration_and_Heading
		# We are using just the pelvis for heading. This is maybe not ideal, since a combined heading of more imus maybe is better



	def create(self):
		save_dir = "/srv/host_data/tmp"
		tmux_yaml_path = "/catkin_ws/src/ros_biomech/acquisition_state_machines/acquisition_of_raw_data/config/"
		tmux_yaml_file = "plus_ik.yaml"
		imu_list = ["torso","pelvis","femur_r","tibia_r","talus_r","femur_l","tibia_l","talus_l"]
		node_start_list = ["/ik"]
		calib_sound_file = "/srv/host_data/calib.wav"
		# x:1421 y:812, x:162 y:458
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.activity_counter = 0
		_state_machine.userdata.activity_save_dir = ""
		_state_machine.userdata.activity_save_name = ""
		_state_machine.userdata.load_nodes = self.run_nodes

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:71 y:247, x:559 y:584
		_sm_recording_trial_0 = OperatableStateMachine(outcomes=['failed', 'done'])

		with _sm_recording_trial_0:
			# x:488 y:12
			OperatableStateMachine.add('start_recording_srv',
										MultiServiceCallState(multi_service_list=node_start_list, predicate="/start_recording", prefix=""),
										transitions={'done': 'start_recording', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:486 y:454
			OperatableStateMachine.add('clear_loggers',
										MultiServiceCallState(multi_service_list=node_start_list, predicate="/clear_loggers", prefix=""),
										transitions={'done': 'done', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:520 y:87
			OperatableStateMachine.add('start_recording',
										LogState(text="Start Recording", severity=Logger.REPORT_HINT),
										transitions={'done': 'Record_time'},
										autonomy={'done': Autonomy.Off})

			# x:493 y:261
			OperatableStateMachine.add('stop_recording_srv',
										MultiServiceCallState(multi_service_list=node_start_list, predicate="/stop_recording", prefix=""),
										transitions={'done': 'write_sto_srv', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:486 y:367
			OperatableStateMachine.add('write_sto_srv',
										MultiServiceCallState(multi_service_list=node_start_list, predicate="/write_sto", prefix=""),
										transitions={'done': 'clear_loggers', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:524 y:156
			OperatableStateMachine.add('Record_time',
										WaitState(wait_time=self.activity_duration),
										transitions={'done': 'stop_recording_srv'},
										autonomy={'done': Autonomy.Off})


		# x:264 y:58, x:130 y:432
		_sm_check_if_devices_are_on_1 = OperatableStateMachine(outcomes=['done', 'fail'])

		with _sm_check_if_devices_are_on_1:
			# x:30 y:42
			OperatableStateMachine.add('is_router_on',
										HostAliveState(hostname="192.168.1.1", waittime=200),
										transitions={'continue': 'done', 'failed': 'turn_on_router'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:61 y:179
			OperatableStateMachine.add('turn_on_router',
										LogState(text="turn on router!!", severity=Logger.REPORT_ERROR),
										transitions={'done': 'fail'},
										autonomy={'done': Autonomy.Off})



		with _state_machine:
			# x:64 y:28
			OperatableStateMachine.add('Model_Scaling_Not_Implemented',
										LogState(text="Model scaling is not implemented. We want to scale the osim, urdf model and generate the moment arm library in this step. ", severity=Logger.REPORT_WARN),
										transitions={'done': 'run_nodes'},
										autonomy={'done': Autonomy.Off})

			# x:636 y:17
			OperatableStateMachine.add('Check_If_Devices_Are_On',
										_sm_check_if_devices_are_on_1,
										transitions={'done': 'imu_startup_sequence', 'fail': 'failed'},
										autonomy={'done': Autonomy.Inherit, 'fail': Autonomy.Inherit})

			# x:666 y:798
			OperatableStateMachine.add('Recording_trial',
										_sm_recording_trial_0,
										transitions={'failed': 'failed', 'done': 'record_another'},
										autonomy={'failed': Autonomy.Inherit, 'done': Autonomy.Inherit})

			# x:642 y:639
			OperatableStateMachine.add('Set_Trial_Filenames_and_Path',
										MultiSetNameAndPathState(multi_service_list=node_start_list, prefix="", suffix="/set_name_and_path", activity_name=self.activity_name, save_dir=save_dir, subject_num=self.subject_num),
										transitions={'done': 'Start_Recording_Question_Mark', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'activity_counter': 'activity_counter', 'activity_save_dir': 'activity_save_dir', 'activity_save_name': 'activity_save_name'})

			# x:642 y:720
			OperatableStateMachine.add('Start_Recording_Question_Mark',
										LogState(text="Is the calibration and the heading OK?\n Proceeding will start recording the trial", severity=Logger.REPORT_HINT),
										transitions={'done': 'Recording_trial'},
										autonomy={'done': Autonomy.Full})

			# x:929 y:546
			OperatableStateMachine.add('addone_to_num_reps',
										CalculationState(calculation=lambda x: x+1),
										transitions={'done': 'calibrate_ik'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'activity_counter', 'output_value': 'activity_counter'})

			# x:647 y:538
			OperatableStateMachine.add('calibrate_ik',
										MultiServiceCallState(multi_service_list="/ik", predicate="/calibrate", prefix=""),
										transitions={'done': 'Set_Trial_Filenames_and_Path', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:645 y:161
			OperatableStateMachine.add('imu_startup_sequence',
										self.use_behavior(imu_startup_sequenceSM, 'imu_startup_sequence'),
										transitions={'finished': 'start_parked_nodes', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'imu_list': 'imu_list'})

			# x:310 y:143
			OperatableStateMachine.add('load_nodes',
										TmuxSetupFromYamlState(session_name="testtt", startup_yaml=tmux_yaml_path+tmux_yaml_file),
										transitions={'continue': 'Check_If_Devices_Are_On', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:928 y:801
			OperatableStateMachine.add('record_another',
										OperatorDecisionState(outcomes=["yes", "no"], hint=None, suggestion=None),
										transitions={'yes': 'addone_to_num_reps', 'no': 'finished'},
										autonomy={'yes': Autonomy.Off, 'no': Autonomy.Off})

			# x:66 y:126
			OperatableStateMachine.add('run_nodes',
										CheckConditionState(predicate=lambda x: bool(x)),
										transitions={'true': 'load_nodes', 'false': 'Check_If_Devices_Are_On'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'input_value': 'load_nodes'})

			# x:647 y:306
			OperatableStateMachine.add('start_parked_nodes',
										MultiServiceCallState(multi_service_list=node_start_list, predicate="/start_now", prefix=""),
										transitions={'done': 'wait_for_nodes_to_be_ready', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:646 y:387
			OperatableStateMachine.add('wait_for_nodes_to_be_ready',
										WaitState(wait_time=5),
										transitions={'done': 'CalibSound'},
										autonomy={'done': Autonomy.Full})

			# x:650 y:459
			OperatableStateMachine.add('CalibSound',
										PlaySoundState(sound_file=calib_sound_file),
										transitions={'continue': 'calibrate_ik', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
