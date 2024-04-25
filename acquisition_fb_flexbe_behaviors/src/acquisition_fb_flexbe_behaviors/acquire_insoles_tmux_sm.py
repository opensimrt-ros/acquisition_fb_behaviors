#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from acquisition_fb_flexbe_states.check_if_alive import HostAliveState
from acquisition_fb_flexbe_states.multi_service_call_state import MultiServiceCallState
from acquisition_fb_flexbe_states.set_name_and_path_state import MultiSetNameAndPathState
from acquisition_fb_flexbe_states.tmux_setup_from_yaml_state import TmuxSetupFromYamlState
from flexbe_states.calculation_state import CalculationState
from flexbe_states.check_condition_state import CheckConditionState
from flexbe_states.log_state import LogState
from flexbe_states.operator_decision_state import OperatorDecisionState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
#from acquisition_fb_flexbe_states.multi_service_call_state import MultiServiceCallState 

# [/MANUAL_IMPORT]


'''
Created on Mon Apr 22 2024
@author: frekle
'''
class acquire_insoles_tmuxSM(Behavior):
	'''
	acquire with tmux and insoles
	'''


	def __init__(self):
		super(acquire_insoles_tmuxSM, self).__init__()
		self.name = 'acquire_insoles_tmux'

		# parameters of this behavior
		self.add_parameter('activity_name', 'walking')
		self.add_parameter('activity_duration', 10)
		self.add_parameter('subject_num', 0)
		self.add_parameter('num_reps', 1)
		self.add_parameter('run_nodes', True)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:

		# O 265 11 /Calibration_and_Heading
		# Right now we are not using these results to calibrate the IK node just yet. |n|nThe only guys that use this are the resolve headings service to show the imus and the external heading calibrator which uses the pelvis avg quaternion

		# O 519 273 /Calibration_and_Heading
		# This published the tfs for showing the IMUs on rViz|n|nNot really necessary, since we are not using this inside the node just yet

		# O 500 109 /Calibration_and_Heading
		# We are using just the pelvis for heading. This is maybe not ideal, since a combined heading of more imus maybe is better



	def create(self):
		save_dir = "/srv/host_data/tmp"
		tmux_yaml_path = "/catkin_ws/src/ros_biomech/acquisition_state_machines/acquisition_of_raw_data/config/"
		tmux_yaml_file = "acquisition_imus.yaml"
		tmux_yaml_file2 = "acquisition_dummy_imus.yaml"
		imu_list = ["torso","pelvis","femur_r","tibia_r","talus_r","femur_l","tibia_l","talus_l"]
		node_start_list = ["/ik_lowerbody_node","/moticon_insoles"]
		tmux_yaml_file3 = "acquisition_imus_insoles.yaml"
		# x:1022 y:731, x:648 y:259
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.activity_counter = 0
		_state_machine.userdata.activity_save_dir = ""
		_state_machine.userdata.activity_save_name = ""
		_state_machine.userdata.load_nodes = self.run_nodes

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:477, x:130 y:477
		_sm_turn_on_imus_0 = OperatableStateMachine(outcomes=['done', 'failed'])

		with _sm_turn_on_imus_0:
			# x:30 y:47
			OperatableStateMachine.add('turn_on_imus',
										LogState(text="Turn on imus in a line", severity=Logger.REPORT_HINT),
										transitions={'done': 'start_imus'},
										autonomy={'done': Autonomy.Full})

			# x:147 y:159
			OperatableStateMachine.add('start_imus',
										MultiServiceCallState(multi_service_list=imu_list, predicate="/start_now", prefix="/ximu_"),
										transitions={'done': 'wait_for_things_to_be_done', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:207 y:40
			OperatableStateMachine.add('wait_for_things_to_be_done',
										WaitState(wait_time=5),
										transitions={'done': 'check_if_imus_on'},
										autonomy={'done': Autonomy.Off})

			# x:396 y:102
			OperatableStateMachine.add('check_if_imus_on',
										LogState(text="Dummy state: check if IMUs are blinking then you can start moving them around. BTW, this should be a wait for transform or somethign.", severity=Logger.REPORT_HINT),
										transitions={'done': 'done'},
										autonomy={'done': Autonomy.Full})


		# x:30 y:477, x:130 y:477
		_sm_recording_trial_1 = OperatableStateMachine(outcomes=['failed', 'done'])

		with _sm_recording_trial_1:
			# x:523 y:40
			OperatableStateMachine.add('start_recording_srv',
										MultiServiceCallState(multi_service_list=node_start_list, predicate="/start_recording", prefix=""),
										transitions={'done': 'start_recording', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:570 y:166
			OperatableStateMachine.add('start_recording',
										LogState(text="Start Recording", severity=Logger.REPORT_HINT),
										transitions={'done': 'Record_time'},
										autonomy={'done': Autonomy.Off})

			# x:261 y:338
			OperatableStateMachine.add('stop_recording_srv',
										MultiServiceCallState(multi_service_list=node_start_list, predicate="/stop_recording", prefix=""),
										transitions={'done': 'write_sto_srv', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:30 y:343
			OperatableStateMachine.add('write_sto_srv',
										MultiServiceCallState(multi_service_list=node_start_list, predicate="/write_sto", prefix=""),
										transitions={'done': 'done', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:490 y:305
			OperatableStateMachine.add('Record_time',
										WaitState(wait_time=self.activity_duration),
										transitions={'done': 'stop_recording_srv'},
										autonomy={'done': Autonomy.Off})


		# x:30 y:477, x:130 y:477
		_sm_external_calibration_and_heading_2 = OperatableStateMachine(outcomes=['failed', 'done'])

		with _sm_external_calibration_and_heading_2:
			# x:102 y:45
			OperatableStateMachine.add('external_pose_calib',
										MultiServiceCallState(multi_service_list=imu_list, predicate="/pose_average/calibrate_pose", prefix="/ik/"),
										transitions={'done': 'calibrate_heading', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:309 y:134
			OperatableStateMachine.add('calibrate_heading',
										MultiServiceCallState(multi_service_list="pelvis", predicate="/wtfh/calibrate_heading", prefix="/ik/"),
										transitions={'done': 'We_re_getting_the_wrong_sing', 'failed': 'failed'},
										autonomy={'done': Autonomy.Full, 'failed': Autonomy.Off})

			# x:465 y:402
			OperatableStateMachine.add('resolve_headings_imus',
										MultiServiceCallState(multi_service_list=imu_list, predicate="/pose_average/resolve_heading", prefix="/ik/"),
										transitions={'done': 'done', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:522 y:255
			OperatableStateMachine.add('We_re_getting_the_wrong_sing',
										LogState(text="There is a bug and the sign of the angle is not been given, or it might just be wrong, so check it", severity=Logger.REPORT_HINT),
										transitions={'done': 'resolve_headings_imus'},
										autonomy={'done': Autonomy.Full})



		with _state_machine:
			# x:58 y:85
			OperatableStateMachine.add('run_nodes',
										CheckConditionState(predicate=lambda x: bool(x)),
										transitions={'true': 'load_nodes', 'false': 'is_router_on'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'input_value': 'load_nodes'})

			# x:1304 y:351
			OperatableStateMachine.add('External_Calibration_and_Heading',
										_sm_external_calibration_and_heading_2,
										transitions={'failed': 'failed', 'done': 'multiple_setpath'},
										autonomy={'failed': Autonomy.Inherit, 'done': Autonomy.Inherit})

			# x:1237 y:605
			OperatableStateMachine.add('Recording_trial',
										_sm_recording_trial_1,
										transitions={'failed': 'failed', 'done': 'record_another'},
										autonomy={'failed': Autonomy.Inherit, 'done': Autonomy.Inherit})

			# x:846 y:55
			OperatableStateMachine.add('Turn_On_IMUs',
										_sm_turn_on_imus_0,
										transitions={'done': 'turn_on_insole_acquisition', 'failed': 'failed'},
										autonomy={'done': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:900 y:391
			OperatableStateMachine.add('addone_to_num_reps',
										CalculationState(calculation=lambda x: x+1),
										transitions={'done': 'calibrate_ik'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'activity_counter', 'output_value': 'activity_counter'})

			# x:1007 y:205
			OperatableStateMachine.add('calibrate_ik',
										MultiServiceCallState(multi_service_list="/ik_lowerbody_node", predicate="/calibrate", prefix=""),
										transitions={'done': 'External_Calibration_and_Heading', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1397 y:52
			OperatableStateMachine.add('don_imus',
										LogState(text="place IMUs", severity=Logger.REPORT_HINT),
										transitions={'done': 'start_ik'},
										autonomy={'done': Autonomy.Full})

			# x:427 y:23
			OperatableStateMachine.add('is_insole_android_device_on',
										HostAliveState(hostname="192.168.1.101", waittime=1000),
										transitions={'continue': 'wear_insole', 'failed': 'turn_on_insole_android_device'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:239 y:35
			OperatableStateMachine.add('is_router_on',
										HostAliveState(hostname="192.168.1.1", waittime=200),
										transitions={'continue': 'is_insole_android_device_on', 'failed': 'turn_on_router'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:42 y:306
			OperatableStateMachine.add('load_nodes',
										TmuxSetupFromYamlState(session_name="testtt", startup_yaml=tmux_yaml_path+tmux_yaml_file3),
										transitions={'continue': 'is_router_on', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1377 y:447
			OperatableStateMachine.add('multiple_setpath',
										MultiSetNameAndPathState(multi_service_list="/ik_lowerbody_node", prefix="", suffix="set_name_and_path", activity_name=self.activity_name, save_dir=save_dir, subject_num=self.subject_num),
										transitions={'done': 'Calibration_OK', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'activity_counter': 'activity_counter', 'activity_save_dir': 'activity_save_dir', 'activity_save_name': 'activity_save_name'})

			# x:1000 y:538
			OperatableStateMachine.add('record_another',
										OperatorDecisionState(outcomes=["yes", "no"], hint=None, suggestion=None),
										transitions={'yes': 'addone_to_num_reps', 'no': 'finished'},
										autonomy={'yes': Autonomy.Off, 'no': Autonomy.Off})

			# x:1376 y:180
			OperatableStateMachine.add('start_ik',
										MultiServiceCallState(multi_service_list=node_start_list, predicate="/start_now", prefix=""),
										transitions={'done': 'wait_for_ik_to_be_ready', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1077 y:30
			OperatableStateMachine.add('turn_on_insole_acquisition',
										LogState(text="Start acquiring insoles", severity=Logger.REPORT_HINT),
										transitions={'done': 'don_imus'},
										autonomy={'done': Autonomy.Full})

			# x:421 y:140
			OperatableStateMachine.add('turn_on_insole_android_device',
										LogState(text="turn on insole android device and make sure it is connected to the router network!", severity=Logger.REPORT_ERROR),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:273 y:214
			OperatableStateMachine.add('turn_on_router',
										LogState(text="turn on router!!", severity=Logger.REPORT_ERROR),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:1367 y:267
			OperatableStateMachine.add('wait_for_ik_to_be_ready',
										WaitState(wait_time=5),
										transitions={'done': 'External_Calibration_and_Heading'},
										autonomy={'done': Autonomy.Full})

			# x:658 y:43
			OperatableStateMachine.add('wear_insole',
										LogState(text="Please put on insoles", severity=Logger.REPORT_HINT),
										transitions={'done': 'Turn_On_IMUs'},
										autonomy={'done': Autonomy.Off})

			# x:1376 y:538
			OperatableStateMachine.add('Calibration_OK',
										LogState(text="Is the calibration and the heading OK?", severity=Logger.REPORT_HINT),
										transitions={'done': 'Recording_trial'},
										autonomy={'done': Autonomy.Full})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
