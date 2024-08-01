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
from flexbe_states.calculation_state import CalculationState
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
class acquire_rawSM(Behavior):
	'''
	acquire from raw imus
	'''


	def __init__(self):
		super(acquire_rawSM, self).__init__()
		self.name = 'acquire_raw'

		# parameters of this behavior
		self.add_parameter('activity_name', 'walking')
		self.add_parameter('activity_duration', 10)
		self.add_parameter('subject_num', 0)
		self.add_parameter('num_reps', 1)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		save_dir = "/srv/host_data/tmp"
		# x:470 y:542, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.activity_counter = 0
		_state_machine.userdata.activity_save_dir = ""
		_state_machine.userdata.activity_save_name = ""

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:31 y:40
			OperatableStateMachine.add('is_router_on',
										HostAliveState(hostname="192.168.1.1", waittime=200),
										transitions={'continue': 'turn_on_imus', 'failed': 'turn_on_router'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:433 y:387
			OperatableStateMachine.add('addone_to_num_reps',
										CalculationState(calculation=lambda x: x+1),
										transitions={'done': 'multiple_setpath'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'activity_counter', 'output_value': 'activity_counter'})

			# x:1070 y:115
			OperatableStateMachine.add('check_heading',
										MultiServiceCallState(multi_service_list="/ik/pelvis/wtfh/calibrate_heading", predicate="", prefix=""),
										transitions={'done': 'start_ik', 'failed': 'failed'},
										autonomy={'done': Autonomy.Full, 'failed': Autonomy.Off})

			# x:641 y:51
			OperatableStateMachine.add('check_if_imus_on',
										LogState(text="Dummy state: check if IMUs are blinking then you can start moving them around. BTW, this should be a wait for transform or somethign.", severity=Logger.REPORT_HINT),
										transitions={'done': 'don_imus'},
										autonomy={'done': Autonomy.Full})

			# x:843 y:50
			OperatableStateMachine.add('don_imus',
										LogState(text="place IMUs", severity=Logger.REPORT_HINT),
										transitions={'done': 'external_pose_calib'},
										autonomy={'done': Autonomy.Full})

			# x:981 y:23
			OperatableStateMachine.add('external_pose_calib',
										MultiServiceCallState(multi_service_list=["/ik/pelvis/pose_average/calibrate_pose"], predicate="", prefix=""),
										transitions={'done': 'check_heading', 'failed': 'external_pose_calib'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:784 y:268
			OperatableStateMachine.add('multiple_setpath',
										MultiSetNameAndPathState(multi_service_list="/ik_lowerbody_node/set_name_and_path", prefix="", suffix="", activity_name=self.activity_name, save_dir=save_dir, subject_num=self.subject_num),
										transitions={'done': 'start_recording', 'failed': 'failed'},
										autonomy={'done': Autonomy.Full, 'failed': Autonomy.Off},
										remapping={'activity_counter': 'activity_counter', 'activity_save_dir': 'activity_save_dir', 'activity_save_name': 'activity_save_name'})

			# x:613 y:544
			OperatableStateMachine.add('record_another',
										OperatorDecisionState(outcomes=["yes", "no"], hint=None, suggestion=None),
										transitions={'yes': 'addone_to_num_reps', 'no': 'finished'},
										autonomy={'yes': Autonomy.Off, 'no': Autonomy.Off})

			# x:1043 y:215
			OperatableStateMachine.add('start_ik',
										MultiServiceCallState(multi_service_list="/ik_lowerbody_node/start_now", predicate="", prefix=""),
										transitions={'done': 'multiple_setpath', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:315 y:130
			OperatableStateMachine.add('start_imus',
										MultiServiceCallState(multi_service_list=["/ximu_torso/start_now",                 "/ximu_pelvis/start_now",                 "/ximu_femur_l/start_now",                 "/ximu_femur_r/start_now",                 "/ximu_tibia_l/start_now",                 "/ximu_tibia_r/start_now",                 "/ximu_talus_l/start_now",                 "/ximu_talus_r/start_now",], predicate="", prefix=""),
										transitions={'done': 'wait_for_things_to_be_done', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:826 y:390
			OperatableStateMachine.add('start_recording',
										LogState(text="Start Recording", severity=Logger.REPORT_HINT),
										transitions={'done': 'Record_time'},
										autonomy={'done': Autonomy.Off})

			# x:263 y:22
			OperatableStateMachine.add('turn_on_imus',
										LogState(text="Turn on imus in a line", severity=Logger.REPORT_HINT),
										transitions={'done': 'start_imus'},
										autonomy={'done': Autonomy.Full})

			# x:30 y:210
			OperatableStateMachine.add('turn_on_router',
										LogState(text="turn on router!!", severity=Logger.REPORT_HINT),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:444 y:54
			OperatableStateMachine.add('wait_for_things_to_be_done',
										WaitState(wait_time=5),
										transitions={'done': 'check_if_imus_on'},
										autonomy={'done': Autonomy.Off})

			# x:828 y:518
			OperatableStateMachine.add('Record_time',
										WaitState(wait_time=self.activity_duration),
										transitions={'done': 'record_another'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
