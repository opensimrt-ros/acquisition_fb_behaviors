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
from acquisition_fb_flexbe_states.tmux_setup_from_yaml_state import TmuxSetupFromYamlState
from acquisition_fb_flexbe_states.wait_for_tfs import WaitForTfsState
from flexbe_states.check_condition_state import CheckConditionState
from flexbe_states.log_state import LogState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Apr 22 2024
@author: frekle
'''
class compare_insolesSM(Behavior):
	'''
	this will start the insoles and the talus imus so we can match their times
	'''


	def __init__(self):
		super(compare_insolesSM, self).__init__()
		self.name = 'compare_insoles'

		# parameters of this behavior
		self.add_parameter('run_nodes', True)

		# references to used behaviors

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

		# O 519 273 /Calibration_and_Heading
		# This published the tfs for showing the IMUs on rViz|n|nNot really necessary, since we are not using this inside the node just yet

		# O 500 109 /Calibration_and_Heading
		# We are using just the pelvis for heading. This is maybe not ideal, since a combined heading of more imus maybe is better



	def create(self):
		save_dir = "/srv/host_data/tmp"
		tmux_yaml_path = "/catkin_ws/src/ros_biomech/acquisition_state_machines/acquisition_of_raw_data/config/"
		tmux_yaml_file = "compare_insoles.yaml"
		imu_list = ["talus_r","talus_l"]
		node_start_list = ["/moticon_insoles"]
		# x:662 y:639, x:162 y:458
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
										transitions={'done': 'check_all_imu_tfs'},
										autonomy={'done': Autonomy.Off})

			# x:514 y:213
			OperatableStateMachine.add('check_all_imu_tfs',
										WaitForTfsState(tf_prefix="imu", tf_list=imu_list, reference_frame="map"),
										transitions={'continue': 'done', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


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
			# x:84 y:33
			OperatableStateMachine.add('run_nodes',
										CheckConditionState(predicate=lambda x: bool(x)),
										transitions={'true': 'load_nodes', 'false': 'Check_If_Devices_Are_On'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'input_value': 'load_nodes'})

			# x:660 y:176
			OperatableStateMachine.add('Turn_On_IMUs',
										_sm_turn_on_imus_0,
										transitions={'done': 'don_imus', 'failed': 'failed'},
										autonomy={'done': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:682 y:290
			OperatableStateMachine.add('don_imus',
										LogState(text="place IMUs", severity=Logger.REPORT_HINT),
										transitions={'done': 'start_parked_nodes'},
										autonomy={'done': Autonomy.Full})

			# x:319 y:106
			OperatableStateMachine.add('load_nodes',
										TmuxSetupFromYamlState(session_name="testtt", startup_yaml=tmux_yaml_path+tmux_yaml_file),
										transitions={'continue': 'Check_If_Devices_Are_On', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:647 y:372
			OperatableStateMachine.add('start_parked_nodes',
										MultiServiceCallState(multi_service_list=node_start_list, predicate="/start_now", prefix=""),
										transitions={'done': 'wait_for_nodes_to_be_ready', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:646 y:455
			OperatableStateMachine.add('wait_for_nodes_to_be_ready',
										WaitState(wait_time=5),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Full})

			# x:636 y:56
			OperatableStateMachine.add('Check_If_Devices_Are_On',
										_sm_check_if_devices_are_on_1,
										transitions={'done': 'Turn_On_IMUs', 'fail': 'failed'},
										autonomy={'done': Autonomy.Inherit, 'fail': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
