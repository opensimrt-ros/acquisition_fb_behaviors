#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from acquisition_fb_flexbe_states.multi_service_call_state import MultiServiceCallState
from flexbe_states.log_state import LogState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

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
		self.add_parameter('activity_name', '"walking"')
		self.add_parameter('activity_duration', 10)
		self.add_parameter('subject_num', 0)
		self.add_parameter('num_reps', 1)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:470 y:542, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.activity_counter = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:64 y:56
			OperatableStateMachine.add('turn_on_imus',
										LogState(text="Turn on imus in a line", severity=Logger.REPORT_HINT),
										transitions={'done': 'start_imus'},
										autonomy={'done': Autonomy.Full})

			# x:633 y:244
			OperatableStateMachine.add('start_ik',
										MultiServiceCallState(multi_service_list="/ik_lowerbody_node/start_now"),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:236 y:117
			OperatableStateMachine.add('start_imus',
										MultiServiceCallState(multi_service_list=["/ximu_torso/start_now",                 "/ximu_pelvis/start_now",                 "/ximu_femur_l/start_now",                 "/ximu_femur_r/start_now",                 "/ximu_tibia_l/start_now",                 "/ximu_tibia_r/start_now",                 "/ximu_talus_l/start_now",                 "/ximu_talus_r/start_now",]),
										transitions={'done': 'don_imus', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:599 y:104
			OperatableStateMachine.add('don_imus',
										LogState(text="place IMUs", severity=Logger.REPORT_HINT),
										transitions={'done': 'start_ik'},
										autonomy={'done': Autonomy.Full})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
