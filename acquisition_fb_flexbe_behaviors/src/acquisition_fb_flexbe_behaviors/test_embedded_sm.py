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
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Sep 19 2024
@author: me
'''
class test_embeddedSM(Behavior):
	'''
	.
	'''


	def __init__(self):
		super(test_embeddedSM, self).__init__()
		self.name = 'test_embedded'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(imu_startup_sequenceSM, 'imu_startup_sequence')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:477, x:130 y:477
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:354 y:117
			OperatableStateMachine.add('imu_startup_sequence',
										self.use_behavior(imu_startup_sequenceSM, 'imu_startup_sequence'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'imu_list': 'imu_list'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
