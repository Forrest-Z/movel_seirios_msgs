#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from movel_flexbe_states.go_to_goal_state import GoToGoalState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import PoseStamped
# [/MANUAL_IMPORT]


'''
Created on Tue Oct 12 2021
@author: Movel AI
'''
class move_base_demoSM(Behavior):
	'''
	move base demo
	'''


	def __init__(self):
		super(move_base_demoSM, self).__init__()
		self.name = 'move_base_demo'

		# parameters of this behavior
		self.add_parameter('goal_1', '[-6.25, -3.0, 1.57]')
		self.add_parameter('goal_2', '[1.0, 2.0, -1.57]')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# home = [-3.0, 1.0, 0.0]
		home = [0.0, 0.0, 0.0]
		# x:855 y:54, x:345 y:295
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.goal = PoseStamped()

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		_state_machine.userdata.goal.header.frame_id = 'odom'
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:65 y:48
			OperatableStateMachine.add('go_to_base',
										GoToGoalState(goal=home),
										transitions={'arrived': 'go_to_goal_1', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off})

			# x:497 y:285
			OperatableStateMachine.add('go_to_base_final',
										GoToGoalState(goal=home),
										transitions={'arrived': 'finished', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off})

			# x:279 y:49
			OperatableStateMachine.add('go_to_goal_1',
										GoToGoalState(goal=self.goal_1),
										transitions={'arrived': 'go_to_goal_2', 'failed': 'go_to_base_final'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off})

			# x:495 y:48
			OperatableStateMachine.add('go_to_goal_2',
										GoToGoalState(goal=self.goal_2),
										transitions={'arrived': 'finished', 'failed': 'go_to_base_final'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	def increase_by_one(self, value):
		return value + 1
	# [/MANUAL_FUNC]
