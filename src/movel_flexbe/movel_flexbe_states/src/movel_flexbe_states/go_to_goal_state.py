#!/usr/bin/env python3

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, Point, Quaternion, Pose2D, PoseStamped
from tf import transformations

class GoToGoalState(EventState):
    """
    Navigates a robot to a desired position and orientation using move_base.

    -- goal     List            Position of goal in map, represented as [x, y, yaw].

    <= arrived                  Navigation to target pose succeeded.
    <= failed                   Navigation to target pose failed.
    """

    def __init__(self, goal):
        """Constructor"""
        super(GoToGoalState, self).__init__(outcomes = ['arrived', 'failed'])
        self._action_topic = '/move_base'
        self._client = ProxyActionClient({self._action_topic: MoveBaseAction})
        self._arrived = False
        self._failed = False

        # goal goal
        # if goal data is string, convert to list
        print("Goal: ", goal)
        print("Type: ",type(goal))
        if type(goal) is str:
            goal = eval(goal, {'__builtins__':None}, {})

        pos = Point(x = goal[0], y = goal[1])
        quat = transformations.quaternion_from_euler(0.0, 0.0, goal[2])
        pose = Pose(position=pos, orientation=Quaternion(*quat))

        self._move_base_goal = MoveBaseGoal()
        self._move_base_goal.target_pose.pose = pose
        self._move_base_goal.target_pose.header.frame_id = 'map'


    def execute(self, userdata):
        """Wait for action result and return outcome accordingly"""
        if self._arrived:
            return 'arrived'
        if self._failed:
            return 'failed'
        if self._client.has_result(self._action_topic):
            status = self._client.get_state(self._action_topic)
            if status == GoalStatus.SUCCEEDED:
                self._arrived = True
                return 'arrived'
            elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED,
                            GoalStatus.RECALLED, GoalStatus.ABORTED]:
                Logger.logwarn('Navigation failed: %s' % str(status))
                self._failed = True
                return 'failed'


    def on_enter(self, userdata):
        """Create and send action goal"""
        self._arrived = False
        self._failed = False
        
        # Send the action goal for execution
        try:
            self._client.send_goal(self._action_topic, self._move_base_goal)
        except Exception as e:
            Logger.logwarn('Unable to send navigation action goal:\n%s' % str(e))
            self._failed = True
            

    def cancel_active_goals(self):
        if self._client.is_available(self._action_topic):
            if self._client.is_active(self._action_topic):
                if not self._client.has_result(self._action_topic):
                    self._client.cancel(self._action_topic)
                    Logger.loginfo('Cancelled move_base active action goal.')


    def on_exit(self, userdata):
        self.cancel_active_goals()


    def on_stop(self):
        self.cancel_active_goals()
    

    def on_pause(self):
        Logger.loginfo('State paused, stopping robot.')
        self.cancel_active_goals()


    def on_resume(self, userdata):
        Logger.loginfo('State resumed, resuming robot navigation.')
        try:
            self._client.send_goal(self._action_topic, self._move_base_goal)
        except Exception as e:
            Logger.logwarn('Unable to send navigation action goal:\n%s' % str(e))
            self._failed = True