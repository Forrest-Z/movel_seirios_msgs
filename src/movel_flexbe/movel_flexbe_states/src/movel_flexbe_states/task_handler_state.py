#!/usr/bin/env python3

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from actionlib_msgs.msg import GoalStatus
from movel_seirios_msgs.msg import RunTaskListGoal, RunTaskListAction

class MovelTaskHandlerState(EventState):
    """
    State for interfacing with task supervisor server.

    -- type         int         Corresponding task type from the list of TS handlers.
    -- payload      string      Task payload.

    <= completed                Task supervisor succeeded.
    <= failed                   Task supervisor failed to complete the task.
    """

    def __init__(self, goal):
        """Constructor"""
        super(MovelTaskHandlerState, self).__init__(outcomes = ['completed', 'failed'])
        self._action_topic = '/task_supervisor'
        self._client = ProxyActionClient({self._action_topic: RunTaskListAction})
        self._completed = False
        self._failed = False

        # construct task supervisor goal
        self._task_supervisor_goal = RunTaskListGoal()


    def execute(self, userdata):
        """Wait for action result and return outcome accordingly"""
        if self._completed:
            return 'completed'
        if self._failed:
            return 'failed'
        if self._client.has_result(self._action_topic):
            status = self._client.get_state(self._action_topic)
            if status == GoalStatus.SUCCEEDED:
                self._completed = True
                return 'arrived'
            elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED,
                            GoalStatus.RECALLED, GoalStatus.ABORTED]:
                Logger.logwarn('Task failed: %s' % str(status))
                self._failed = True
                return 'failed'


    def on_enter(self, userdata):
        """Create and send action goal"""
        self._completed = False
        self._failed = False
        
        # Send the action goal for execution
        try:
            self._client.send_goal(self._action_topic, self._task_supervisor_goal)
        except Exception as e:
            Logger.logwarn('Unable to send task supervisor action goal:\n%s' % str(e))
            self._failed = True
            

    def cancel_active_goals(self):
        if self._client.is_available(self._action_topic):
            if self._client.is_active(self._action_topic):
                if not self._client.has_result(self._action_topic):
                    self._client.cancel(self._action_topic)
                    Logger.loginfo('Cancelled task supervisor active action goal.')


    def on_exit(self, userdata):
        self.cancel_active_goals()


    def on_stop(self):
        self.cancel_active_goals()
    

    def on_pause(self):
        pass


    def on_resume(self, userdata):
        pass