#!/usr/bin/env python3

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from actionlib_msgs.msg import GoalStatus
from movel_seirios_msgs.msg import RunTaskListGoal, RunTaskListAction, Task
import pymongo
import json

class SeiriosRunNavigationState(EventState):
    """
    State for single point navigation on seirios.

    -- goal_name    string      Goal name.
    -- linear_vel   float       Linear velocity.
    -- angular_vel  float       Angular velocity.

    <= arrived                  Navigation task succeeds, robot's arrived to the destination.
    <= failed                   Navigation task fails.
    """

    def __init__(self, goal_name, linear_vel, angular_vel):
        """Constructor"""
        super(SeiriosRunNavigationState, self).__init__(outcomes = ['arrived', 'failed'])

        self._goal_name = goal_name
        self._linear_vel = linear_vel
        self._angular_vel = angular_vel

        self._action_topic = '/task_supervisor'
        self._client = ProxyActionClient({self._action_topic: RunTaskListAction})
        
        self._completed = False
        self._failed = False

        self._task_supervisor_goal = RunTaskListGoal()


    def execute(self, userdata):
        """Wait for action result and return outcome accordingly"""
        if self._completed:
            return 'arrived'
        if self._failed:
            return 'failed'
        if self._client.has_result(self._action_topic):
            status = self._client.get_state(self._action_topic)
            if status == GoalStatus.SUCCEEDED:
                self._completed = True
                Logger.loginfo('[%s] Navigation completed: %s' % (self.name, str(status)))
                return 'arrived'
            elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED,
                            GoalStatus.RECALLED, GoalStatus.ABORTED]:
                Logger.logwarn('[%s] Navigation failed: %s' % (self.name, str(status)))
                self._failed = True
                return 'failed'


    def on_enter(self, userdata):
        """Create and send action goal"""
        self._completed = False
        self._failed = False

        # retrieve goal coordinate from db
        try:
            Logger.loghint('[%s] Initialising MongoDB Client' % self.name)
            mongo_client = pymongo.MongoClient('mongodb://localhost:27017/')
        except pymongo.errors.ConnectionFailure as e:
            Logger.logerr('[%s] Cannot initialise MongoDB client:\n%s' % (self.name, str(e)))
            self._failed = True
            return

        try:
            Logger.loghint('[%s] Attempting to connect to MongoDB and retrieve goal data' % self.name)

            db = mongo_client.get_database('movelweb')

            goal = db.get_collection('Goals').find_one({'name': self._goal_name})
            if goal is None:
                Logger.logerr('[%s] Goal with name %s does not exist in collection' % (self.name, self._goal_name))
                self._failed = True
                return

            bot = db.get_collection('BotIn').find_one()
            if 'currentMapId' not in bot or goal['mapId'] != bot['currentMapId']:
                Logger.logerr('[%s] Map of requested goal does not match current map' % self.name)
                self._failed = True
                return
        except (pymongo.errors.ConnectionFailure, pymongo.errors.ServerSelectionTimeoutError) as e:
            Logger.logerr('[%s] Cannot establish connection to MongoDB server:\n%s' % (self.name, str(e)))
            self._failed = True
            return

        # construct task supervisor goal
        Logger.loghint('[%s] Constructing message and transmitting goal to task supervisor' % self.name)

        ts_task = Task()
        ts_task.name = 'Goto'
        ts_task.type = 3
        ts_task.mapId = str(goal['mapId'])
        
        payload_dict = goal['goal']
        payload_dict['from_map'] = ts_task.mapId
        payload_dict['to_map'] = ts_task.mapId

        ts_task.payload = json.dumps(payload_dict)

        ts_task.linear_velocity = self._linear_vel
        ts_task.angular_velocity = self._angular_vel

        self._task_supervisor_goal.task_list.tasks.append(ts_task)

        # Send the action goal for execution
        try:
            self._client.send_goal(self._action_topic, self._task_supervisor_goal)
        except Exception as e:
            Logger.logerr('[%s] Unable to send task supervisor action goal:\n%s' % (self.name, str(e)))
            self._failed = True
            

    def cancel_active_goals(self):
        if self._client.is_available(self._action_topic):
            if self._client.is_active(self._action_topic):
                if not self._client.has_result(self._action_topic):
                    self._client.cancel(self._action_topic)
                    Logger.loginfo('[%s] Cancelled task supervisor active action goal.' % self.name)


    def on_exit(self, userdata):
        self.cancel_active_goals()


    def on_stop(self):
        self.cancel_active_goals()
    

    def on_pause(self):
        Logger.loginfo('[%s] State paused, stopping robot.' % self.name)
        self.cancel_active_goals()


    def on_resume(self, userdata):
        Logger.loginfo('[%s] State resumed, resuming robot navigation.' % self.name)
        try:
            self._client.send_goal(self._action_topic, self._move_base_goal)
        except Exception as e:
            Logger.logerr('[%s] Unable to send task supervisor action goal:\n%s' % (self.name, str(e)))
            self._failed = True