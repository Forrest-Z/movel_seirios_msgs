#!/usr/bin/env python3

import rospy
import json

from simple_recorder import SimpleRecorder
from std_msgs.msg import String, Empty


class RecorderNode:
    
    def __init__(self):
        self.recorder = None
        self.init_recorder_sub = rospy.Subscriber("/init_recorder", String, self.init_recorder_CB)
        self.start_sub = rospy.Subscriber("/start", Empty, self.start_CB)
        self.stop_sub = rospy.Subscriber("/stop", Empty, self.stop_CB)

    def init_recorder_CB(self, msg):
        # json string follows recorder constructor kwargs
        record_params = json.loads(msg.data)
        self.recorder = SimpleRecorder(**record_params)
        rospy.logwarn(f"record_params: {record_params}")
    
    def start_CB(self, msg):
        self.recorder.start()

    def stop_CB(self, msg):
        self.recorder.stop()


if __name__ == "__main__":
    rospy.init_node('simple_recorder_node')
    node = RecorderNode()
    rospy.spin()