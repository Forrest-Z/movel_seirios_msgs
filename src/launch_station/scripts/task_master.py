import rospy, rosnode, roslaunch
from roslaunch import roslaunch_logs

class RosTask:
    def __init__(self):
        self.connected = False
        print("TM init")

    def pub_connections(self):
        print("badum")

    def connect(self, reconnect_delay):
        print ("tm attempt")
        while not self.connected:
            self.connected = True
            print("tm connected")
            return self.connected
