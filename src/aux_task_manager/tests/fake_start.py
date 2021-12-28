#!/usr/bin/python3

import rospy
import json
from std_msgs.msg import String

def pub_request():
    fake_pub = rospy.Publisher("/aux_task_manager/request", String, queue_size=20)

    # Fake requests
    dummy_roslaunch = {"request_type": "start", "task_id": "task1","payload": { "launch_type": "roslaunch", "package": None, "launch_file": "/home/movel/seirios_ws/seirios_ros/src/movel/launch/parallel_mapping.launch", "args": None}, "timeout": 10.0}
    #dummy_rosrun = {"request_type": "start", "task_id": "task2","payload": { "launch_type": "rosrun", "package": "my_robot", "executable": "mypose.py"}, "timeout": 10.0}
    #dummy_executable = {"request_type": "start", "task_id": "task3","payload": { "launch_type": "executable", "executable": "/home/movel/bash_arg_test.bash", "args": ["/home/movel/noisymp3.mp3"]}, "timeout": 10.0}

    ## Select fake request you want to test
    msg = json.dumps(dummy_roslaunch)
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        connections = fake_pub.get_num_connections()
        if connections > 0:
            fake_pub.publish(msg)
            rospy.signal_shutdown("done")
        else:
            rate.sleep()

if __name__=='__main__':
    # need to use rosrun to run this file
    rospy.init_node('fake_start', anonymous=True)
    
    pub_request() # publish fake request only once
