#!/usr/bin/python3
import rospy
import json
from std_msgs.msg import String

def fake_exec_pub():
    # Publish multiple roslaunch
    # Poll them
    # Cancel them
    fake_pub = rospy.Publisher("/aux_task_manager/request", String, queue_size=20)

    dummy_executable1 = {
        "RequestType": "start", 
        "TaskId": "task1",
        "LaunchType": "executable", 
        "Payload": { 
            "Executable": "/home/movel/seirios_ws/seirios_ros/src/aux_task_manager/tests/bash_test2.sh", 
            "Args": None,
            "Timeout": 10.0
            }, 
        }

    dummy_executable2 = {
        "RequestType": "start", 
        "TaskId": "task2",
        "LaunchType": "executable",
        "Payload": { 
            "Executable": "/home/movel/seirios_ws/seirios_ros/src/aux_task_manager/tests/bash_test.sh", 
            "Args": ["/home/movel/seirios_ws/seirios_ros/src/aux_task_manager/tests/get_odom.py"],
            "Timeout": 10.0
            },
        }

    dummy_executable_wrong = {
        "RequestType": "start", 
        "TaskId": "task3",
        "LaunchType": "executable",
        "Payload": { 
            "Executable": "iamanonexistentexe.sh", 
            "Args": None,
            "Timeout": 10.0
            }, 
        }

    dummy_cancel_all = {"RequestType": "cancel_all"}

    requests = [dummy_executable1, dummy_executable2, dummy_executable_wrong, dummy_cancel_all]

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        connections = fake_pub.get_num_connections()
        if connections > 0:
            for r in requests:
                fake_pub.publish(json.dumps(r))
                rospy.sleep(10)
            rospy.signal_shutdown("done")
        
        else:
            rate.sleep()

if __name__=='__main__':
    # need to use rosrun to run this file
    rospy.init_node('test_executable', anonymous=True)
    
    fake_exec_pub() # publish fake request only once