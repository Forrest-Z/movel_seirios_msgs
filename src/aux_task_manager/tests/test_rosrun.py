#!/usr/bin/python3
import rospy
import json
from std_msgs.msg import String

def fake_rosrun_pub():
    # Publish multiple roslaunch
    # Poll them
    # Cancel them
    fake_pub = rospy.Publisher("/aux_task_manager/request", String, queue_size=20)

    dummy_rosrun = {
        "RequestType": "start", 
        "TaskId": "task1",
        "LaunchType": "rosrun",
        "Payload": { 
            "Pkg": "aux_task_manager", 
            "Executable": "get_odom.py",
            "Timeout": 10.0
            }, 
        }
    
    dummy_cancel= {
        "RequestType": "cancel", 
        "TaskId": "task1"
        }

    dummy_rosrun_wrong = {
        "RequestType": "start", 
        "TaskId": "task1",
        "LaunchType": "rosrun",
        "Payload": { 
            "Pkg": "aux_task_manager", 
            "Executable": "idontexist.py",
            "Timeout": 10.0
        }, 
    }

    dummy_rosrun_wrongagain= {
        "RequestType": "start", 
        "TaskId": "task2",
        "LaunchType": "rosrun",
        "Payload": {
            "Pkg": "aux_task_manager", 
            "Executable": None,
            "Timeout": 10.0
            }, 
        }
    dummy_poll_all = {"RequestType": "poll_all"}
    dummy_cancel_all = {"RequestType": "cancel_all"}

    requests = [dummy_rosrun, dummy_cancel, dummy_rosrun_wrong, dummy_rosrun_wrongagain, dummy_poll_all, dummy_cancel_all]

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
    rospy.init_node('test_rosrun', anonymous=True)
    fake_rosrun_pub() # publish fake request only once