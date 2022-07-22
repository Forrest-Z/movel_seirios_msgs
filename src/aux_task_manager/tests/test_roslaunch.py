#!/usr/bin/python3
import rospy
import json
from std_msgs.msg import String

def fake_roslaunch_pub():
    # Publish multiple roslaunch
    # Poll them
    # Cancel them
    fake_pub = rospy.Publisher("/aux_task_manager/request", String, queue_size=20)

    dummy_roslaunch1 = {
        "RequestType": "start", 
        "TaskId": "task1",
        "LaunchType": "roslaunch",
        "Payload": { 
            "Pkg": None, 
            "Launchfile": "/home/movel/seirios_ws/seirios_ros/src/movel/launch/parallel_mapping.launch", 
            "Args": None, 
            "Timeout": 10.0
            }, 
        }
    
    dummy_roslaunch2 = {
        "RequestType": "start", 
        "TaskId": "task2",
        "LaunchType": "roslaunch", 
        "Payload": { 
            "Pkg": "broadcast_pose", 
            "Launchfile": "broadcast_pose.launch", 
            "Args": None,
            "Timeout": 10.0
            }, 
        }

    dummy_roslaunch_wrong = {
        "RequestType": "start", 
        "TaskId": "task3",
        "LaunchType": "roslaunch", 
        "Payload": { 
            "Pkg": "movel", 
            "Launchfile": None, 
            "Args": None,
            "Timeout": 10.0
            }, 
        }

    dummy_poll_all = {"RequestType": "poll_all"}
    dummy_cancel_all = {"RequestType": "cancel_all"}

    dummy_roslaunch_wrong2 = {
        "RequestType": "start", 
        "TaskId": "task3",
        "LaunchType": "roslaunch", 
        "Payload": { 
            "Pkg": "movel", 
            "Launchfile": "iamawrongfile.launch", 
            "Args": None,
            "Timeout": 10.0
            }, 
        }

    requests = [dummy_roslaunch1, dummy_roslaunch2, dummy_roslaunch_wrong, dummy_poll_all, dummy_cancel_all, dummy_roslaunch_wrong2]

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
    rospy.init_node('test_roslaunch', anonymous=True)
    
    fake_roslaunch_pub() # publish fake request only once