#!/usr/bin/python3
import rospy
import json
from std_msgs.msg import String

def fake_roslaunch_pub():
    # Publish multiple roslaunch
    # Poll them
    # Cancel them
    fake_pub = rospy.Publisher("/aux_task_manager/request", String, queue_size=20)

    dummy_roslaunch1 = {"request_type": "start", "task_id": "task1","payload": { "launch_type": "roslaunch", "package": None, "launch_file": "/home/movel/seirios_ws/seirios_ros/src/movel/launch/parallel_mapping.launch", "args": None}, "timeout": 10.0}

    requests = [dummy_roslaunch1]

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        connections = fake_pub.get_num_connections()
        if connections > 0:
            for r in requests:
                fake_pub.publish(json.dumps(r))
                rospy.sleep(10)
            # fake_pub.publish(json.dumps(dummy_roslaunch1))
            # rospy.sleep(3)
            # fake_pub.publish(json.dumps(dummy_roslaunch2))
            # rospy.sleep(3)
            # fake_pub.publish(json.dumps(dummy_poll_all))
            # rospy.sleep(3)
            # fake_pub.publish(json.dumps(dummy_cancel_all))
            rospy.signal_shutdown("done")
        
        else:
            rate.sleep()

if __name__=='__main__':
    # need to use rosrun to run this file
    rospy.init_node('test_roslaunch', anonymous=True)
    
    fake_roslaunch_pub() # publish fake request only once