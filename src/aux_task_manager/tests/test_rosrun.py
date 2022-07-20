#!/usr/bin/python3
import rospy
import json
from std_msgs.msg import String

def fake_rosrun_pub():
    # Publish multiple roslaunch
    # Poll them
    # Cancel them
    fake_pub = rospy.Publisher("/aux_task_manager/request", String, queue_size=20)

    dummy_rosrun = {"request_type": "start", "task_id": "task1","payload": { "launch_type": "rosrun", "package": "aux_task_manager", "executable": "get_odom.py"}, "timeout": 10.0}
    dummy_cancel= {"request_type": "cancel", "task_id": "task1"}
    dummy_rosrun_wrong = {"request_type": "start", "task_id": "task1","payload": { "launch_type": "rosrun", "package": "aux_task_manager", "executable": "idontexist.py"}, "timeout": 10.0}
    dummy_rosrun_wrongagain= {"request_type": "start", "task_id": "task2","payload": { "launch_type": "rosrun", "package": "aux_task_manager", "executable": None}, "timeout": 10.0}
    dummy_poll_all = {"request_type": "poll_all"}
    dummy_cancel_all = {"request_type": "cancel_all"}

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