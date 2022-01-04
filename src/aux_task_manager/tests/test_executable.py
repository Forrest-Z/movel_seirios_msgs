#!/usr/bin/python3
import rospy
import json
from std_msgs.msg import String

def fake_exec_pub():
    # Publish multiple roslaunch
    # Poll them
    # Cancel them
    fake_pub = rospy.Publisher("/aux_task_manager/request", String, queue_size=20)

    dummy_executable1 = {"request_type": "start", "task_id": "task1","payload": { "launch_type": "executable", "executable": "/home/movel/seirios_ws/seirios_ros/src/aux_task_manager/tests/bash_test2.sh", "args": None}, "timeout": 10.0}
    dummy_executable2 = {"request_type": "start", "task_id": "task2","payload": { "launch_type": "executable", "executable": "/home/movel/seirios_ws/seirios_ros/src/aux_task_manager/tests/bash_test.sh", "args": ["/home/movel/seirios_ws/seirios_ros/src/aux_task_manager/tests/get_odom.py"]}, "timeout": 10.0}
    dummy_executable_wrong = {"request_type": "start", "task_id": "task3","payload": { "launch_type": "executable", "executable": "iamanonexistentexe.sh", "args": None}, "timeout": 10.0}

    dummy_cancel_all = {"request_type": "cancel_all"}

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