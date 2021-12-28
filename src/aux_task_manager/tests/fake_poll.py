#!/usr/bin/python3

import rospy
import json
from std_msgs.msg import String

def pub_request():
    fake_pub = rospy.Publisher("/aux_task_manager/request", String, queue_size=20)

    #dummy_cancel= {"request_type": "cancel", "task_id": "task1"}
    dummy_poll = {"request_type": "poll", "task_id": "task1"}

    msg = json.dumps(dummy_poll)
    #rospy.loginfo(msg)

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
    rospy.init_node('fake_poll', anonymous=True)
    
    pub_request()
    #rospy.spin()