#!/usr/bin/python3

import rospy
import json
from std_msgs.msg import String

def pub_request():
    fake_pub = rospy.Publisher("/aux_task_manager/request", String, queue_size=20)

    ## Fake requests
    dummy_cancel= {"request_type": "cancel", "task_id": "task1"}
    dummy_cancel_all = {"request_type": "cancel_all"}

    msg = json.dumps(dummy_cancel_all)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        connections = fake_pub.get_num_connections()
        if connections > 0:
            fake_pub.publish(msg)
            rospy.signal_shutdown("done")
        else:
            rate.sleep()

if __name__=='__main__':
    rospy.init_node('fake_stop', anonymous=True)
    
    pub_request()
    #rospy.spin()