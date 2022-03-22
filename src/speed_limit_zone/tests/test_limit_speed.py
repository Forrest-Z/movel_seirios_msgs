#!/usr/bin/python3

import rospy
from movel_seirios_msgs.srv import ThrottleSpeed

if __name__=="__main__":
  # simple script to test `limit_robot_speed` service 
  # check if robot's speed got reduced by echoing cmd_vel
  rospy.init_node("test_speed_limiter")
  rospy.wait_for_service("limit_robot_speed")

  try:
    speed_limit_client = rospy.ServiceProxy("limit_robot_speed", ThrottleSpeed)
    resp = speed_limit_client(True, 0.5)
    print(f"{resp.success}, {resp.message}")
  except rospy.ServiceException as e:
    rospy.logerr("Service failed to launch" + str(e))
