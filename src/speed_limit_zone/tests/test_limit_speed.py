#!/usr/bin/python3

import rospy
from movel_seirios_msgs.srv import ThrottleSpeed

if __name__=="__main__":
  # simple script to test `limit_robot_speed` service 
  rospy.init_node("test_speed_limiter")
  rospy.wait_for_service("limit_robot_speed")
  #global_name = rospy.get_param("/global_name")
  local_planner = rospy.get_param("/move_base/base_local_planner")
  local_planner = local_planner.split("/")[1]
  linear_topic = "/move_base/" + local_planner + "/max_vel_x"
  angular_topic = "/move_base/" + local_planner + "/max_vel_theta"
  default_linear_x = rospy.get_param(linear_topic)
  default_angular_z = rospy.get_param(angular_topic)
  print(f"Old speeds are linear: {default_linear_x} angular: {default_angular_z}")

  try:
    # while the robot is moving, run script
    # throttle speed, sleep 3 sec, revert back to normal speed
    speed_limit_client = rospy.ServiceProxy("limit_robot_speed", ThrottleSpeed)
    resp = speed_limit_client(True, 0.5) # change this value to how many % you want to slow down robot by
    new_linear_x = rospy.get_param(linear_topic)
    new_angular_x = rospy.get_param(angular_topic)
    print(f"New speeds are linear: {new_linear_x} angular: {new_angular_x}")
    print(f"{resp.success}, {resp.message}")

    rospy.sleep(3)
    resp2 = speed_limit_client(False, 1.0)
    new_linear_x2 = rospy.get_param(linear_topic)
    new_angular_x2 = rospy.get_param(angular_topic)
    print(f"Revert to old speeds linear: {new_linear_x2} angular: {new_angular_x2}")
    print(f"{resp.success}, {resp.message}")
  except rospy.ServiceException as e:
    rospy.logerr("Service failed to launch" + str(e))
