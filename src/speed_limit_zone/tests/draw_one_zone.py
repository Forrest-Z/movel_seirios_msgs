#!/usr/bin/python3

import rospy
from movel_seirios_msgs.srv import ZonePolygon
from movel_seirios_msgs.msg import Zones
from geometry_msgs.msg import Point32, Polygon
from std_msgs.msg import Header

if __name__=="__main__":
  rospy.init_node("draw_one_zone")
  rospy.wait_for_service("reduce_speed_zone")

  try:
    draw_one_polygon = rospy.ServiceProxy("reduce_speed_zone", ZonePolygon)
    header = Header()
    header.stamp = rospy.Time.now()
    speed_zone = Zones()
    
    one_polygon = Polygon()
    # find 4 points on the map that can make a polygon
    p1, p2, p3, p4 = Point32(), Point32(), Point32(), Point32()
    p1.x, p1.y, p1.z = -2.0, 10.5, 0.0
    p2.x, p2.y, p2.z = -2.5, 3, 0.0
    p3.x, p3.y, p3.z = 7.5, 10.5, 0.0
    p4.x, p4.y, p4.z = 7.5, 3.0, 0.0
    one_polygon.points = [p1, p2, p3, p4]

    speed_zone.polygons = one_polygon
    speed_zone.percentage_reduction = 0.5
    
    response = draw_one_polygon(header, [speed_zone])
    print(f"Zone: {speed_zone.polygons}, % reduction: {speed_zone.percentage_reduction}")
    print(f"{response.success}, {response.message}")

  except rospy.ServiceException as e:
    rospy.logerr("Service failed to launch" + str(e))