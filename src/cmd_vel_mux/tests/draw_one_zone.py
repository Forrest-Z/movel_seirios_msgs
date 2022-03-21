#!/usr/bin/python3

import rospy
from movel_seirios_msgs.srv import ZonePolygon
from movel_seirios_msgs.msg import Zones
from geometry_msgs.msg import Point32, Polygon

if __name__=="__main__":
  rospy.init_node("draw_one_zone")
  rospy.wait_for_service("reduce_speed_zone")

  try:
    draw_one_polygon = rospy.ServiceProxy("reduce_speed_zone", ZonePolygon)
    speed_zone = Zones()
    
    #TODO: construct a polygon
    #TODO: construct a zone - polygon + label 
    one_polygon = Polygon()
    p1, p2, p3, p4 = Point32(), Point32(), Point32(), Point32()
    p1.x, p1.y, p1.z = 0.0, 4.0, 0.0
    p2.x, p2.y, p2.z = 0.0, 2.0, 0.0
    p3.x, p3.y, p3.z = 2.0, 2.0, 0.0
    p4.x, p4.y, p4.z = 2.0, 4.0, 0.0
    one_polygon = [p1, p2, p3, p4]

    speed_zone.polygons = one_polygon
    speed_zone.percentage_reduction = 0.5
    response = draw_one_polygon(speed_zone)
    print(f"Zone: {speed_zone.polygons}, % reduction: {speed_zone.percentage_reduction}")
    print(f"{response.success}, {response.message}")

  except rospy.ServiceException as e:
    rospy.logerr("Service failed to launch" + str(e))