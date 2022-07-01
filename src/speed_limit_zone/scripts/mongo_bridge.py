#!/usr/bin/env python3
from tkinter.messagebox import NO
import rospy
from movel_seirios_msgs.srv import SpeedZones, SpeedZonesRequest, SpeedZonesResponse, StringTrigger, StringTriggerRequest, StringTriggerResponse
from movel_seirios_msgs.msg import SpeedZone
from geometry_msgs.msg import Polygon, Point32

import pymongo
from pymongo import MongoClient
from bson.objectid import ObjectId

NODE_NAME = "mongo_bridge"

class MongoBridge:
    def __init__(self):
        self.speed_service = rospy.Service('~get_speed_zones', StringTrigger, self.speedZoneHandle)
        rospy.loginfo("[%s] Initialized", NODE_NAME)

    def speedZoneHandle(self, req):
        rospy.loginfo("[%s] Get speed zones", NODE_NAME)
        map_id = req.input
        service_response = StringTriggerResponse()

        # Connect to movelweb database
        try:
            client = MongoClient(serverSelectionTimeoutMS=1000)
            client.server_info()
            dbnames = client.list_database_names()
            if 'movelweb' in dbnames:
                db = client['movelweb']
                rospy.loginfo("[%s] Connnected to movelweb", NODE_NAME)
            else:
                rospy.logerror("[%s] movelweb does not exist", NODE_NAME)
                service_response.success = False
                return service_response
        except pymongo.errors.ServerSelectionTimeoutError as err:
            rospy.logerr("[%s] %s", NODE_NAME, err)
            service_response.success = False
            return service_response

        # Query for Polygons collection
        polygons = db.Polygons
        #polygons = db.Waypoints
        if polygons.count_documents({"mapId": ObjectId(map_id)}) == 0:
            rospy.logwarn("[%s] No speed zone in selected map", NODE_NAME)
            #service_response.success = True
            #return service_response
        
        # Call /reduce_speed_zone service
        try:
            speed_zone = rospy.ServiceProxy('/reduce_speed_zone', SpeedZones)
            speed_req = SpeedZonesRequest()

            for polygon in polygons.find({"mapId": ObjectId(map_id)}):
                if polygon["disabled"]:
                    continue
                zone = SpeedZone()
                try:
                    zone.linear = polygon["velocity"]["linear_velocity"]
                    zone.angular = polygon["velocity"]["angular_velocity"]
                except TypeError as e:
                    continue
                polygon_points = Polygon()
                for pose in polygon["poses"]:
                    point = Point32()
                    point.x = pose["x"]
                    point.y = pose["y"]
                    #point.x = pose["position"]["x"]
                    #point.y = pose["position"]["y"]
                    point.z = 0
                    polygon_points.points.append(point)
                zone.polygons = polygon_points
                speed_req.zone_data.append(zone)

            if len(speed_req.zone_data) == 0:
                rospy.logwarn("[%s] No enabled speed zone for selected map", NODE_NAME)
                #service_response.success = True
                #return service_response

            res = speed_zone(speed_req)
            service_response.success = res.success
            return service_response
        except rospy.ServiceException as e:
            rospy.logerr("[%s] Service call failed: %s", NODE_NAME, e)
            service_response.success = False
            return service_response

def main():
    rospy.init_node(NODE_NAME)
    m = MongoBridge()
    rospy.spin()

if __name__=="__main__":
    main()
