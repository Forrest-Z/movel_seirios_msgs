#!/usr/bin/env python3
import rospy    
import pymongo
import numpy as np
from movel_seirios_msgs.srv import ZonePolygon, ZonePolygonRequest, ZonePolygonResponse, StringTrigger, StringTriggerRequest, StringTriggerResponse
from geometry_msgs.msg import Polygon, Point32
from movel_seirios_msgs.msg import Zones
from std_srvs.srv import Trigger, TriggerRequest
from pymongo import MongoClient
from bson.objectid import ObjectId


NODE_NAME = "prohibition_layer_mongo"


class MongoBridgeProhib:
    
    def __init__(self):
        self.success_list = []
        self.service_list = [
            '/move_base/global_costmap/costmap_prohibition_layer/',
            '/move_base/local_costmap/costmap_prohibition_layer/',
            '/planner_utils_node/aux_clean_map/costmap_prohibition_layer/',
        ]

        self.prohib_layer_service = rospy.Service('~get_prohib_layer', StringTrigger, self.NoGoZoneHandle)
        rospy.loginfo("[%s] Initialized", NODE_NAME)


    def NoGoZoneHandle(self, req):
        rospy.loginfo("[%s] Getting No Go Zones in Map: %s", NODE_NAME, req.input)
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
                rospy.logerr("[%s] movelweb does not exist", NODE_NAME)
                service_response.success = False
                return service_response
        except pymongo.errors.ServerSelectionTimeoutError as err:
            rospy.logerr("[%s] %s", NODE_NAME, err)
            service_response.success = False
            return service_response

        # Query for RosPolygons collection
        polygons = db.RosPolygons

        # Clear previous map RosPolygons 
        if polygons.count_documents({"mapId": ObjectId(map_id)}) == 0:
            rospy.logwarn("[%s] No prohibition layer enabled for selected map", NODE_NAME)
            clear_req = TriggerRequest()
            try:
                for service in self.service_list:
                    clear_polygon = rospy.ServiceProxy(service+'clear',Trigger) 
                    resp_clear = clear_polygon(clear_req)
                    self.success_list.append(resp_clear.success)
                service_response.success = np.prod(self.success_list)
                return service_response
            except rospy.ServiceException as e:
                rospy.logerr("[%s] Service call failed: %s", NODE_NAME, e)
                service_response.success = False
                return service_response
     
        
        # Call /no_go_zone service
        nogo_req = ZonePolygonRequest()
        for polygon in polygons.find({"mapId": ObjectId(map_id)}):
            polygon_points = Polygon()
            zone = Zones()
            for pose in polygon["poses"]:
                point = Point32()
                point.x = pose["x"]
                point.y = pose["y"]
                point.z = 0
                polygon_points.points.append(point)
            zone.polygons = polygon_points
            zone.labels = 0
            zone.percentage_reduction = 0
            nogo_req.zone_data.append(zone)

        try:
            for service in self.service_list:
                prohib_layer = rospy.ServiceProxy(service+'nogo_zone', ZonePolygon)
                resp_add = prohib_layer(nogo_req)
                self.success_list.append(resp_add.success)
            service_response.success = np.prod(self.success_list)
            rospy.loginfo("[%s] Prohibition layer added", NODE_NAME)
            return service_response    
        except rospy.ServiceException as e:
            rospy.logerr("[%s] Service call failed: %s", NODE_NAME, e)
            service_response.success = False
            return service_response


if __name__=="__main__":
    rospy.init_node(NODE_NAME)
    m = MongoBridgeProhib()
    rospy.spin()