#ifndef MAP_EDITOR_H_
#define MAP_EDITOR_H_

// Dependencies
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <movel_seirios_msgs/DrawMultiPolygons.h>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc.hpp>

// Class Definition
class MapEditor
{
public:
    // Constructor
    MapEditor(){}

    // Load params from rosparam file
    void loadParams();

    // Main callback for editing map
    bool updateMultiPolygonsCb(movel_seirios_msgs::DrawMultiPolygons::Request &req,
                                movel_seirios_msgs::DrawMultiPolygons::Response &res);

    // Callback for restoring map
    bool restoreMapCb(movel_seirios_msgs::DrawMultiPolygons::Request &req,
                      movel_seirios_msgs::DrawMultiPolygons::Response &res);

    // Service Server and Client
    ros::ServiceServer make_multi_polygons_;
    ros::ServiceServer restore_map_;
    ros::ServiceClient relaunch_map_server_;

protected:
    std::string path_to_polygon_txt_;
    std::string nav_map_path_;
    std::string loc_map_path_;

    std::string map_restore_path_;
    std::string filename_;
    int line_width_;
};

#endif