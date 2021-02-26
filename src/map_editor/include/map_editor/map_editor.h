#ifndef MAP_EDITOR_H_
#define MAP_EDITOR_H_

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <movel_seirios_msgs/DrawPolygon.h>
#include <movel_seirios_msgs/Pixel.h>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc.hpp>

class MapEditor
{
public:
    MapEditor(){}

    void loadParams();

    bool updateCb(movel_seirios_msgs::DrawPolygon::Request &req,
                  movel_seirios_msgs::DrawPolygon::Response &res);

    bool restoreMapCb(movel_seirios_msgs::DrawPolygon::Request &req,
                      movel_seirios_msgs::DrawPolygon::Response &res);

    ros::ServiceServer make_polygon_;
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