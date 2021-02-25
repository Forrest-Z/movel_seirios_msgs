#ifndef MAP_EDITOR_H_
#define MAP_EDITOR_H_

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <map_editor/Polygon.h>
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

    bool updateCb(map_editor::Polygon::Request &req,
                  map_editor::Polygon::Response &res);

    bool restoreMapCb(map_editor::Polygon::Request &req,
                      map_editor::Polygon::Response &res);

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