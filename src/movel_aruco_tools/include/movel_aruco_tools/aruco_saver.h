#ifndef ARUCO_SAVER_H_
#define ARUCO_SAVER_H_

#include <ros/ros.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <fstream>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <movel_seirios_msgs/StringTrigger.h>
#include <movel_hasp_vendor/license.h>

typedef struct
{
    double x;
    double y;
    double theta;
    uint8_t n;
    double xvar;
    double yvar;
}aruco;

class ARUCOSaver
{
public:
    ARUCOSaver();
    ~ARUCOSaver(){};

    void setupTopics();
    void setupParams();

    void arucoCallback(const fiducial_msgs::FiducialTransformArrayPtr &msg);
    bool savePoseToFile(movel_seirios_msgs::StringTrigger::Request& req,
                               movel_seirios_msgs::StringTrigger::Response& res);

protected:
    ros::Subscriber aruco_sub_;
    ros::Publisher aruco_pub_;
    ros::ServiceServer save_aruco_;
    geometry_msgs::TransformStamped camera_to_bl_;
    std::map<int, aruco> aruco_map_;
    double correction_range_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_ear_;
    std::string optical_frame_;
};

#endif