#ifndef ARUCO_AMCL_H_
#define ARUCO_AMCL_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>
#include <fstream>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <movel_seirios_msgs/StringTrigger.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <movel_hasp_vendor/license.h>

typedef struct
{
    double x;
    double y;
    double theta;
    int n;
}aruco;

class ARUCOAMCL
{
public:
    ARUCOAMCL();
    ~ARUCOAMCL(){};

    void setupTopics();
    void setupParams();

    bool loadArucoPoseFile(movel_seirios_msgs::StringTrigger::Request& req,
                          movel_seirios_msgs::StringTrigger::Response& res);
    void arucoCallback(const fiducial_msgs::FiducialTransformArrayPtr &msg);

protected:
    ros::Subscriber aruco_sub_;
    ros::Publisher initpose_pub_;
    ros::ServiceServer load_aruco_serv_;
    std::map<int, aruco> aruco_map_;
    std::map<int, aruco> aruco_flush_;

    ros::Time last_init_pose_;
    bool initialized_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_ear_;

    // Params
    double max_error_x_;
    double max_error_y_;
    double max_error_theta_;
    double max_dev_x_;
    double max_dev_y_;
    double max_dev_theta_;
    int num_holding_;
    double cooldown_time_;
    double correction_range_;
    geometry_msgs::TransformStamped camera_to_bl_;
    std::string optical_frame_;
};

#endif


