#pragma once
#include <assert.h>
#include <sys/time.h>
#include <unistd.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>


#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <list>
#include <string>
#include <boost/algorithm/string.hpp>
#include<geometry_msgs/Twist.h>
#include <fstream>
//#include <cameralidarDock/camera_lidar_docking.h>
#include<lidar_docking_/LidarDocking.hpp>
#include <geometry_msgs/Twist.h>

using namespace std;
using namespace cv;


class FiducialsNode {

protected:
    ros::Publisher * vertices_pub;
    ros::Publisher * pose_pub;

    ros::Subscriber caminfo_sub;
    ros::Subscriber ignore_sub;
    image_transport::ImageTransport it;
    image_transport::Subscriber img_sub;
    tf2_ros::TransformBroadcaster broadcaster;

    ros::ServiceServer service_enable_detections;

    // if set, we publish the images that contain fiducials
    bool publish_images;
    bool enable_detections;

    double fiducial_len;
    double kd;
    double kp;
    double max_speed_forward_;
    double max_speed_angular_;
    double goal_position_orientation_;
    bool doPoseEstimation;
    bool haveCamInfo;
    bool publishFiducialTf;

    cv::Mat cameraMatrix;
    cv::Mat distortionCoeffs;
    int frameNum;
    std::string frameId;
    std::vector<int> ignoreIds;
    std::map<int, double> fiducialLens;
    ros::NodeHandle nh;
    ros::NodeHandle pnh;


    image_transport::Publisher image_pub;
    ros::Publisher pub;

    cv::Ptr<aruco::DetectorParameters> detectorParams;
    cv::Ptr<aruco::Dictionary> dictionary;



    float current_angle;
    float previous_angle = 0;
    float previous_error = 0;
    geometry_msgs::Twist ptr;
    float y_error = 0;
    float prev_y_error = 0;
    double marker_for_docking_;
    double max_y_error_;
    float start_angle_rotation_recovery_;
    float end_angle_rotation_recovery_;
    double time_serach_marker_;
    int time_counter_search_marker=0;

public:
    lidar_docking::LidarDocking* obj;
    FiducialsNode();
    void handleIgnoreString(const std::string& str);
bool found_marker_dock = false;
    void estimatePoseSingleMarkers(const vector<int> &ids,
                                   const vector<vector<Point2f > >&corners,
                                   float markerLength,
                                   const cv::Mat &cameraMatrix,
                                   const cv::Mat &distCoeffs,
                                   vector<Vec3d>& rvecs, vector<Vec3d>& tvecs,
                                   vector<double>& reprojectionError);


//    void ignoreCallback(const std_msgs::String &msg);
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg);
//    void configCallback(aruco_detect::DetectorParamsConfig &config, uint32_t level);

    bool enableDetectionsCallback(std_srvs::SetBool::Request &req,
                                  std_srvs::SetBool::Response &res);

    void getSingleMarkerObjectPoints(float markerLength, vector<Point3f>& objPoints);
    double dist(const cv::Point2f &p1, const cv::Point2f &p2);
    double calcFiducialArea(const std::vector<cv::Point2f> &pts);
    double getReprojectionError(const vector<Point3f> &objectPoints,
                                const vector<Point2f> &imagePoints,
                                const Mat &cameraMatrix, const Mat  &distCoeffs,
                                const Vec3d &rvec, const Vec3d &tvec);
    float computeImageSide(Mat image, vector <vector <Point2f> > corners, vector<int>ids);
    void move(float linear_speed, float angular_speed);

//    dynamic_reconfigure::Server<aruco_detect::DetectorParamsConfig> configServer;
//    dynamic_reconfigure::Server<aruco_detect::DetectorParamsConfig>::CallbackType callbackType;

};


