#include <assert.h>
#include <sys/time.h>
#include <unistd.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include "nav_msgs/Odometry.h"
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <list>
#include <string>
#include <boost/algorithm/string.hpp>
#include<geometry_msgs/Twist.h>
#include <fstream>
#include<std_msgs/Bool.h>
#include<map>
#include<tf/transform_broadcaster.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;
using namespace cv;

class FiducialsNode {
private:
    ros::Publisher * vertices_pub;
    ros::Publisher * pose_pub;
    ros::Publisher  detection_side;
    ros::Subscriber caminfo_sub;
    ros::Subscriber caminfo_sub_rear;
    ros::Subscriber ignore_sub;
    ros::Subscriber search_marker_sub;
    image_transport::ImageTransport it;
    image_transport::Subscriber img_sub;
    image_transport::Subscriber img_sub_rear;
    ros::ServiceServer service_enable_detections;

    // if set, we publish the images that contain fiducials
    bool publish_images;
    bool enable_detections;

    double fiducial_len;

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
    vector<double>markers_tvec_x ;
    vector<double>markers_tvec_y ;
    vector<double>markers_rvec_theta ;
    bool ready_for_docking_ = false;
    bool search_for_markers_ = false;

public:
    FiducialsNode();
    void handleIgnoreString(const std::string& str);

    void estimatePoseSingleMarkers(const vector<int> &ids,
                                   const vector<vector<Point2f > >&corners,
                                   float markerLength,
                                   const cv::Mat &cameraMatrix,
                                   const cv::Mat &distCoeffs,
                                   vector<Vec3d>& rvecs, vector<Vec3d>& tvecs,
                                   vector<double>& reprojectionError);


    void ignoreCallback(const std_msgs::String &msg);
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    void imageCallback_rear(const sensor_msgs::ImageConstPtr &msg);

    void serachMarkerCallback(const std_msgs::Bool::ConstPtr &msg);
    void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg);

    bool enableDetectionsCallback(std_srvs::SetBool::Request &req,
                                  std_srvs::SetBool::Response &res);

    void getSingleMarkerObjectPoints(float markerLength, vector<Point3f>& objPoints);
    double dist(const cv::Point2f &p1, const cv::Point2f &p2);
    double calcFiducialArea(const std::vector<cv::Point2f> &pts);
    double getReprojectionError(const vector<Point3f> &objectPoints,
                                const vector<Point2f> &imagePoints,
                                const Mat &cameraMatrix, const Mat  &distCoeffs,
                                const Vec3d &rvec, const Vec3d &tvec);
    void move(float translation_speed, float angular_speed);
    void goto_position();
    void rotate(float speed_translation, float speed_angular);

};


