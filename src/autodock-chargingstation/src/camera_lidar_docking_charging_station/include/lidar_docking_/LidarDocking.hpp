#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>
#include<pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include<pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
//#include <cameralidarDock/camera_lidar_docking.h>
#include "nav_msgs/Odometry.h"
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "camera_lidar_docking_charging_station/FinishedDock.h"

namespace lidar_docking{


class LidarDocking
{

protected:
    ros::NodeHandle nodeHandle_;
    ros::Subscriber point_cloud_subscriber_;
    ros::Publisher pub;
    ros::Publisher cloud_pub_;
    float max_linear_speed = 0.7;
    float max_angular_speed = 0.7;
    float prev_x = 0;
    float prev_y = 0;
    float prev_theta = 0;
    float final_position_x = 0;
    float final_position_y = 0;
    float final_theta = 0;
    geometry_msgs::Twist ptr;
    float prev_err_slope = 0;
    float res;
    float dist_to_dock_station;
    float marker_slope_lidar_threshold;
    float marker_intensity_; //= 200;        // marker intensity used for clustering based on intensity values
    float intensity_threshold_ ;//= 25;      // threshold value to be added to and subtracted from markerintensity so as to create a window of values like 175 to 225
    float cluster_length_;
    float cluster_length_threshold_;
    float speed_translation_max_;
    float speed_angular_max_;
    float max_delta_error_slope_;

    float min_points_obstacle_;
    float charger_x_;
    float charger_y_;
    float charger_theta_;
    ros::Subscriber pose_subscriber;
    float robot_theta;
    float robot_x_;
    float robot_y_;
    float kp_ , kd_;
    float time_for_obstacle_clearance_;
    float time_counter_ = 0;
    ros::ServiceClient client;

public:
    /*!
     * Constructor.
     */
    float distance = 0;
    float docking_complete_ = false;
    bool robot_ready_for_docking = false;
    bool camera_detection = true;
    float dist_camera_to_lidar_switch_ = 0.2;
    bool marker_visible_lidar_ = false;
    bool obstacle_found_ = false;
    bool rotation_completed = false;
    float max_error_y_axis_;
    LidarDocking(/*ros::NodeHandle& nodeHandle*/);
    LidarDocking(ros::NodeHandle& nodeHandle);
    void read_point_cloud(const  sensor_msgs::LaserScan& msg);

    void readParams(ros::NodeHandle &nodeHandle);
    /*!
     * Destructor.
     */
    virtual ~LidarDocking();

    int computeMean(std::vector<int> arr);
    void clusters(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void intensityClusters(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    float find_length(std::vector<pcl::PointXYZI> cluster);
    void move(float linear_speed, float angular_speed);

    void pose_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void start_recovery();
    void rotate_and_check();
    float lineFit(pcl::PointCloud<pcl::PointXYZI> cloud);
    pcl::PointCloud<pcl::PointXYZI>cluster_to_point_cloud(std::vector<pcl::PointXYZI> cluster);

};

} /* namespace */
