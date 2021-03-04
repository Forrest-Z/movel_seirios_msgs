#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>
#include<pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include<pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
#include "nav_msgs/Odometry.h"
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include "camera_lidar_docking_pallet/DropObject.h"
#include<bits/stdc++.h>
#include <math.h>
#include<std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "camera_lidar_docking_pallet/StartAutoDock.h"
#include <std_srvs/Trigger.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


namespace lidar_docking{


class LidarDocking
{

protected:
    ros::NodeHandle nodeHandle_;
    ros::Subscriber point_cloud_subscriber_;
    ros::Subscriber point_cloud_subscriber_rear_;
    ros::Subscriber detection_side;
    ros::Publisher pub;
    ros::Publisher cloud_pub_;
    ros::Publisher search_for_markers_pub_;
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
    float pallet_intensity_; //= 200;        // pallet intensity used for clustering based on intensity values
    float intensity_threshold_ ;//= 25;      // threshold value to be added to and subtracted from markerintensity so as to create a window of values like 175 to 225
    float max_delta_error_slope_;
    float pallet_length_;


    float min_points_obstacle_;
    ros::Subscriber pose_subscriber;

    float kp_ , kd_;
    float time_counter_ = 0;
    std::string topic_name_;
    double cluster_depth_threshold_;
    float angle_;
    bool ready_for_docking_ = false;
    float pallet_point_x, pallet_point_y;
    bool robot_ready_for_picking_object_ = false;
    bool object_dropping_operation_ = false;
    bool reached_dropping_pallet_location_ = false;
    bool object_picked_ = false;
    bool robot_ready_to_drop_object_ = false;
    bool go_for_pickup_ = false;
    bool go_for_dropping_ = false;
    std_msgs::Bool search_for_markers_;
    float kp_rotation_;
    float kd_rotation_;
    float kd_orientation_, kp_orientation_;
    float kp_movement_, kd_movement_;
    float robot_inside_pallet_ = false;
    std::string detection_side_;
    float pallet_orientation_angle_;
    float max_linear_speed_;
    float max_angular_speed_;

    float dropping_x_;
    float dropping_y_;
    float dropping_theta_;
    float pickup_x_;
    float pickup_y_;
    float pickup_theta_;
    float robot_x_;
    float robot_y_;
    float robot_theta_;
    camera_lidar_docking_pallet::StartAutoDock last_goal_;
    bool paused_;
    bool active_;
    bool cancel_dock_;
    std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> nav_ac_ptr_;

public:
    /*!
     * Constructor.
     */
    bool finished_docking_ = false;
    ros::ServiceServer startAutoDock;
    ros::ServiceClient nav_client;
    ros::Subscriber pause_sub_;
    ros::Subscriber resume_sub_;
    ros::Subscriber cancel_sub_;
    ros::Publisher status_pub_;
    
    float distance = 0;
    bool robot_ready_for_docking = false;
    bool camera_detection = true;
    bool marker_visible_lidar_ = false;
    bool obstacle_found_ = false;
    bool rotation_completed = false;
    LidarDocking(/*ros::NodeHandle& nodeHandle*/);
    LidarDocking(ros::NodeHandle& nodeHandle);
    void read_point_cloud(const  sensor_msgs::LaserScan& msg);
    void read_point_cloud_concat(const  sensor_msgs::LaserScan& msg);
    void read_point_cloud_rear(const sensor_msgs::LaserScan& msg);

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
    float minimum_depth(std::vector<pcl::PointXYZI> cluster);
    void find_closest_cluster(std::vector<std::vector<pcl::PointXYZI> > all_clusters);
    int minimum(std::vector<float> depth);
    //    std::vector<pcl::PointXYZI> compute_closest_points(std::vector<pcl::PointXYZI>closest_cluster, float minimum_depth);
    std::vector<pcl::PointXYZI>compute_closest_points(std::vector<std::vector<pcl::PointXYZI> > all_clusters, float minimum_depth);

    pcl::PointCloud<pcl::PointXYZI>cluster_to_point_cloud(std::vector<pcl::PointXYZI> cluster);
    float compute_centroid(pcl::PointCloud<pcl::PointXYZI> cloud, std::string axis);
    float lineFit(pcl::PointCloud<pcl::PointXYZI> cloud);
    void correct_pose(float centroid_x, float centroid_y);
    float maximum(pcl::PointCloud<pcl::PointXYZI> cloud, std::string axis);
    float minimum(pcl::PointCloud<pcl::PointXYZI> cloud, std::string axis);
    void goto_docking_position();
    bool dropObject(camera_lidar_docking_pallet::DropObject::Request  &req, camera_lidar_docking_pallet::DropObject::Response &res);
    bool goForDropping();
    bool goForPickUp();
    void visualize_clusters(std::vector<std::vector<pcl::PointXYZI> > all_clusters);
    std::vector<std::pair<float, int> > compute_allcluster_centroids(std::vector<std::vector<pcl::PointXYZI> > all_clusters);
    void find_closest_cluster_centroid(std::vector<std::pair<float, int> >centroids ,std::vector<std::vector<pcl::PointXYZI> > all_clusters);
    std::vector<std::vector<pcl::PointXYZI> > distance_based_clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void intensityClusters_(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void detection_side_callback(const  std_msgs::String::ConstPtr& msg);
    std::vector<std::pair<int, int> > compute_pairs(std::vector<std::pair<float, int> > centroid_disctances);
    std::vector<pcl::PointXYZI> concatenate_clusters(std::vector<pcl::PointXYZI> cluster_a, std::vector<pcl::PointXYZI> cluster_b);
    void generate_velocity_commands (pcl::PointCloud<pcl::PointXYZI> nearest_cluster );
    bool autoDock(camera_lidar_docking_pallet::StartAutoDock::Request  &req,
                  camera_lidar_docking_pallet::StartAutoDock::Response &res);
    bool executeDocking();
    void navigationLoop(move_base_msgs::MoveBaseGoal goal);

    void pauseCb(std_msgs::Bool msg);
    void resumeCb(std_msgs::Bool msg);
    void cancelCb(std_msgs::Bool msg); 

};

} /* namespace */
