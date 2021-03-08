#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <bits/stdc++.h>
#include <math.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include "navigation_based_docking/StartAutoDock.h"
#include <geometry_msgs/Pose2D.h>
#include <std_srvs/Trigger.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Docking
{

protected:
    ros::NodeHandle nodeHandle_;
    std::vector<float> data;

    float eps_;
    std::string path_of_waypoints_;

    float param_value_;
    std::string param_name_;
    std::string service_call_topic_;
    double inflation_temp_;
    bool rotation_temp_;
    bool paused_;
    uint8_t current_point_;
    bool navigating_;
    bool cancel_dock_;
    bool active_;
    std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> nav_ac_ptr_;

public:

    bool finished_docking_ = false;
    ros::ServiceServer startAutoDock;
    ros::Subscriber pause_sub_;
    ros::Subscriber resume_sub_;
    ros::Subscriber cancel_sub_;

    ros::Publisher status_pub_;
    /**
     * @brief Docking - constructor
     * @param nodeHandle
     */
    Docking(ros::NodeHandle& nodeHandle);

    /**
     * @brief readParams - readng params from config file
     * @param nodeHandle
     */
    void readParams(ros::NodeHandle &nodeHandle);
    /**
     * @brief goto_docking_position - function to move robot to specific position using move base
     * @param pt
     * @param wait
     */
    void goto_docking_position(std::vector<float> pt);
    /**
     * @brief autoDock - service to carry out pallet docking task
     * @param req
     * @param res
     * @return
     */
    bool autoDock(navigation_based_docking::StartAutoDock::Request  &req,
                  navigation_based_docking::StartAutoDock::Response &res);

    std::vector<float> getPointData(int i);

    void pauseCb(std_msgs::Bool msg);
    void resumeCb(std_msgs::Bool msg);
    void cancelCb(std_msgs::Bool msg);

    void iteratePoints();
    void reconfigureParams(std::string param, std::string op);
    
};
