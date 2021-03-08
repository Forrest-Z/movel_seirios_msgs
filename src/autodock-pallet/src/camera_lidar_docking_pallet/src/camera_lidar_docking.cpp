#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "lidar_docking_/LidarDocking.hpp"
#include "cameralidarDock/camera_lidar_docking.h"
#include "navigation_based_docking/navigation_based_docking.hpp"

using namespace std;

CameraLidar::CameraLidar(ros::NodeHandle& nodeHandle)
{
}

/**
 * @brief lidar_docking::LidarDocking::autoDock - service to trigger the robot for docking
 * @param req
 * @param res
 * @return
 */
bool lidar_docking::LidarDocking::autoDock(camera_lidar_docking_pallet::StartAutoDock::Request  &req,
                                           camera_lidar_docking_pallet::StartAutoDock::Response &res)
{
    active_ = true;
    last_goal_.request = req; 
    res.success = true;
    return true;
}

/**
 * @brief lidar_docking::LidarDocking::executeDocking - function for taking the robot to the docking position
 * @return
 */
bool lidar_docking::LidarDocking::executeDocking()
{
    if(active_)
    {
        move_base_msgs::MoveBaseGoal goal; 

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        if(last_goal_.request.operation == "pickup")       // Variable assignment for start_recovery method
        {
            pickup_x_ = last_goal_.request.x;
            pickup_y_ = last_goal_.request.y;
            pickup_theta_ = last_goal_.request.theta;
        }
        if(last_goal_.request.operation == "drop")
        {
            dropping_x_ = last_goal_.request.x;
            dropping_y_ = last_goal_.request.y;
            dropping_theta_ = last_goal_.request.theta;
        }

        double qz = sin(last_goal_.request.theta * 0.5);
        double qw = cos(last_goal_.request.theta * 0.5);

        goal.target_pose.pose.position.x = last_goal_.request.x;
        goal.target_pose.pose.position.y = last_goal_.request.y;
        goal.target_pose.pose.orientation.z = qz;
        goal.target_pose.pose.orientation.w = qw;
        
        ROS_INFO("Sending goal");

        navigationLoop(goal);
        if (!cancel_dock_)
        {
            std::cout<<"reached position for "<<last_goal_.request.operation<<std::endl;
            // ros::Duration(0.1).sleep();
            goto_docking_position();
            active_ = false;
            if(ready_for_docking_)
            {
                std::cout<<"going under the pallet for "<<last_goal_.request.operation<<std::endl;
                
                navigation_based_docking::StartAutoDock srv;
                srv.request.pallet_number = last_goal_.request.id_pallet;          // id of pallet
                srv.request.operation = last_goal_.request.operation;       // operation
                ros::service::waitForService("/navigation_based_docking/NavigationDocking");
                if(nav_client.call(srv)){
                    ROS_INFO("Navigation-based-docking is finished");
                }
                else
                {
                    ROS_INFO("Failed to call service NavigationDocking");
                }
                ready_for_docking_ = 0;
            }
            std_msgs::UInt8 done;
            done.data = 2;
            status_pub_.publish(done);
        }
        
    }
    return true;
}

/**
 * @brief lidar_docking::LidarDocking::navigationLoop- main loop for going to the dock postition, and aligning using camera.
 * @param goal
 * @return
 */
void lidar_docking::LidarDocking::navigationLoop(move_base_msgs::MoveBaseGoal goal)
{
    while(!nav_ac_ptr_->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    nav_ac_ptr_->sendGoal(goal);

    bool navigating = true;
    while(nav_ac_ptr_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        if(paused_ && navigating)
        {
            ROS_INFO("Navigation paused");
            navigating = false;
            nav_ac_ptr_->cancelGoal();
        }
        if (!paused_ && !navigating)
        {
            ROS_INFO("Navigation resumed");
            navigating = true;
            nav_ac_ptr_->sendGoal(goal);
        }
        if (cancel_dock_)
        {
            ROS_INFO("Docking cancelled");
            nav_ac_ptr_->cancelGoal();

            std_msgs::UInt8 done;
            done.data = 3;
            status_pub_.publish(done);
            break;
        }
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
}
/**
 * @brief lidar_docking::LidarDocking::pauseCb - Callback for pause 
 * @param msg
 * @return
 */
void lidar_docking::LidarDocking::pauseCb(std_msgs::Bool msg)
{
    paused_ = true;
}

/**
 * @brief lidar_docking::LidarDocking::resumeCb - Callback for resuming
 * @param msg
 * @return
 */
void lidar_docking::LidarDocking::resumeCb(std_msgs::Bool msg)
{
    paused_ = false;
}

/**
 * @brief lidar_docking::LidarDocking::cancelCb - Callback for canceling task
 * @param msg
 * @return
 */
void lidar_docking::LidarDocking::cancelCb(std_msgs::Bool msg)
{
    if (active_)
        cancel_dock_ = true;
}
/**
 * @brief lidar_docking::LidarDocking::LidarDocking constructor with arguments
 * @param nodeHandle
 */
lidar_docking::LidarDocking::LidarDocking(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle),  active_(false), paused_(false), cancel_dock_(false)
{   
    point_cloud_subscriber_ = nodeHandle_.subscribe("/scan_front", 10, &LidarDocking::read_point_cloud, this);
    point_cloud_subscriber_rear_ = nodeHandle_.subscribe("/scan_rear", 10, &LidarDocking::read_point_cloud_rear, this);
    point_cloud_subscriber_ = nodeHandle_.subscribe("/concatenate_point_cloud/scan", 10, &LidarDocking::read_point_cloud_concat, this);
    pose_subscriber = nodeHandle.subscribe("/odom", 10, &lidar_docking::LidarDocking::pose_callback, this);
    
    pub = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    search_for_markers_pub_ = nodeHandle.advertise<std_msgs::Bool>("search_markers",1000);
    nav_ac_ptr_ = std::make_shared<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);

    // Lidar-based-docking
    // cloud_pub_ = nodeHandle.advertise<sensor_msgs::PointCloud2>("point_cloud", 10);

    readParams(nodeHandle);
}


/* --------------------- END OF CAMERA DOCKING --------------- */





/* -------- BELOW THIS, THIS IS THE CODE FOR LIDAR DOCKING --------- */


/**
 * @brief lidar_docking::LidarDocking::LidarDocking constructor
 */
lidar_docking::LidarDocking::LidarDocking() :  active_(false), paused_(false)
{
    ros::NodeHandle nodeHandle("~");
    nodeHandle_ = nodeHandle;

    point_cloud_subscriber_ = nodeHandle_.subscribe("/scan_front", 10, &LidarDocking::read_point_cloud, this);
    point_cloud_subscriber_rear_ = nodeHandle_.subscribe("/scan_rear", 10, &LidarDocking::read_point_cloud_rear, this);
    pose_subscriber = nodeHandle.subscribe("/odom", 10, &lidar_docking::LidarDocking::pose_callback, this);

    pub = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    search_for_markers_pub_ = nodeHandle.advertise<std_msgs::Bool>("search_markers",10);

    search_for_markers_.data = false;
    nav_ac_ptr_ = std::make_shared<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);

    // Lidar-based-docking
    // cloud_pub_ = nodeHandle.advertise<sensor_msgs::PointCloud2>("point_cloud", 10);

    readParams(nodeHandle);
}


/**
 * @brief lidar_docking::LidarDocking::pose_callback
 * subscribing to odometry msgs and on the basis of difference of robot and  pallet's mid points, deciding
 * when to pick up or drop an object
 * @param msg
 */
// Lidar-based-docking
void lidar_docking::LidarDocking::pose_callback(const nav_msgs::Odometry::ConstPtr& msg)
{

    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    robot_theta_ = msg->pose.pose.orientation.z ;

    //    std::cout<<"data from  pose is "<<robot_x <<"   "<<robot_y<<" "<<robot_theta<<std::endl;
    float err_x, err_y;

    err_y = pallet_point_y - robot_y_;
    err_x = pallet_point_x - robot_x_;
    //    std::cout<<"total err  "<<err_x*err_x + err_y*err_y<<std::endl;

    if( err_x*err_x + err_y*err_y <= 0.2)   //if difference with robot points less than 10cm , start object picking or dropping
    {
        finished_docking_ = true;
    }
    if(err_x*err_x + err_y*err_y < 1)
    {
        robot_inside_pallet_ = true;
    }
}


/**
 * @brief LidarDocking::read_point_cloud - reading front point cloud
 * also looks for obstacles and stops the vehicle if obstale closer than the mentioned threshold value
 * @param msg
 */
void lidar_docking::LidarDocking::read_point_cloud(const sensor_msgs::LaserScan& msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    //change on the basis of the point cloud area that needs to be considered
    int obstacle_points_count = 0;
    for(int i = 0; i < msg.ranges.size(); i++)    // greater than 270
    {
        if( msg.ranges[i] < 1 and msg.intensities[i]!=pallet_intensity_)
        {
            obstacle_points_count+=1;
        }

    }
    if(obstacle_points_count >=3)
    {
        std::cout<<"obstacle_encountered at the front of the robot"<<std::endl;
        move(0,0);
    }
    //    search_for_markers_pub_.publish(search_for_markers_);
}


/**
 * @brief lidar_docking::LidarDocking::read_point_cloud_rear read rear point clound subscriber and check for obstacles
 * @param msg
 */
void lidar_docking::LidarDocking::read_point_cloud_rear(const sensor_msgs::LaserScan& msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    //change on the basis of the point cloud area that needs to be considered
    int obstacle_points_count = 0;
    for(int i = 0; i < msg.ranges.size(); i++)    // greater than 270
    {
        if( msg.ranges[i] < 1 and msg.intensities[i]!=pallet_intensity_)
        {
            obstacle_points_count+=1;
        }
    }
    if(obstacle_points_count >= 3)
    {
        std::cout<<"obstacle encountered at the rear of the vehicle "<<std::endl;
        move(0,0);
    }
}

void lidar_docking::LidarDocking::read_point_cloud_concat(const sensor_msgs::LaserScan& msg)  //read fused point cloud
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    //change on the basis of the point cloud area that needs to be considered
    for(int i = 0; i < msg.ranges.size(); i++)    // greater than 270
    {
        //        if( msg.ranges[i] < 10)
        {
            pcl::PointXYZI pt;
            pt.y = msg.ranges[i]*sin(msg.angle_min + i*msg.angle_increment);
            pt.x = msg.ranges[i]*cos(msg.angle_min + i*msg.angle_increment);
            pt.z = 0;
            pt.intensity = msg.intensities[i];
            point_cloud->points.push_back(pt);
        }

    }
    search_for_markers_pub_.publish(search_for_markers_);

    //////////////uncomment if you wish to visualise the point cloud //////////////////////////////
    //        sensor_msgs::PointCloud2 msg2;
    //        pcl::toROSMsg(*point_cloud, msg2);
    //        msg2.header.stamp = ros::Time::now();
    //        msg2.header.frame_id= "/base_link";

    //            cloud_pub_.publish(msg2);

    //            intensityClusters_(point_cloud);
    ///////////////////////////////////////////////////////////////////////////////////////////////

    if(ready_for_docking_)
    {
        intensityClusters_(point_cloud);

        if (finished_docking_)
        {
            move(0,0);
            ready_for_docking_ = false;
        }
    }

}


