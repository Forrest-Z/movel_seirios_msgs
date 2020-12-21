
#include "lidar_docking_/LidarDocking.hpp"
#include "camera_lidar_docking/StartAutoDock.h"

#include <cameralidarDock/camera_lidar_docking.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
namespace lidar_docking {


/**
 * @brief LidarDocking::~LidarDocking contructor
 */
LidarDocking::~LidarDocking()
{

}


/**
 * @brief LidarDocking::move - move function to move the robot
 * @param linear_speed
 * @param angular_speed
 */
void LidarDocking::move(float linear_speed, float angular_speed)
{
    ptr.linear.x = linear_speed;
    ptr.linear.y = 0;
    ptr.linear.z = 0;
    ptr.angular.x = 0;
    ptr.angular.y = 0;
    ptr.angular.z = angular_speed;

    pub.publish(ptr);
}


/**
 * @brief LidarDocking::find_length - finding length of the cluster
 * @param cluster
 * @return
 */
float LidarDocking::find_length(std::vector<pcl::PointXYZI> cluster)
{

    float length = (cluster[0].x - cluster[cluster.size()-1].x)*(cluster[0].x - cluster[cluster.size()-1].x) +
            (cluster[0].y - cluster[cluster.size()-1].y)*(cluster[0].y - cluster[cluster.size()-1].y);
    //    std::cout<<"cluster size :"<<cluster.size()<<" ";
    //    std::cout<<"length of cluster is "<<sqrt(length)<<" ";
    //    std::cout<<"mean x :" << (cluster[0].x + cluster[cluster.size()-1].x)/2 <<" ";
    //    std::cout<<"mean y :" << (cluster[0].y + cluster[cluster.size()-1].y)/2 <<std::endl;

    return sqrt(length);

}


/**
 * @brief LidarDocking::computeMean - computing mean value of the array
 * @param arr
 * @return
 */
int LidarDocking::computeMean(std::vector<int> arr)
{
    return arr[arr.size()/2];
}


/**
 * @brief LidarDocking::read_point_cloud -construction of a point cloud on the basis of how the lidar
 * has been mounted i.e its orientation
 * also looks for obstacles and stops the vehicle if obstale closer than the mentioned threshold value
 * @param msg
 */
void LidarDocking::read_point_cloud(const sensor_msgs::LaserScan& msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    //change on the basis of the point cloud area that needs to be considered
    for(int i = 0; i < msg.ranges.size(); i++)    // greater than 270
    {
//        if( msg.ranges[i] < 2)
        {
            pcl::PointXYZI pt;
            pt.y = msg.ranges[i]*sin(msg.angle_min + i*msg.angle_increment);
            pt.x = msg.ranges[i]*cos(msg.angle_min + i*msg.angle_increment);
            pt.z = 0;
            pt.intensity = msg.intensities[i];
            point_cloud->points.push_back(pt);
        }

    }

//    for(int i = 7*msg.ranges.size()/8; i < msg.ranges.size(); i++)    // greater than 270
//    {
////        if( msg.ranges[i] < 2)
//        {
//            pcl::PointXYZI pt;
//            pt.y = msg.ranges[i]*sin(msg.angle_min + i*msg.angle_increment);
//            pt.x = msg.ranges[i]*cos(msg.angle_min + i*msg.angle_increment);
//            pt.z = 0;
//            pt.intensity = msg.intensities[i];
//            point_cloud->points.push_back(pt);
//        }

//    }

//    for(int i = 0; i < msg.ranges.size()/8; i++)    // 0 - 45deg
//    {
////        if( msg.ranges[i] < 2)
//        {
//            pcl::PointXYZI pt;
//            pt.y = msg.ranges[i]*sin(msg.angle_min + i*msg.angle_increment);
//            pt.x = msg.ranges[i]*cos(msg.angle_min + i*msg.angle_increment);
//            pt.z = 0;
//            pt.intensity = msg.intensities[i];
//            point_cloud->points.push_back(pt);
//        }
//    }

    sensor_msgs::PointCloud2 msg2;
    pcl::toROSMsg(*point_cloud, msg2);
    msg2.header.stamp = ros::Time::now();
    msg2.header.frame_id= "base_link";

    cloud_pub_.publish(msg2);

    int count_points_obstacle = 0;
//    for(int i = 0; i < point_cloud->size(); i++)
//    {
//        if(point_cloud->points[i].x < distance - 0.1 and abs(point_cloud->points[i].y) < 0.3)
//        {
//            count_points_obstacle += 1;
//        }
//    }
    if(count_points_obstacle >= min_points_obstacle_)     //3
    {
        move(0,0);
        obstacle_found_ = true;
        std::cout<<"obstacle encountered"<<std::endl;
    }
    else
    {
        obstacle_found_ = false;
        intensityClusters(point_cloud);
    }
    if(obstacle_found_)
    {
        time_counter_+=1;
        if(time_counter_ > time_for_obstacle_clearance_ )
        {
            std::cout<<"aborting the docking operation as obstacle not getting out of the way"<<std::endl;
            exit(0);
        }
       ros::Duration(1.0).sleep();

    }
    else
    {
        time_counter_ = 0;
    }
}


float lidar_docking::LidarDocking::lineFit(pcl::PointCloud<pcl::PointXYZI> cloud)
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_LINE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.1);
    seg.setInputCloud (cloud.makeShared());
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a LINE model for the given dataset.");
    }
    float angle=2*asin(coefficients->values[4])*180.0/M_PI;//formulae for quarternions.
    std::cout <<"deviation from the pallet: "<<angle<<std::endl;
//    angle_  = angle;
        return angle;

}


/**
 * @brief LidarDocking::start_recovery - recover to the starting docking position
 */
void LidarDocking::start_recovery()
{

    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = charger_x_;
    goal.target_pose.pose.position.y = charger_y_;
    goal.target_pose.pose.orientation.z = charger_theta_;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();
    std::cout<<"done via recovery"<<std::endl;

}

pcl::PointCloud<pcl::PointXYZI>lidar_docking::LidarDocking::cluster_to_point_cloud(vector<pcl::PointXYZI> cluster)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for(int i=0;i < cluster.size(); i++ )
    {
        pcl::PointXYZI pt;
        pt.x = cluster[i].x;
        pt.y =  cluster[i].y;
        pt.z =  cluster[i].z;
        pt.intensity =  cluster[i].intensity;
        point_cloud->push_back(pt);
    }
    return *point_cloud;
}


} /* namespace */
