#include <iostream>
#include <fstream>
#include<vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <movel_hasp_vendor/license.h>

#include <fstream>

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

std::ofstream myfile;
bool user_io;
char input;
/**
 * @brief pose_callback subscriber to odom topic and saves the wayoints when s is pressed or quits when q is presseds
 * @param msg
 */
void pose_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (user_io)
    {
        cout<<"enter command "<<std::endl;
        cin>>input;
    }

    // Change type
    geometry_msgs::PoseStamped odom_pose;
    odom_pose.pose.position= msg->pose.pose.position;
    odom_pose.pose.orientation= msg->pose.pose.orientation;

    if (!user_io)
    {
        // Transform
        tf2_ros::Buffer tfBuffer;           // Buffer
        tf2_ros::TransformListener tfListener(tfBuffer);        // Listener

        geometry_msgs::TransformStamped transformStamped;
        geometry_msgs::PoseStamped map_pose;

        try{
            tfBuffer.canTransform("map", "odom", ros::Time(0), ros::Duration(1.0));
            transformStamped = tfBuffer.lookupTransform("map", "odom", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            return;
        }
        tf2::doTransform(odom_pose.pose, map_pose.pose, transformStamped);

        // Print
        cout<<endl;
        cout<<"x: "<<map_pose.pose.position.x<<" ";
        cout<<"y: "<<map_pose.pose.position.y<<endl;
        cout<<"qz: "<<map_pose.pose.orientation.z<<" " ;
        cout<<"qw: "<<map_pose.pose.orientation.w<<endl<<endl ;
        
        if(input=='q')
        {
            exit(0);
        }

        if(input=='s')
        {
            cout<<"Saving waypoints "<<endl;
            myfile << map_pose.pose.position.x<<std::endl;
            myfile << map_pose.pose.position.y<<std::endl;
            myfile << map_pose.pose.orientation.z<<std::endl;
            myfile << map_pose.pose.orientation.w<<std::endl;
            myfile << std::endl;
        }
        else
        {
            cout<<"Please type 's' for save and 'q' for quit"<<endl;
        }
        user_io = 1;
    }
    else
    {
        user_io = false;
    }
    
}


int main(int argc, char * argv[])
{
    #ifdef MOVEL_LICENSE                                                                                                    
        MovelLicense ml(18);                                                                                                   
        if (!ml.login())                                                                                                      
            return 1;                                                                                                           
    #endif
    std::vector<float>data;
    ros::init(argc, argv, "navigation_based_docking");
    ros::NodeHandle nodeHandle("~");
    ros::Subscriber  pose_subscriber = nodeHandle.subscribe("/odom", 10, pose_callback);
    string file_name_;
    nodeHandle.getParam("/record_waypoints/waypoints_file", file_name_);
    std::cout<<file_name_<<std::endl;
    myfile.open(file_name_);
    user_io = true;
    ros::spin();

    #ifdef MOVEL_LICENSE                                                                                                    
        ml.logout();          
    #endif     


    return 0;
}
