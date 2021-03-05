
#include <ros/ros.h>
#include "lidar_docking_/LidarDocking.hpp"
#include<cameralidarDock/camera_lidar_docking.h>
#include "navigation_based_docking/navigation_based_docking.hpp"
#include <movel_hasp_vendor/license.h>

using namespace std;


int main(int argc, char ** argv) {


    #ifdef MOVEL_LICENSE                                                                                                    
        MovelLicense ml(18);                                                                                                   
        if (!ml.login())                                                                                                      
            return 1;                                                                                                           
    #endif

    ros::init(argc, argv, "camera_lidar_docking");
    ros::NodeHandle nodeHandle("~");

    lidar_docking::LidarDocking LidarDocking(nodeHandle);
    
    LidarDocking.startAutoDock = nodeHandle.advertiseService("/autodock_pallet/StartAutoDocking", &lidar_docking::LidarDocking::autoDock, &LidarDocking);
    LidarDocking.nav_client = nodeHandle.serviceClient<navigation_based_docking::StartAutoDock>("/navigation_based_docking/NavigationDocking");
    LidarDocking.pause_sub_ = nodeHandle.subscribe("/autodock_pallet/pauseDocking",1, &lidar_docking::LidarDocking::pauseCb, &LidarDocking);
    LidarDocking.resume_sub_ = nodeHandle.subscribe("/autodock_pallet/resumeDocking",1, &lidar_docking::LidarDocking::resumeCb, &LidarDocking);
    LidarDocking.cancel_sub_ = nodeHandle.subscribe("/autodock_pallet/cancelDocking", 1, &lidar_docking::LidarDocking::cancelCb, &LidarDocking);

    LidarDocking.status_pub_ = nodeHandle.advertise<std_msgs::UInt8>("/autodock_pallet/status", 1);
    while(ros::ok())
    {
        LidarDocking.executeDocking();
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    #ifdef MOVEL_LICENSE                                                                                                    
        ml.logout();          
    #endif      
        
    return 0;
}
