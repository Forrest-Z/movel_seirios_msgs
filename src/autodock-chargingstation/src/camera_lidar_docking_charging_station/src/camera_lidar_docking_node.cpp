
#include <ros/ros.h>
#include <camera_docking_/camera_docking.h>
//#include "lidar_docking_/LidarDocking.hpp"
#include <cameralidarDock/camera_lidar_docking.h>
#include <movel_hasp_vendor/license.h>

using namespace std;
using namespace cv;


int main(int argc, char ** argv) {
    #ifdef MOVEL_LICENSE                                                                                                    
        MovelLicense ml;                                                                                                   
        if (!ml.login())                                                                                                      
            return 1;                                                                                                           
    #endif

    ros::init(argc, argv, "camera_lidar_docking");
    ros::NodeHandle nodeHandle("~");


    CameraLidar camera_lidar(nodeHandle);
    camera_lidar.startAutoDock = nodeHandle.advertiseService("StartAutoDocking", &CameraLidar::autoDock, &camera_lidar);
    ros::spin();
    #ifdef MOVEL_LICENSE                                                                                                    
        ml.logout();          
    #endif     

    return 0;
}
