
#include <ros/ros.h>
#include <camera_docking_/camera_docking.h>
//#include "lidar_docking_/LidarDocking.hpp"
#include<cameralidarDock/camera_lidar_docking.h>

using namespace std;
using namespace cv;


int main(int argc, char ** argv) {
    ros::init(argc, argv, "camera_lidar_docking");
    ros::NodeHandle nodeHandle("~");


    CameraLidar camera_lidar(nodeHandle);
    camera_lidar.startAutoDock = nodeHandle.advertiseService("StartAutoDocking", &CameraLidar::autoDock, &camera_lidar);
    ros::spin();

    return 0;
}
