#include<iostream>
#include "camera_lidar_docking/StartAutoDock.h"
#include "camera_lidar_docking/FinishedDock.h"
#include "nav_msgs/Odometry.h"

using namespace std;

class CameraLidar {

protected:
    bool auto_dock_start_position_reached = false;
    ros::NodeHandle nodeHandle_;
public:
    CameraLidar(ros::NodeHandle &nodeHandle);
    ros::ServiceServer startAutoDock;

    bool autoDock(camera_lidar_docking::StartAutoDock::Request  &req,
                  camera_lidar_docking::StartAutoDock::Response &res);



};


