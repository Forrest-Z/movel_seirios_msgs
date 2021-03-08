#include<iostream>
#include "camera_lidar_docking_charging_station/StartAutoDock.h"
#include "camera_lidar_docking_charging_station/FinishedDock.h"
#include "nav_msgs/Odometry.h"

using namespace std;

class CameraLidar {

protected:
    bool auto_dock_start_position_reached = false;
    ros::NodeHandle nodeHandle_;
public:
    CameraLidar(ros::NodeHandle &nodeHandle);
    ros::ServiceServer startAutoDock;

    bool autoDock(camera_lidar_docking_charging_station::StartAutoDock::Request  &req,
                  camera_lidar_docking_charging_station::StartAutoDock::Response &res);



};


