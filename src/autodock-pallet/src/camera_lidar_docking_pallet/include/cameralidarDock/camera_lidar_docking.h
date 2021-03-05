#include<iostream>
#include "nav_msgs/Odometry.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

using namespace std;

class CameraLidar {

protected:
    bool auto_dock_start_position_reached = false;

public:
    CameraLidar(ros::NodeHandle &nodeHandle);



};


