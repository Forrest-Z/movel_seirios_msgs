#ifndef POSE_RECORDER_H
#define POSE_RECORDER_H
#include "ros/ros.h"
#include <movel_seirios_msgs/TeleopPlanner.h>
#include <movel_seirios_msgs/StringTrigger.h>
#include <cstdlib>
#include <nav_msgs/Path.h>
#include <std_srvs/SetBool.h>
namespace pose_recorder
{


class PoseRecorder 
{
  public:

    PoseRecorder();
    // ~PoseRecorder();
    bool receiveStopCommand(movel_seirios_msgs::StringTrigger::Request &req, movel_seirios_msgs::StringTrigger::Response &res);
    void poseCb(const geometry_msgs::Pose& msg);

  private:
    ros::NodeHandle n;
    std::vector<geometry_msgs::PoseStamped> recorder_pose;
    nav_msgs::Path path_;
    bool record_trigger;
    size_t i;
    float last_x,last_y;


    // subscribers and publishers
    ros::ServiceClient client ;
    ros::ServiceServer record;
    ros::Subscriber sub ;

};

}  

#endif  
