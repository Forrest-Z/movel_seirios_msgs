#include "teleop_global_planner/pose_client.h"
// #include <movel_hasp_vendor/license.h>


namespace pose_recorder{


PoseRecorder::PoseRecorder(){
   ROS_INFO("[PoseRecorder] Intialized "); 
  client = n.serviceClient<movel_seirios_msgs::TeleopPlanner>("move_base/TeleopGlobalPlanner/teleop_plan"), this;
  record = n.advertiseService("teleop_recorder", &PoseRecorder::receiveStopCommand, this);
  sub = n.subscribe("robot_pose", 1, &PoseRecorder::poseCb, this);
  record_trigger = false;
  i = 0;
  last_x = 0;
  last_y = 0;
  recorder_pose.clear();
}


void PoseRecorder::poseCb(const geometry_msgs::Pose& msg)
{
  if(record_trigger){
      //  ROS_INFO("points %f  %f ",(int)(msg.position.x  * 100.0) / 100.0, last_x);
      float roundoff_x = ((int)(msg.position.x  * 100.0) / 100.0) ;
      float roundoff_y = ((int)(msg.position.x  * 100.0) / 100.0) ;
      if (roundoff_x != last_x  || roundoff_y != last_y ){
        if (i > 0){
        ROS_INFO("Data: count: %d  %f  %f ", i, msg.position.x, msg.position.y); 
        geometry_msgs::PoseStamped poses;
        poses.header.seq = static_cast<int>(i) ;
        poses.header.frame_id = "map";
        poses.pose.position.x = msg.position.x;
        poses.pose.position.y = msg.position.y;
        poses.pose.orientation.z = msg.orientation.z;
        poses.pose.orientation.w = msg.orientation.w;
        recorder_pose.push_back(poses);
        }
        i++;

      }
      last_y = roundoff_y ;
      last_x = roundoff_x ;
    }
  else{
    i = 0;
  }

}

bool PoseRecorder::receiveStopCommand(movel_seirios_msgs::StringTrigger::Request &req, movel_seirios_msgs::StringTrigger::Response &res) {
  if(req.input == "on" ){
    record_trigger = true;
    res.message = "recorder  enabled";
  }
  if(req.input == "off" ){
    record_trigger = false;
    res.message = "recorder  disabled";
  }
  if(req.input == "send" ){
    if (!record_trigger){
      path_.poses.clear();
      if(!recorder_pose.empty()){
        path_.header.frame_id = "map";
        path_.poses = recorder_pose;
        recorder_pose.clear();
        res.message = "Pose  sent";

        movel_seirios_msgs::TeleopPlanner srv;
        srv.request.teleop_path = path_;
        // srv.request.b = atoll(argv[2]);
        if (client.call(srv))
        {
          ROS_INFO("Sum: %ld", (long int)srv.response.success);
        }
        else
        {
          res.success = false;
          ROS_ERROR("Failed to call service teleop_plan");
          return 1;
        }
      }
      else{
        res.message = "Pose   empty";
      }
    }
    else{
      res.message = "Turn off recorder";
    }
  }
  res.success = true;
  return true;
}
}


int main(int argc, char **argv)
{
  // #ifdef MOVEL_LICENSE
  //   MovelLicense ml();
  //   if (!ml.login())
  //     return 1;
  // #endif

  ros::init(argc, argv, "pose_client");
  pose_recorder::PoseRecorder poseclient;
  ros::spin();

  // #ifdef MOVEL_LICENSE
  //   ml.logout();
  // #endif
  return 0;
}