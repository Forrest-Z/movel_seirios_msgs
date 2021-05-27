#include <ros/ros.h>
#include <dafan_docking/ToggleDocking.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void setGoalMsg(move_base_msgs::MoveBaseGoal& goal, 
            float& g_x, float& g_y, float& g_z, 
            float& g_qx, float& g_qy, float& g_qz, float& g_qw){

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  //set position
  goal.target_pose.pose.position.x = g_x;
  goal.target_pose.pose.position.y = g_y;
  goal.target_pose.pose.position.z = g_z;
  //set orientation
  goal.target_pose.pose.orientation.x = g_qx;
  goal.target_pose.pose.orientation.y = g_qy;
  goal.target_pose.pose.orientation.z = g_qz;
  goal.target_pose.pose.orientation.w = g_qw;
}


void toggleDocking(ros::ServiceClient& toggle_docking_client, bool dock_){

  dafan_docking::ToggleDocking srv_;
  srv_.request.toggle = dock_; 

  if (toggle_docking_client.call(srv_)){
    ROS_INFO("Service call to auto-docking successful");
    if (srv_.response.april_detected == false){
      ROS_WARN("April tags not detected, unable to find docking goal");
    }
    else{
      ROS_INFO("April tags detected, starting to dock.");
    }
  }
  else{
    ROS_ERROR("Service call to auto-docking failed");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pre_docking");
  ros::NodeHandle nh_;

  //listen to battery percentage
  ros::Subscriber batt_;

  //init services
  ros::ServiceClient toggle_docking_client;
  toggle_docking_client = nh_.serviceClient<dafan_docking::ToggleDocking>("toggle_docking");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  //get params
  float g_x, g_y, g_z;
  float g_qx, g_qy, g_qz, g_qw;
  ros::param::param<float>("~g_x", g_x, 0.0);
  ros::param::param<float>("~g_y", g_y, 0.0);
  ros::param::param<float>("~g_z", g_z, 0.0);
  ros::param::param<float>("~goal_qx", g_qx, 0.0);
  ros::param::param<float>("~goal_qy", g_qy, 0.0);
  ros::param::param<float>("~goal_qz", g_qz, 0.0);
  ros::param::param<float>("~goal_qw", g_qw, 0.0);

  //send goal
  move_base_msgs::MoveBaseGoal goal;
  setGoalMsg(goal, g_x, g_y, g_z, g_qx, g_qy, g_qz, g_qw);
  ROS_INFO("Going to pre-docking position");
  ac.sendGoal(goal);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Successfully reached pre-docking goal. Calling auto-docking service.");
    ros::Rate r(0.25);
    r.sleep();
    toggleDocking(toggle_docking_client, true);
  }
  else{
    ROS_WARN("Pre-docking goal failed, will not call auto-docking service.");
  }

  return 0;

}
