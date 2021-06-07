#include "move_base_msgs/MoveBaseFeedback.h"
#include <actionlib_msgs/GoalID.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetPlan.h>
#include <movel_seirios_msgs/ChooseTarget.h>
#include <movel_seirios_msgs/ClearTarget.h>
#include <movel_seirios_msgs/TrackArray.h>
#include <unordered_map>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

class VelocityCommands {
   public:
      VelocityCommands();  //constructor
      ~VelocityCommands(){} //destructor
   private:
      geometry_msgs::PoseStamped goal_;
      actionlib_msgs::GoalID move_base_goal_id_;
      void getNearestPseudoGoal(geometry_msgs::Pose pos); 
      ros::ServiceClient plan_client_; //!< Get path plan from move_base
      void populateClient(nav_msgs::GetPlan &srv,geometry_msgs::Pose target_pose); 
      int y_samples_;
      bool following_;
      float goal_dist_thresh_;
      bool getParams(); //for initialization
      bool setupTopics();
      bool stop_track_;
      float target_offset_;
      int target_id_;
      //int identities_[50];
      ros::NodeHandle n_;
      ros::NodeHandle n_private;
      ros::Publisher target_pub_;
      ros::Publisher cancel_pub_;
      ros::Publisher goal_pub_;
      ros::Subscriber sub_;
      ros::Subscriber feedback_sub_;
      ros::Subscriber pose_sub_;
      ros::ServiceServer choose_target_;
      ros::ServiceServer clear_target_;
      bool choose_target(movel_seirios_msgs::ChooseTarget::Request &req, movel_seirios_msgs::ChooseTarget::Response &res);
      bool clear_target(movel_seirios_msgs::ClearTarget::Request &req, movel_seirios_msgs::ClearTarget::Response &res);
      float radius_;
      float calcDist(float x, float y);
      void callback(const movel_seirios_msgs::TrackArray &msg);
      void feedback_cb(const move_base_msgs::MoveBaseActionFeedback::ConstPtr &msg);
      void pose_cb(const geometry_msgs::Pose &msg);
      void nav_robot(float position_x,float position_y);
      geometry_msgs::Pose pose_;
      std::unordered_map<int,float> unordered_map_; 
      tf2_ros::Buffer buffer_;
      tf2_ros::TransformListener listener_;      
};

