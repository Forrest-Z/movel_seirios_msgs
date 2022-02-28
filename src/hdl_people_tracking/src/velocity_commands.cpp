#include <hdl_people_tracking/people_tracker.hpp>
#include <hdl_people_tracking/velocity_commands.hpp>
#include <movel_hasp_vendor/license.h>

VelocityCommands::VelocityCommands(): target_id_(-2), n_private("~"), following_(false), listener_(buffer_)
{
    if (!getParams() || !setupTopics())
        {ROS_ERROR("Parameter error. Please try again.");}
    ROS_INFO("Finished Initialization");
    while(ros::ok()){
    ros::spinOnce();
    }

}

bool VelocityCommands::getParams()
{   
    ros::param::param<float>("~goal_dist_thresh", goal_dist_thresh_, 0.5);
    ros::param::param<float>("~target_offset", target_offset_, 0.9);
    ros::param::param<float>("~radius", radius_, 1.50);
    ros::param::param<int>("~y_samples", y_samples_, 5);
    return true;
}

bool VelocityCommands::setupTopics()
{
  //Publishers
  cancel_pub_ = n_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
  goal_pub_ = n_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  target_pub_ = n_.advertise<std_msgs::Int32>("/velocity_commands/target_id", 1);
  //Subscribers
  sub_ = n_.subscribe("/hdl_people_tracking_nodelet/tracks", 1, &VelocityCommands::callback, this);
  feedback_sub_ = n_.subscribe("/move_base/feedback", 1, &VelocityCommands::feedback_cb, this);
  pose_sub_ = n_.subscribe("/pose", 1, &VelocityCommands::pose_cb, this);
  //Services
  choose_target_ = n_.advertiseService("/hdl_people_tracking/choose_target", &VelocityCommands::choose_target, this);
  clear_target_ = n_.advertiseService("/hdl_people_tracking/clear_target", &VelocityCommands::clear_target, this);
  plan_client_ = n_.serviceClient<nav_msgs::GetPlan>("/move_base/GlobalPlanner/make_plan", true);

  return true;
}

void VelocityCommands::pose_cb(const geometry_msgs::Pose &msg)
{
    pose_ = msg;
    return;
}

float VelocityCommands::calcDist(float x, float y)
{
    float ans =  pow(x,2) + pow(y,2);
    return sqrt(ans);
}

void VelocityCommands::callback(const movel_seirios_msgs::TrackArray &msg)
{   
    //identities_[50] = {-1};
    unordered_map_.clear();
    unordered_map_.reserve(msg.tracks.size());
	  for (movel_seirios_msgs::Track track: msg.tracks)
	  {   //identities_[idx] = track.id; idx++;
                float depth = calcDist(track.pos.x, track.pos.y); 
                std::pair<int,float> a_pair (track.id,depth);
                unordered_map_.insert(a_pair);
		if (track.id == target_id_ && depth > target_offset_){
                nav_robot(track.pos.x, track.pos.y);
                return;
	        }
    else {continue;}
    }
}

void VelocityCommands::nav_robot(float position_x,float position_y)
{
    geometry_msgs::PoseStamped laser_pose;
    laser_pose.header.frame_id = "laser";
    laser_pose.pose.position.x = position_x - 1.0;
    laser_pose.pose.position.y = position_y;
    laser_pose.pose.orientation.w = 1.0;
    geometry_msgs::PoseStamped map_pose = buffer_.transform(laser_pose, "map");
    float diff_x = map_pose.pose.position.x - goal_.pose.position.x;
    float diff_y = map_pose.pose.position.y - goal_.pose.position.y;
    float dist = calcDist(diff_x, diff_y);
    if (dist >= goal_dist_thresh_ || following_ == false){
    following_ = true;
    nav_msgs::GetPlan srv;
    populateClient(srv, map_pose.pose); 
    goal_.header.frame_id = "map";
    goal_.header.stamp = ros::Time::now();
    try {
    plan_client_.call(srv);
    if (srv.response.plan.poses.size() > 0){
    goal_= map_pose;
    }
    else{getNearestPseudoGoal(map_pose.pose);}
    ROS_INFO_STREAM("SENDING GOAL's coordinates, x: " << goal_.pose.position.x << ", y: " << goal_.pose.position.y);
    goal_pub_.publish(goal_);
    return;
    } 
    catch (...)
    {
      ROS_ERROR("Failed to call service /move_base/GlobalPlanner/make_plan");
    }
    return;
    }
 }

bool VelocityCommands::choose_target(movel_seirios_msgs::ChooseTarget::Request &req, movel_seirios_msgs::ChooseTarget::Response &res)
{
    //bool exists = std::find(std::begin(identities_), std::end(identities_), req.target_id) != std::end(identities_);
    bool cannot_find = unordered_map_.find(req.target_id) == unordered_map_.end();
    if (!cannot_find){
    if (unordered_map_.at(req.target_id) <= radius_)
    {std_msgs::Int32 target_msg; 
    target_id_ = req.target_id; 
    res.target_selected_success = true; 
    target_msg.data = target_id_; 
    target_pub_.publish(target_msg);
    return true; 
    }
    else {
      ROS_ERROR("Target is outside radius."); 
      res.target_selected_success = false; 
      return false;}
    }
    if (cannot_find){
    ROS_ERROR("Target not found."); 
    res.target_selected_success = false; 
    return false;
    }
    return false;
}

bool VelocityCommands::clear_target(movel_seirios_msgs::ClearTarget::Request &req, movel_seirios_msgs::ClearTarget::Response &res)
{
  if (req.clear_target) {
    target_id_ = -2; 
    res.target_cleared_success = true; 
    actionlib_msgs::GoalID cancel_path;
    cancel_pub_.publish(move_base_goal_id_);
    cancel_pub_.publish(cancel_path);
    return true;}
  else {
    res.target_cleared_success = false; 
    return false;}
}

void VelocityCommands::getNearestPseudoGoal(geometry_msgs::Pose pos) 
{
  for (int i = 1; i <= y_samples_; i++)
  { float alpha_step = 0.3*i;
    for (int j = 0; j < 4; j++)
  {geometry_msgs::Pose estimated_nearby = pos;
   if (j == 0)
   {   estimated_nearby.position.x = pos.position.x + alpha_step*pos.position.x;
       estimated_nearby.position.y = pos.position.y + alpha_step*pos.position.y;}
   else if (j == 1)
   {   estimated_nearby.position.x = pos.position.x + alpha_step*pos.position.x;
       estimated_nearby.position.y = pos.position.y - alpha_step*pos.position.y;}
   else if (j == 2)
   {   estimated_nearby.position.x = pos.position.x - alpha_step*pos.position.x;
       estimated_nearby.position.y = pos.position.y + alpha_step*pos.position.y;}
   else if (j == 3)
   {   estimated_nearby.position.x = pos.position.x - alpha_step*pos.position.x;
       estimated_nearby.position.y = pos.position.y - alpha_step*pos.position.y;}
    
   nav_msgs::GetPlan srv;
   populateClient(srv, estimated_nearby);
   try {
      plan_client_.call(srv);
      ROS_INFO("iter %d, %d, plan size %lu", i, j, srv.response.plan.poses.size());
      //! Check if plan to waypoint is viable
      if (srv.response.plan.poses.size() > 0){
          goal_.pose = estimated_nearby;return;}
      }
   catch (...)
    {
      ROS_ERROR("Failed to call service /move_base/GlobalPlanner/make_plan");
    }
  }
  } 
  ROS_ERROR("Failed to find a viable goal. Please check for obstacles.");
} 

void VelocityCommands::populateClient(nav_msgs::GetPlan &srv,
                                      geometry_msgs::Pose target_pose)
{
  srv.request.start.header.frame_id = "map";
  
  srv.request.start.pose.position.x = pose_.position.x;
  srv.request.start.pose.position.y = pose_.position.y;
  srv.request.start.pose.position.z = pose_.position.z;
  srv.request.start.pose.orientation.x = pose_.orientation.x;
  srv.request.start.pose.orientation.y = pose_.orientation.y;
  srv.request.start.pose.orientation.z = pose_.orientation.z;
  srv.request.start.pose.orientation.w = pose_.orientation.w;

  srv.request.goal.header.frame_id = "map";
  srv.request.goal.pose.position.x = target_pose.position.x;
  srv.request.goal.pose.position.y = target_pose.position.y;
  srv.request.goal.pose.position.z = target_pose.position.z;
  srv.request.goal.pose.orientation.x = target_pose.orientation.x;
  srv.request.goal.pose.orientation.y = target_pose.orientation.y;
  srv.request.goal.pose.orientation.z = target_pose.orientation.z;
  srv.request.goal.pose.orientation.w = target_pose.orientation.w;

  srv.request.tolerance = 0;
}

void VelocityCommands::feedback_cb(const move_base_msgs::MoveBaseActionFeedback::ConstPtr &msg) 
{
  move_base_goal_id_ = msg->status.goal_id;
  return;
}

int main(int argc, char** argv) 
{
   #ifdef MOVEL_LICENSE   
     MovelLicense ml;                                                                                                   
    if (!ml.login())                                                                                                      
      return 1;                                                                                                           
  #endif

   ros::init(argc, argv, "velocity_commands_node");
   VelocityCommands vc;
   #ifdef MOVEL_LICENSE                                                                                                    
    ml.logout();          
  #endif   
   return 0;
}
