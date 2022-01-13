#include <task_supervisor/plugins/multi_point_navigation_handler.h>
#include <task_supervisor/json.hpp>
#include <pluginlib/class_list_macros.h>
#include <actionlib_msgs/GoalID.h>
#include <movel_seirios_msgs/GetReachableSubplan.h>
#include <type_traits>

PLUGINLIB_EXPORT_CLASS(task_supervisor::MultiPointNavigationHandler, task_supervisor::TaskHandler);

using json = nlohmann::json;

namespace task_supervisor
{

MultiPointNavigationHandler::MultiPointNavigationHandler() : 
  task_cancelled_(false),
  isHealthy_(true)
{
}

bool MultiPointNavigationHandler::setupHandler(){

  ROS_INFO("[%s] M.Point SETUP TEST 1 Reached", name_.c_str());
  if (!loadParams()) {
    ROS_FATAL("[%s] Error during parameter loading. Shutting down.", name_.c_str());
    return false;
  }

  /*
  mfn_map_change_server_ = nh_handler_.advertiseService("/mfn_change_map", &MultiFloorNavigationHandler::MFNChangeMapHandle, this);
  map_change_client_ = nh_handler_.serviceClient<nav_msgs::LoadMap>("/change_map");
  map_nav_change_client_ = nh_handler_.serviceClient<nav_msgs::LoadMap>("/change_map_nav");
  clear_costmap_client_ = nh_handler_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  initial_pose_pub_ = nh_handler_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
  map_changed_pub_ = nh_handler_.advertise<std_msgs::String>("map_changed", 10);
  */
  major_marker_pub_ = nh_handler_.advertise<visualization_msgs::Marker>("/major_marker", 10);
  minor_marker_pub_ = nh_handler_.advertise<visualization_msgs::Marker>("/minor_marker", 10);
  current_marker_pub_ = nh_handler_.advertise<visualization_msgs::Marker>("/current_marker", 10);
  robot_pose_sub_ = nh_handler_.subscribe("/pose", 1, &MultiPointNavigationHandler::robotPoseCB, this);
  cmd_vel_pub_ = nh_handler_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/autonomous", 1);
  obstacle_sub_ = nh_handler_.subscribe("/obst", 1, &MultiPointNavigationHandler::obstacleCB, this);

  obstructed_ = true;

  ROS_INFO("[%s] M.Point SETUP TEST 3 Reached", name_.c_str());

  return true;
}

template <typename param_type>
bool MultiPointNavigationHandler::load_param_util(std::string param_name, param_type& output)
{
  if (!nh_handler_.getParam(param_name, output)) {
    ROS_ERROR("[%s] Failed to load parameter: %s", name_.c_str(), param_name.c_str());
    return false;
  }
  else {  
    if (std::is_same<param_type, bool>::value) {   // bool
      ROS_INFO_STREAM("[" << name_ << "] " << param_name << ": " << output ? "true" : "false");
    }
    else {   // all others 
      ROS_INFO_STREAM("[" << name_ << "] " << param_name << ": " << output);
    }
    return true;
  }
}

bool MultiPointNavigationHandler::loadParams(){
  ROS_INFO("[%s] M.Point SETUP TEST 2 Reached", name_.c_str());
  ROS_WARN("[%s] Loading of plugin parameters by ros_utils has not been implemented. Loading directly from Parameter "
           "Server instead.",
           name_.c_str());
/*
  if (!load_param_util("mfn_map_folder_path", p_map_folder_path_)) { return false; }
  if (!load_param_util("mfn_map_nav_folder_path", p_map_nav_folder_path_)) { return false; }
  if (!load_param_util("mfn_graph_folder_path", p_graph_folder_path_)) { return false; }
  if (!load_param_util("mfn_transit_folder_path", p_transit_folder_path_)) { return false; }
*/
  if (!load_param_util("points_distance", p_point_gen_dist_)){return false;}
  if (!load_param_util("point_goal_tolerance_x", p_goal_tolerance_x_)){return false;}
  if (!load_param_util("point_goal_tolerance_y", p_goal_tolerance_y_)){return false;}
  if (!load_param_util("angular_vel", p_angular_vel_)){return false;}
  if (!load_param_util("linear_vel", p_linear_vel_)){return false;}

  return true;
}

ReturnCode MultiPointNavigationHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message){
  task_active_ = true;
  task_parsed_ = false;
  isHealthy_ = true;
  start_ = ros::Time::now();

  ROS_INFO("[%s] Task payload %s", name_.c_str(), task.payload.c_str());
  json payload = json::parse(task.payload);
  ROS_INFO("[%s] M.Point RUN TEST 0 Reached", name_.c_str());

  if (payload.find("total_points") != payload.end()){
    ROS_INFO("[%s] M.Point RUN TEST 1 Reached", name_.c_str());
    
    int total_points = payload["total_points"].get<int>();
    std::cout<<"Total Points : " << total_points << std::endl;
    std::vector<std::vector<float>> rcvd_coords;
    for(int i = 0 ; i < total_points; i++){
      std::vector<float> coord_instance;
      coord_instance.push_back(payload["points"][i]["x"].get<float>());
      coord_instance.push_back(payload["points"][i]["y"].get<float>());
      rcvd_coords.push_back(coord_instance);
    }
    ROS_INFO("[%s] M.Point RUN TEST 2 Reached", name_.c_str());
    pointsGen(rcvd_coords);
    ROS_INFO("[%s] M.Point RUN TEST 4 Reached", name_.c_str());
    

    if(coords_for_nav_.size()>0){
      // loop 
      // iterate through generated points
      for(int i = 0; i < coords_for_nav_.size(); i++){
        // Show current nav goal
        showCurrentGoal(i);

        // call custom nav in every iteration
        if(!navToPoint(coords_for_nav_[i])){
          // If navigation was unsuccessful, cancel
          setMessage("Navigation to point unsuccessful");
          error_message = message_;
          setTaskResult(false);
          return code_;
        }
      }
      // Successful navigation
      ROS_INFO("[%s] Multi-point nav successfully completed", name_.c_str());
      
    }
    else{
      setMessage("Navigational coordinates vector empty");
      error_message = message_;
      setTaskResult(false);
    }
    
  }
  else{
    setMessage("Malformed payload, Example: {\"total_points\":3, \"points\":[{\"x\":1.0,\"y\":1.0}, {\"x\":3.0,\"y\":3.0}, {\"x\":5.0,\"y\":5.0}]}");
    error_message = message_;
    setTaskResult(false);
  }
  return code_;
}

void MultiPointNavigationHandler::pointsGen(std::vector<std::vector<float>> rcvd_multi_coords){
  coords_for_nav_.clear();
  // loop
  for(int i = 0; i < rcvd_multi_coords.size()-1; i++){
    // coords_for_nav.push_back(rcvd_multi_coords[i]);
    // what if slope is infinite
    float slope;
    float maj_point_distance = std::sqrt(pow((rcvd_multi_coords[i+1][0] - rcvd_multi_coords[i][0]),2)+pow((rcvd_multi_coords[i+1][1] - rcvd_multi_coords[i][1]),2));
    float num_of_points = maj_point_distance/p_point_gen_dist_;
    
    if((num_of_points - int(num_of_points))*p_point_gen_dist_ < 0.1){
      num_of_points--;
    }
    
  
    if((rcvd_multi_coords[i+1][0] - rcvd_multi_coords[i][0]) != 0){
      slope = (rcvd_multi_coords[i+1][1] - rcvd_multi_coords[i][1])/(rcvd_multi_coords[i+1][0] - rcvd_multi_coords[i][0]);
     
      for(int j = 0; j <= int(num_of_points); j++){
        std::vector<float> generated_min_point;
        if(rcvd_multi_coords[i][0] > rcvd_multi_coords[i+1][0]){
          // minus
          generated_min_point.push_back(rcvd_multi_coords[i][0] - ((p_point_gen_dist_*j)*(sqrt(1/(1+pow(slope,2))))));
        }
        else{
          // plus
          generated_min_point.push_back(rcvd_multi_coords[i][0] + ((p_point_gen_dist_*j)*(sqrt(1/(1+pow(slope,2))))));
        }
        if(((rcvd_multi_coords[i][1] > rcvd_multi_coords[i+1][1]) && slope > 0) || ((rcvd_multi_coords[i][1] < rcvd_multi_coords[i+1][1]) && slope < 0)){
          // minus
          generated_min_point.push_back(rcvd_multi_coords[i][1] - ((p_point_gen_dist_*j*slope)*(sqrt(1/(1+pow(slope,2))))));
        }
        else if(((rcvd_multi_coords[i][1] < rcvd_multi_coords[i+1][1]) && slope > 0) || ((rcvd_multi_coords[i][1] > rcvd_multi_coords[i+1][1]) && slope < 0)){
          // 
          generated_min_point.push_back(rcvd_multi_coords[i][1] + ((p_point_gen_dist_*j*slope)*(sqrt(1/(1+pow(slope,2))))));
        }
        else{
          generated_min_point.push_back(rcvd_multi_coords[i][1]);
        }

        coords_for_nav_.push_back(generated_min_point);
      }
    }
    else{
      for(int j = 0; j < int(num_of_points); j++){
        std::vector<float> generated_min_point;
        
        generated_min_point.push_back(rcvd_multi_coords[i][0]);
        
        if(rcvd_multi_coords[i][1] > rcvd_multi_coords[i+1][1]){
          // minus
          generated_min_point.push_back(rcvd_multi_coords[i][1] - (p_point_gen_dist_*j));
        }
        else if(rcvd_multi_coords[i][1] < rcvd_multi_coords[i+1][1]){
          // plus
          generated_min_point.push_back(rcvd_multi_coords[i][1] + (p_point_gen_dist_*j));
        }
        else{
          generated_min_point.push_back(rcvd_multi_coords[i][1]);
        }
        coords_for_nav_.push_back(generated_min_point);
      }
    }
  }
  coords_for_nav_.push_back(rcvd_multi_coords.back());
  ROS_INFO("[%s] M.Point RUN TEST 3 Reached", name_.c_str());
  showAllPoints(rcvd_multi_coords);
}

void MultiPointNavigationHandler::showAllPoints(std::vector<std::vector<float>> rcvd_multi_coords){
  visualization_msgs::Marker major_marker;
  geometry_msgs::Vector3 cube_scale;
  std_msgs::ColorRGBA cube_color;
  
  cube_color.r = 1;
  cube_color.g = 0;
  cube_color.b = 0;
  cube_color.a = 1;
  cube_scale.x = 0.07;
  cube_scale.y = 0.07;
  cube_scale.z = 0.07;

  major_marker.header.frame_id = "map";
  major_marker.header.stamp = ros::Time();
  major_marker.id = 0;
  major_marker.type = 6;
  major_marker.action = 0;
  major_marker.pose.orientation.w = 1.0;
  major_marker.scale = cube_scale;
  major_marker.color = cube_color;
  //major_marker.lifetime = 0;

  std::cout << std::endl << "Major points : \n";
  for(int i = 0; i < rcvd_multi_coords.size(); i++){
    std::cout << "⦿ [" << rcvd_multi_coords[i][0] << ", " << rcvd_multi_coords[i][1] << "]" << std::endl;

    geometry_msgs::Point mark_pose;
    mark_pose.x = rcvd_multi_coords[i][0];
    mark_pose.y = rcvd_multi_coords[i][1];
    
    major_marker.points.push_back(mark_pose);
  }

  visualization_msgs::Marker minor_marker;
  geometry_msgs::Vector3 sphere_scale;
  std_msgs::ColorRGBA sphere_color;
  sphere_color.r = 0;
  sphere_color.g = 1;
  sphere_color.b = 0;
  sphere_color.a = 1;
  sphere_scale.x = 0.07;
  sphere_scale.y = 0.07;
  sphere_scale.z = 0.07;
  minor_marker.header.frame_id = "map";
  minor_marker.header.stamp = ros::Time();
  minor_marker.id = 0;
  minor_marker.type = 7;
  minor_marker.action = 0;
  minor_marker.pose.orientation.w = 1.0;
  minor_marker.scale = sphere_scale;
  minor_marker.color = sphere_color;
  //minor_marker.lifetime = 0;

  std::cout << std::endl << "Minor points : \n";
  for(int i = 0; i < coords_for_nav_.size() ; i++){
    std::cout << "• [" << coords_for_nav_[i][0] << ", " << coords_for_nav_[i][1] << "]" << std::endl;

    geometry_msgs::Point mark_pose;
    mark_pose.x = coords_for_nav_[i][0];
    mark_pose.y = coords_for_nav_[i][1];
    
    minor_marker.points.push_back(mark_pose);
  }
  major_marker_pub_.publish(major_marker);
  minor_marker_pub_.publish(minor_marker);

}

void MultiPointNavigationHandler::showCurrentGoal(int point_index){
  visualization_msgs::Marker current_goal_marker;
  geometry_msgs::Vector3 cube_scale;
  std_msgs::ColorRGBA cube_color;
  
  cube_color.r = 0;
  cube_color.g = 0;
  cube_color.b = 1;
  cube_color.a = 1;
  cube_scale.x = 0.1;
  cube_scale.y = 0.1;
  cube_scale.z = 0.1;

  current_goal_marker.header.frame_id = "map";
  current_goal_marker.header.stamp = ros::Time();
  current_goal_marker.id = 0;
  current_goal_marker.type = 6;
  current_goal_marker.action = 0;
  current_goal_marker.pose.orientation.w = 1.0;
  current_goal_marker.scale = cube_scale;
  current_goal_marker.color = cube_color;

  geometry_msgs::Point mark_pose;
  mark_pose.x = coords_for_nav_[point_index][0];
  mark_pose.y = coords_for_nav_[point_index][1];
  
  current_goal_marker.points.push_back(mark_pose);
  current_marker_pub_.publish(current_goal_marker);
}

void MultiPointNavigationHandler::robotPoseCB(const geometry_msgs::Pose::ConstPtr& msg){
  if (task_active_) { robot_pose_ = *msg; }
}

bool MultiPointNavigationHandler::navToPoint(std::vector<float> instance_point){
  // if robot pose not within tolerance, point towards it 
  while((std::abs(robot_pose_.position.x - instance_point[0]) > p_goal_tolerance_x_) || (std::abs(robot_pose_.position.y - instance_point[1]) > p_goal_tolerance_y_)){
    // Get angle of robot with instance point goal
    float angle_to_point = std::atan2((instance_point[1]-robot_pose_.position.y),(instance_point[0]-robot_pose_.position.x));
    
    // Get robot orientation theta
    tf::Quaternion q(
      robot_pose_.orientation.x,
      robot_pose_.orientation.y,
      robot_pose_.orientation.z,
      robot_pose_.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, theta;
    m.getRPY(roll, pitch, theta);

    geometry_msgs::Twist to_cmd_vel;
    float dtheta = angle_to_point - theta;

    if(dtheta>M_PI){dtheta = dtheta-(2*M_PI);}
    if(dtheta<-M_PI){dtheta = dtheta+(2*M_PI);}

    // Check obstruction
    if(!obstructed_){
      if(std::abs(dtheta) > 0.1){
        to_cmd_vel.linear.x = 0.0;
      }
      else{
        to_cmd_vel.linear.x = p_linear_vel_;
      }

      to_cmd_vel.angular.z = pidFn(dtheta,0);
    }
    else{
      ROS_WARN("[%s] Robot Obstructed", name_.c_str());
      to_cmd_vel.linear.x = 0.0;
      to_cmd_vel.angular.z = 0.0;
    }

    

    // Publish cmd_vel
    cmd_vel_pub_.publish(to_cmd_vel);
  }

  geometry_msgs::Twist stop;
  cmd_vel_pub_.publish(stop);

  if((std::abs(robot_pose_.position.x - instance_point[0]) <= p_goal_tolerance_x_) && (std::abs(robot_pose_.position.y - instance_point[1]) <= p_goal_tolerance_y_)){
    // Successful navigation
    return true;
  }
  return false;
}

float MultiPointNavigationHandler::pidFn(float dtheta, float set_point){
  static float prev_value = 0;
  static float i_err = 0;

  float error = set_point - dtheta; 
  float pTerm = kp_ * error; 

  static float iTerm = 0;
  iTerm += ki_ * error; 

  float dTerm = kd_ * (dtheta - prev_value); 
  prev_value = dtheta;

  float return_val = pTerm + dTerm;

  //std::cout<<"before: p: "<<pTerm<<", i: " <<iTerm<< ", d: "<< dTerm<<", RV: " << return_val << std::endl;

  if(return_val > p_angular_vel_){
    return_val = p_angular_vel_;
  }
  else if(return_val < -p_angular_vel_){
    return_val = -p_angular_vel_;
  }

  //std::cout<<"after val : "<< return_val << std::endl;
  
  return return_val;
}

void MultiPointNavigationHandler::cancelTask()
{
  geometry_msgs::Twist stop;
  cmd_vel_pub_.publish(stop);

  task_cancelled_ = true;
  task_parsed_ = true;
  task_active_ = false;
  task_paused_ = false;
}

void MultiPointNavigationHandler::obstacleCB(const std_msgs::Bool::ConstPtr& obst_msg){
  obstructed_ = obst_msg->data;
}
}  // namespace task_supervisor
