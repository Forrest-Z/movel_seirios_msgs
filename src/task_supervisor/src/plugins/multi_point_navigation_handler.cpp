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
  isHealthy_(true),
  tf_ear_(tf_buffer_)
{

}

bool MultiPointNavigationHandler::setupHandler(){

  /* TODO :
  unsigned int for index if needed
  correct bypass degree if needed
  */

  

  ROS_INFO("[%s] M.Point SETUP TEST 1 Reached", name_.c_str());
  if (!loadParams()) {
    ROS_FATAL("[%s] Error during parameter loading. Shutting down.", name_.c_str());
    return false;
  }

  if(p_obstruction_timeout_ < min_obst_timeout_){

    ROS_WARN("[%s] Obstruction timeout too low, resetting to minimum %f sec", name_.c_str(), min_obst_timeout_);
    p_obstruction_timeout_ = min_obst_timeout_;
  }

  
  major_marker_pub_ = nh_handler_.advertise<visualization_msgs::Marker>("/major_marker", 10);
  minor_marker_pub_ = nh_handler_.advertise<visualization_msgs::Marker>("/minor_marker", 10);
  smooth_marker_pub_ = nh_handler_.advertise<visualization_msgs::Marker>("/smooth_marker", 10);
  current_marker_pub_ = nh_handler_.advertise<visualization_msgs::Marker>("/current_marker", 10);
  robot_pose_sub_ = nh_handler_.subscribe("/pose", 1, &MultiPointNavigationHandler::robotPoseCB, this);
  cmd_vel_pub_ = nh_handler_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/autonomous", 1);
  obstacle_sub_ = nh_handler_.subscribe("/obst", 1, &MultiPointNavigationHandler::obstacleCB, this);
  sync_costmap_ptr_ = std::make_shared<costmap_2d::Costmap2DROS>("aux_sync_map", tf_buffer_);

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

  if (!load_param_util("points_distance", p_point_gen_dist_)){return false;}
  if (!load_param_util("point_goal_tolerance_x", p_goal_tolerance_x_)){return false;}
  if (!load_param_util("point_goal_tolerance_y", p_goal_tolerance_y_)){return false;}
  if (!load_param_util("angular_vel", p_angular_vel_)){return false;}
  if (!load_param_util("linear_vel", p_linear_vel_)){return false;}
  if (!load_param_util("spline_enable", p_spline_enable_)){return false;}
  if (!load_param_util("obstacle_timeout", p_obstruction_timeout_)){return false;}

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

  if (payload.find("path") != payload.end()){
    ROS_INFO("[%s] M.Point RUN TEST 1 Reached", name_.c_str());
    /*
    int total_points = payload["total_points"].get<int>();
    
    std::vector<std::vector<float>> rcvd_coords;
    for(int i = 0 ; i < total_points; i++){
      std::vector<float> coord_instance;
      coord_instance.push_back(payload["points"][i]["x"].get<float>());
      coord_instance.push_back(payload["points"][i]["y"].get<float>());
      rcvd_coords.push_back(coord_instance);
    }
    */
    std::vector<std::vector<float>> rcvd_coords;
    for(auto& elem : payload["path"]){
      std::vector<float> coord_instance;
      coord_instance.push_back(elem["position"]["x"].get<float>());
      coord_instance.push_back(elem["position"]["y"].get<float>());
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
        if(!navToPoint(i)){
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
  // Time
  ros::Time starttime=ros::Time::now();

  coords_for_spline_.clear();

  // Will track indices of major points in the collection of all coords
  std::vector<int> major_indices {0};

  // loop through major points to generate minor (breadcrumb) points
  for(int i = 0; i < rcvd_multi_coords.size()-1; i++){
    float slope;
    float maj_point_distance = std::sqrt(pow((rcvd_multi_coords[i+1][0] - rcvd_multi_coords[i][0]),2)+pow((rcvd_multi_coords[i+1][1] - rcvd_multi_coords[i][1]),2));
    float num_of_points = maj_point_distance/p_point_gen_dist_; 
    
    if((num_of_points - int(num_of_points))*p_point_gen_dist_ < 0.1){
      num_of_points--;
    }
    
    major_indices.push_back(major_indices.back());

    
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

        coords_for_spline_.push_back(generated_min_point);
        major_indices.back() = major_indices.back() + 1;
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
        coords_for_spline_.push_back(generated_min_point);
        major_indices.back() = major_indices.back() + 1;
      }
    }
  }
  coords_for_spline_.push_back(rcvd_multi_coords.back());

  // Check if spline/smoothening enabled
  if(p_spline_enable_){
    if(rcvd_multi_coords.size()>2 && coords_for_spline_.size()>0 && getPointsToSpline(rcvd_multi_coords,major_indices)){
      // Spline/smoothen points
      splinePoints();
    }
    else{
      // No spline
      coords_for_nav_ = coords_for_spline_;
    }
  }
  else{
    // No spline
    coords_for_nav_ = coords_for_spline_;
  }

  // Time
  ros::Time endtime=ros::Time::now();
  ros::Duration time_taken=endtime-starttime;

  ROS_INFO("[%s] Info : Major points - %ld , Total nav points - %ld , Spline enable - %d , Gen. time - %f", name_.c_str(), rcvd_multi_coords.size(), coords_for_nav_.size(), p_spline_enable_, time_taken.toSec());

  // Show generated nav points
  showAllPoints(rcvd_multi_coords);
}

void MultiPointNavigationHandler::splinePoints(){
  coords_for_nav_.clear();
  
  // TODO: Correct bypass_degree if needed
  
  // Check if points_to_spline_ is not empty
  if(points_to_spline_.size() > 0){

    int index_pointer = 0;
    for(int i = 0; i < points_to_spline_.size(); i++){
      // insert into coords_for_nav, upto the indice_to_smoothen - bypass_degree
      // make the smooth points and push into coords_for_nav
      // skip coords_for_smooth indices by (bypass_degree*2)-1
      // next loop should start from new index
      // push the last points

      for(int j = index_pointer; j <= (points_to_spline_[i]-bypass_degree_); j++){
        coords_for_nav_.push_back(coords_for_spline_[j]);
        index_pointer++;
      }
      
      // Declare Points
      coord_pair pCminus3 = std::make_pair(coords_for_spline_[points_to_spline_[i]-3][0], coords_for_spline_[points_to_spline_[i]-3][1]);
      coord_pair pCminus2 = std::make_pair(coords_for_spline_[points_to_spline_[i]-2][0], coords_for_spline_[points_to_spline_[i]-2][1]);
      coord_pair pCminus1 = std::make_pair(coords_for_spline_[points_to_spline_[i]-1][0], coords_for_spline_[points_to_spline_[i]-1][1]);
      coord_pair pC = std::make_pair(coords_for_spline_[points_to_spline_[i]][0], coords_for_spline_[points_to_spline_[i]][1]);
      coord_pair pCplus1 = std::make_pair(coords_for_spline_[points_to_spline_[i]+1][0], coords_for_spline_[points_to_spline_[i]+1][1]);
      coord_pair pCplus2 = std::make_pair(coords_for_spline_[points_to_spline_[i]+2][0], coords_for_spline_[points_to_spline_[i]+2][1]);
      coord_pair pCplus3 = std::make_pair(coords_for_spline_[points_to_spline_[i]+3][0], coords_for_spline_[points_to_spline_[i]+3][1]);

      // For 1st spline path point
      coords_for_nav_.push_back(intersectPoint(pCminus3, midPoint(pC,pCplus1), midPoint(pCminus2,pCminus3) , pCplus1));

      // For 2nd spline path point
      coords_for_nav_.push_back(intersectPoint(midPoint(pCminus2,pCminus3), pCplus1, pCminus2, midPoint(pCplus1,pCplus2)));

      // For 3rd spline path point
      coords_for_nav_.push_back(intersectPoint(pCminus2, midPoint(pCplus1, pCplus2), midPoint(pCminus1, pCminus2), pCplus2));

      // For 4th spline path point
      coords_for_nav_.push_back(intersectPoint(midPoint(pCminus1, pCminus2), pCplus2, pCminus1, midPoint(pCplus2, pCplus3)));

      // For 5th spline path point
      coords_for_nav_.push_back(intersectPoint(pCminus1, midPoint(pCplus2, pCplus3), midPoint(pC, pCminus1), pCplus3));

      index_pointer = index_pointer + (2*bypass_degree_) - 1;
    }
    for(int i = index_pointer ; i < coords_for_spline_.size(); i++){
      coords_for_nav_.push_back(coords_for_spline_[i]);
      index_pointer++;
    }
  }

  // If empty, do not spline
  else{
    ROS_WARN("[%s] Points to spline was empty, aborting spline", name_.c_str());
    coords_for_nav_ = coords_for_spline_;
  }
}

bool MultiPointNavigationHandler::getPointsToSpline(std::vector<std::vector<float>> rcvd_multi_coords, std::vector<int> major_indices){

  points_to_spline_.clear();

  for(int i = 1; i < major_indices.size()-1; i++){
    // Check if straight line
    bool slope1exists = false, slope2exists = false;
    float slope1, slope2;
    if((rcvd_multi_coords[i][0] - rcvd_multi_coords[i-1][0]) != 0){
      // Slope 1 exists
      slope1 = (rcvd_multi_coords[i][1] - rcvd_multi_coords[i-1][1])/(rcvd_multi_coords[i][0] - rcvd_multi_coords[i-1][0]);
      slope1exists = true;
    }
    if((rcvd_multi_coords[i+1][0] - rcvd_multi_coords[i][0]) != 0){
      // Slope 2 exists
      slope2 = (rcvd_multi_coords[i+1][1] - rcvd_multi_coords[i][1])/(rcvd_multi_coords[i+1][0] - rcvd_multi_coords[i][0]);
      slope2exists = true;
    }

    if((slope1exists != slope2exists) || (slope1exists && slope2exists && (slope1 != slope2))){
      // Check if enough points are there between major points, to accomodate bypass_degree (3)
      if(((major_indices[i]-major_indices[i-1]) >= 6) && ((major_indices[i+1]-major_indices[i]) >= 6)){
        points_to_spline_.push_back(major_indices[i]);
      }
    }
  }

  if(points_to_spline_.size()==0){
    return false;
  }
  return true;
}

coord_pair MultiPointNavigationHandler::midPoint(coord_pair P1, coord_pair P2){
  coord_pair mid_point = std::make_pair((P1.first + P2.first)/2,(P1.second + P2.second)/2);
  return mid_point;
}

std::vector<float> MultiPointNavigationHandler::intersectPoint(coord_pair line1A, coord_pair line1B, coord_pair line2C, coord_pair line2D){
  // Line 1 AB represented as a1x + b1y = c1
  float a1 = line1B.second - line1A.second;
  float b1 = line1A.first - line1B.first;
  float c1 = a1*line1A.first + b1*line1A.second;

  // Line 2 CD represented as a2x + b2y = c2
  float a2 = line2D.second - line2C.second;
  float b2 = line2C.first - line2D.first;
  float c2 = a2*line2C.first + b2*line2C.second;

  float determinant = a1*b2 - a2*b1;

  std::vector<float> return_vec;

  // Push X value of intersecting point
  return_vec.push_back((b2*c1 - b1*c2)/determinant);
  // Push Y value of intersecting point
  return_vec.push_back((a1*c2 - a2*c1)/determinant);

  return return_vec;
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
  sphere_color.r = 0.8;
  sphere_color.g = 0.65;
  sphere_color.b = 0;
  sphere_color.a = 1;
  sphere_scale.x = 0.05;
  sphere_scale.y = 0.05;
  sphere_scale.z = 0.05;
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
  for(int i = 0; i < coords_for_spline_.size() ; i++){
    std::cout << "• [" << coords_for_spline_[i][0] << ", " << coords_for_spline_[i][1] << "]" << std::endl;

    geometry_msgs::Point mark_pose;
    mark_pose.x = coords_for_spline_[i][0];
    mark_pose.y = coords_for_spline_[i][1];
    
    minor_marker.points.push_back(mark_pose);
  }

  visualization_msgs::Marker smooth_marker;
  sphere_color.r = 0.1;
  sphere_color.g = 0.5;
  sphere_color.b = 0;
  sphere_color.a = 1;
  sphere_scale.x = 0.07;
  sphere_scale.y = 0.07;
  sphere_scale.z = 0.07;
  smooth_marker.header.frame_id = "map";
  smooth_marker.header.stamp = ros::Time();
  smooth_marker.id = 0;
  smooth_marker.type = 7;
  smooth_marker.action = 0;
  smooth_marker.pose.orientation.w = 1.0;
  smooth_marker.scale = sphere_scale;
  smooth_marker.color = sphere_color;

  std::cout << std::endl << "Smooth points : \n";
  for(int i = 0; i < coords_for_nav_.size() ; i++){
    std::cout << "•~ [" << coords_for_nav_[i][0] << ", " << coords_for_nav_[i][1] << "]" << std::endl;

    geometry_msgs::Point mark_pose;
    mark_pose.x = coords_for_nav_[i][0];
    mark_pose.y = coords_for_nav_[i][1];
    
    smooth_marker.points.push_back(mark_pose);
  }

  major_marker_pub_.publish(major_marker);
  minor_marker_pub_.publish(minor_marker);
  smooth_marker_pub_.publish(smooth_marker);
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

bool MultiPointNavigationHandler::navToPoint(int instance_index){
  
  ros::Time obs_start_time;
  bool obs_timeout_started = false;
  obstructed_ == obstacleCheck(instance_index);
  ros::Time prev_check_time = ros::Time::now();

  // if robot pose not within tolerance, point towards it 
  while((std::abs(robot_pose_.position.x - coords_for_nav_[instance_index][0]) > p_goal_tolerance_x_) || (std::abs(robot_pose_.position.y - coords_for_nav_[instance_index][1]) > p_goal_tolerance_y_)){
    
    // Obstruction timeout check
    if(!obstructed_ && obs_timeout_started){
      obs_timeout_started = false;
    }
    if(obstructed_ && !obs_timeout_started){
      obs_timeout_started = true;
      obs_start_time = ros::Time::now();
    }
    if(obstructed_ && obs_timeout_started && (ros::Time::now() - obs_start_time > ros::Duration(p_obstruction_timeout_))){
      ROS_ERROR("[%s] Obstruction time out reached. Cancelling task", name_.c_str());
      return false;
    }
    // Get angle of robot with instance point goal
    float angle_to_point = std::atan2((coords_for_nav_[instance_index][1]-robot_pose_.position.y),(coords_for_nav_[instance_index][0]-robot_pose_.position.x));
    
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

    // Update obstruction status
    if(ros::Time::now() - prev_check_time >= ros::Duration(obst_check_freq_)){
      obstructed_ = obstacleCheck(instance_index);
      prev_check_time = ros::Time::now();
      if(obstructed_){
        ROS_WARN("[%s] Robot Obstructed", name_.c_str());
      }
    }

    // Nav cmd velocity if not obstructed
    if(!obstructed_){
      if(std::abs(dtheta) > 0.2){
        to_cmd_vel.linear.x = 0.0;
      }
      else{
        to_cmd_vel.linear.x = p_linear_vel_;
      }

      to_cmd_vel.angular.z = pidFn(dtheta,0);
    }
    else{
      // ROS_WARN("[%s] Robot Obstructed", name_.c_str());
      to_cmd_vel.linear.x = 0.0;
      to_cmd_vel.angular.z = 0.0;
    }

    // Publish cmd_vel
    cmd_vel_pub_.publish(to_cmd_vel);
  }

  geometry_msgs::Twist stop;
  cmd_vel_pub_.publish(stop);

  if((std::abs(robot_pose_.position.x - coords_for_nav_[instance_index][0]) <= p_goal_tolerance_x_) && (std::abs(robot_pose_.position.y - coords_for_nav_[instance_index][1]) <= p_goal_tolerance_y_)){
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

bool MultiPointNavigationHandler::obstacleCheck(int nav_coords_index){
  /*
  //Check if planned points exist
  if(coords_for_nav_.size()==0){
    ROS_ERROR("[%s] Navigation Co-ords vector empty", name_.c_str());
    return true;
  }

  enum ObstructionType { LETHAL, INSCRIBED_INFLATED };
  ObstructionType obstruction_type = LETHAL;
  
  costmap_2d::Costmap2D* sync_costmap = sync_costmap_ptr_->getCostmap();

  int max_i_count = 3;

  if(nav_coords_index > coords_for_nav_.size()-3){
    max_i_count = coords_for_nav_.size() - nav_coords_index;
  }

  for(int i = 0; i < max_i_count; i++){
    unsigned int mx, my;
    double wx = coords_for_nav_[i][0];
    double wy = coords_for_nav_[i][1];
    
    // check for obstacles
    // TODO : Fix worldToMap returning false
    if (sync_costmap->worldToMap(wx, wy, mx, my)){
      unsigned char cost_i = sync_costmap->getCost(mx, my);
      if (cost_i == costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        ROS_INFO("[%s] Found obstruction on C-space inscribed inflation", name_.c_str());
        obstruction_type = INSCRIBED_INFLATED;
        return true;
      }
      if (cost_i == costmap_2d::LETHAL_OBSTACLE) {
        ROS_WARN("[%s] Found obstruction on C-space lethal obstacle", name_.c_str());
        obstruction_type = LETHAL;
        return true;
      }
    }
    // out of bounds
    else {
      ROS_INFO("[%s] Out of bounds", name_.c_str());
      obstruction_type = LETHAL;
      return true;
    }
  }
  // only for testing
  ROS_INFO("[%s] Path is clear for index %d(out of %d)", name_.c_str(), nav_coords_index,coords_for_nav_.size());
  return false;
  */

  return false;
}

}  // namespace task_supervisor
