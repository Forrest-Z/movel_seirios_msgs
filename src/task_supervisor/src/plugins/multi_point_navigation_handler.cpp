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
  -> unsigned int for index if needed
  -> documentation
  -> fix costmap_common_params overwriting
  -> confirm format for current goal publish
  -> if name changes, change name of srv file too
  
  V3
  -> acceleration config
  -> check dublin spline library
  -> dynamic reconfigure
  -> euclidean distance tolerance
  -> reverse/going back for linear
  */

  if (!loadParams()) {
    ROS_FATAL("[%s] Error during parameter loading. Shutting down.", name_.c_str());
    return false;
  }

  // Check minimum obstacle timeout
  if(p_obstruction_timeout_ < min_obst_timeout_){
    ROS_WARN("[%s] Obstruction timeout too low, resetting to minimum %f sec", name_.c_str(), min_obst_timeout_);
    p_obstruction_timeout_ = min_obst_timeout_;
  }
  // Obstacle checking frequency
  if(p_obst_check_freq_ > 0.5 && p_obst_check_freq_ < 10.0){
    obst_check_interval_ = 1/p_obst_check_freq_;
  }
  // Obstacle look ahead points from distance
  if(int(p_look_ahead_dist_/p_point_gen_dist_) > look_ahead_points_){
    look_ahead_points_ = int(p_look_ahead_dist_/p_point_gen_dist_);
  }
  // Angular tolerance
  if(p_angular_tolerance_ > angular_tolerance_){
    angular_tolerance_ = p_angular_tolerance_;
  }
  
  path_visualize_pub_ = nh_handler_.advertise<visualization_msgs::MarkerArray>("path", 10);
  robot_pose_sub_ = nh_handler_.subscribe("/pose", 1, &MultiPointNavigationHandler::robotPoseCB, this);
  cmd_vel_pub_ = nh_handler_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/autonomous", 1);
  current_goal_pub_ = nh_handler_.advertise<movel_seirios_msgs::MultipointProgress>("current_goal", 1);
  path_srv_ = nh_handler_.advertiseService("generate_path", &MultiPointNavigationHandler::pathServiceCb, this);
  

  obstructed_ = true;

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
  ROS_WARN("[%s] Loading of plugin parameters by ros_utils has not been implemented. Loading directly from Parameter "
           "Server instead.",
           name_.c_str());

  if (!load_param_util("points_distance", p_point_gen_dist_)){return false;}
  if (!load_param_util("look_ahead_distance", p_look_ahead_dist_)){return false;}
  if (!load_param_util("obst_check_freq", p_obst_check_freq_)){return false;}
  if (!load_param_util("goal_tolerance_x", p_goal_tolerance_x_)){return false;}
  if (!load_param_util("goal_tolerance_y", p_goal_tolerance_y_)){return false;}
  if (!load_param_util("angular_tolerance", p_angular_tolerance_)){return false;}
  if (!load_param_util("spline_enable", p_spline_enable_)){return false;}
  if (!load_param_util("obstacle_timeout", p_obstruction_timeout_)){return false;}
  if (!load_param_util("kp", p_kp_)){return false;}
  if (!load_param_util("ki", p_ki_)){return false;}
  if (!load_param_util("kd", p_kd_)){return false;}

  return true;
}

ReturnCode MultiPointNavigationHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message){
  task_cancelled_ = false;
  task_active_ = true;
  task_parsed_ = false;
  isHealthy_ = true;
  start_ = ros::Time::now();

  ROS_INFO("[%s] Task payload %s", name_.c_str(), task.payload.c_str());
  json payload = json::parse(task.payload);

  if (payload.find("path") != payload.end()){
    
    // Get received coordinates
    std::vector<std::vector<float>> rcvd_coords;
    for(auto& elem : payload["path"]){
      std::vector<float> coord_instance;
      coord_instance.push_back(elem["position"]["x"].get<float>());
      coord_instance.push_back(elem["position"]["y"].get<float>());
      rcvd_coords.push_back(coord_instance);
    }
    
    // Set task velocities
    if(task.angular_velocity > min_angular_vel_ && task.angular_velocity < max_angular_vel_){angular_vel_ = task.angular_velocity;}
    else{
      angular_vel_ = min_angular_vel_;
      ROS_WARN("[%s] Angular velocity out of bounds, setting default %f", name_.c_str(), angular_vel_);
    }
    if(task.linear_velocity > min_linear_vel_ && task.linear_velocity < max_linear_vel_){linear_vel_ = task.linear_velocity;}
    else{
      linear_vel_ = min_linear_vel_;
      ROS_WARN("[%s] Linear velocity out of bounds, setting default %f", name_.c_str(), linear_vel_);
    }
    // If robot is already near starting major point
    if(payload["at_start_point"].get<bool>()){
      at_start_point_ = payload["at_start_point"].get<bool>();
    }

    coords_for_nav_.clear();

    // Local vector (made to accomodate path generation service)
    std::vector<std::vector<float>> coords_for_nav;

    costmap_ptr_ = std::make_shared<costmap_2d::Costmap2DROS>("multi_point_map", tf_buffer_);
    
    // Generate all minor points
    if(pointsGen(rcvd_coords, coords_for_nav, true)){
      // Store coords in global
      coords_for_nav_ = coords_for_nav;
      if(coords_for_nav_.size()>0){
        ROS_INFO("[%s] Starting Multi-point navigation across %ld generated points", name_.c_str(), coords_for_nav_.size());
        // Loop through generated points
        for(int i = 0; i < coords_for_nav_.size(); i++){
          // Visualize on rviz
          visualizePath(i, false);

          //Publish current goal (mainly for FMS)
          publishCurrentGoal(i);

          // Call nav for instance point
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
        setTaskResult(true);
      }
      else{
        setMessage("Navigational coordinates vector empty");
        error_message = message_;
        setTaskResult(false);
      }
    }
    else{
      setMessage("Major points too close");
      error_message = message_;
      setTaskResult(false);
    }
  }
  else{
    setMessage("Malformed payload");
    error_message = message_;
    setTaskResult(false);
  }
  // Clean rviz topic
  visualizePath(0, true);
  return code_;
}

/// Generating Path

bool MultiPointNavigationHandler::pointsGen(std::vector<std::vector<float>> rcvd_multi_coords, std::vector<std::vector<float>>& coords_for_nav, bool for_nav){
  // Time
  ros::Time starttime=ros::Time::now();

  std::vector<std::vector<float>> coords_for_spline;
  std::vector<int> points_to_spline;

  // Only if generating points for task (not service)
  if(for_nav){
    if(at_start_point_){
      // Robot already near starting point, can remove starting point from navigation
      ROS_INFO("[%s] Robot already near starting point (%.2f,%.2f), ignoring it", name_.c_str(), rcvd_multi_coords[0][0], rcvd_multi_coords[0][1]);
      rcvd_multi_coords.erase(rcvd_multi_coords.begin());
    }

    // Add current robot pose to the front of the major points list
    boost::shared_ptr<geometry_msgs::Pose const> shared_current_pose;
    shared_current_pose = ros::topic::waitForMessage<geometry_msgs::Pose>("/pose",ros::Duration(2.0));
    if(shared_current_pose != NULL){
      std::vector<float> robot_pose_vec = {float(shared_current_pose->position.x), float(shared_current_pose->position.y)};
      rcvd_multi_coords.insert(rcvd_multi_coords.begin(), robot_pose_vec);
    }
    else{
      ROS_ERROR("[%s] Robot pose unavailable", name_.c_str());
      return false;
    }
  }

  // Will track indices of major points in the collection of all coords
  std::vector<int> major_indices {0};

  // Loop through major points to generate minor (breadcrumb) points
  for(int i = 0; i < rcvd_multi_coords.size()-1; i++){

    // Check if 2 major points are the same
    if(rcvd_multi_coords[i][0] == rcvd_multi_coords[i+1][0] && rcvd_multi_coords[i][1] == rcvd_multi_coords[i+1][1]){
      ROS_ERROR("[%s] 2 Major points with same coordinates (%.2f, %.2f)", name_.c_str(), rcvd_multi_coords[i][0], rcvd_multi_coords[i][1]);
      return false;
    }

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
          generated_min_point.push_back(rcvd_multi_coords[i][0] - ((p_point_gen_dist_*j)*(sqrt(1/(1+pow(slope,2))))));
        }
        else{
          generated_min_point.push_back(rcvd_multi_coords[i][0] + ((p_point_gen_dist_*j)*(sqrt(1/(1+pow(slope,2))))));
        }
        if(((rcvd_multi_coords[i][1] > rcvd_multi_coords[i+1][1]) && slope > 0) || ((rcvd_multi_coords[i][1] < rcvd_multi_coords[i+1][1]) && slope < 0)){
          generated_min_point.push_back(rcvd_multi_coords[i][1] - ((p_point_gen_dist_*j*slope)*(sqrt(1/(1+pow(slope,2))))));
        }
        else if(((rcvd_multi_coords[i][1] < rcvd_multi_coords[i+1][1]) && slope > 0) || ((rcvd_multi_coords[i][1] > rcvd_multi_coords[i+1][1]) && slope < 0)){
          generated_min_point.push_back(rcvd_multi_coords[i][1] + ((p_point_gen_dist_*j*slope)*(sqrt(1/(1+pow(slope,2))))));
        }
        else{
          generated_min_point.push_back(rcvd_multi_coords[i][1]);
        }

        coords_for_spline.push_back(generated_min_point);
        major_indices.back() = major_indices.back() + 1;
      }
    }
    else{
      for(int j = 0; j < int(num_of_points); j++){
        std::vector<float> generated_min_point;
        
        generated_min_point.push_back(rcvd_multi_coords[i][0]);
        
        if(rcvd_multi_coords[i][1] > rcvd_multi_coords[i+1][1]){
          generated_min_point.push_back(rcvd_multi_coords[i][1] - (p_point_gen_dist_*j));
        }
        else if(rcvd_multi_coords[i][1] < rcvd_multi_coords[i+1][1]){
          generated_min_point.push_back(rcvd_multi_coords[i][1] + (p_point_gen_dist_*j));
        }
        else{
          generated_min_point.push_back(rcvd_multi_coords[i][1]);
        }
        coords_for_spline.push_back(generated_min_point);
        major_indices.back() = major_indices.back() + 1;
      }
    }
  }
  coords_for_spline.push_back(rcvd_multi_coords.back());

  // Check if spline/smoothening enabled
  if(p_spline_enable_){
    if(rcvd_multi_coords.size()>2 && coords_for_spline.size()>0 && getPointsToSpline(rcvd_multi_coords, major_indices, points_to_spline)){
      // Spline/smoothen points
      splinePoints(coords_for_spline, points_to_spline, coords_for_nav);
    }
    else{
      // No spline
      coords_for_nav = coords_for_spline;
    }
  }
  else{
    // No spline
    coords_for_nav = coords_for_spline;
  }

  // Time
  ros::Time endtime=ros::Time::now();
  ros::Duration time_taken=endtime-starttime;

  if(for_nav){
    // Only for visualization
    rcvd_multi_coords_ = rcvd_multi_coords;
    major_indices_ = major_indices;

    ROS_INFO("[%s] Info :\n Major points - %ld,\n Total nav points - %ld,\n Spline enable - %d,\n Obstacle check interval - %0.2f s,\n Obstacle timeout - %0.2f s,\n Look ahead points - %d,\n Gen. time - %f", 
          name_.c_str(), rcvd_multi_coords.size(), coords_for_nav.size(), p_spline_enable_, obst_check_interval_, p_obstruction_timeout_, look_ahead_points_, time_taken.toSec());

    // Print generated nav points
    printGeneratedPath(rcvd_multi_coords);
  }

  return true;
}

void MultiPointNavigationHandler::splinePoints(std::vector<std::vector<float>>& coords_for_spline ,std::vector<int> points_to_spline, std::vector<std::vector<float>>& coords_for_nav){
  coords_for_nav.clear();
  
  // Check if points_to_spline is not empty
  if(points_to_spline.size() > 0){

    int index_pointer = 0;
    for(int i = 0; i < points_to_spline.size(); i++){
      // insert into coords_for_nav, upto the indice_to_smoothen - bypass_degree
      // make the smooth points and push into coords_for_nav
      // skip coords_for_smooth indices by (bypass_degree*2)-1
      // next loop should start from new index
      // push the last points

      for(int j = index_pointer; j <= (points_to_spline[i]-bypass_degree_); j++){
        coords_for_nav.push_back(coords_for_spline[j]);
        index_pointer++;
      }
      
      // Declare Points
      co_ord_pair pCminus3 = std::make_pair(coords_for_spline[points_to_spline[i]-3][0], coords_for_spline[points_to_spline[i]-3][1]);
      co_ord_pair pCminus2 = std::make_pair(coords_for_spline[points_to_spline[i]-2][0], coords_for_spline[points_to_spline[i]-2][1]);
      co_ord_pair pCminus1 = std::make_pair(coords_for_spline[points_to_spline[i]-1][0], coords_for_spline[points_to_spline[i]-1][1]);
      co_ord_pair pC = std::make_pair(coords_for_spline[points_to_spline[i]][0], coords_for_spline[points_to_spline[i]][1]);
      co_ord_pair pCplus1 = std::make_pair(coords_for_spline[points_to_spline[i]+1][0], coords_for_spline[points_to_spline[i]+1][1]);
      co_ord_pair pCplus2 = std::make_pair(coords_for_spline[points_to_spline[i]+2][0], coords_for_spline[points_to_spline[i]+2][1]);
      co_ord_pair pCplus3 = std::make_pair(coords_for_spline[points_to_spline[i]+3][0], coords_for_spline[points_to_spline[i]+3][1]);

      // For 1st spline path point
      coords_for_nav.push_back(intersectPoint(pCminus3, midPoint(pC,pCplus1), midPoint(pCminus2,pCminus3) , pCplus1));

      // For 2nd spline path point
      coords_for_nav.push_back(intersectPoint(midPoint(pCminus2,pCminus3), pCplus1, pCminus2, midPoint(pCplus1,pCplus2)));

      // For 3rd spline path point
      coords_for_nav.push_back(intersectPoint(pCminus2, midPoint(pCplus1, pCplus2), midPoint(pCminus1, pCminus2), pCplus2));

      // For 4th spline path point
      coords_for_nav.push_back(intersectPoint(midPoint(pCminus1, pCminus2), pCplus2, pCminus1, midPoint(pCplus2, pCplus3)));

      // For 5th spline path point
      coords_for_nav.push_back(intersectPoint(pCminus1, midPoint(pCplus2, pCplus3), midPoint(pC, pCminus1), pCplus3));

      index_pointer = index_pointer + (2*bypass_degree_) - 1;
    }
    for(int i = index_pointer ; i < coords_for_spline.size(); i++){
      coords_for_nav.push_back(coords_for_spline[i]);
      index_pointer++;
    }
  }

  // If empty, do not spline
  else{
    ROS_WARN("[%s] Points to spline was empty, aborting spline", name_.c_str());
    coords_for_nav = coords_for_spline;
  }
}

bool MultiPointNavigationHandler::getPointsToSpline(std::vector<std::vector<float>> rcvd_multi_coords, std::vector<int> major_indices, std::vector<int>& points_to_spline){

  points_to_spline.clear();

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
        points_to_spline.push_back(major_indices[i]);
      }
    }
  }

  if(points_to_spline.size()==0){
    return false;
  }
  return true;
}

co_ord_pair MultiPointNavigationHandler::midPoint(co_ord_pair P1, co_ord_pair P2){
  co_ord_pair mid_point = std::make_pair((P1.first + P2.first)/2,(P1.second + P2.second)/2);
  return mid_point;
}

std::vector<float> MultiPointNavigationHandler::intersectPoint(co_ord_pair line1A, co_ord_pair line1B, co_ord_pair line2C, co_ord_pair line2D){
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

///////-----///////

/// Visualize, topics and service

void MultiPointNavigationHandler::visualizePath(int point_index, bool delete_all){
  int marker_action = 0;
  if(delete_all){
    point_index = 0;
    marker_action = 2;
  }
  visualization_msgs::MarkerArray marker_array;

  // markers[0] is to visualize Major points - Sphere
  visualization_msgs::Marker major_marker;
  geometry_msgs::Vector3 sphere_scale;
  std_msgs::ColorRGBA sphere_color;
  sphere_color.r = 1;
  sphere_color.g = 0;
  sphere_color.b = 0;
  sphere_color.a = 1;
  sphere_scale.x = 0.07;
  sphere_scale.y = 0.07;
  sphere_scale.z = 0.07;
  major_marker.header.frame_id = "map";
  major_marker.header.stamp = ros::Time();
  major_marker.id = 0;
  major_marker.type = 7;
  major_marker.action = marker_action;
  major_marker.pose.orientation.w = 1.0;
  major_marker.scale = sphere_scale;
  major_marker.color = sphere_color;

  for(int i = 0; i < rcvd_multi_coords_.size(); i++){
    if(point_index <= major_indices_[i]){
      geometry_msgs::Point mark_pose;
      mark_pose.x = rcvd_multi_coords_[i][0];
      mark_pose.y = rcvd_multi_coords_[i][1];
      major_marker.points.push_back(mark_pose);
    }
  }

  marker_array.markers.push_back(major_marker);

  // markers[1] is to visualize generated path - Line strip 
  visualization_msgs::Marker path_marker;
  geometry_msgs::Vector3 line_scale;
  std_msgs::ColorRGBA line_color;
  line_color.r = 0;
  line_color.g = 0;
  line_color.b = 0;
  line_color.a = 0.5;
  line_scale.x = 0.01;
  path_marker.header.frame_id = "map";
  path_marker.header.stamp = ros::Time();
  path_marker.id = 1;
  path_marker.type = 4;
  path_marker.action = marker_action;
  path_marker.pose.orientation.w = 1.0;
  path_marker.scale = line_scale;
  path_marker.color = line_color;

  for(int i = point_index; i < coords_for_nav_.size(); i++){
    geometry_msgs::Point mark_pose;
    mark_pose.x = coords_for_nav_[i][0];
    mark_pose.y = coords_for_nav_[i][1];
    path_marker.points.push_back(mark_pose);
  }
  if(path_marker.points.size()>1){
    marker_array.markers.push_back(path_marker);
  }
  else{
    path_marker.action = 2;
    marker_array.markers.push_back(path_marker);
  }

  // markers[2] is to visualize current goal point - Sphere
  visualization_msgs::Marker current_marker;
  sphere_color.r = 0;
  sphere_color.g = 0;
  sphere_color.b = 1;
  sphere_color.a = 1;
  sphere_scale.x = 0.08;
  sphere_scale.y = 0.08;
  sphere_scale.z = 0.08;
  current_marker.header.frame_id = "map";
  current_marker.header.stamp = ros::Time();
  current_marker.id = 2;
  current_marker.type = 7;
  current_marker.action = marker_action;
  current_marker.pose.orientation.w = 1.0;
  current_marker.scale = sphere_scale;
  current_marker.color = sphere_color;

  geometry_msgs::Point mark_pose;
  mark_pose.x = coords_for_nav_[point_index][0];
  mark_pose.y = coords_for_nav_[point_index][1];
  current_marker.points.push_back(mark_pose);
  marker_array.markers.push_back(current_marker);

  // markers[3] is to visualize obstacle check look ahead path - Line strip 
  visualization_msgs::Marker look_ahead_marker;
  line_color.r = 0;
  line_color.g = 1;
  line_color.b = 0;
  line_color.a = 1;
  line_scale.x = 0.02;
  look_ahead_marker.header.frame_id = "map";
  look_ahead_marker.header.stamp = ros::Time();
  look_ahead_marker.id = 3;
  look_ahead_marker.type = 4;
  look_ahead_marker.action = marker_action;
  look_ahead_marker.pose.orientation.w = 1.0;
  look_ahead_marker.scale = line_scale;
  look_ahead_marker.color = line_color;

  int max_i_count = look_ahead_points_;

  if(max_i_count > coords_for_nav_.size()-1){
    max_i_count = 1;
  }

  if(point_index > coords_for_nav_.size() - max_i_count){
    max_i_count = coords_for_nav_.size() - point_index;
  }

  for(int i = 0; i < max_i_count; i++){
    geometry_msgs::Point mark_pose;
    mark_pose.x = coords_for_nav_[point_index + i][0];
    mark_pose.y = coords_for_nav_[point_index + i][1];
    look_ahead_marker.points.push_back(mark_pose);
  }
  marker_array.markers.push_back(look_ahead_marker);

  path_visualize_pub_.publish(marker_array);
}

void MultiPointNavigationHandler::printGeneratedPath(std::vector<std::vector<float>> rcvd_multi_coords){
  // Only for debugging
  std::cout << std::endl << "Major points : \n";
  for(int i = 0; i < rcvd_multi_coords.size(); i++){
    std::cout << "⦿ [" << rcvd_multi_coords[i][0] << ", " << rcvd_multi_coords[i][1] << "]" << std::endl;
  }

  std::cout << std::endl << "Generated points : \n";
  for(int i = 0; i < coords_for_nav_.size() ; i++){
    std::cout << "• [" << coords_for_nav_[i][0] << ", " << coords_for_nav_[i][1] << "]" << std::endl;
  }
}

void MultiPointNavigationHandler::publishCurrentGoal(int nav_coords_index){
  if(nav_coords_index != 0){
    for(int i = 0; i < major_indices_.size(); i++){
      if(nav_coords_index <= major_indices_[i]){
        movel_seirios_msgs::MultipointProgress to_publish;

        to_publish.from_major_index = i - 1;
        to_publish.to_major_index = i;

        to_publish.from_minor_index = nav_coords_index - 1;
        to_publish.to_minor_index = nav_coords_index;

        to_publish.from_major_pose.position.x = rcvd_multi_coords_[i-1][0];
        to_publish.from_major_pose.position.y = rcvd_multi_coords_[i-1][1];

        to_publish.to_major_pose.position.x = rcvd_multi_coords_[i][0];
        to_publish.to_major_pose.position.y = rcvd_multi_coords_[i][1];

        for(int j = 0; j < rcvd_multi_coords_.size(); j++){
          geometry_msgs::Pose instance_pose;
          instance_pose.position.x = rcvd_multi_coords_[j][0];
          instance_pose.position.y = rcvd_multi_coords_[j][1];

          to_publish.major_points_path.push_back(instance_pose);
        }
        current_goal_pub_.publish(to_publish);
        break;
      }
    }
  }
}

bool MultiPointNavigationHandler::pathServiceCb(movel_seirios_msgs::MultipointPath::Request& req, movel_seirios_msgs::MultipointPath::Response& res){
  if(req.major_points.size() > 1){
    // Call points gen for visualization/UI
    std::vector<std::vector<float>> rcvd_srv_coords;
    for(auto& elem : req.major_points){
      std::vector<float> coord_instance;
      coord_instance.push_back(elem.position.x);
      coord_instance.push_back(elem.position.y);
      rcvd_srv_coords.push_back(coord_instance);
    }

    // Will store generated path, passed reference
    std::vector<std::vector<float>> coords_for_nav;

    if(pointsGen(rcvd_srv_coords, coords_for_nav, false)){
      for(auto& elem : coords_for_nav){
        geometry_msgs::Point coord_point;
        coord_point.x = elem[0];
        coord_point.y = elem[1];
        res.generated_path.push_back(coord_point);
      }
      res.result = "Successful";
    }
    else{
      res.result = "Failure in path generation";
    }
  }
  else{
    res.result = "Failure, not enough major points";
  }

  return true;
}

void MultiPointNavigationHandler::robotPoseCB(const geometry_msgs::Pose::ConstPtr& msg){
  if (task_active_) { robot_pose_ = *msg; }
}

///////-----///////

/// Navigation

bool MultiPointNavigationHandler::navToPoint(int instance_index){
  ros::Time obs_start_time;
  bool obs_timeout_started = false;
  obstructed_ == obstacleCheck(instance_index);
  ros::Time prev_check_time = ros::Time::now();

  // If robot pose not within tolerance, point towards it 
  while((std::abs(robot_pose_.position.x - coords_for_nav_[instance_index][0]) > p_goal_tolerance_x_) || (std::abs(robot_pose_.position.y - coords_for_nav_[instance_index][1]) > p_goal_tolerance_y_)){
    if(task_cancelled_){
      geometry_msgs::Twist stop_cmd;
      cmd_vel_pub_.publish(stop_cmd);
      ROS_ERROR("[%s] Published stop command", name_.c_str());
      return false;
    }

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
    if(ros::Time::now() - prev_check_time >= ros::Duration(obst_check_interval_)){
      //clearCostmapFn();
      obstructed_ = obstacleCheck(instance_index);
      prev_check_time = ros::Time::now();
      if(obstructed_){
        ROS_WARN("[%s] Robot Obstructed", name_.c_str());
      }
    }

    // Nav cmd velocity if not obstructed and not paused
    if(!obstructed_ && !isTaskPaused()){
      if(std::abs(dtheta) > angular_tolerance_){
        if((std::abs(dtheta) > M_PI - angular_tolerance_) && (std::abs(dtheta) < M_PI + angular_tolerance_) && !p_forward_only_){
          to_cmd_vel.linear.x = -linear_vel_;
          to_cmd_vel.angular.z = -pidFn(dtheta, 0);
          ROS_INFO("[%s] Reversing", name_.c_str());
        }
        else{
          to_cmd_vel.linear.x = 0.0;
          to_cmd_vel.angular.z = pidFn(dtheta,0);
        }
      }
      else{
        to_cmd_vel.linear.x = linear_vel_;
        to_cmd_vel.angular.z = pidFn(dtheta,0);
      }
    }
    else{
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
  float pTerm = p_kp_ * error; 

  static float iTerm = 0;
  iTerm += p_ki_ * error; 

  float dTerm = p_kd_ * (dtheta - prev_value); 
  prev_value = dtheta;

  float return_val = pTerm + dTerm;

  if(return_val > angular_vel_){
    return_val = angular_vel_;
  }
  else if(return_val < -angular_vel_){
    return_val = -angular_vel_;
  }
  
  return return_val;
}

bool MultiPointNavigationHandler::obstacleCheck(int nav_coords_index){
  //Check if planned points exist
  if(coords_for_nav_.size()==0){
    ROS_ERROR("[%s] Navigation Co-ords vector empty", name_.c_str());
    return true;
  }

  enum ObstructionType { LETHAL, INSCRIBED_INFLATED };
  ObstructionType obstruction_type = LETHAL;
  
  costmap_2d::Costmap2D* sync_costmap = costmap_ptr_->getCostmap();

  int max_i_count = look_ahead_points_;

  if(max_i_count > coords_for_nav_.size()-1){
    max_i_count = 1;
  }

  if(nav_coords_index > coords_for_nav_.size()-max_i_count){
    max_i_count = coords_for_nav_.size() - nav_coords_index;
  }

  // Check for obstacles on the points ahead
  for(int i = 0; i < max_i_count; i++){
    unsigned int mx, my;
    double wx, wy;
    
    wx = coords_for_nav_[nav_coords_index + i][0];
    wy = coords_for_nav_[nav_coords_index + i][1];
    
    if(sync_costmap->worldToMap(wx, wy, mx, my)){
      unsigned char cost_i = sync_costmap->getCost(mx, my);
      if(cost_i == costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
        ROS_ERROR("[%s] Found obstruction - Inscribed inflation", name_.c_str());
        obstruction_type = INSCRIBED_INFLATED;
        return true;
      }
      if(cost_i == costmap_2d::LETHAL_OBSTACLE){
        ROS_ERROR("[%s] Found obstruction - Lethal obstacle", name_.c_str());
        obstruction_type = LETHAL;
        return true;
      }
      if(cost_i == costmap_2d::NO_INFORMATION){
        ROS_ERROR("[%s] Found obstruction - No Information", name_.c_str());
        return true;
      }
    }
    // Out of bounds
    else{
      ROS_ERROR("[%s] Out of bounds", name_.c_str());
      obstruction_type = LETHAL;
      return true;
    }

    // Also check the middle co-ordinates between nav points
    if(!(nav_coords_index == 0 && i == 0)){
      unsigned int mx_mid, my_mid;
      double wx_mid, wy_mid;

      co_ord_pair nav_coord_1 = std::make_pair(coords_for_nav_[nav_coords_index + i - 1][0],coords_for_nav_[nav_coords_index + i -1][1]);
      co_ord_pair nav_coord_2 = std::make_pair(coords_for_nav_[nav_coords_index + i][0],coords_for_nav_[nav_coords_index + i][1]);

      co_ord_pair mid_nav_coord = midPoint(nav_coord_1, nav_coord_2);

      wx_mid = mid_nav_coord.first;
      wy_mid = mid_nav_coord.second;
      
      if(sync_costmap->worldToMap(wx_mid, wy_mid, mx_mid, my_mid)){
        unsigned char cost_i = sync_costmap->getCost(mx_mid, my_mid);
        if(cost_i == costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
          ROS_ERROR("[%s] Found obstruction - Inscribed inflation", name_.c_str());
          obstruction_type = INSCRIBED_INFLATED;
          return true;
        }
        if(cost_i == costmap_2d::LETHAL_OBSTACLE){
          ROS_ERROR("[%s] Found obstruction - Lethal obstacle", name_.c_str());
          obstruction_type = LETHAL;
          return true;
        }
        if(cost_i == costmap_2d::NO_INFORMATION){
          ROS_ERROR("[%s] Found obstruction - No Information", name_.c_str());
          return true;
        }
      }
      // Out of bounds
      else{
        ROS_ERROR("[%s] Out of bounds", name_.c_str());
        obstruction_type = LETHAL;
        return true;
      }
    }
  }
  // Only for debugging
  // ROS_INFO("[%s] Path is clear for index %d(out of %ld)", name_.c_str(), nav_coords_index,coords_for_nav_.size());
  return false;
}

///////-----///////

void MultiPointNavigationHandler::cancelTask(){
  geometry_msgs::Twist stop_cmd;
  cmd_vel_pub_.publish(stop_cmd);
  setTaskResult(false);

  task_cancelled_ = true;
  task_parsed_ = true;
  task_active_ = false;
  task_paused_ = false;
}

}  // namespace task_supervisor
