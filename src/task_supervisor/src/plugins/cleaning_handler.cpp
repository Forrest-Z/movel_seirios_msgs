#include <pluginlib/class_list_macros.h>
#include <task_supervisor/plugins/cleaning_handler.h>
#include <ros_utils/ros_utils.h>  //For loadParams function contents

// Includes for path_recall services
#include <path_recall/PathInfo.h>
#include <path_recall/PathName.h>
#include <path_recall/PathCheck.h>
#include <crop_map/crop_map.h>
#include <ipa_room_exploration_msgs/RoomExplorationClient.h>
#include <stdio.h>
#include <deque>


// For path_point_density reduction service 
#include <movel_seirios_msgs/StringTrigger.h>
#include <task_supervisor/json.hpp>
#include <nav_msgs/Path.h>
#include <movel_fms_utils/path_dist_utils.hpp>


using json = nlohmann::json;

PLUGINLIB_EXPORT_CLASS(task_supervisor::CleaningHandler, task_supervisor::TaskHandler);

namespace task_supervisor
{ 
// Path status received from path_load module
void CleaningHandler::onPathStatus(const std_msgs::BoolConstPtr& msg)
{
  path_load_ended_ = !msg->data;
}

void CleaningHandler::plannerResultCB(const nav_msgs::PathConstPtr& path)
{

  ROS_INFO("[%s] coverage path received, %lu", name_.c_str(), path->poses.size());
  bool valid_size = false;
  bool valid_length = false;

  if (path->poses.size() < 2)
  {
    message_ = "Path contains " + std::to_string(path->poses.size()) + " points, needs to contain atleast 2 points.";
    ROS_INFO("[%s] %s", name_.c_str(), message_.c_str());
    setTaskResult(false);
    path_planned_ = true;
    return;
  }
  else
    valid_size = true;

  double length = 0.0;
  for (int i = 0; i < path->poses.size() - 2; ++i)
  {
    length += sqrt(pow((path->poses[i + 1].pose.position.x - path->poses[i].pose.position.x), 2) +
                   pow((path->poses[i + 1].pose.position.y - path->poses[i].pose.position.y), 2));
  }

  if (length < (robot_radius_ * p_radius_multiplier_))
  {
    message_ = "Path length is " + std::to_string(length) + " , needs be atleast " +
               std::to_string(robot_radius_ * p_radius_multiplier_);
    ROS_INFO("[%s] %s", name_.c_str(), message_.c_str());
    setTaskResult(false);
    path_planned_ = true;
    return;
  }
  else
    valid_length = true;

  // if (valid_size and valid_length)
  path_planned_ = true;

  ROS_INFO("[%s] planning success %d", name_.c_str(), path_planned_);
}


bool CleaningHandler::parseArgs(std::string payload, arg_flags& flags)
{
  bool name_go = false;
  bool poly_go = false;
  YAML::Node config = YAML::Load(payload);
  if (config["name"])
  {
    // ROS_INFO("parsing name");
    std::string path_name = config["name"].as<std::string>();
    std::string path_full_path = p_yaml_path_ + path_name + ".yaml";
    FILE* file = fopen(path_full_path.c_str(), "r");
    if (file == NULL)
    {
      ROS_INFO("[%s] failed to open %s", name_.c_str(), path_full_path.c_str());
      name_go = false;
    }
    else
    {
      ROS_INFO("[%s] loading path %s", name_.c_str(), path_name.c_str());

      flags.run_now = true;
      flags.use_name = true;
      flags.name = path_name;
      flags.use_poly = false;
      run_immediately_ = true;
      return true;
    }
  }

  if (config["poly"])
  {
    // ROS_INFO("parsing poly");
    std::string poly_path = config["poly"].as<std::string>();
    FILE* file = fopen(poly_path.c_str(), "r");
    if (file == NULL)
    {
      ROS_INFO("[%s] cannot open polygon spec in %s", name_.c_str(), poly_path.c_str());
      poly_go = false;
    }
    else
    {
      path_to_polygon_txt_ = poly_path;
      ROS_INFO("[%s] saving new path in %s", name_.c_str(), poly_path.c_str());
      flags.use_name = false;
      flags.use_poly = true;
      poly_go = true;
    }

    if (config["bigmap"])
    {
      // ROS_INFO("parsing bigmap");
      std::string bigmap_path = config["bigmap"].as<std::string>();
      FILE* file = fopen(bigmap_path.c_str(), "r");

      if (file != NULL)
      {
        path_to_big_map_ = bigmap_path;
      }
      ROS_INFO("[%s] planning new path in map %s", name_.c_str(), path_to_big_map_.c_str());
    }
    else
    {
      ROS_ERROR_STREAM("Poly argument cannot be without BigMap argument");
      return false;
    }

    if (config["run"])
    {
      // ROS_INFO("parsing run");
      bool run = config["run"].as<bool>();
      if (run)
      {
        ROS_INFO("[%s] running the new path after planning", name_.c_str());
        flags.run_now = true;
        run_immediately_ = true;
      }
      else
      {
        ROS_INFO("[%s] just plan, do not run immediately", name_.c_str());
        flags.run_now = false;
        run_immediately_ = false;
      }
    }
  }

  // ROS_INFO("name_go %d poly_go %d", name_go, poly_go);
  return name_go || poly_go;
}
// ROS path reduction service call and functions 

bool CleaningHandler::pathDensityReductionCB(movel_seirios_msgs::StringTrigger::Request& req, movel_seirios_msgs::StringTrigger::Response& res)
{
ROS_INFO("Task payload %s", req.input.c_str());
  std::vector<geometry_msgs::Pose> path_in;
  std::vector<geometry_msgs::Pose> path_out; 
  json payload = json::parse(req.input);
  std::cout <<"\n" << payload["poses"].size()<<"\n";
  double d_min_;
  auto is_density_set = payload.contains("distance_between_points");
  if(!is_density_set){
   res.success = false;
   res.message = "please set a distance_between_points";
    return true;
  }

  d_min_ = payload["distance_between_points"];

  std::cout<<d_min_<<std::endl;
   for(int i =0 ; i < payload["poses"].size();i++){
    geometry_msgs::Pose waypoint;
   // std::cout<< payload["poses"][i]["position"]["x"]<<"\n";
    waypoint.position.x = payload["poses"][i]["position"]["x"].get<double>();
    waypoint.position.y = payload["poses"][i]["position"]["y"].get<double>();
    waypoint.position.z = payload["poses"][i]["position"]["z"].get<double>();
    waypoint.orientation.x = payload["poses"][i]["orientation"]["x"].get<double>();
    waypoint.orientation.y = payload["poses"][i]["orientation"]["y"].get<double>();
    waypoint.orientation.z = payload["poses"][i]["orientation"]["z"].get<double>();
    waypoint.orientation.w = payload["poses"][i]["orientation"]["w"].get<double>();
    path_in.push_back(waypoint);
   }
   
   //check if there is enough points to reduce if less than 2 then return false, too less points to reduce 
   if(path_in.size() < 2){ 
   res.success = false;
   res.message = "Too less points to reduce( points provided)";
   
   return true;
   }
 
   //start position will always be inside the new path 
   path_out.push_back(path_in[0]); 
   geometry_msgs::Pose last_pose = path_in[0];

   //checking of points in between start and end points. 
   for (int i = 1; i < path_in.size()-1; i++)
    {
      double dee = movel_fms_utils::eclideadDist(last_pose, path_in[i]);
      
      if (dee > d_min_)
      {
        // setting the pose that was pushed into the plan_out as the last pose so next comparision will be with this pose. 
        path_out.push_back(path_in[i]);
        last_pose = path_in[i];
      }
    }

   //End point will always be inside the new path 
    path_out.push_back(path_in[path_in.size()-1]);

  //if path have more than 2 points then check the 2nd last point distance with the last point's distance if it is
  //less than distance apart stated, 2nd last point will be removed.else 
   if(path_out.size() > 2){
       double dee = movel_fms_utils::eclideadDist(path_out[path_out.size()-2], path_out[path_out.size()-1]);

    if (dee < d_min_)
      {
        path_out.erase(path_out.begin()+(path_out.size()-2));
      }
   }else if(path_out.size() == 2){

   double dee = movel_fms_utils::eclideadDist(path_out[0], path_out[1]);

    if (dee < d_min_)
      {
              res.success = false;
              res.message = "The distance between the first to last point is less than distance stated to be apart,reduce distance";
              return true;
      }

   }

   json j2; 
   std::vector<json> arr1;
   
   // add array of poses in json
  for (int i = 0; i < path_out.size(); i++){

     arr1.push_back(movel_fms_utils::pose_to_json(path_out[i]));
  } 

   j2["poses"] = arr1;
   //std::cout<<j2<<std::endl;
   std::cout <<"path_in size:"<<path_in.size()<<"\n";
   std::cout <<"path_out size:"<<path_out.size()<<"\n";
   res.success = true;
   std::string reduced_path = j2.dump();  //This will change the json created into a string
   //std::cout<<j2.dump(4)<<std::endl;
   //std::cout<< reduced_path;
   res.message = reduced_path;               //path reduced;

  return true;
}
// end of rosservice call function

ReturnCode CleaningHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message)
{
  task_active_ = true;
  task_parsed_ = false;
  path_load_ended_ = false;
  start_ = ros::Time::now();
  // Load require params
  if (!loadParams())
  {
    setTaskResult(false);
    return code_;
  }

  ROS_INFO("[%s] payload: %s", name_.c_str(), task.payload.c_str());

  // Parse task payload, should be a text file of coordinates for polygon
  arg_flags flags;
  if (!parseArgs(task.payload, flags))
  {
    error_message = "[" + name_ + "] Payload parsing failed, check payload format";
    setTaskResult(false);
    return code_;
  }

  // Start all required launches
  if (flags.run_now)
  {
    startAllLaunch();

    // Check if all launches started
    if (!(path_recovery_id_ && path_load_id_ && planner_server_id_ && planner_client_id_ && 
          path_saver_id_))
    {
      stopAllLaunch();
      error_message = "[" + name_ + "] Failed to start required launch files";
      setTaskResult(false);
      return code_;
    }
  }
  else
  {
    planner_server_id_ = startLaunch("ipa_room_exploration", "room_exploration_action_server.launch", "");
    planner_client_id_ = startLaunch("ipa_room_exploration", "room_exploration_client.launch", "");    
  }

  // Subscribe to path_load's "start" topic. Start topic is gives state of path_load
  ros::Subscriber path_state_sub = nh_handler_.subscribe("/path_load/start", 1, &CleaningHandler::onPathStatus, this);
  health_check_pub_ = nh_handler_.advertise<movel_seirios_msgs::Reports>("/task_supervisor/health_report", 1);
  
  if (flags.use_poly)
  {

    if (!(getPath() && isTaskActive()))
    {
      error_message = "[" + name_ + "] Path planning failed. " + message_;
      stopAllLaunch();
      setTaskResult(false);
      return code_;
    }

    ROS_INFO("[%s] get path OK", name_.c_str());
    if (!flags.run_now)
    {
      ROS_INFO("[%s] not running path immediately, teardown handler", name_.c_str());
      stopLaunch(planner_server_id_);
      stopLaunch(planner_client_id_);
      planner_server_id_ = planner_client_id_ = 0;
      setTaskResult(true);
      return code_;
    }

    std::string sep = "/";
    std::string map_name = path_to_big_map_;
    std::string ext = ".pgm";
    std::string mname;
    size_t i = map_name.find_last_of(sep, map_name.length());
    if (i != std::string::npos)
    {
      ROS_WARN_STREAM("EDITING");
      mname = map_name.substr(i + 1, map_name.length() - i - ext.length() - 1);
    }

    // rename saved path
    std::string new_path_name;
    std::string new_path_name_stem;
    std::size_t idx_ext = path_to_polygon_txt_.find(".txt");
    if (idx_ext != std::string::npos)
    {
      new_path_name = path_to_polygon_txt_.substr(0, idx_ext);
      std::size_t idx_path = path_to_polygon_txt_.find_last_of("/");
      std::size_t stem_len = idx_ext - idx_path - 1;
      new_path_name_stem = path_to_polygon_txt_.substr(idx_path + 1, stem_len);

      new_path_name += ".yaml";
      std::string old_path_name = p_yaml_path_ + p_planned_path_name_ + ".yaml";
      // std::rename(old_path_name.c_str(), new_path_name.c_str());

      YAML::Node path_yaml = YAML::LoadFile(old_path_name);
      for (auto it = path_yaml.begin(); it != path_yaml.end(); ++it)
      {
        if (it->first.as<std::string>() == p_planned_path_name_)
        {
          it->first = mname + ">" + new_path_name_stem;
          break;
        }
      }

      std::ofstream fout(new_path_name);
      fout << path_yaml;
      fout.close();

      flags.name = new_path_name_stem;
      error_message = "[" + name_ + "] Name of path saved: " + new_path_name_stem;
    }
  }

  // Get path, send planned path to path_load, provided not cancelled
  if (flags.use_name || (flags.use_poly && flags.run_now))
  {
    if (!(isTaskActive() && startPath(flags.name)))
    {
      ROS_ERROR("%s", message_.c_str());
      error_message = "[" + name_ + "] Path retrieval or starting failed";
      stopAllLaunch();
      setTaskResult(false);
      return code_;
    }
  }
  else
  {
    path_load_ended_ = true;
  }

  // Wait until path completion or error
  ros::Rate r(p_loop_rate_);
  while (ros::ok())
  {
    // Cancellation is called
    if (!isTaskActive())
    {
      ROS_INFO("[%s] Task cancelled, running required cleanup tasks", name_.c_str());
      // Send cancel to path_load, and wait until cancelled
      cancelPath();
      while (!path_load_ended_)
        ;  // Set on cancel by path load
      path_load_ended_ = false;

      // Close all launches
      stopAllLaunch();
      error_message = "[" + name_ + "] Task cancelled";
      setTaskResult(false);
      return code_;
    }

    // Path completed, maybe cancelled
    if (path_load_ended_)
    {
      ROS_INFO("[%s] Path completed", name_.c_str());

      // TODO Implement path_recovery function
      stopAllLaunch();
      setTaskResult(true);
      return code_;
    }
    r.sleep();
  }

  error_message = "[" + name_ + "] Task failed, ROS was shutdown";
  setTaskResult(false);
  return code_;
}

bool CleaningHandler::loadParams()
{
  ros_utils::ParamLoader param_loader(nh_handler_);

  param_loader.get_optional("loop_rate", p_loop_rate_, 5.0);
  param_loader.get_optional("planning_timeout", p_planning_timeout_, 10.0);  // Set to 0 to disable
  //	param_loader.get_optional("pose_distance_threshold", p_pose_distance_threshold_, 0.5); //What unit?
  param_loader.get_required("start_distance_thresh", p_start_distance_thresh_);
  param_loader.get_required("yaml_path", p_yaml_path_);
  //	param_loader.get_required("polygon_path", path_to_polygon_txt_);
  param_loader.get_required("cropped_map_path", path_to_cropped_map_);
  param_loader.get_required("cropped_coordinates_path", path_to_coordinates_txt_);
  // param_loader.get_required("move_base_launch", p_move_base_launch_);
  // param_loader.get_required("move_base_package", p_move_base_package_);
  param_loader.get_required("planned_path_name", p_planned_path_name_);
  param_loader.get_required("radius_multiplier", p_radius_multiplier_);

  return param_loader.params_valid();
}

bool CleaningHandler::setupHandler()
{
  if (!loadParams())
    return false;
  else
  start_path_density_reduction_ = nh_handler_.advertiseService("reduce_path_density", &CleaningHandler::pathDensityReductionCB, this);
    return true;
}

void CleaningHandler::cropMap()
{
  ROS_ERROR("Inside function cropMap()");
  // Create cropping object
  CropMap crop_map;
  crop_map.setCoordinatesSavePath(path_to_coordinates_txt_);
  crop_map.setCroppedSavePath(path_to_cropped_map_);

  // read map origin and resolution from big map
  // 1. deduce big map yaml
  float map_origin_x = -50.0;
  float map_origin_y = -50.0;
  float map_res = 0.05;
  std::size_t ext = path_to_big_map_.find(".pgm");
  if (ext != std::string::npos)
  {
    std::string big_map_config = path_to_big_map_.substr(0, ext);
    big_map_config += ".yaml";
    // ROS_INFO("[%s] big map config yaml %s", name_.c_str(), big_map_config.c_str());

    // 2. read relevant parameters, populate CropMap object
    YAML::Node config = YAML::LoadFile(big_map_config);
    if (config["resolution"])
      map_res = config["resolution"].as<float>();

    if (config["origin"])
    {
      map_origin_x = config["origin"][0].as<float>();
      map_origin_y = config["origin"][1].as<float>();
    }
  }
  // ROS_INFO("map origin (%f, %f)", map_origin_x, map_origin_y);
  // ROS_INFO("map resolution %f", map_res);

  crop_map.setMapOrigin(map_origin_x, map_origin_y);
  crop_map.setResolution(map_res);

  ROS_INFO("cropping map %s", path_to_big_map_.c_str());
  bool cropping_result = crop_map.cropMap(path_to_big_map_, path_to_polygon_txt_);
  if (cropping_result)
  {
    double area = crop_map.getContourArea();
    double sqm_area = area * map_res * map_res;
    if (!ros::param::get("/room_exploration_client/robot_radius", robot_radius_)) {
    //   message_ = "Unable to obtain robot's radius for crop validation calculation.";
    //   return false;
      ROS_ERROR("[%s] Unable to obtain robot's radius for crop validation calculation.", name_.c_str());
      crop_fail_ = true;
    }
    double area_threshold = M_PI * robot_radius_ * robot_radius_ * p_radius_multiplier_;
    ROS_INFO("area_threshold is %f", area_threshold);
    ROS_INFO("sqm area is %f", sqm_area);
    if (sqm_area < area_threshold)
    {
      // message_ = "Cropped area is " + std::to_string(sqm_area) + ", which is less than area threshold of " +
      //            std::to_string(area_threshold);
      ROS_ERROR("[%s] Cropped area smaller than threshold, cropping failed", name_.c_str());
      crop_fail_ = true;
    }
  }
  else 
  {
    crop_fail_ = true;
  }
}

bool CleaningHandler::getPath()
{
  ROS_ERROR_STREAM("Entered Getting Path");

  cropMap();
  if(crop_fail_){
    message_ = "Unable to crop map";
    return false;
  }
  // Subscribe to know if planning is complete
  ros::Subscriber planner_result_sub =
    nh_handler_.subscribe("/room_exploration_server/coverage_path", 1, &CleaningHandler::plannerResultCB, this);

  // Call room exploration client service
  ros::ServiceClient planner_srv = 
    nh_handler_.serviceClient<ipa_room_exploration_msgs::RoomExplorationClient>("/room_exploration_client/start");
  ipa_room_exploration_msgs::RoomExplorationClient planner;
  planner.request.path_to_coordinates_txt = path_to_coordinates_txt_;
  planner.request.path_to_cropped_map = path_to_cropped_map_;

  planner_srv.waitForExistence(ros::Duration(5.0));
  // Crop map using given polygon
  // if (!cropMap())
  //   return false;
  if (!planner_srv.call(planner))
  {
    message_ = "[" + name_ + "] Unable to call planner service /room_exploration_client/start";
    return false;
  }
  if (!planner.response.success)
  {
    message_ = "[" + name_ + "] Planner failed";
    return false;
  }
  ROS_INFO("[%s] Path Planning Started!", name_.c_str());
  // Wait for planning to complete, timeout applied
  ros::Time startTime = ros::Time::now();
  ros::Rate r(p_loop_rate_);
  while (!path_planned_)
  {
    if (ros::Time::now() - startTime > ros::Duration(p_planning_timeout_))
    {
      ROS_ERROR("[%s] Planning timed out, failed to plan path for cleaning", name_.c_str());
      message_ = "[" + name_ + "] Path planning timed out, unable to get path";
      return false;
    }
    ros::spinOnce();
    r.sleep();
    // ROS_INFO("wait for plan %d", path_planned_);
  }

  path_planned_ = false;
  return true;
}

bool CleaningHandler::checkPathWithinThresh(std::string path_name)
{
  // Check if path is within starting distance threshold
  ros::ServiceClient path_load_check = nh_handler_.serviceClient<path_recall::PathCheck>("/path_load/check");
  path_recall::PathCheck path_check_thresh;
  path_check_thresh.request.distance_thresh = p_start_distance_thresh_;
  // path_check_thresh.request.name = p_planned_path_name_;
  path_check_thresh.request.name = path_name;

  path_load_check.waitForExistence(ros::Duration(10.0));
  if (!path_load_check.call(path_check_thresh))
  {
    message_ = "[" + name_ + "] Path checking failed, unable to call /path_load/check service";
    return false;
  }

  if (!path_check_thresh.response.pass)
  {
    message_ = "[" + name_ +
               "] Path checking failed, distance between start point of path and current robot position "
               "exceeds threshold";
    return false;
  }

  return true;
}

// Begin path_load
bool CleaningHandler::startPath(std::string path_name)
{
  //	ros::Duration(30.0).sleep();
  if (!checkPathWithinThresh(path_name))
    return false;

  // Send path to path_load/load which starts moving the robot
  ros::ServiceClient path_load_load = nh_handler_.serviceClient<path_recall::PathName>("/path_load/load");
  path_recall::PathName path_load_name;
  // path_load_name.request.name = p_planned_path_name_;
  path_load_name.request.name = path_name;

  path_load_load.waitForExistence(ros::Duration(10.0));
  if (!path_load_load.call(path_load_name))
  {
    message_ = "[" + name_ + "] Path loading failed, unable to call /path_load/load service";
    return false;
  }

  if (!path_load_name.response.success)
  {
    message_ = "Path failed";
    return false;
  }

  return true;
}

// Cancel path_load
bool CleaningHandler::cancelPath()
{
  ros::ServiceClient path_load_cancel = nh_handler_.serviceClient<std_srvs::Trigger>("/path_load/cancel");
  std_srvs::Trigger cancel;
  path_load_cancel.call(cancel);
  return cancel.response.success;
}

void CleaningHandler::stopAllLaunch()
{
  ROS_WARN("[%s] Stopping all launch files", name_.c_str());
  stopLaunch(path_recovery_id_);
  stopLaunch(path_load_id_);
  stopLaunch(path_saver_id_);
  stopLaunch(planner_server_id_);
  stopLaunch(planner_client_id_);

  path_recovery_id_ = path_load_id_ = path_saver_id_ = planner_server_id_ = planner_client_id_ = 0;
}

void CleaningHandler::startAllLaunch()
{
  // Start planners last as they are dependent on other launches
  path_recovery_id_ = startLaunch("path_recall", "path_recovery.launch", "");
  path_load_id_ = startLaunch("path_recall", "path_load_segments.launch", "yaml_path:=" + p_yaml_path_);
  path_saver_id_ = startLaunch("path_recall", "path_saver.launch", "yaml_path:=" + p_yaml_path_);
  planner_server_id_ = startLaunch("ipa_room_exploration", "room_exploration_action_server.launch", "");
  planner_client_id_ = startLaunch("ipa_room_exploration", "room_exploration_client.launch", "");
}

bool CleaningHandler::healthCheck()
{
  static int failcount = 0;
  bool isHealthy_planner_server = launchStatus(planner_server_id_);
  bool isHealthy_planner_client = launchStatus(planner_client_id_);

  bool isHealthy = (isHealthy_planner_server || isHealthy_planner_client);
  if (!isHealthy && planner_server_id_ && planner_client_id_)
  {
    failcount += 1;
    if (failcount >= 2*p_watchdog_rate_)
    {
      ROS_INFO("[%s] one or more zone planner nodes have failed", name_.c_str());
      movel_seirios_msgs::Reports report;
      report.header.stamp = ros::Time::now();
      report.handler = "cleaning_handler";
      report.task_type = task_type_;
      report.healthy = false;
      report.message = "some cleaning_handler nodes are not running";
      health_check_pub_.publish(report);
      
      cancelTask();
      stopAllLaunch();
      setTaskResult(false);

      failcount = 0;
    }

  }
  else 
    failcount = 0;
  return isHealthy;
}


}  // namespace task_supervisor
