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
  bool valid_size = false;
  bool valid_length = false;

  if (path->poses.size() < 2)
  {
    message_ = "Path contains " + std::to_string(path->poses.size()) + " points, needs to contain atleast 2 points.";
    setTaskResult(false);
  }
  else
    valid_size = true;

  double length;
  for (int i; i < path->poses.size() - 2; ++i)
  {
    length += sqrt(pow((path->poses[i + 1].pose.position.x - path->poses[i].pose.position.x), 2) +
                   pow((path->poses[i + 1].pose.position.y - path->poses[i].pose.position.y), 2));
  }

  if (length < (robot_radius_ * p_radius_multiplier_))
  {
    message_ = "Path length is " + std::to_string(length) + " , needs be atleast " +
               std::to_string(robot_radius_ * p_radius_multiplier_);
    setTaskResult(false);
  }
  else
    valid_length = true;

  if (valid_size and valid_length)
    path_planned_ = true;
}

bool CleaningHandler::parseArgs(std::string payload)
{
  std::deque<std::string> unordered_parsed_args;
  std::string delim = " ";
  size_t pos;

  // Split input payload strings
  if (payload.find(delim) != std::string::npos || payload.size() != 0)
  {
    // Append space to the back, otherwise while loop clears all args except last
    payload += " ";

    while ((pos = payload.find(delim)) != std::string::npos)
    {
      std::string token = payload.substr(0, pos);
      unordered_parsed_args.push_back(token);
      payload.erase(0, pos + delim.length());
    }
  }

  if (unordered_parsed_args.size() == 1 || unordered_parsed_args.size() == 2)
  {
    // Find '.txt' in first argument
    if (unordered_parsed_args.front().find(".txt") != std::string::npos)
    {
      path_to_polygon_txt_ = unordered_parsed_args.front();
      unordered_parsed_args.pop_front();
    }

    //.txt not found in first argument
    else if (unordered_parsed_args.back().find(".txt") != std::string::npos)
    {
      path_to_polygon_txt_ = unordered_parsed_args.back();
      unordered_parsed_args.pop_back();
    }

    else
    {
      ROS_ERROR("[%s] A .txt file containing the cropping polygon is required as an argument", name_.c_str());
      return false;
    }

    {
      FILE* file = fopen(path_to_polygon_txt_.c_str(), "r");
      if (file == NULL)
      {
        ROS_ERROR("[%s] Specified polygon .txt file is not readable or does not exist", name_.c_str());
        return false;
      }
    }

    // Parse 2nd arg if present
    if (unordered_parsed_args.size())
    {
      if (unordered_parsed_args.front().find(".pgm") != std::string::npos)
      {
        path_to_big_map_ = unordered_parsed_args.front();
        ROS_INFO("[%s] Map file provided, cropping provided map file at %s", name_.c_str(), path_to_big_map_.c_str());

        // Test if map is readable
        FILE* file = fopen(path_to_big_map_.c_str(), "r");
        if (file == NULL)
        {
          ROS_ERROR("[%s] Map file specified is not readable or does not exist", name_.c_str());
          return false;
        }
      }

      // Map format invalid
      else
      {
        ROS_ERROR("[%s] Provided 2nd argument is invalid. Only .pgm map files are accepted", name_.c_str());
        return false;
      }
    }

    else
    {
      ROS_INFO("[%s] No map file provided, cropping map pointed to at %s", name_.c_str(), path_to_big_map_.c_str());
    }
  }

  else
  {
    ROS_ERROR("[%s] Number of arguments incorrect, ensure no double spaces or spaces at the end.", name_.c_str());
    ROS_ERROR("[%s] Only 1 or 2 arguments are accepted. 1 .txt file and 1 .pgm file, in any order.", name_.c_str());
    return false;
  }

  return true;
}

bool CleaningHandler::parseArgs2(std::string payload, arg_flags& flags)
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
  if (!parseArgs2(task.payload, flags))
  {
    error_message = "[" + name_ + "] Payload parsing failed, check payload format";
    setTaskResult(false);
    return code_;
  }

  // Start all required launches
  startAllLaunch();

  // Check if all launches started
  if (!(path_recovery_id_ && path_load_id_ && planner_server_id_ && planner_client_id_ && path_saver_id_))
  {
    stopAllLaunch();
    error_message = "[" + name_ + "] Failed to start required launch files";
    setTaskResult(false);
    return code_;
  }

  // Subscribe to path_load's "start" topic. Start topic is gives state of path_load
  ros::Subscriber path_state_sub = nh_handler_.subscribe("/path_load/start", 1, &CleaningHandler::onPathStatus, this);

  if (flags.use_poly)
  {
    if (!(getPath() && isTaskActive()))
    {
      error_message = "[" + name_ + "] Path planning failed. " + message_;
      stopAllLaunch();
      setTaskResult(false);
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
    return true;
}

bool CleaningHandler::cropMap()
{
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
    if (!ros::param::get("/room_exploration_client/robot_radius", robot_radius_))
    {
      message_ = "Unable to obtain robot's radius for crop validation calculation.";
      return false;
    }

    double area_threshold = M_PI * robot_radius_ * robot_radius_ * p_radius_multiplier_;
    ROS_INFO("area_threshold is %f", area_threshold);
    ROS_INFO("sqm area is %f", sqm_area);

    if (sqm_area < area_threshold)
    {
      message_ = "Cropped area is " + std::to_string(sqm_area) + ", which is less than area threshold of " +
                 std::to_string(area_threshold);
      return false;
    }
  }
  else
    message_ = "Unable to crop map";

  return cropping_result;
}

bool CleaningHandler::getPath()
{
  ROS_ERROR_STREAM("Entered Getting Path");
  // Crop map using given polygon
  if (!cropMap())
    return false;

  // Subscribe to know if planning is complete
  ros::Subscriber planner_result_sub =
      nh_handler_.subscribe("/room_exploration_server/coverage_path", 1, &CleaningHandler::plannerResultCB, this);

  // Call room exploration client service
  ros::ServiceClient planner_srv = nh_handler_.serviceClient<ipa_room_exploration_msgs::RoomExplorationClient>("/room_"
                                                                                                               "explora"
                                                                                                               "tion_"
                                                                                                               "client/"
                                                                                                               "start");
  ipa_room_exploration_msgs::RoomExplorationClient planner;
  planner.request.path_to_coordinates_txt = path_to_coordinates_txt_;
  planner.request.path_to_cropped_map = path_to_cropped_map_;

  planner_srv.waitForExistence(ros::Duration(5.0));
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

  // Wait for planning to complete, timeout applied
  ros::Time startTime = ros::Time::now();
  while (!path_planned_)
  {
    if (ros::Time::now() - startTime > ros::Duration(p_planning_timeout_))
    {
      ROS_ERROR("[%s] Planning timed out, failed to plan path for cleaning", name_.c_str());
      message_ = "[" + name_ + "] Path planning timed out, unable to get path";
      return false;
    }
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
  // stopLaunch(move_base_id_);

  path_recovery_id_ = path_load_id_ = path_saver_id_ = planner_server_id_ = planner_client_id_ = 0;
  // move_base_id_ = 0;
}

void CleaningHandler::startAllLaunch()
{
  // Start planners last as they are dependent on other launches
  // move_base_id_ = startLaunch(p_move_base_package_, p_move_base_launch_, "");
  path_recovery_id_ = startLaunch("path_recall", "path_recovery.launch", "");

  path_load_id_ = startLaunch("path_recall", "path_load_segments.launch", "yaml_path:=" + p_yaml_path_);

  path_saver_id_ = startLaunch("path_recall", "path_saver.launch", "yaml_path:=" + p_yaml_path_);

  planner_server_id_ = startLaunch("ipa_room_exploration", "room_exploration_action_server.launch", "");

  planner_client_id_ = startLaunch("ipa_room_exploration", "room_exploration_client.launch", "");
}
}  // namespace task_supervisor
