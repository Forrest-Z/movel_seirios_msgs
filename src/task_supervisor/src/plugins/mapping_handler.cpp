#include <pluginlib/class_list_macros.h>
#include <task_supervisor/plugins/mapping_handler.h>
#include <ros_utils/ros_utils.h>  //For loadParams function contents
#include <tf/transform_listener.h>

PLUGINLIB_EXPORT_CLASS(task_supervisor::MappingHandler, task_supervisor::TaskHandler);

namespace fs = boost::filesystem;

namespace task_supervisor
{
/**
 * Callback for /task_supervisor/map_saver/save_map service. Once save_map service is called, this method
 * starts the map_saver launch file and sets the map_topic depending on the yaml config file of task_supervisor.
 *
 * Where the map is saved is determined during the call to save_map service, a fully defined path can be passed
 * as an argument, otherwise the default location of $HOME is used.
 *
 * Default location can be modified in the launch file of map_saver, found in
 * (task_supervisor package)/launch/map_saver.launch
 */

void MappingHandler::orbTransCallback(std_msgs::Bool msg)
{
  ui_done_ = true;
}

std::string MappingHandler::mongo_object_id()
{
  std::stringstream result;
  std::time_t timestamp = std::time(nullptr);
  result << std::hex << timestamp;
  
  static const char hex_digits[] = "0123456789abcdef";
  for (int i = 0; i < 16; ++i) {
      result << hex_digits[std::rand() % 16];
  }

  return result.str();
}

bool MappingHandler::copyMapFiles(const std::string& source_folder_path, const std::string& destination_folder_path)
{
  try
  {
    if (!fs::exists(source_folder_path))
    {
      ROS_ERROR("[%s] Source folder does not exist: %s", name_.c_str(), source_folder_path.c_str());
      return false;
    }

    if (!fs::exists(destination_folder_path))
    {
      ROS_ERROR("[%s] Destination folder: %s doesn't exist!", name_.c_str(), destination_folder_path.c_str());

      return false;
    }

    for (fs::directory_iterator file(source_folder_path); file != fs::directory_iterator(); ++file)
    {
      if (fs::is_regular_file(file->status()))
      {
        std::string file_name = file->path().filename().string();
        std::string file_name_without_extension = file->path().stem().string();
        std::string file_extension = fs::extension(file->path());
        // std::cout<<"file name: " << file_name << std::endl;
        std::string map_id = mongo_object_id();
        if (file_extension == ".pgm")
        {
          std::string new_file_name = map_id;
          // std::cout << "parent path: " << file->path().parent_path().string() <<std::endl; //full path
          // std::cout << "without extension: " << file->path().parent_path().string() + "/" + file_name_without_extension <<std::endl; //full path
          std::string source_file_without_extension = file->path().parent_path().string();
          fs::copy_file(source_file_without_extension + "/" + file_name_without_extension + ".yaml", destination_folder_path + "/" + new_file_name + ".yaml");
          fs::copy_file(source_file_without_extension + "/" + file_name_without_extension + ".pgm", destination_folder_path + "/" + new_file_name + ".pgm");
          
          try
          {
              const std::string filename = destination_folder_path + "/" + new_file_name + ".yaml";
              // Load the YAML file
              YAML::Node config = YAML::LoadFile(filename);

              // Check if the 'image' parameter exists
              if (config["image"])
              {
                // Modify the 'image' parameter
                config["image"] = "/home/movel/.config/movel/maps/" + map_id + ".pgm";

                // Save the modified YAML back to the file
                std::ofstream fout(filename);
                fout << config;
                fout.close();

                ROS_INFO("Successfully modified the 'image' parameter.");
              }
              else
              {
                ROS_ERROR("The 'image' parameter does not exist in the YAML file.");
              }
          }
          catch (const YAML::Exception& e)
          {
            ROS_ERROR_STREAM("Error while processing the YAML file: " << e.what());
          }

          ROS_INFO("[%s] Files were successfully copied: %s (pgm and yaml) to: %s", name_.c_str(), source_file_without_extension.c_str(), new_file_name.c_str());
        }
      }
    }
  }
  catch (const fs::filesystem_error& e)
  {
    ROS_INFO("[%s] File system error: %s", name_.c_str(), e.what());

    return false;
  }
  return true;
}

bool MappingHandler::onOrbRestartServiceCall(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  if(!p_orb_slam_)
  {
    res.success = false;
    res.message = "Orb slam not active";
    return true;
  }

  orb_slam2_ros::SaveMap orb_map_name;
  orb_map_name.request.name = "/home/movel/.config/movel/maps/resume_orb_map_";
  serv_orb_save_.call(orb_map_name);
  if(!orb_map_name.response.success)
  {
    ROS_INFO("[%s] Orb Save Failed", name_.c_str());
    res.success = false;
    res.message = "Some issue on saving map";
    ROS_INFO("[%s] Orb Save Failed", name_.c_str());
    return false;
  }
  stopLaunch(orb_map_launch_id_, p_orb_map_launch_nodes_);
  while (launchExists(orb_map_launch_id_))
    ;

  std::string launch_args = " map_name:=" + orb_map_name.request.name;
  std::string rgb_color_topic_ = " rgb_color_topic:=" + p_rgb_color_topic_;
  std::string rgbd_depth_topic_ = " rgbd_depth_topic:=" + p_rgbd_depth_topic_;
  std::string rgbd_camera_info_topic_ = " rgbd_camera_info:=" + p_rgbd_camera_info_;
  orb_map_launch_id_ = startLaunch("orbomator", "orb_map_resume.launch", launch_args + 
                                                                        rgb_color_topic_ +
                                                                      rgbd_depth_topic_ +
                                                                      rgbd_camera_info_topic_);
  if (!orb_map_launch_id_)
  {
    ROS_ERROR("[%s] Failed to restart orbslam mapping launch file", name_.c_str());
    return false;
  }
  res.success = true;
  res.message = "Orb slam mapping restarted";
  return true;

}

bool MappingHandler::onSaveServiceCall(movel_seirios_msgs::StringTrigger::Request& req,
                                       movel_seirios_msgs::StringTrigger::Response& res)
{
  if(p_orb_slam_)
  {
    orb_slam2_ros::SaveMap orb_map_name;
    orb_map_name.request.name = req.input;
    serv_orb_save_.call(orb_map_name);
    if(!orb_map_name.response.success)
    {
      ROS_INFO("[%s] Orb Save Failed", name_.c_str());
      stopLaunch(orb_map_launch_id_, p_orb_map_launch_nodes_);
      while (launchExists(orb_map_launch_id_))
        ;
      res.success = false;
      orb_map_launch_id_ = 0;
      ROS_INFO("[%s] ORB Mapping STOPPED", name_.c_str());
      res.success = saveMap(req.input);
      
      saved_ = true;
      while(!mapping_launches_stopped_)
        ros::Duration(2).sleep();
      mapping_launches_stopped_ = false;
      return false;
    }
    stopLaunch(orb_map_launch_id_, p_orb_map_launch_nodes_);
    while (launchExists(orb_map_launch_id_))
      ;
    std::string launch_args = " map_name:=" + req.input;
    std::string rgb_color_topic_ = " rgb_color_topic:=" + p_rgb_color_topic_;
    std::string rgbd_depth_topic_ = " rgbd_depth_topic:=" + p_rgbd_depth_topic_;
    std::string rgbd_camera_info_topic_ = " rgbd_camera_info:=" + p_rgbd_camera_info_;

    orb_ui_launch_id = startLaunch("orbomator", "orbomator_ui.launch", launch_args + 
                                                                          rgb_color_topic_ +
                                                                        rgbd_depth_topic_ +
                                                                        rgbd_camera_info_topic_);
    while(!ui_done_)
    {
      ros::spinOnce();
      ros::Duration(2).sleep();
    }
    ui_done_ = false;
    stopLaunch(orb_ui_launch_id,p_orb_ui_launch_nodes_);
    while (launchExists(orb_ui_launch_id))
      ;
    ROS_INFO("[%s] ORB Save complete", name_.c_str());
    orb_ui_launch_id = 0;
  }
  
  res.success = saveMap(req.input);
  // res.success = true;
  saved_ = true;
  // while(!mapping_launches_stopped_)
  //   ros::Duration(1).sleep();
  // mapping_launches_stopped_ = false;
  return true;
}

bool MappingHandler::onAsyncSave(movel_seirios_msgs::StringTrigger::Request& req,
                                 movel_seirios_msgs::StringTrigger::Response& res)
{
  res.success = saveMap(req.input);
  res.success = true;
  return true;
}

bool MappingHandler::onStatus(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  res.success = launchStatus(mapping_launch_id_);
  return true;
}

bool MappingHandler::saveMap(std::string map_name)
{
  if (p_use_aruco_)
  {
    ros::ServiceClient aruco_client = nh_handler_.serviceClient<movel_seirios_msgs::StringTrigger>("/movel_aruco_saver/save_aruco");
    movel_seirios_msgs::StringTrigger write_aruco;
    write_aruco.request.input = map_name + ".txt";
    aruco_client.call(write_aruco);
    if (write_aruco.response.success == false)
    {
        ROS_ERROR("[%s] Failed to save aruco file", map_name.c_str());
    }
  }
  map_name_save_ = map_name;
  // Set path to save file
  std::string launch_args = " map_topic:=" + p_map_topic_;

  // If user has rotated the map, save the rotated map instead
  if (map_rotation_rad_ != 0.0)
  {
    ROS_INFO("[%s] Rotating the map one last time and saving the rotated map.", name_.c_str());
    launch_args += "_rotated";

    rotateMap(original_map_, rotated_map_, map_rotation_rad_);
    publishRotatedMap();
  }
  
  if (!map_name.empty())
  {
    launch_args = launch_args + " file_path:=" + map_name;
    std::string map_nav_name (map_name);
    std::string key ("/");
    std::size_t idx = map_name.rfind(key);
    if (idx != std::string::npos)
    {
      map_nav_name.replace(idx, key.length(), "/nav/");
      ROS_INFO("map name %s", map_name.c_str());
      ROS_INFO("map nav name %s", map_nav_name.c_str());
      launch_args = launch_args + " file_path_nav:=" + map_nav_name;
    }
  }

  ROS_INFO("launch args %s", launch_args.c_str());
  // Call map saving through launch manager service
  ROS_INFO("[%s] Saving map %s", name_.c_str(), map_name.size() != 0 ? ("to " + map_name).c_str() : "");
  unsigned int map_saver_id = startLaunch("task_supervisor", "map_saver.launch", launch_args);

  // Check if startLaunch succeeded
  if (!map_saver_id)
  {
    ROS_ERROR("[%s] Failed to start map saver", name_.c_str());
    return false;
  }

  // While loop until timeout
  ros::Time start_time = ros::Time::now();
  ros::Rate r(p_loop_rate_);
  while (ros::Time::now().toSec() - start_time.toSec() < p_save_timeout_)
  {
    // TODO map_saver might die before saving
    if (!launchExists(map_saver_id))
    {
      ROS_INFO("[%s] Save complete", name_.c_str());
      if (p_split_map_)
      {
        // launch map splitter
        std::string map_splitter_args;
        unsigned int map_splitter_id = startLaunch("map_splitter", "map_splitter.launch", map_splitter_args);
        if (!map_splitter_id)
        {
          ROS_ERROR("[%s] map splitter failed to start", name_.c_str());
          return false;
        }

        // call its service
        ros::ServiceClient map_splitter_srv = nh_supervisor_.serviceClient<movel_seirios_msgs::StringTrigger>("/split_map");
        ROS_INFO("expect split map service on %s", nh_supervisor_.resolveName("split_map").c_str());
        if (!map_splitter_srv.waitForExistence(ros::Duration(10.0)))
        {
          ROS_ERROR("[%s] map splitting service failed to start", name_.c_str());
          return false;
        }
        movel_seirios_msgs::StringTrigger srv;
        srv.request.input = map_name;
        ROS_INFO("[%s] map name: %s ", name_.c_str(), map_name.c_str());
        if (!map_splitter_srv.call(srv))
        {
          ROS_ERROR("[%s] map splitting service failed to run", name_.c_str());
          stopLaunch(map_splitter_id);
          return false;
        }
        if (p_save_split_map_to_library_)
        {
          // map_name: "/home/$USER/catkin_ws/movel_ai/maps/64abb4439a5ff8e3ae252b03";
          // default_path: "/home/$USER/catkin_ws/movel_ai/maps/";

          std::string default_path = map_name;

          // Find the last occurrence of '/'
          size_t lastSlashPos = default_path.rfind('/');

          if (lastSlashPos != std::string::npos) 
          {
            // Erase the characters from lastSlashPos onwards
            default_path.erase(lastSlashPos+1); // +1 to keep the trailing '/'
          }
          else
          {
            // Handle the case where no '/' was found
            ROS_ERROR("[%s] Failed to save split map to library - No '/' found in the path.", name_.c_str());
            return false; // Exit with an error code
          }

          //copy the split map files on the folder to the library / default directory (movel_ai/maps/)
          if(copyMapFiles(map_name, default_path))
          {
            ROS_INFO("[%s] Save split map to library - done.", name_.c_str());
          }
          else
          {
            ROS_ERROR("[%s] Failed to save split map to library - error copying map files.", name_.c_str());
          }
        }

        // teardown
        stopLaunch(map_splitter_id);
        return true;
      }
      return true;
    }

    r.sleep();
  }

  ROS_WARN("[%s] Timeout occurred, save failed", name_.c_str());
  stopLaunch(map_saver_id);
  return false;
}

bool MappingHandler::setSpeed(double linear_velocity, double angular_velocity)
{
    ROS_INFO("[%s] Received velocity value(s) to be set: (%.2lf, %.2lf)", 
             name_.c_str(), linear_velocity, angular_velocity);

    if (linear_velocity == 0 || angular_velocity == 0)
    {
      ROS_ERROR("[%s] Failed to set velocity, both velocities must not be zero!", name_.c_str());
      return false;
    }

    ros::ServiceClient velocity_client =
      nh_handler_.serviceClient<movel_seirios_msgs::SetSpeed>("/velocity_setter_node/set_speed");    
    movel_seirios_msgs::SetSpeed set_speed;
    set_speed.request.linear = linear_velocity;
    set_speed.request.angular = angular_velocity;

    if (!velocity_client.call(set_speed))
    {
      ROS_ERROR("[%s] Failed to call /velocity_setter_node/set_speed service", name_.c_str());
      return false;
    }

    if (!set_speed.response.success)
    {
      ROS_ERROR("[%s] Failed to set new linear and angular velocities", name_.c_str());
      return false;
    }

    ROS_INFO("[%s] Linear velocity set to: %.2lf, angular velocity set to: %.2lf", name_.c_str(), set_speed.request.linear, set_speed.request.angular);
    return true;
}

/**
 * Method called by runTask() which starts up the mapping node. Mapping package and launch file is defined in the
 * task_supervisor
 * yaml under 'mapping_launch_package' and 'mapping_launch_file'. Launch file should only launch the mapping node, such
 * as gmapping,
 * and no other nodes eg move_base.
 *
 * Once mapping is started by task_supervisor, mapping_handler eventually blocks in this method and waits for
 * cancellation
 * or saving to be done via save_map service. Once save_map service is called, this method will return
 * and the mapping node will be shut down.
 */
bool MappingHandler::runMapping()
{
  ROS_INFO("[%s] Starting mapping package: %s, launch file: %s", name_.c_str(), p_mapping_launch_package_.c_str(),
           p_mapping_launch_file_.c_str());

  std::string auto_arg;
  if(p_auto_)
    auto_arg = " auto:=true";

  // Run mapping asynchronously
  mapping_launch_id_ = startLaunch(p_mapping_launch_package_, p_mapping_launch_file_, auto_arg);
  if (!mapping_launch_id_)
  {
    ROS_ERROR("[%s] Failed to launch mapping launch file", name_.c_str());
    return false;
  }
  
  if(p_orb_slam_)
  {
    std::string rgb_color_topic_ = "rgb_color_topic:=" + p_rgb_color_topic_;
    std::string rgbd_depth_topic_ = " rgbd_depth_topic:=" + p_rgbd_depth_topic_;
    std::string rgbd_camera_info_topic_ = " rgbd_camera_info:=" + p_rgbd_camera_info_;

    orb_map_launch_id_ = startLaunch(p_orb_map_launch_package_, p_orb_map_launch_file_, rgb_color_topic_ +
                                                                                      rgbd_depth_topic_ +
                                                                                      rgbd_camera_info_topic_);
    if (!orb_map_launch_id_)
    {
      ROS_ERROR("[%s] Failed to launch orbslam mapping launch file", name_.c_str());
      return false;
    }

  }

  if(p_auto_)
  {
    automap_launch_id_ = startLaunch("automapping_handler", "automapping.launch", "");
    if (!automap_launch_id_)
    {
      ROS_ERROR("[%s] Failed to launch automapping launch file", name_.c_str());
      return false;
    }
  }

  // Loop until save callback is called
  ros::Rate r(p_loop_rate_);
  bool speed_initialized = false;
  while (!saved_)
  {
    if (p_auto_ && !speed_initialized && setSpeed(task_linear_velocity_, task_angular_velocity_))
      speed_initialized = true;
    
    // Check if cancellation has been called
    if (!isTaskActive())
    {
      ROS_INFO("[%s] Waiting for mapping thread to exit", name_.c_str());

      if(p_auto_ && automap_launch_id_)
      {
        stopLaunch(automap_launch_id_);
        while(launchExists(automap_launch_id_))
          ;
        automap_launch_id_ = 0;
        actionlib_msgs::GoalID cancel;
        cancel_pub_.publish(cancel);
      }

      stopLaunch(mapping_launch_id_, p_mapping_launch_nodes_);
      if(p_orb_slam_)
      {
        stopLaunch(orb_map_launch_id_, p_orb_map_launch_nodes_);
        if(launchExists(orb_ui_launch_id))
          {
            stopLaunch(orb_ui_launch_id,p_orb_ui_launch_nodes_);
            ui_done_ = false;
            while (launchExists(orb_map_launch_id_) || launchExists(mapping_launch_id_) || launchExists(orb_ui_launch_id))
            ;
            return false; 
          }
        while (launchExists(orb_map_launch_id_) || launchExists(mapping_launch_id_))
          ;
        return false;        
      }
      while (launchExists(mapping_launch_id_))
        ;
      return false;
      
    }

    // Check if pause request is received
    if (isTaskPaused() && p_auto_)
    {
      while (isTaskPaused())
      {
        if(automap_launch_id_)
        {
          stopLaunch(automap_launch_id_);
          while(launchExists(automap_launch_id_))
            ;
          automap_launch_id_ = 0;
          actionlib_msgs::GoalID cancel;
          cancel_pub_.publish(cancel);
        }
      }
      if(isTaskActive())
        automap_launch_id_ = startLaunch("automapping_handler", "automapping.launch", "");
    }

    r.sleep();
  }

  if(p_auto_ && automap_launch_id_)
  {
    ROS_INFO("[%s] Stopping automapping node", name_.c_str());
    stopLaunch(automap_launch_id_);
    while(launchExists(automap_launch_id_))
      ;
    automap_launch_id_ = 0;
    ROS_INFO("[%s] Automapping node stopped", name_.c_str());
  }

  if(p_orb_slam_)
  {
    // Append a type to the yaml file
    ROS_INFO("[%s] Configuring yaml file!!!", name_.c_str());
    std::ofstream yaml_file(map_name_save_ + ".yaml", std::ios::out | std::ios::app);
    if(yaml_file.is_open())
    {
      yaml_file<<"type: orb_slam\n";
      yaml_file.close();
    }
    else
    {
      ROS_ERROR("[%s] Error opening a yaml file!", name_.c_str());
    }

    ROS_INFO("[%s] Stopping mapping", name_.c_str());
    stopLaunch(mapping_launch_id_, p_mapping_launch_nodes_);
    stopLaunch(orb_map_launch_id_, p_orb_map_launch_nodes_);
    while (launchExists(orb_map_launch_id_) || launchExists(mapping_launch_id_))
    ;
    mapping_launch_id_ = 0;
    orb_map_launch_id_ = 0;
    ROS_INFO("[%s] ORB Mapping stopped", name_.c_str());
    saved_ = false;
    mapping_launches_stopped_ = true;
    return true;
  } 
  else
  {
    // Append a type to the yaml file
    ROS_INFO("[%s] Configuring yaml file!!!", name_.c_str());
    std::ofstream yaml_file(map_name_save_ + ".yaml", std::ios::out | std::ios::app);
    if(yaml_file.is_open())
    {
      yaml_file<<"type: 2d\n";
      yaml_file.close();
    }
    else
    {
      ROS_ERROR("[%s] Error opening a yaml file!", name_.c_str());
    }

    ROS_INFO("[%s] Stopping mapping", name_.c_str());
    stopLaunch(mapping_launch_id_, p_mapping_launch_nodes_);    
    while (launchExists(mapping_launch_id_))
      ;
    mapping_launch_id_ = 0;
    ROS_INFO("[%s] Mapping stopped", name_.c_str());
    saved_ = false;
    // mapping_launches_stopped_ = true;
    return true;
  }
}

/**
 * runTask is called by task_supervisor once it receives a goal that includes a task of type 2. No payload is processed
 * by mapping_handler
 *
 * Once called, this method advertises the save_map service, as well as calls the runMapping method, which will carry
 * out all required tasks to start mapping.
 *
 * Once mapping is complete, save_map service is shut down and the task is signalled as complete
 */
ReturnCode MappingHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message)
{
  task_active_ = true;
  task_parsed_ = false;
  start_ = ros::Time::now();

  task_linear_velocity_ = task.linear_velocity;
  task_angular_velocity_ = task.angular_velocity;

  if(p_auto_)
  {
    cancel_pub_ = nh_handler_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
    stopped_pub_ = nh_handler_.advertise<std_msgs::Empty>("stopped", 1);
  }

  ros::Subscriber status_sub_ = nh_handler_.subscribe("/rosout",1, &MappingHandler::logCB, this);
  ros::ServiceServer serv_status_ = nh_handler_.advertiseService("status", &MappingHandler::onStatus, this);
  ros::ServiceServer serv_save_ = nh_handler_.advertiseService("save_map", &MappingHandler::onSaveServiceCall, this);
  ros::ServiceServer serv_save_async_ = nh_handler_.advertiseService("save_map_async", &MappingHandler::onAsyncSave, this);
  
  ros::Subscriber map_sub_ = nh_handler_.subscribe(p_map_topic_, 1, &MappingHandler::mapCB, this);
  ros::Subscriber map_rotation_sub_ = nh_handler_.subscribe("/movel_gui_mapping_rotation", 1, &MappingHandler::mapRotationCB, this);
  ros::Timer rotated_map_pub_timer_ = nh_handler_.createTimer(ros::Duration(1.0), std::bind(&MappingHandler::publishRotatedMap, this));
  rotated_map_pub_ = nh_handler_.advertise<nav_msgs::OccupancyGrid>(std::string(p_map_topic_) + "_rotated", 1, true);
  map_rotation_rad_ = 0.0;

  ros::ServiceServer orb_map_restart_ = nh_handler_.advertiseService("/orb_slam/mapping/restart", &MappingHandler::onOrbRestartServiceCall, this);
  orb_trans_ui_ =  nh_handler_.subscribe("/orb_ui/status", 1, &MappingHandler::orbTransCallback, this);
  serv_orb_save_ = nh_handler_.serviceClient<orb_slam2_ros::SaveMap>("/orb_slam2/save_map");
  p_orb_ui_launch_nodes_ = "/GUI_Orbomator /orbomateur /orb_slam2";  
  mapping_launches_stopped_ = false;

  bool mapping_done = runMapping();

  // Reset all state variables and shutdown services
  setTaskResult(mapping_done);
  return code_;
}

/**
 * Loads all required and optional parameters for the use of mapping_handler. Parameters can be configured
 * in the task_supervisor's yaml file, under the 'mapping_handler' section
 *
 * Parameters are loaded on startup of task_supervisor, not on task execution
 */
bool MappingHandler::loadParams()
{
  ros_utils::ParamLoader param_loader(nh_handler_);

  param_loader.get_optional("loop_rate", p_loop_rate_, 5.0);
  param_loader.get_optional("save_timeout", p_save_timeout_, 5.0);
  param_loader.get_optional("map_topic", p_map_topic_, std::string("/map"));
  param_loader.get_optional("split_map", p_split_map_, false);
  param_loader.get_optional("save_split_map_to_library", p_save_split_map_to_library_, false);
  param_loader.get_optional("auto", p_auto_, false);

  param_loader.get_optional("orb_slam", p_orb_slam_, false);
  param_loader.get_optional("rgb_color_topic", p_rgb_color_topic_, std::string("/camera/rgb/image_raw" ));
  param_loader.get_optional("rgbd_depth_topic", p_rgbd_depth_topic_, std::string("/camera/depth_registered/image_raw"));
  param_loader.get_optional("rgbd_camera_info", p_rgbd_camera_info_, std::string("/camera/rgb/camera_info"));
    
  param_loader.get_required("mapping_launch_package", p_mapping_launch_package_);
  param_loader.get_required("mapping_launch_file", p_mapping_launch_file_);
  param_loader.get_required("mapping_launch_nodes", p_mapping_launch_nodes_);

  param_loader.get_optional("use_aruco", p_use_aruco_, false);
  if(p_orb_slam_)
  {
    param_loader.get_required("orb_map_launch_package", p_orb_map_launch_package_);
    param_loader.get_required("orb_map_launch_file", p_orb_map_launch_file_);
    param_loader.get_required("orb_map_launch_nodes", p_orb_map_launch_nodes_);
  }

  return param_loader.params_valid();
}

/**
 *	Setup method called once task_supervisor starts and loads all plugins (task handlers)
 */
bool MappingHandler::setupHandler()
{
  if (!loadParams())
    return false;
  else
  {
    return true;
  }
}

bool MappingHandler::healthCheck()
{
  static int failcount = 0;
  if (mapping_launch_id_ == 0)
  {
    failcount = 0;
    return true;
  }
  bool healthy = launchStatus(mapping_launch_id_, failcount >= 30*p_watchdog_rate_);
  if (!healthy && mapping_launch_id_)
  {
    // it is possible for launchStatus to return false right after the nodes are launched
    // so failure assessment must give it time to stabilise
    // --> only declare unhealth after several seconds of consistent report
    failcount += 1;
    ROS_INFO("[%s] fail count %d", name_.c_str(), failcount);
    if (failcount >= 30*p_watchdog_rate_)
    {
      // report bad health
      ROS_INFO("[%s] one or more mapping nodes have failed %d, %5.2f", 
        name_.c_str(), failcount, 2*p_watchdog_rate_);
      
      // notify UI about unhealthy nodes
      bool healthy = launchStatus(mapping_launch_id_, true);

      // tear down task
      cancelTask();
      // stopLaunch(mapping_launch_id_);
      // saved_ = true;
      // setTaskResult(false);

      // reset flags
      failcount = 0;
      // mapping_launch_id_ = 0;
    }
  }
  else
    failcount = 0;
  return healthy;
}

void MappingHandler::logCB(const rosgraph_msgs::LogConstPtr& msg)
{
  if (p_auto_)
  {
    // Retrieve log message
    if (msg->name == "/explore" && msg->level == 2)
    {
      std::size_t found;
      found = msg->msg.find("Exploration stopped.");
      if (found != std::string::npos)
      {
        //ended_ = true;
        ROS_INFO("[%s] Automapping complete.", name_.c_str());
        stopped_pub_.publish(std_msgs::Empty());
      }
    }
  }
}

void MappingHandler::mapCB(const nav_msgs::OccupancyGridConstPtr& msg)
{
  original_map_ = *msg;

  if (map_rotation_rad_ == 0.0)
    return;

  ROS_INFO("[%s] Received map, rotating.", name_.c_str());
  rotateMap(original_map_, rotated_map_, map_rotation_rad_);

  publishRotatedMap();
}

void MappingHandler::mapRotationCB(const std_msgs::Float64ConstPtr& msg)
{
  map_rotation_rad_ = msg->data;
}

void MappingHandler::publishRotatedMap()
{
  rotated_map_pub_.publish(rotated_map_);
}

void MappingHandler::rotateMap(
  const nav_msgs::OccupancyGrid& input_map,
  nav_msgs::OccupancyGrid& output_map,
  double rotation_rad)
{
  // The rotated map has to be enlarged or some areas might get cropped out
  // Smallest safe shape is a square with its sides being the original map's diagonal
  int width = input_map.info.width;
  int height = input_map.info.height;
  double diag_length = std::sqrt(width * width + height * height);
  
  int output_size = static_cast<int>(std::ceil(diag_length));
  double output_center_x = (output_size - 1) / 2.0;
  double output_center_y = (output_size - 1) / 2.0;

  // Set the map's metadata
  output_map.header = input_map.header;
  output_map.info.resolution = input_map.info.resolution;
  output_map.info.width = output_size;
  output_map.info.height = output_size;
  
  // Calculate origin such that the center cell is at (0.0 m, 0.0 m, 0.0 rad)
  output_map.info.origin.position.x = -output_center_x * input_map.info.resolution;
  output_map.info.origin.position.y = -output_center_y * input_map.info.resolution;
  output_map.info.origin.position.z = 0.0;
  output_map.info.origin.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // Increase the number of map cells to accomodate the larger map size, initialize to unknown
  output_map.data.resize(output_size * output_size, -1);

  double cos_theta = cos(rotation_rad);
  double sin_theta = sin(rotation_rad);

  // Rotate and translate the map
  for (int y = 0; y < output_size; ++y)
  {
    for (int x = 0; x < output_size; ++x)
    {
      int old_x = static_cast<int>(cos_theta * (x - output_center_x) + sin_theta * (y - output_center_y) + width / 2);
      int old_y = static_cast<int>(-sin_theta * (x - output_center_x) + cos_theta * (y - output_center_y) + height / 2);

      if (old_x >= 0 && old_x < width && old_y >= 0 && old_y < height)
      {
        output_map.data[y * output_size + x] = input_map.data[old_y * width + old_x];
      }
    }
  }
}

}  // namespace task_supervisor