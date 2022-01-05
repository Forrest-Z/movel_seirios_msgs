namespace task_supervisor
{
/* ------------ START -------------*/
bool PCLLocalizationHandler::startDynamicMappingLaunch()
{
  if(!localizing_.data)
  {
    // copy db file to temp
    try
    {
      boost::filesystem::path mySourcePath(loc_map_path_+".db");
      boost::filesystem::path myTargetPath(loc_map_path_+"_temp.db");
      boost::filesystem::copy_file(mySourcePath, myTargetPath, boost::filesystem::copy_option::overwrite_if_exists);
      ROS_INFO("[%s] Make a copy of db file!", name_.c_str());
    }
    catch (const boost::filesystem::filesystem_error& e)
    {
      std::cout << "Error Message: " << e.code().message() << '\n';
      message_ = e.code().message();
    }

    // Create payload for move base
    std::string launch_args = "";
    if (!loc_map_path_.empty())
    {
      launch_args += " database_path:="+loc_map_path_+"_temp.db";
    }

    // Launch 3d movebase launch file
    ROS_INFO("[%s] Launching Move Base Launch File", name_.c_str());
    localization_launch_id_ = startLaunch(p_localization_launch_package_, p_dyn_map_move_base_launch_file_, launch_args);
    if (!localization_launch_id_)
    {
      ROS_ERROR("[%s] Failed to launch move base launch file", name_.c_str());
      message_ = "Failed to launch move base launch file";
      return false;
    }

    // Dynamic mapping launch
    launch_args += " localization:=true";
    ROS_INFO("[%s] Launching Mapping Launch File", name_.c_str());
    dynamic_map_launch_id_ = startLaunch(p_dyn_map_launch_package_, p_dyn_map_launch_file_, launch_args);
    if (!dynamic_map_launch_id_)
    {
      ROS_ERROR("[%s] Failed to launch mapping launch file", name_.c_str());
      message_ = "Failed to launch mapping launch file";
      return false;
    }

  }
  localizing_.data = true;
  return true;
}

bool PCLLocalizationHandler::startDynamicMappingCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  if (!localizing_.data && !isDynamicMapping_)
  {
    ROS_ERROR("[%s] Could not start dynamic mapping, localization was not started. Please start localization first!", name_.c_str());
    message_ = "Localization was not started. Please start localization first!";
    res.success = false;
    res.message = message_;
    return true;
  }
  else if (localizing_.data && isDynamicMapping_)
  {
    ROS_ERROR("[%s] Dynamic mapping is already running", name_.c_str());
    message_ = "Dynamic mapping is already running";
    res.success = false;
    res.message = message_;
    return true;
  }

  loc_health_timer_.stop();

  // Kill map_server and its friend
  if (loc_map_server_launch_id_ && nav_map_server_launch_id_)
  {
    stopLaunch(loc_map_server_launch_id_, "/map_server");
    stopLaunch(nav_map_server_launch_id_, "/map_server_nav");
    stopLaunch(map_name_pub_id_, "/map_name_pub");
    stopLaunch(map_editor_id_, "/map_editor");
    loc_map_server_launch_id_ = 0;    
    nav_map_server_launch_id_ = 0;
    map_name_pub_id_ = 0;
    map_editor_id_ = 0;
  }

  // Stopping localization node (RTABMAP)
  ROS_INFO("[%s] Stopping localization", name_.c_str());
  stopLaunch(localization_launch_id_, p_localization_launch_nodes_);
  localization_launch_id_ = 0;
  localizing_.data = false;

  // Relaunch new movebase launch
  bool status = startDynamicMappingLaunch();
  if(!status)
  {
    localizing_.data =false;
    if (localization_launch_id_ != 0)
    {
      stopLaunch(localization_launch_id_, p_localization_launch_nodes_);
      localization_launch_id_ = 0;
    }
    else if (point_mapping_launch_id_ != 0)
    {
      stopLaunch(point_mapping_launch_id_, p_localization_launch_nodes_);
      point_mapping_launch_id_ = 0;
    }
    res.success = false;
    res.message = message_;
    return true;
  }

  // Waiting for the service to come up
  ros::service::waitForService("/rtabmap/set_mode_mapping", ros::Duration(20));
  ros::service::waitForService("/move_base/clear_costmaps", ros::Duration(20));

  // Call service to switch mode to mapping
  std_srvs::Empty switch_mode;
  if(!start_dyn_mapping_.call(switch_mode))
  {
    ROS_INFO("[%s] Failed to switch mode to mapping mode", name_.c_str());
  }

  ROS_INFO("[%s] Dynamic Mapping has been started !", name_.c_str());
  isDynamicMapping_ = true;

  loc_health_timer_.start();    // Health check dynamic mapping version

  res.success = true;
  return true;
}

/* ------------ SAVE -------------*/
bool PCLLocalizationHandler::saveDynamicMappingCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  if (!localizing_.data || !isDynamicMapping_)
  {
    ROS_ERROR("[%s] Could not save the updated map, please start dynamic mapping first!", name_.c_str());
    message_ = "Could not save the updated map, please start dynamic mapping first!";
    res.success = false;
    res.message = message_;
    return true;
  }

  // Call service to switch mode to localization
  std_srvs::Empty switch_mode;
  if(!stop_dyn_mapping_.call(switch_mode))
  {
    ROS_INFO("[%s] Failed to switch mode to localization mode", name_.c_str());
    message_ = "Failed to launch dynamic mapping!";
    res.success = false;
    res.message = message_;
    return true;
  }

  // Save map. 
  bool status = saveMap("");
  if (!status)
  {
    ROS_ERROR("[%s] Failed to update The Map!", name_.c_str());
    message_ = "Failed to update The Map!";
    res.success = false;
    res.message = message_;
    return true;
  }

  loc_health_timer_.stop();

  // Kill dynamic mapping move_base
  ROS_INFO("[%s] Stopping movebase launch", name_.c_str());
  stopLaunch(localization_launch_id_, p_localization_launch_nodes_);
  localization_launch_id_ = 0;

  ROS_INFO("[%s] Stopping dynamic mapping", name_.c_str());
  stopLaunch(dynamic_map_launch_id_, p_localization_launch_nodes_);
  dynamic_map_launch_id_ = 0;
  localizing_.data = false;

  isDynamicMapping_ = false;

  // Copy map
  try
  {
    boost::filesystem::path mySourcePath(loc_map_path_+"_temp.db");
    boost::filesystem::path myTargetPath(loc_map_path_+".db");
    boost::filesystem::copy_file(mySourcePath, myTargetPath, boost::filesystem::copy_option::overwrite_if_exists);
    boost::filesystem::remove(loc_map_path_+"_temp.db");
    ROS_INFO("[%s] Save the temporary db file to the original one!", name_.c_str());
  }
  catch (const boost::filesystem::filesystem_error& e)
  {
    ROS_ERROR("[%s] ERROR: %s ", name_.c_str(),  e.code().message().c_str());
    message_ = e.code().message();
    res.success = false;
    res.message = message_;
    return true;
  }

  // Relaunch all the things
  bool result = startLocalization();
  if (!result)
  {
    ROS_ERROR("[%s] Error while starting localization. Please reload the map!", name_.c_str());
    message_ = "Error while starting localization. Please reload the map!";
    res.success = false;
    res.message = message_;
    return true;
  }
  loc_health_timer_.start();
  res.success = true;
  return true;
}

bool PCLLocalizationHandler::saveMap(std::string map_path)
{
  // Save 3D map
  movel_seirios_msgs::StringTrigger srv;
  if (map_path.empty())
    srv.request.input = loc_map_path_;
  else
    srv.request.input = map_path;

  if (save_map_client_rtabmap_.call(srv))
      ROS_INFO("[%s] PCL Map Save complete", name_.c_str());
  else
  {
      ROS_ERROR("[%s] Failed to save PCL", name_.c_str());
      return false;
  }

  std::string launch_args = " map_topic:=/map";
  if (!map_path.empty())
  {
    launch_args = launch_args + " file_path:=" + map_path;

    std::string map_name_nav = map_path;
    std::string key ("/");
    std::size_t idx = map_name_nav.rfind(key);
    if (idx != std::string::npos)
    {
      map_name_nav.replace(idx, key.length(), "/nav/");
    }
    launch_args = launch_args + " file_path_nav:=" + map_name_nav;
  }
  else
  {
    launch_args = launch_args + " file_path:=" + loc_map_path_;
    std::string new_nav_path = nav_map_path_.substr(0, nav_map_path_.size() -5);
    launch_args = launch_args + " file_path_nav:=" + new_nav_path;
  }

  unsigned int map_saver_id = startLaunch(p_map_saver_package_, p_map_saver_launch_, launch_args);
  // Check if startLaunch succeeded
  if (!map_saver_id)
  {
    ROS_ERROR("[%s] Failed to start map saver", name_.c_str());
    return false;
  }

  // While loop until timeout
  ros::Rate r(p_loop_rate_);
  ros::Time start_time = ros::Time::now();
  while (ros::Time::now().toSec() - start_time.toSec() < 5.0)
  {
    // TODO map_saver might die before saving
    if (!launchExists(map_saver_id))
    {
      ROS_INFO("[%s] Save complete", name_.c_str());
      // stopLaunch(conversion_id);

      ROS_INFO("[%s] Checking for 3D and 2D map files", name_.c_str());

      // 2D map checking
      FILE* pgm = fopen( (srv.request.input + ".pgm").c_str(), "r");
      if (pgm == NULL)
      {
          ROS_ERROR("[%s] 2D map file is NOT FOUND in %s", name_.c_str(), (srv.request.input + ".pgm").c_str());
          return false;
      }
      fclose(pgm);
      ROS_INFO("[%s] 2D map file is AVAILABLE in %s", name_.c_str(), (srv.request.input + ".pgm").c_str());

      return true;
    }
    r.sleep();
  }
  return true;
}

/* ------------ CANCEL -------------*/
bool PCLLocalizationHandler::cancelDynamicMappingCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  if (!localizing_.data || !isDynamicMapping_)
  {
    ROS_ERROR("[%s] Dynamic Mapping as not started. Nothing happened", name_.c_str());
    message_ = "Dynamic Mapping as not started. Nothing happened";
    res.success = false;
    res.message = message_;
    return true;
  }

  // Save Current pose
  try {
      last_pose_map_ = tf_buffer_.lookupTransform("map", "base_link" , ros::Time(0.0), ros::Duration(1.0)); 
  }
  catch (tf2::TransformException &ex) {
      ROS_WARN("[%s] Transform lookup failed %s", name_.c_str(), ex.what());
      message_ = "Failed to save latest pose of the robot";
      res.success = false;
      res.message = message_;
      return true;
  }

   // Call service to switch mode to localization
  if (launchStatus(dynamic_map_launch_id_))
  {
    std_srvs::Empty switch_mode;
    if(!stop_dyn_mapping_.call(switch_mode))
    {
      ROS_INFO("[%s] Failed to switch mode to localization mode", name_.c_str());
      message_ = "Failed to switch mode to localization mode";
      res.success = false;
      res.message = message_;
      return true;
    }
  }

  loc_health_timer_.stop();

  // Stopping Dynamic Mapping
  ROS_INFO("[%s] Stopping localization", name_.c_str());
  stopLaunch(localization_launch_id_, p_localization_launch_nodes_);
  localization_launch_id_ = 0;
  
  ROS_INFO("[%s] Stopping dynamic mapping", name_.c_str());
  stopLaunch(dynamic_map_launch_id_, p_localization_launch_nodes_);
  dynamic_map_launch_id_ = 0;
  localizing_.data = false;

  isDynamicMapping_ = false;
  // Delete DB
  try
  {
    boost::filesystem::remove(loc_map_path_+"_temp.db");
  }
  catch (const boost::filesystem::filesystem_error& e)
  {
    ROS_ERROR("[%s] ERROR: %s ", name_.c_str(),  e.code().message().c_str());
  }

  // relaunch original localiation
  bool result = startLocalization();
  if (!result)
  {
    ROS_ERROR("[%s] Error while starting localization. Please reload the map!", name_.c_str());
    message_ = "Error while starting localization. Please reload the map!";
    res.success = false;
    res.message = message_;
    return true;
  }

  // Wait for movebase and rtabmap
  ros::service::waitForService("/rtabmap/set_mode_mapping", ros::Duration(20));
  ros::service::waitForService("/move_base/clear_costmaps", ros::Duration(20));

  // Call Initial Pose
  geometry_msgs::PoseWithCovarianceStamped message;
  message.header.stamp = ros::Time::now();
  message.header.frame_id = "map";
  message.pose.pose.position.x = last_pose_map_.transform.translation.x;
  message.pose.pose.position.y = last_pose_map_.transform.translation.y;
  message.pose.pose.orientation = last_pose_map_.transform.rotation;
  message.pose.covariance[0] = 0.1;
  message.pose.covariance[7] = 0.1;
  message.pose.covariance[35] = 0.1;
  initpose_pub_.publish(message);

  loc_health_timer_.start();
  res.success = true;
  return true;
}
}