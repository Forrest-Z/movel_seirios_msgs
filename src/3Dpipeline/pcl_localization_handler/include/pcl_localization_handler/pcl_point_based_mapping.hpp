namespace task_supervisor
{
/* --------------------------------
        POINT BASED MAPPING
   --------------------------------
*/

/* ------------ START -------------*/
bool PCLLocalizationHandler::startPointBMappingCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  // If localization is running
  if (localizing_.data && !isPBMapping_ && !isDynamicMapping_)   // Kill Localization first
  {
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
  }
  else if (localizing_.data && !isPBMapping_ && isDynamicMapping_)
  {
    ROS_WARN("[%s] Dynamic Mapping is Running, please stop it first", name_.c_str());
    message_ = "Dynamic Mapping is Running, please stop it first";
    res.success =false;
    res.message = message_;
    return true;
  }
  else if (isPBMapping_)     // If Point based mapping is already running
  {
    ROS_WARN("[%s] Point Based Mapping is already running!", name_.c_str());
    message_ = "Point based mapping is already running";
    res.success =false;
    res.message = message_;
    return true;
  }
  // If there is no localization node running right now, do nothing

  // Launch Point based Mapping
  bool status = startPointBMappingLaunch();
  if(!status)
  {
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

  isPBMapping_ = true;
  loc_health_timer_.start();        // health check point based mapping version
  res.success = true;
  return true;
}

bool PCLLocalizationHandler::startPointBMappingLaunch()
{
  // Create payload for move base
  std::string launch_args = "";
  if (!p_map_name_.empty())
  {
    launch_args += " database_path:="+p_map_name_+".db";
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

  // Point Based mapping launch
  ROS_INFO("[%s] Launching Mapping Launch File", name_.c_str());
  point_mapping_launch_id_ = startLaunch(p_dyn_map_launch_package_, p_dyn_map_launch_file_, launch_args);
  if (!point_mapping_launch_id_)
  {
    ROS_ERROR("[%s] Failed to launch point-based mapping launch file", name_.c_str());
    message_ = "Failed to launch point-based mapping launch file";
    return false;
  }

  return true;
}

/* ------------ SAVE -------------*/
bool PCLLocalizationHandler::savePointBMappingCB(movel_seirios_msgs::StringTrigger::Request& req, movel_seirios_msgs::StringTrigger::Response& res)
{
  // If localizing. Do nothing
  if (localizing_.data)
  {
    ROS_ERROR("[%s] Normal 3D localization is running. Nothing happened", name_.c_str());
    message_ = "Normal 3D localization is running. Nothing happened";
    res.success = false;
    res.message = message_;
    return true;
  }
  else if (!isPBMapping_)   // If Point based mapping is not running
  {
    ROS_ERROR("[%s] Could not save the updated map, please start point based mapping first!", name_.c_str());
    message_ = "Could not save the updated map, please start point based mapping first!";
    res.success = false;
    res.message = message_;
    return true;
  }

  // Save map. 
  bool status = saveMap(req.input);
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

  ROS_INFO("[%s] Stopping point based mapping", name_.c_str());
  stopLaunch(point_mapping_launch_id_, p_localization_launch_nodes_);
  point_mapping_launch_id_ = 0;

  isPBMapping_ = false;
  // Copy map
  try
  {
    boost::filesystem::path mySourcePath(p_map_name_+".db");
    boost::filesystem::path myTargetPath(req.input+".db");
    boost::filesystem::copy_file(mySourcePath, myTargetPath, boost::filesystem::copy_option::overwrite_if_exists);
    boost::filesystem::remove(p_map_name_+".db");
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
  
  res.success = true;
  return true;
}

/* ------------ CANCEL -------------*/
bool PCLLocalizationHandler::cancelPointBMappingCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  // If normal localization is running
  if (localizing_.data)
  {
    ROS_ERROR("[%s] Normal 3D localization is running. Nothing happened when canceling point based mapping", name_.c_str());
    message_ = "Normal 3D localization is running. Nothing happened when canceling point based mapping";
    res.success = false;
    res.message = message_;
    return true;
  }
  else if (!localizing_.data && !isPBMapping_)    // No mapping is running
  {
    ROS_ERROR("[%s] Point Based Mapping is not started. Nothing happened", name_.c_str());
    message_ = "Point Based Mapping is not started. Nothing happened";
    res.success = false;
    res.message = message_;
    return true;
  }

  loc_health_timer_.stop();
  
  // Stopping Point Based Mapping
  ROS_INFO("[%s] Stopping localization", name_.c_str());
  stopLaunch(localization_launch_id_, p_localization_launch_nodes_);
  localization_launch_id_ = 0;
  
  ROS_INFO("[%s] Stopping dynamic mapping", name_.c_str());
  stopLaunch(point_mapping_launch_id_, p_localization_launch_nodes_);
  point_mapping_launch_id_ = 0;

  isPBMapping_ = false;

  // Delete DB
  try
  {
    boost::filesystem::remove(p_map_name_+".db");
  }
  catch (const boost::filesystem::filesystem_error& e)
  {
    ROS_ERROR("[%s] ERROR: %s ", name_.c_str(),  e.code().message().c_str());
  }

  res.success = true;
  return true;
}

}