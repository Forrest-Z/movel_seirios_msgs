namespace task_supervisor
{
/* ------------ START -------------*/
bool PCLLocalizationHandler::startDynamicMappingLaunch()
{
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

  ROS_INFO("[%s] TURNING ON DYNAMIC MAPPING!", name_.c_str());

  // Make a new copy of the map as temp
  try
  {
    boost::filesystem::path mySourcePath(loc_map_path_+".db");
    boost::filesystem::path myTargetPath(loc_map_path_+"_temp.db");
    boost::filesystem::copy_file(mySourcePath, myTargetPath, boost::filesystem::copy_option::overwrite_if_exists);
    ROS_INFO("[%s] Make a copy of db file for temp file!", name_.c_str());
  }
  catch (const boost::filesystem::filesystem_error& e)
  {
    std::cout << "Error Message: " << e.code().message() << '\n';
    message_ = e.code().message();
  }

  // Save pose
  bool status_pose = savePose();
  if (!status_pose)
  {
    res.success = false;
    res.message = message_;
    return true;
  }

  // Change db file to the temp one
  rtabmap_ros_multi::LoadDatabase change_srv;
  change_srv.request.clear = false;
  change_srv.request.database_path = loc_map_path_ + "_temp.db";
  if(!change_db_rtabmap_.call(change_srv))
  {
    ROS_INFO("[%s] Failed to switch db files!", name_.c_str());
  }
  
  // Waiting for the service to come up
  ros::service::waitForService("/rtabmap/set_mode_mapping", ros::Duration(20));
  ros::service::waitForService("/move_base/clear_costmaps", ros::Duration(20));

  // Call initial pose
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

  ros::Duration(2.0).sleep();

  // Pointcloud saver launch
  ROS_INFO("[%s] Launching Pointcloud Saver Launch File", name_.c_str());
  dynamic_map_launch_id_ = startLaunch("pcl_slam_handler", "pointcloud_saver.launch", "");
  if (!dynamic_map_launch_id_)
  {
    ROS_ERROR("[%s] Failed to launch Pointcloud Saver launch file", name_.c_str());
    message_ = "Failed to launch Pointcloud Saver launch file";
    res.success = false;
    res.message = message_;
    return false;
  }

  // // Update parameters
  // ROS_INFO("[%s] Updating Parameters for dynamic mapping!", name_.c_str());
  // int update_launch = startLaunch("pcl_localization_handler", "mapping_param.launch","");
  // std_srvs::Empty msg;
  // if (!update_params_.call(msg))
  // {
  //   ROS_INFO("[%s] Failed to update params", name_.c_str());
  // }

  // Call service to switch mode to mapping
  std_srvs::Empty switch_mode;
  if(!start_dyn_mapping_.call(switch_mode))
  {
    ROS_INFO("[%s] Failed to switch mode to mapping mode", name_.c_str());
  }

  ROS_INFO("[%s] DYNAMIC MAPPING HAS BEEN TURNED ON!", name_.c_str());

  isDynamicMapping_ = true;

  res.success = true;
  return true;
}

/* ------------ SAVE -------------*/
bool PCLLocalizationHandler::saveDynamicMappingCB(movel_seirios_msgs::StringTrigger::Request& req, movel_seirios_msgs::StringTrigger::Response& res)
{
  if (!localizing_.data || !isDynamicMapping_)
  {
    ROS_ERROR("[%s] Could not save the updated map, please start dynamic mapping first!", name_.c_str());
    message_ = "Could not save the updated map, please start dynamic mapping first!";
    res.success = false;
    res.message = message_;
    return true;
  }

  ROS_INFO("[%s] ENTERING SAVING MODE FOR DYNAMIC MAPPING!", name_.c_str());

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

  // // Update parameters
  // ROS_INFO("[%s] Updating Parameters for dynamic mapping!", name_.c_str());
  // int update_launch = startLaunch("pcl_localization_handler", "loc_param.launch","");
  // std_srvs::Empty msg;
  // if (!update_params_.call(msg))
  // {
  //   ROS_INFO("[%s] Failed to update params", name_.c_str());
  // }

  // Save pose
  bool status_pose = savePose();
  if (!status_pose)
  {
    res.success = false;
    res.message = message_;
    return true;
  }

  // Choose within update map and saving to a new map
  bool isUpdateMode;
  if (req.input == "")
    isUpdateMode = true;
  else
    isUpdateMode = false;

  // Save map. 
  bool status;
  if (isUpdateMode)     // Update Mode
    status = saveMap("");
  else              // Create a new map
    status = saveMap(req.input);
  if (!status)
  {
    ROS_ERROR("[%s] Failed to update The Map!", name_.c_str());
    message_ = "Failed to update The Map!";
    res.success = false;
    res.message = message_;
    return true;
  }

  ROS_INFO("[%s] Stopping pointcloud saver", name_.c_str());
  stopLaunch(dynamic_map_launch_id_, "/pointcloud_saver");
  dynamic_map_launch_id_ = 0;

  // Make new copy of the original file
  try
  {
    boost::filesystem::path mySourcePath(loc_map_path_+".db");
    boost::filesystem::path myTargetPath(loc_map_path_+"_temp2.db");
    boost::filesystem::copy_file(mySourcePath, myTargetPath, boost::filesystem::copy_option::overwrite_if_exists);
    ROS_INFO("[%s] Make a copy of db file for temp file!", name_.c_str());
  }
  catch (const boost::filesystem::filesystem_error& e)
  {
    std::cout << "Error Message: " << e.code().message() << '\n';
    message_ = e.code().message();
  }

  ros::Duration(2.0).sleep();

  // Change db file to the temp one
  rtabmap_ros_multi::LoadDatabase change_srv;
  change_srv.request.clear = false;
  change_srv.request.database_path = loc_map_path_ + "_temp2.db";
  if(!change_db_rtabmap_.call(change_srv))
  {
    ROS_INFO("[%s] Failed to switch db files!", name_.c_str());
  }

  isDynamicMapping_ = false;
  std::string target_path = req.input;
  // Copy map
  try
  {
    boost::filesystem::path mySourcePath(loc_map_path_+"_temp.db");
    boost::filesystem::path myTargetPath(target_path+".db");
    boost::filesystem::copy_file(mySourcePath, myTargetPath, boost::filesystem::copy_option::overwrite_if_exists);
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


  // Append a type to the yaml file
  ROS_INFO("[%s] Configuring yaml file!!!", name_.c_str());
  std::ofstream yaml_file(target_path + ".yaml", std::ios::out | std::ios::app);
  if(yaml_file.is_open())
  {
    yaml_file<<"type: 3d_rtabmap\n";
    yaml_file.close();
  }
  else
  {
    ROS_ERROR("[%s] Error opening a yaml file!", name_.c_str());
  }

  // Reload the new db file
  change_srv.request.clear = false;
  change_srv.request.database_path = target_path + ".db";
  if(!change_db_rtabmap_.call(change_srv))
  {
    ROS_INFO("[%s] Failed to switch db files!", name_.c_str());
  }

  // Update pose
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

  ros::Duration(2.0).sleep();


  // Delete the temp db file
  try
  {
    boost::filesystem::remove(loc_map_path_+"_temp.db");
    boost::filesystem::remove(loc_map_path_+"_temp2.db");
  }
  catch (const boost::filesystem::filesystem_error& e)
  {
    ROS_ERROR("[%s] ERROR: %s ", name_.c_str(),  e.code().message().c_str());
    message_ = e.code().message();
    res.success = false;
    res.message = message_;
    return true;
  }
  ROS_INFO("[%s] DYNAMIC MAPPING SUCCEEDED!", name_.c_str());

  if (!isUpdateMode) //Update the new loc_map_path_
  {
    loc_map_path_ = req.input;
  }

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

  ROS_INFO("[%s] CANCELING DYNAMIC MAPPING!", name_.c_str());
  bool status_pose = savePose();
  if (!status_pose)
  {
    res.success = false;
    res.message = message_;
    return true;
  }

   // Call service to switch mode to localization
  if (launchStatus(localization_launch_id_))
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

  ROS_INFO("[%s] Stopping pointcloud saver", name_.c_str());
  stopLaunch(dynamic_map_launch_id_, "/pointcloud_saver");
  dynamic_map_launch_id_ = 0;

  isDynamicMapping_ = false;

  // Change database
  rtabmap_ros_multi::LoadDatabase change_srv;
  change_srv.request.clear = false;
  change_srv.request.database_path = loc_map_path_ + ".db";
  if(!change_db_rtabmap_.call(change_srv))
  {
    ROS_INFO("[%s] Failed to switch db files!", name_.c_str());
  }

  // Delete DB
  try
  {
    boost::filesystem::remove(loc_map_path_+"_temp.db");
  }
  catch (const boost::filesystem::filesystem_error& e)
  {
    ROS_ERROR("[%s] ERROR: %s ", name_.c_str(),  e.code().message().c_str());
  }

  // Wait for movebase and rtabmap
  ros::service::waitForService("/rtabmap/set_mode_mapping", ros::Duration(20));
  ros::service::waitForService("/move_base/clear_costmaps", ros::Duration(20));

  // // Update parameters
  // ROS_INFO("[%s] Updating Parameters for dynamic mapping!", name_.c_str());
  // int update_launch = startLaunch("pcl_localization_handler", "loc_param.launch","");
  // std_srvs::Empty msg;
  // if (!update_params_.call(msg))
  // {
  //   ROS_INFO("[%s] Failed to update params", name_.c_str());
  // }

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

  ROS_INFO("[%s] DYNAMIC MAPPING HAS BEEN CANCELLED", name_.c_str());
  res.success = true;
  return true;
}

bool PCLLocalizationHandler::savePose()
{
  // Save Current pose
  try {
      last_pose_map_ = tf_buffer_.lookupTransform("map", "base_link" , ros::Time(0.0), ros::Duration(1.0)); 
  }
  catch (tf2::TransformException &ex) {
      ROS_WARN("[%s] Transform lookup failed %s", name_.c_str(), ex.what());
      message_ = "Failed to save latest pose of the robot";
      return false;
  }
  return true;
}
}