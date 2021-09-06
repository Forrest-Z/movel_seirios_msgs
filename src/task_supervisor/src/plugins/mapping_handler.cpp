#include <pluginlib/class_list_macros.h>
#include <task_supervisor/plugins/mapping_handler.h>
#include <ros_utils/ros_utils.h>  //For loadParams function contents

PLUGINLIB_EXPORT_CLASS(task_supervisor::MappingHandler, task_supervisor::TaskHandler);

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
  // return true;
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
  // Set path to save file
  std::string launch_args = " map_topic:=" + p_map_topic_;
  
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
  ROS_INFO("[%s] Saving map %s", name_.c_str(), map_name.size() != 0 ? ("to" + map_name).c_str() : "");
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
        if (!map_splitter_srv.call(srv))
        {
          ROS_ERROR("[%s] map splitting service failed to run", name_.c_str());
          stopLaunch(map_splitter_id);
          return false;
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

  // Run mapping asynchronously
  mapping_launch_id_ = startLaunch(p_mapping_launch_package_, p_mapping_launch_file_, "");
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

  // Loop until save callback is called
  ros::Rate r(p_loop_rate_);
  while (!saved_)
  {
    // Check if cancellation has been called
    if (!isTaskActive())
    {
      ROS_INFO("[%s] Waiting for mapping thread to exit", name_.c_str());
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

    r.sleep();
  }


  if(p_orb_slam_)
  {
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


  ros::ServiceServer serv_status_ = nh_handler_.advertiseService("status", &MappingHandler::onStatus, this);
  ros::ServiceServer serv_save_ = nh_handler_.advertiseService("save_map", &MappingHandler::onSaveServiceCall, this);
  ros::ServiceServer serv_save_async_ = nh_handler_.advertiseService("save_map_async", &MappingHandler::onAsyncSave, this);
  
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
  
  param_loader.get_optional("orb_slam", p_orb_slam_, false);
  param_loader.get_optional("rgb_color_topic", p_rgb_color_topic_, std::string("/camera/rgb/image_raw" ));
  param_loader.get_optional("rgbd_depth_topic", p_rgbd_depth_topic_, std::string("/camera/depth_registered/image_raw"));
  param_loader.get_optional("rgbd_camera_info", p_rgbd_camera_info_, std::string("/camera/rgb/camera_info"));
  
  
  param_loader.get_required("mapping_launch_package", p_mapping_launch_package_);
  param_loader.get_required("mapping_launch_file", p_mapping_launch_file_);
  param_loader.get_required("mapping_launch_nodes", p_mapping_launch_nodes_);

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
    health_check_pub_ = nh_handler_.advertise<movel_seirios_msgs::Reports>("/task_supervisor/health_report", 1);
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
  bool healthy = launchStatus(mapping_launch_id_);
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
      movel_seirios_msgs::Reports report;
      report.header.stamp = ros::Time::now();
      report.handler = "mapping_handler";
      report.task_type = task_type_;
      report.healthy = false;
      report.message = "some mapping nodes are not running";
      health_check_pub_.publish(report);

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
}  // namespace task_supervisor
