#include <pluginlib/class_list_macros.h>
#include <rtabmap_handler/rtabmap_handler.h>
//#include <task_supervisor/plugins/mapping_handler.h>
#include <ros_utils/ros_utils.h>  //For loadParams function contents

PLUGINLIB_EXPORT_CLASS(rtabmap_handler::RtabmapHandler, task_supervisor::TaskHandler);

namespace rtabmap_handler
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
bool RtabmapHandler::onSaveServiceCall(movel_seirios_msgs::StringTrigger::Request& req,
                                       movel_seirios_msgs::StringTrigger::Response& res)
{
  res.success = saveMap(req.input);
  saved_ = true;
  return true;
}

bool RtabmapHandler::onAsyncSave(movel_seirios_msgs::StringTrigger::Request& req,
                                       movel_seirios_msgs::StringTrigger::Response& res)
{
  res.success = saveMap(req.input);
  return true;
}

bool RtabmapHandler::onStatus(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  res.success = launchStatus(mapping_launch_id_);
  return true;
}

bool RtabmapHandler::saveMap(std::string map_name)
{
  // Set path to save file
  std::string launch_args = " map_topic:=" + p_map_topic_;
  
  try
  {
    boost::filesystem::path mySourcePath(map_name_+".db");
    boost::filesystem::path myTargetPath(map_name+".db");
    boost::filesystem::copy_file(mySourcePath, myTargetPath, boost::filesystem::copy_option::overwrite_if_exists);
  }
  catch(...)
  {
    ROS_WARN("[%s] Something went wrong while copying DB", name_.c_str());
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
bool RtabmapHandler::runMapping()
{
  ROS_INFO("[%s] Starting mapping package: %s, launch file: %s", name_.c_str(), p_mapping_launch_package_.c_str(),
           p_mapping_launch_file_.c_str());

  std::string database_args = "database_path:=" + map_name_ + ".db";
  
  size_t camera_quantity = p_camera_names_.size();
  std::string camera_names_args, camera_quantity_args, auto_args;

  if(camera_quantity >= 1 && camera_quantity <= 3)
    camera_quantity_args = " rgbd_cameras:=" + std::to_string((int)camera_quantity);
  else if(camera_quantity > 3)
    camera_quantity_args = " rgbd_cameras:=" + std::to_string((int)4);

  if(camera_quantity == 1)
    camera_names_args = " camera:=" + p_camera_names_[0];
  else if(camera_quantity == 2)
    camera_names_args = " camera1:=" + p_camera_names_[0] +
                        " camera2:=" + p_camera_names_[1];
  else if (camera_quantity == 3)
    camera_names_args = " camera1:=" + p_camera_names_[0] +
                        " camera2:=" + p_camera_names_[1] +
                        " camera3:=" + p_camera_names_[2];
  else if (camera_quantity >= 4)
    camera_names_args = " camera1:=" + p_camera_names_[0] +
                        " camera2:=" + p_camera_names_[1] +
                        " camera3:=" + p_camera_names_[2] +
                        " camera4:=" + p_camera_names_[3];

  if(p_auto_)
    auto_args = " auto:=true";

  // Run mapping asynchronously
  mapping_launch_id_ = startLaunch(p_mapping_launch_package_, p_mapping_launch_file_, database_args +
                                                                                      camera_names_args +
                                                                                      camera_quantity_args +
                                                                                      auto_args);
  if (!mapping_launch_id_)
  {
    ROS_ERROR("[%s] Failed to launch mapping launch file", name_.c_str());
    return false;
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
  while (!saved_)
  {
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

      stopLaunch(mapping_launch_id_);
      while (launchExists(mapping_launch_id_))
        ;
      ros::Duration(3.0).sleep();
      try
      {
        boost::filesystem::remove(map_name_+".db");
      }
      catch(...)
      {
        ROS_WARN("[%s] Something went wrong wwhile copying DB", name_.c_str());
      }
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

  ROS_INFO("[%s] Stopping mapping", name_.c_str());
  stopLaunch(mapping_launch_id_);
  while (launchExists(mapping_launch_id_))
    ;
  mapping_launch_id_ = 0;
  ROS_INFO("[%s] Mapping stopped", name_.c_str());

  saved_ = false;
  ros::Duration(3.0).sleep();
  try
  {
    boost::filesystem::remove(map_name_+".db");
  }
  catch(...)
  {
    ROS_WARN("[%s] Something went wrong wwhile copying DB", name_.c_str());
  }
  return true;
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
task_supervisor::ReturnCode RtabmapHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message)
{
  task_active_ = true;
  task_parsed_ = false;
  start_ = ros::Time::now();
  map_name_ = "/home/movel/.config/movel/maps/temp_rtabmap_save_";

  if(p_auto_)
  {
    cancel_pub_ = nh_handler_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
    stopped_pub_ = nh_handler_.advertise<std_msgs::Empty>("stopped", 1);
  }

  ros::Subscriber status_sub_ = nh_handler_.subscribe("/rosout",1, &RtabmapHandler::logCB, this);
  ros::ServiceServer serv_status_ = nh_handler_.advertiseService("status", &RtabmapHandler::onStatus, this);
  ros::ServiceServer serv_save_ = nh_handler_.advertiseService("save_map", &RtabmapHandler::onSaveServiceCall, this);
  ros::ServiceServer serv_save_async_ =
      nh_handler_.advertiseService("save_map_async", &RtabmapHandler::onAsyncSave, this);

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
bool RtabmapHandler::loadParams()
{
  ros_utils::ParamLoader param_loader(nh_handler_);

  param_loader.get_optional("loop_rate", p_loop_rate_, 5.0);
  param_loader.get_optional("save_timeout", p_save_timeout_, 5.0);
  param_loader.get_optional("map_topic", p_map_topic_, std::string("/map"));

  param_loader.get_required("mapping_launch_package", p_mapping_launch_package_);
  param_loader.get_required("mapping_launch_file", p_mapping_launch_file_);
  
  param_loader.get_optional("split_map", p_split_map_, false);
  param_loader.get_optional("auto", p_auto_, false);

  if (nh_handler_.hasParam("camera_names"))
    nh_handler_.getParam("camera_names", p_camera_names_);
  else
    return false;
    
  if (p_camera_names_.size() == 0)
    return false; 
  
  return param_loader.params_valid();
}

/**
 *	Setup method called once task_supervisor starts and loads all plugins (task handlers)
 */
bool RtabmapHandler::setupHandler()
{
  if (!loadParams())
    return false;
  else
  {
    health_check_pub_ = nh_handler_.advertise<movel_seirios_msgs::Reports>("/task_supervisor/health_report", 1);
    return true;
  }
}

bool RtabmapHandler::healthCheck()
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
      mapping_launch_id_ = 0;
    }
  }
  else
    failcount = 0;
  return healthy;
}

void RtabmapHandler::logCB(const rosgraph_msgs::LogConstPtr& msg)
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

}  // namespace rtabmap_handler
