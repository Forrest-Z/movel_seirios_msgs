#include <movel_zone_coverage/zone_coverage_server.h>

ZoneCoverageServer::ZoneCoverageServer(ros::NodeHandle nh) : tf_listener_(tf_buffer_), nh_private_("~")
{
  nh_ = nh;

  if (!loadParams())
  {
    ROS_FATAL("[zone_coverage_server] Error during parameter loading. Shutting down.");
    return;
  }
  ROS_INFO("[zone_coverage_server] All parameters loaded. Launching.");

  coverage_percentage_publisher_ =
      std::make_shared<ros::Publisher>(nh_private_.advertise<std_msgs::Float64>("coverage_percentage", 1));
  coverage_cell_update_publisher_ =
      std::make_shared<ros::Publisher>(nh_private_.advertise<movel_seirios_msgs::PointArray>("cell_update", 1));

  map_ = std::make_shared<costmap_2d::Costmap2DROS>("zone_coverage_map", tf_buffer_);

  redis_client_ = std::make_shared<ZoneCoverageRedisClient>(redis_host, redis_port, redis_timeout);

  current_active_task_ = "";

  ros::Rate r(20.0);
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
}

ZoneCoverageServer::~ZoneCoverageServer()
{
}

void ZoneCoverageServer::shutdownHandler()
{
  ROS_INFO("[zone_coverage_server] gracefully stopping zone coverage jobs");
  for (std::map<std::string, WorkerPtr>::iterator it = workers_.begin(); it != workers_.end(); ++it)
  {
    if (it->second->getStatus() != WorkerStatus::FINISHED)
      it->second->stopWorker();
    
    while (it->second->getStatus() != WorkerStatus::FINISHED)
      ;
  }

  worker_threads_.clear();
  workers_.clear();

  ROS_INFO("[zone_coverage_server] all zone coverage jobs stopped");
}

bool ZoneCoverageServer::loadParams()
{
  XmlRpc::XmlRpcValue robot_footprint_param;

  if (!nh_.getParam("/zone_coverage_server/zone_coverage_map/footprint", robot_footprint_param))
  {
    ROS_FATAL("[zone_coverage_server] cannot load parameter /move_base/local_costmap/footprint");
    return false;
  }

  if (robot_footprint_param.getType() != XmlRpc::XmlRpcValue::Type::TypeArray)
  {
    ROS_FATAL("[zone_coverage_server] robot_footprint parameter type is different from what is expected");
    return false;
  }

  for (int i = 0; i < robot_footprint_param.size(); ++i)
  {
    geometry_msgs::Point p;
    p.x = double(robot_footprint_param[i][0]);
    p.y = double(robot_footprint_param[i][1]);
    robot_footprint_.push_back(p);
  }

  return true;
}

bool ZoneCoverageServer::startServiceCb(movel_seirios_msgs::StartZoneCoverageStats::Request& req,
                                        movel_seirios_msgs::StartZoneCoverageStats::Response& res)
{
  if (workers_.size() > 0 && hasRunningWorkers())
  {
    res.success = false;
    res.message = "A zone coverage task is still active. Pause or stop the current task before starting a new one";
    ROS_ERROR("[zone_coverage_server] %s", res.message.c_str());
    return true;
  }

  if (workers_.find(req.task_id) != workers_.end())
  {
    res.success = false;
    res.message = "A task with the same taskId already started.";
    ROS_ERROR("[zone_coverage_server] %s", res.message.c_str());
    return true;
  }

  WorkerPtr& new_worker = createWorker(req.task_id, req.zone_polygon);
  worker_threads_[req.task_id] = std::make_unique<std::thread>([&]() { new_worker->workerThread(); });
  new_worker->startWorker();

  current_active_task_ = req.task_id;

  res.success = true;
  return true;
}

bool ZoneCoverageServer::resumeServiceCb(movel_seirios_msgs::StartZoneCoverageStats::Request& req,
                                         movel_seirios_msgs::StartZoneCoverageStats::Response& res)
{
  if (workers_.size() > 0 && hasRunningWorkers())
  {
    res.success = false;
    res.message = "A zone coverage task is still active. Pause or stop the current task before resuming another task";
    ROS_ERROR("[zone_coverage_server] %s", res.message.c_str());
    return true;
  }

  if (workers_.find(req.task_id) == workers_.end())
  {
    res.success = false;
    res.message = "A task with such taskId doesn't exist. Please start a new one.";
    ROS_ERROR("[zone_coverage_server] %s", res.message.c_str());
    return true;
  }

  WorkerPtr& worker = workers_.at(req.task_id);
  worker->startWorker();

  current_active_task_ = req.task_id;

  res.success = true;
  return true;
}

bool ZoneCoverageServer::stopServiceCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  if (!hasRunningWorkers())
  {
    res.success = false;
    res.message = "Nothing to stop, no active tasks";
    ROS_ERROR("[zone_coverage_server] %s", res.message.c_str());
    return true;
  }

  WorkerPtr& worker = workers_.at(current_active_task_);
  worker->stopWorker();

  while (worker->getStatus() != WorkerStatus::FINISHED)
    ;

  worker_threads_.erase(current_active_task_);
  workers_.erase(current_active_task_);

  current_active_task_ = "";

  res.success = true;
  return true;
}

bool ZoneCoverageServer::pauseServiceCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  if (!hasRunningWorkers())
  {
    res.success = false;
    res.message = "Nothing to pause, no active tasks";
    ROS_ERROR("[zone_coverage_server] %s", res.message.c_str());
    return true;
  }

  WorkerPtr& worker = workers_.at(current_active_task_);
  worker->pauseWorker();

  current_active_task_ = "";

  res.success = true;
  return true;
}

WorkerPtr& ZoneCoverageServer::createWorker(const std::string& task_id,
                                            const std::vector<geometry_msgs::Point>& zone_polygon)
{
  workers_[task_id] = std::make_unique<ZoneCoverageWorker>(
      task_id, zone_polygon, map_, redis_client_, coverage_percentage_publisher_, coverage_cell_update_publisher_);
  workers_[task_id]->setRobotFootprintPolygon(robot_footprint_);
  return workers_.at(task_id);
}

WorkerPtr& ZoneCoverageServer::createWorker(const std::string& task_id,
                                            const std::vector<geometry_msgs::Point>& zone_polygon,
                                            const std::vector<geometry_msgs::Point>& robot_footprint)
{
  workers_[task_id] = std::make_unique<ZoneCoverageWorker>(
      task_id, zone_polygon, map_, redis_client_, coverage_percentage_publisher_, coverage_cell_update_publisher_);
  workers_[task_id]->setRobotFootprintPolygon(robot_footprint);
  return workers_.at(task_id);
}

WorkerPtr& ZoneCoverageServer::createWorker(const std::string& task_id,
                                            const std::vector<geometry_msgs::Point>& zone_polygon,
                                            const double& robot_radius)
{
  workers_[task_id] = std::make_unique<ZoneCoverageWorker>(
      task_id, zone_polygon, map_, redis_client_, coverage_percentage_publisher_, coverage_cell_update_publisher_);
  workers_[task_id]->setRobotFootprintRadius(robot_radius);
  return workers_.at(task_id);
}

bool ZoneCoverageServer::hasRunningWorkers()
{
  for (std::map<std::string, WorkerPtr>::iterator it = workers_.begin(); it != workers_.end(); ++it)
  {
    if (it->second->getStatus() == WorkerStatus::RUNNING)
      return true;
  }
  return false;
}