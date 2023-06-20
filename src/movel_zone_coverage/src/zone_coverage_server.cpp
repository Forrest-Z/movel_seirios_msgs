#include <movel_zone_coverage/zone_coverage_server.h>

ZoneCoverageServer::ZoneCoverageServer(ros::NodeHandle nh) : tf_listener_(tf_buffer_), nh_private_("~")
{
  nh_ = nh;
  tf_buffer_.setUsingDedicatedThread(true);
  if (!loadParams())
  {
    ROS_FATAL("[ZoneCoverageServer] Error during parameter loading. Shutting down.");
    return;
  }
  ROS_INFO("[ZoneCoverageServer] All parameters loaded. Launching.");

  map_ = std::make_shared<costmap_2d::Costmap2DROS>("zone_coverage_map", tf_buffer_);

  redis_client_ = std::make_shared<ZoneCoverageRedisClient>(redis_host_, redis_port_, redis_timeout_);

  start_service_ = nh_private_.advertiseService("start", &ZoneCoverageServer::startServiceCb, this);
  resume_service_ = nh_private_.advertiseService("resume", &ZoneCoverageServer::resumeServiceCb, this);
  stop_service_ = nh_private_.advertiseService("stop", &ZoneCoverageServer::stopServiceCb, this);
  pause_service_ = nh_private_.advertiseService("pause", &ZoneCoverageServer::pauseServiceCb, this);

  coverage_percentage_publisher_ =
      std::make_shared<ros::Publisher>(nh_private_.advertise<std_msgs::Float64>("coverage_percentage", 1));
  coverage_cell_update_publisher_ =
      std::make_shared<ros::Publisher>(nh_private_.advertise<movel_seirios_msgs::PointArray>("cell_update", 1));
  status_publisher_ = std::make_shared<ros::Publisher>(nh_private_.advertise<std_msgs::String>("status", 1));

  worker_ = std::make_unique<ZoneCoverageWorker>(map_, redis_client_, coverage_percentage_publisher_,
                                                 coverage_cell_update_publisher_, status_publisher_);
  worker_thread_ = std::thread([this]() { worker_->threadFunction(); });
}

ZoneCoverageServer::~ZoneCoverageServer()
{
}

void ZoneCoverageServer::shutdownHandler()
{
  ROS_INFO("[ZoneCoverageServer] gracefully stopping zone coverage worker");
  worker_->terminate();
  worker_thread_.join();
  ROS_INFO("[ZoneCoverageServer] zone coverage worker stopped");
}

bool ZoneCoverageServer::loadParams()
{
  if (!nh_private_.hasParam("redis_host"))
  {
    ROS_FATAL("[ZoneCoverageServer] cannot load parameter redis_host");
    return false;
  }
  nh_private_.getParam("redis_host", redis_host_);

  if (!nh_private_.hasParam("redis_port"))
  {
    ROS_FATAL("[ZoneCoverageServer] cannot load parameter redis_port");
    return false;
  }
  std::string redis_port_str;
  nh_private_.getParam("redis_port", redis_port_str);
  redis_port_ = std::stoi(redis_port_str);

  if (!nh_private_.hasParam("socket_timeout"))
  {
    ROS_FATAL("[ZoneCoverageServer] cannot load parameter socket_timeout");
    return false;
  }
  nh_private_.getParam("socket_timeout", redis_timeout_);

  XmlRpc::XmlRpcValue robot_footprint_param;

  if (!nh_.getParam("/zone_coverage_server/zone_coverage_map/footprint", robot_footprint_param))
  {
    ROS_FATAL("[ZoneCoverageServer] cannot load parameter /zone_coverage_server/zone_coverage_map/footprint");
    return false;
  }

  if (robot_footprint_param.getType() != XmlRpc::XmlRpcValue::Type::TypeArray)
  {
    ROS_FATAL("[ZoneCoverageServer] robot_footprint parameter type is different from what is expected");
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
  if (!worker_->startTask(req.task_id, req.zone_polygon, robot_footprint_))
  {
    res.success = false;
    res.message = "Failed to start task " + req.task_id;
    ROS_ERROR("[ZoneCoverageServer] %s", res.message.c_str());
    return true;
  }

  res.success = true;
  return true;
}

bool ZoneCoverageServer::resumeServiceCb(movel_seirios_msgs::StartZoneCoverageStats::Request& req,
                                         movel_seirios_msgs::StartZoneCoverageStats::Response& res)
{
  if (!worker_->resumeTask(req.task_id))
  {
    res.success = false;
    res.message = "Failed to start task " + req.task_id;
    ROS_ERROR("[ZoneCoverageServer] %s", res.message.c_str());
    return true;
  }

  res.success = true;
  return true;
}

bool ZoneCoverageServer::stopServiceCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  worker_->stopCurrentTask();
  res.success = true;
  return true;
}

bool ZoneCoverageServer::pauseServiceCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  worker_->pauseCurrentTask();
  res.success = true;
  return true;
}