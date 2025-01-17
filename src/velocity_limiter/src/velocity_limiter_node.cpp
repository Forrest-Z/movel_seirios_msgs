#include <velocity_limiter/velocity_limiter_node.h>
#include <movel_hasp_vendor/license.h>

using namespace sw::redis;

VelocityLimiterNode::VelocityLimiterNode()
  : nh_private_("~")
  , is_autonomous_safety_enabled_(true)
  , is_teleop_safety_enabled_(true)
  , current_profile_("")
  , first_cloud_received_(false)
  , first_vel_received_(false)
  , is_stopped_(false)
  , is_teleop_velocity_overridden_(false)
{
  ros::Time::waitForValid();

  if (!loadParams())
  {
    ROS_FATAL("[velocity_limiter] Error during parameter loading. Shutting down.");
    return;
  }

  ROS_INFO("[velocity_limiter] All parameters loaded. Launching.");

  setupTopics();

  if (!computeVelocityGrids())
  {
    ROS_FATAL("[velocity_limiter] Error during velocity grid loading. Shutting down.");
    return;
  }
  autonomous_velocity_grid_ = p_velocity_grid_map_[current_profile_];
  safe_teleop_velocity_grid_ = p_velocity_grid_map_[p_safe_teleop_limit_set_];

  // redis
  opts_.host = redis_host_;
  opts_.port = stoi(redis_port_);
  opts_.socket_timeout = std::chrono::milliseconds(socket_timeout_);

  redis_autonomous_velo_lim_key_ = "autonomous_safety_enabled";
  redis_teleop_velo_lim_key_ = "teleop_safety_enabled";

  auto redis = Redis(opts_);
  auto sub = redis.subscriber();

  auto val_auto = redis.get(redis_autonomous_velo_lim_key_);
  if(*val_auto=="true")
    is_autonomous_safety_enabled_ = true;
  else if(*val_auto=="false")
    is_autonomous_safety_enabled_ = false;

  auto val_teleop = redis.get(redis_teleop_velo_lim_key_);
  if(*val_teleop=="true")
    is_teleop_safety_enabled_ = true;
  else if(*val_teleop=="false")
    is_teleop_safety_enabled_ = false;

  ros::Rate r(20.0);
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
    updater_.update();
  }
}

VelocityLimiterNode::~VelocityLimiterNode()
{
}

bool VelocityLimiterNode::loadParams()
{
  ros_utils::ParamLoader loader(nh_private_);

  loader.get_required("base_frame", p_base_frame_);
  loader.get_required("merging_frame", p_merging_frame_);
  loader.get_required("low_pass_gain", p_low_pass_gain_);
  loader.get_required("publish_pcl", p_publish_pcl_);
  loader.get_required("cloud_persistence", p_cloud_persistence_);
  loader.get_required("initial_limit_set", p_initial_limit_set_);
  loader.get_required("safe_teleop_limit_set", p_safe_teleop_limit_set_);
  loader.get_required("grid_resolution", p_grid_resolution_);
  loader.get_required("stop_timeout", p_stop_timeout_);
  // loader.get_required("start_enabled", p_start_enabled_);
  // loader.get_required("start_teleop_enabled", p_start_teleop_enabled_);
  loader.get_required("obstruction_threshold", p_obstruction_threshold_);
  // is_autonomous_safety_enabled_ = p_start_enabled_;
  // is_teleop_safety_enabled_ = p_start_teleop_enabled_;

  // redis
  loader.get_required("redis_host", redis_host_);
  loader.get_required("redis_port", redis_port_);
  loader.get_required("socket_timeout", socket_timeout_);

  if (!loadLimitSetMap(p_limit_set_map_))
    return false;
  if (!isValidLimitSetMap(p_limit_set_map_))
    return false;
  autonomous_limit_set_ = p_limit_set_map_[p_initial_limit_set_];
  current_profile_ = p_initial_limit_set_;
  safe_teleop_limit_set_ = p_limit_set_map_[p_safe_teleop_limit_set_];
  

  return loader.params_valid();
}

void VelocityLimiterNode::setupTopics()
{
  autonomous_velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel_mux/autonomous", 1, &VelocityLimiterNode::onAutonomousVelocity, this);
  teleop_velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel_mux/teleop/keyboard", 1, &VelocityLimiterNode::onTeleopVelocity, this);
  costmap_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/costmap_node/costmap/costmap", 1, &VelocityLimiterNode::onCostmap, this);
  clicked_point_sub_ =
      nh_.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, &VelocityLimiterNode::onClickedPoint, this);

  autonomous_velocity_limited_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/capped", 1);
  teleop_velocity_limited_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/teleop/capped", 1);
  is_teleop_velocity_overridden_pub_ = nh_.advertise<std_msgs::Bool>("/cmd_vel_mux/teleop/keyboard/overridden", 1);
  velocity_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/velocity_grid", 1, true);
  velocity_frontiers_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("/velocity_frontiers", 1);
  merged_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud/persisted", 1);
  task_pause_pub_ = nh_.advertise<std_msgs::Bool>("/universal_handler/pause", 1);
  stopped_time_pub_ = nh_.advertise<std_msgs::Float64>("/velocity_limiter/stopped_time", 1);

  enable_srv_ = nh_.advertiseService("/enable_velocity_limiter", &VelocityLimiterNode::onEnableAutonomousSafety, this);
  enable_safe_teleop_srv_ = nh_.advertiseService("/enable_safe_teleop", &VelocityLimiterNode::onEnableTeleopSafety, this);
  switch_limit_set_srv_ = nh_.advertiseService("/switch_limit_set", &VelocityLimiterNode::onSwitchLimitSet, this);
  publish_zones_srv_ = nh_.advertiseService("/publish_limit_zones", &VelocityLimiterNode::onPublishZones, this);
  publish_grid_srv_ = nh_.advertiseService("/publish_velocity_grid", &VelocityLimiterNode::onPublishGrid, this);
  safe_teleop_checker = nh_.advertiseService("/check_safe_teleop", &VelocityLimiterNode::onCheckTeleopSafety, this);
  safe_autonomy_checker = nh_.advertiseService("/check_safe_autonomy", &VelocityLimiterNode::onCheckAutonomousSafety, this);

  autonomous_safety_timer_ = nh_.createTimer(ros::Duration(p_stop_timeout_), &VelocityLimiterNode::safeAutonomyTimerCb, this);
  autonomous_safety_timer_.stop();

  updater_.setHardwareID("Velocity limiter");
  updater_.add("Node state", this, &VelocityLimiterNode::nodeState);
  updater_.add("Velocity limit enabled", this, &VelocityLimiterNode::limitEnabled);
  updater_.add("Safe teleop enabled", this, &VelocityLimiterNode::safeTeleopEnabled);
  updater_.add("Current profile", this, &VelocityLimiterNode::profile);
  updater_.add("Time since last cloud", this, &VelocityLimiterNode::timeLastCloud);
  updater_.add("Time since last velocity", this, &VelocityLimiterNode::timeLastVel);
}

/**
 * Check whether the list of zones is valid.
 *
 * @param zone_list the list of zones being checked.
 * @return whether the list of zones is valid.
 */
bool VelocityLimiterNode::isValidZoneList(const std::vector<Zone>& zone_list)
{
  std::map<ZoneId, int> zone_type_counter;
  for (int i = 0; i < zone_list.size(); ++i)
  {
    if (zone_type_counter.find(zone_list[i].id) == zone_type_counter.end())
    {
      zone_type_counter[zone_list[i].id] = 1;
    }
    else
    {
      ROS_FATAL_STREAM("Invalid zone list: multiple instances of " << zone_list[i].id << " (should be unique)");
      return false;
    }
  }
  return true;
}

/**
 * Load the map of limit set name and linit set.
 *
 * @param limit_set_map store the resultant map.
 * @return whether it succeed.
 * @see buildLimitSet
 */
bool VelocityLimiterNode::loadLimitSetMap(std::map<std::string, Set>& limit_set_map)
{
  XmlRpc::XmlRpcValue limit_set_map_param;

  if (!nh_private_.getParam("/velocity_limiter/zone_sets", limit_set_map_param))
  {
    ROS_ERROR("[velocity_limiter] Failed to read zone sets on param server");
    return false;
  }

  if (!checkType(limit_set_map_param, XmlRpc::XmlRpcValue::TypeStruct, "zone_sets"))
    return false;
  for (auto& kv : limit_set_map_param)
  {
    ROS_INFO_STREAM("[velocity_limiter] Loading set " << kv.first);
    Set set;
    std::string path = "/velocity_limiter/zone_sets/" + static_cast<std::string>(kv.first);
    set = YamlUtils::required<Set>(path);
    if (!limiter_.buildLimitSet(set))
      return false;

    limit_set_map[kv.first] = set;
    ROS_INFO_STREAM("[velocity_limiter] Loaded set " << kv.first);
  }
  return true;
}
// if (!loadCriticalFrontier(p_critical_frontier_)) return false;
// if (!loadZoneList(p_zone_list_)) return false;
// if (!isValidZoneList(p_zone_list_)) return false;

/**
 * Check whether the limit set map is valid.
 *
 * @param limit_set_map the map being checked.
 * @return whether the limit set map is valid.
 * @see isValidZoneList
 */
bool VelocityLimiterNode::isValidLimitSetMap(std::map<std::string, Set>& limit_set_map)
{
  bool initial_set_found = false;
  for (auto& kv : limit_set_map)
  {
    if (!isValidZoneList(kv.second.zone_list))
      return false;
    if (kv.first == p_initial_limit_set_)
      initial_set_found = true;
  }
  if (!initial_set_found)
  {
    ROS_ERROR_STREAM("Set " << p_initial_limit_set_ << " not found in provided sets");
    return false;
  }
  return true;
}

/**
 * Check whether the limit set map is valid.
 *
 * @return whether it secceed.
 */
bool VelocityLimiterNode::computeVelocityGrids()
{
  for (auto set : p_limit_set_map_)
  {
    ROS_INFO_STREAM("[velocity_limiter] Computing velocity grids for limit set: " << set.first);
    VelocityGrid velocity_grid;
    if (!velocity_grid.load(set.second.zone_list, p_grid_resolution_))
    {
      ROS_FATAL_STREAM("Failed to compute velocity grids for limit set: " << set.first);
      return false;
    }
    p_velocity_grid_map_.insert(std::pair<std::string, VelocityGrid>(set.first, velocity_grid));
  }

  ROS_INFO("[velocity_limiter] Computed all velocity grids");
  return true;

  // velocity_grid_.load(limit_set_.zone_list, p_grid_resolution_, limit_set_.critical_frontier);
  // velocity_grid_pub_.publish(velocity_grid_.toOccupancyGrid(LINEAR_POSITIVE_X));
}

/**
 * Update the velocity limit values based on the cloud received.
 *
 * @param cloud the filtered cloud data.
 * @see velocity_grid::getVelocityLimit
 */
void VelocityLimiterNode::updateVelocityLimits(VelocityLimit& velocity_limit, VelocityGrid& velocity_grid, const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  VelocityLimit velocity_limit_new;      // = velocity_grid_.getMaxVelocityLimit();
  VelocityLimit velocity_limit_current;  // = velocity_grid_.getMaxVelocityLimit();
  for (const auto& point : cloud.points)
  {
    if (!velocity_grid.getVelocityLimit(point.x, point.y, velocity_limit_current))
      continue;

    velocity_limit_new.linear.positive.x =
        velocity_limit_current.linear.positive.x < velocity_limit_new.linear.positive.x ?
            velocity_limit_current.linear.positive.x :
            velocity_limit_new.linear.positive.x;
    velocity_limit_new.linear.negative.x =
        velocity_limit_current.linear.negative.x < velocity_limit_new.linear.negative.x ?
            velocity_limit_current.linear.negative.x :
            velocity_limit_new.linear.negative.x;
    velocity_limit_new.angular.positive.z =
        velocity_limit_current.angular.positive.z < velocity_limit_new.angular.positive.z ?
            velocity_limit_current.angular.positive.z :
            velocity_limit_new.angular.positive.z;
    velocity_limit_new.angular.negative.z =
        velocity_limit_current.angular.negative.z < velocity_limit_new.angular.negative.z ?
            velocity_limit_current.angular.negative.z :
            velocity_limit_new.angular.negative.z;
  }
  velocity_limit = velocity_limit_new;
}

bool VelocityLimiterNode::onPublishZones(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  Set *current_set;
  if (is_teleop_safety_enabled_)
  {
    current_set = &safe_teleop_limit_set_;
  }
  else
  {
    current_set = &autonomous_limit_set_;
  }

  resp.success = true;
  for (auto& zone : current_set->zone_list)
  {
    for (auto& frontier : zone.frontier_list)
    {
      velocity_frontiers_pub_.publish(limiter_.toPolygonMsg(frontier, p_base_frame_));
    }
  }
  return true;
}

bool VelocityLimiterNode::onPublishGrid(movel_seirios_msgs::Uint8::Request& req,
                                        movel_seirios_msgs::Uint8::Response& resp)
{
  resp.success = true;

  VelocityGrid *velocity_grid;
  if (is_teleop_safety_enabled_)
  {
    velocity_grid = &safe_teleop_velocity_grid_;
  }
  else
  {
    velocity_grid = &autonomous_velocity_grid_;
  }

  switch (req.input)
  {
    case LINEAR_POSITIVE_X:
      velocity_grid_pub_.publish(velocity_grid->toOccupancyGrid(LINEAR_POSITIVE_X, p_base_frame_));
      break;
    case LINEAR_NEGATIVE_X:
      velocity_grid_pub_.publish(velocity_grid->toOccupancyGrid(LINEAR_NEGATIVE_X, p_base_frame_));
      break;
    case LINEAR_POSITIVE_Y:
      velocity_grid_pub_.publish(velocity_grid->toOccupancyGrid(LINEAR_POSITIVE_Y, p_base_frame_));
      break;
    case LINEAR_NEGATIVE_Y:
      velocity_grid_pub_.publish(velocity_grid->toOccupancyGrid(LINEAR_NEGATIVE_Y, p_base_frame_));
      break;
    case LINEAR_POSITIVE_Z:
      velocity_grid_pub_.publish(velocity_grid->toOccupancyGrid(LINEAR_POSITIVE_Z, p_base_frame_));
      break;
    case LINEAR_NEGATIVE_Z:
      velocity_grid_pub_.publish(velocity_grid->toOccupancyGrid(LINEAR_NEGATIVE_Z, p_base_frame_));
      break;
    case ANGULAR_POSITIVE_X:
      velocity_grid_pub_.publish(velocity_grid->toOccupancyGrid(ANGULAR_POSITIVE_X, p_base_frame_));
      break;
    case ANGULAR_NEGATIVE_X:
      velocity_grid_pub_.publish(velocity_grid->toOccupancyGrid(ANGULAR_NEGATIVE_X, p_base_frame_));
      break;
    case ANGULAR_POSITIVE_Y:
      velocity_grid_pub_.publish(velocity_grid->toOccupancyGrid(ANGULAR_POSITIVE_Y, p_base_frame_));
      break;
    case ANGULAR_NEGATIVE_Y:
      velocity_grid_pub_.publish(velocity_grid->toOccupancyGrid(ANGULAR_NEGATIVE_Y, p_base_frame_));
      break;
    case ANGULAR_POSITIVE_Z:
      velocity_grid_pub_.publish(velocity_grid->toOccupancyGrid(ANGULAR_POSITIVE_Z, p_base_frame_));
      break;
    case ANGULAR_NEGATIVE_Z:
      velocity_grid_pub_.publish(velocity_grid->toOccupancyGrid(ANGULAR_NEGATIVE_Z, p_base_frame_));
      break;
    case 12:
      velocity_grid_pub_.publish(velocity_grid->toOccupancyGrid(12, p_base_frame_));
      break;
    default:
      resp.success = false;
      return false;
  }
  return true;
}

/**
 * Check whether the cloud is outdated given the persistence.
 *
 * @param cloud the unfiltered cloud data.
 * @return whether the cloud is outdated given the persistence.
 */
bool VelocityLimiterNode::isCloudOutdated(const sensor_msgs::PointCloud2& cloud)
{
  ros::Time now = ros::Time::now();
  // ROS_INFO("[velocity_limiter] dt: %.4f", (now - pcl_conversions::fromPCL(cloud.header).stamp).toSec());

  return (now - cloud.header.stamp).toSec() > p_cloud_persistence_;
}

/**
 * Update cloud_buffer_ with the cloud data that are not outdated.
 *
 * @param cloud the filtered cloud data.
 */
void VelocityLimiterNode::updateCloudBuffer(sensor_msgs::PointCloud2 new_cloud)
{
  sensor_msgs::PointCloud2 cloud_merging_frame;
  try
  {
    pcl_ros::transformPointCloud(p_merging_frame_, new_cloud, cloud_merging_frame, tf_listener_);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("[velocity_limiter] %s", ex.what());
  }

  cloud_buffer_.push_back(cloud_merging_frame);

  cloud_buffer_.erase(std::remove_if(cloud_buffer_.begin(), cloud_buffer_.end(),
                                     boost::bind(&VelocityLimiterNode::isCloudOutdated, this, _1)),
                      cloud_buffer_.end());
}

void VelocityLimiterNode::safeAutonomyTimerCb(const ros::TimerEvent& event)
{
  autonomous_safety_timer_.stop();

  if (!is_stopped_ || !is_stopped_during_autonomy_)
  {
    ROS_INFO("[velocity_limiter] Timer is up, but not pausing task");
    is_stopped_during_autonomy_ = false;
    stopped_time_pub_.publish(std_msgs::Float64{});
    return;
  }

  double dt = (ros::Time::now() - t_stopped_).toSec();
  ROS_INFO("[velocity_limiter] We have been stopped for %.2lf s, pausing task", dt);
  
  std_msgs::Bool pause_msg;
  pause_msg.data = true;
  task_pause_pub_.publish(pause_msg);

  std_msgs::Float64 stopped_time;
  stopped_time.data = dt;
  stopped_time_pub_.publish(stopped_time);
}

void VelocityLimiterNode::onAutonomousVelocity(const geometry_msgs::Twist::ConstPtr& velocity)
{
  if (!first_vel_received_)
    first_vel_received_ = true;
  last_vel_time_ = ros::Time::now();

  geometry_msgs::Twist velocity_limited;
  limiter_.limitVelocity(*velocity, velocity_limited, autonomous_velocity_limit_);
  if (is_autonomous_safety_enabled_)
  {
    autonomous_velocity_limited_pub_.publish(velocity_limited);

    // check for stoppage
    std_msgs::Float64 stopped_time;
    double dx = fabs(velocity->linear.x - velocity_limited.linear.x);
    double dz = fabs(velocity->angular.z - velocity_limited.angular.z);
    bool stopped_linear = dx > 1.0e-3 && fabs(velocity_limited.linear.x) < 1.0e-3;
    bool stopped_angular = dz > 1.0e-3 && fabs(velocity_limited.angular.z) < 1.0e-3;
    if (stopped_linear || stopped_angular)
    {
      if (!is_stopped_)
      {
        ROS_INFO("[velocity_limiter] Safe autonomy stopped the robot motion. v_lim: (%.5f, %.5f), v: (%.5f, %.5f)",
                 velocity_limited.linear.x, velocity_limited.angular.z, velocity->linear.x, velocity->angular.z);
        is_stopped_ = true;
        t_stopped_ = ros::Time::now();
        is_stopped_during_autonomy_ = true;
        autonomous_safety_timer_.start();
      }
    }
    else
    {
      if (is_stopped_)
      {
        ROS_INFO("[velocity_limiter] Safe autonomy resumed the robot motion. v_lim: (%.5f, %.5f), v: (%.5f, %.5f)",
                 velocity_limited.linear.x, velocity_limited.angular.z, velocity->linear.x, velocity->angular.z);
      }

      is_stopped_ = false;
      stopped_time_pub_.publish(stopped_time);   // pub zero
      is_stopped_during_autonomy_ = false;
      autonomous_safety_timer_.stop();
    }
  }
  else
  {
    autonomous_velocity_limited_pub_.publish(*velocity);
  }
}

void VelocityLimiterNode::onTeleopVelocity(const geometry_msgs::Twist::ConstPtr& velocity)
{
  if (!first_vel_received_)
    first_vel_received_ = true;
  last_vel_time_ = ros::Time::now();

  geometry_msgs::Twist velocity_limited;
  limiter_.limitVelocity(*velocity, velocity_limited, safe_teleop_velocity_limit_);
  if (is_teleop_safety_enabled_)
  {
    teleop_velocity_limited_pub_.publish(velocity_limited);

    // check for stoppage
    double dx = fabs(velocity->linear.x - velocity_limited.linear.x);
    if (dx > 0.01 && fabs(velocity_limited.linear.x) < 1.0e-3)
    {
      if (!is_stopped_)
      {
        ROS_INFO("[velocity_limiter] Teleop safety stop");
        is_stopped_ = true;
        t_stopped_ = ros::Time::now();
      }
      is_teleop_velocity_overridden_ = true;
    }
    else
    {
      is_teleop_velocity_overridden_ = false;
      is_stopped_ = false;
    }
  }
  else
  {
    is_teleop_velocity_overridden_ = false;
    teleop_velocity_limited_pub_.publish(*velocity);
  }

  std_msgs::Bool status;
  status.data = is_teleop_velocity_overridden_;
  is_teleop_velocity_overridden_pub_.publish(status);
}

void VelocityLimiterNode::onCostmap(const nav_msgs::OccupancyGrid::ConstPtr& costmap)
{
  if (!first_cloud_received_)
    first_cloud_received_ = true;
  last_cloud_time_ = ros::Time::now();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);
  for(size_t i = 0; i < costmap->info.width * costmap->info.height; i++)
  {
    if(costmap->data[i] >= p_obstruction_threshold_)
    {
      pcl::PointXYZ p;
      p.x = (i % costmap->info.width) * costmap->info.resolution + costmap->info.origin.position.x;
      p.y = (i / costmap->info.width) * costmap->info.resolution + costmap->info.origin.position.y;
      p.z = 0.0;
      cloud_raw->push_back(p);
    }
  }

  sensor_msgs::PointCloud2 cloud_merged;
  pcl::toROSMsg(*cloud_raw, cloud_merged);
  cloud_merged.header.frame_id = costmap->header.frame_id;

  sensor_msgs::PointCloud2 cloud_base;
  int x_idx = pcl::getFieldIndex (cloud_merged, "x");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (x_idx != -1) 
  {
    try
    {
        pcl_ros::transformPointCloud(p_base_frame_, cloud_merged, cloud_base, tf_listener_);
        if (p_publish_pcl_)
          merged_cloud_pub_.publish(cloud_base);

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(cloud_base, pcl_pc2);
      
        int x_idx_x = pcl::getFieldIndex (pcl_pc2, "x");
        if (x_idx_x != -1) 
            pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("[velocity_limiter] %s", ex.what());
    }
  } 
  // ROS_INFO("[velocity_limiter] Cloud size: %lu", cloud->points.size());

  pcl::PointCloud<pcl::PointXYZ>::Ptr autonomous_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  BoxHull autonomous_boundary = autonomous_velocity_grid_.getBoundary();
  pcl::CropBox<pcl::PointXYZ> autonomousBoxFilter;
  autonomousBoxFilter.setMin(Eigen::Vector4f(autonomous_boundary.bottom_right.x(), autonomous_boundary.bottom_right.y(), -10.0, 1.0));
  // autonomousBoxFilter.setMin(Eigen::Vector4f(1., -1., -10.0, 1.0));
  autonomousBoxFilter.setMax(Eigen::Vector4f(autonomous_boundary.top_left.x(), autonomous_boundary.top_left.y(), 10.0, 1.0));
  // autonomousBoxFilter.setMax(Eigen::Vector4f(2., 0., 10.0, 1.0));
  autonomousBoxFilter.setInputCloud(cloud);
  autonomousBoxFilter.filter(*autonomous_cloud_filtered);

  pcl::PointCloud<pcl::PointXYZ>::Ptr safe_teleop_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  BoxHull safe_teleop_boundary = safe_teleop_velocity_grid_.getBoundary();
  pcl::CropBox<pcl::PointXYZ> safeTeleopBoxFilter;
  safeTeleopBoxFilter.setMin(Eigen::Vector4f(safe_teleop_boundary.bottom_right.x(), safe_teleop_boundary.bottom_right.y(), -10.0, 1.0));
  // safeTeleopBoxFilter.setMin(Eigen::Vector4f(1., -1., -10.0, 1.0));
  safeTeleopBoxFilter.setMax(Eigen::Vector4f(safe_teleop_boundary.top_left.x(), safe_teleop_boundary.top_left.y(), 10.0, 1.0));
  // safeTeleopBoxFilter.setMax(Eigen::Vector4f(2., 0., 10.0, 1.0));
  safeTeleopBoxFilter.setInputCloud(cloud);
  safeTeleopBoxFilter.filter(*safe_teleop_cloud_filtered);

  // ROS_INFO("[velocity_limiter] Unfiltered size: %d, %d", cloud->width, cloud->height);
  // ROS_INFO("[velocity_limiter] Filtered size: %d, %d", cloud_filtered->width, cloud_filtered->height);

  updateVelocityLimits(autonomous_velocity_limit_, autonomous_velocity_grid_, *autonomous_cloud_filtered);
  updateVelocityLimits(safe_teleop_velocity_limit_, safe_teleop_velocity_grid_, *safe_teleop_cloud_filtered);
}

/**
 * Publish the velocity limit values of the clicked point.
 * see velocity_grid::getOrigin
 * see velocity_grid::getVelocityLimit
 */
void VelocityLimiterNode::onClickedPoint(const geometry_msgs::PointStamped::ConstPtr& point)
{
  VelocityLimit limit;
  ROS_INFO("[velocity_limiter] Clicked on the point: (%.2f, %.2f)", point->point.x, point->point.y);

  double origin_x;
  double origin_y;

  VelocityGrid *velocity_grid;
  if (is_teleop_safety_enabled_)
  {
    velocity_grid = &safe_teleop_velocity_grid_;
  }
  else
  {
    velocity_grid = &autonomous_velocity_grid_;
  }

  velocity_grid->getOrigin(origin_x, origin_y);
  velocity_grid->getVelocityLimit(point->point.x + origin_x, point->point.y + origin_y, limit);
  if (limit.linear.positive.x < DBL_MAX)
    ROS_INFO("[velocity_limiter] Linear positive: %.2f", limit.linear.positive.x);
  else
    ROS_INFO("[velocity_limiter] Linear positive: DBL_MAX");
  if (limit.linear.negative.x < DBL_MAX)
    ROS_INFO("[velocity_limiter] Linear negative: %.2f", limit.linear.negative.x);
  else
    ROS_INFO("[velocity_limiter] Linear positive: DBL_MAX");
  if (limit.angular.positive.z < DBL_MAX)
    ROS_INFO("[velocity_limiter] Angular positive: %.2f", limit.angular.positive.z);
  else
    ROS_INFO("[velocity_limiter] Linear positive: DBL_MAX");
  if (limit.angular.negative.z < DBL_MAX)
    ROS_INFO("[velocity_limiter] Angular negative: %.2f", limit.angular.negative.z);
  else
    ROS_INFO("[velocity_limiter] Linear positive: DBL_MAX");
}

bool VelocityLimiterNode::onEnableTeleopSafety(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
{
  if (req.data)
  {
    ROS_INFO("[velocity_limiter] Safe teleop enabled");
    resp.message = "Safe teleop enabled";
  }
  else
  {
    ROS_INFO("[velocity_limiter] Safe teleop disabled");
    resp.message = "Safe teleop disabled";
  }

  is_teleop_safety_enabled_ = req.data;
  // resp.success = true;

  auto redis = sw::redis::Redis(opts_);
  bool success = true;
  try
  {
    if (is_teleop_safety_enabled_)
      redis.set(redis_teleop_velo_lim_key_, "true");
    else
      redis.set(redis_teleop_velo_lim_key_, "false");
  }
  catch (const sw::redis::Error& e)
  {
    ROS_FATAL_STREAM("[velocity_limiter] Failed to toggle enable/disable teleop safety using service call: " << e.what());
    success = false;
  }

  resp.success = success;
  return true;
}

bool VelocityLimiterNode::onSwitchLimitSet(movel_seirios_msgs::StringTrigger::Request& req,
                                           movel_seirios_msgs::StringTrigger::Response& resp)
{
  std::string new_profile = req.input;
  if (new_profile == current_profile_)
  {
    ROS_WARN("[velocity_limiter] Ignoring request for switch to the current limit set");
    resp.success = true;
    return true;
  }

  if (switchLimitSet(new_profile))
  {
    resp.success = true;
    return true;
  }
  else
  {
    resp.success = false;
    return false;
  }
}

bool VelocityLimiterNode::switchLimitSet(std::string new_profile)
{
  bool set_found = false;
  for (auto& kv : p_limit_set_map_)
  {
    if (kv.first == new_profile)
      set_found = true;
  }
  if (!set_found)
  {
    ROS_ERROR_STREAM("Set " << new_profile << " not found in existing sets");
    return false;
  }
  ROS_INFO_STREAM("[velocity_limiter] Set " << new_profile << " loaded");
  autonomous_limit_set_ = p_limit_set_map_[new_profile];
  autonomous_velocity_grid_ = p_velocity_grid_map_[new_profile];
  ROS_INFO("[velocity_limiter] Velocity grid loaded");
  current_profile_ = new_profile;
  return true;
}

bool VelocityLimiterNode::onEnableAutonomousSafety(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
{
  if (req.data)
  {
    ROS_WARN("[velocity_limiter] Velocity limiter enabled");
  }
  else
  {
    ROS_WARN("[velocity_limiter] Velocity limiter disabled");
  }
  is_autonomous_safety_enabled_ = req.data;

  auto redis = sw::redis::Redis(opts_);
  bool success = true;
  try
  {
    if (is_autonomous_safety_enabled_)
      redis.set(redis_autonomous_velo_lim_key_, "true");
    else
      redis.set(redis_autonomous_velo_lim_key_, "false");
  }
  catch (const sw::redis::Error& e)
  {
    ROS_FATAL_STREAM("[velocity_limiter] Failed to toggle enable/disable autonomous safety using service call: " << e.what());
    success = false;
  }

  resp.success = success;
  return true;
}

/* Sync globals with UI -get safe teleop status */
bool VelocityLimiterNode::onCheckTeleopSafety(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) 
{
  if (is_teleop_safety_enabled_) {
    res.success = true;
    res.message = "Safe teleop is enabled";
  }
  else {
    res.success = false;
    res.message = "Safe teleop not enabled";               
  }
  return true;
}

bool VelocityLimiterNode::onCheckAutonomousSafety(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) 
{
  if (is_autonomous_safety_enabled_) {
    res.success = true;
    res.message = "Safe autonomy is enabled";
  }
  else {
    res.success = false;
    res.message = "Safe autonomy is not enabled";               
  }
  return true;
}

/**
 * Diagnostic task: whether the node is running.
 */
void VelocityLimiterNode::nodeState(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Node is running.");
}
/**
 * Diagnostic task: whether the velocity limit is enabled.
 */
void VelocityLimiterNode::limitEnabled(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  if (is_autonomous_safety_enabled_)
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Enabled");
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Not enabled");
}
/**
 * Diagnostic task: whether safe teleop is enabled.
 */
void VelocityLimiterNode::safeTeleopEnabled(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  if (is_teleop_safety_enabled_)
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Safe teleop enabled");
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Safe teleop not enabled");
}
/**
 * Diagnostic task: print the current profile.
 */
void VelocityLimiterNode::profile(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, current_profile_);
}
/**
 * Diagnostic task: print the time since last cloud data was received.
 */
void VelocityLimiterNode::timeLastCloud(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  if (!first_cloud_received_)
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Cloud not received yet");
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, std::to_string((ros::Time::now() - last_cloud_time_).toSec()));
}
/**
 * Diagnostic task: print the time since last velocity data was received.
 */
void VelocityLimiterNode::timeLastVel(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  if (!first_vel_received_)
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Velocity not received yet");
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, std::to_string((ros::Time::now() - last_vel_time_).toSec()));
}

int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE
  MovelLicense ml;
  if (!ml.login())
    return 1;
#endif

  ros::init(argc, argv, "velocity_limiter");
  VelocityLimiterNode velocity_limiter;
#ifdef MOVEL_LICENSE
  ml.logout();
#endif

  return (0);
}
