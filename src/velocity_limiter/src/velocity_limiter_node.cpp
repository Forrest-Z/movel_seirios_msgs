#include <velocity_limiter/velocity_limiter_node.h>
#include <movel_hasp_vendor/license.h>

VelocityLimiterNode::VelocityLimiterNode()
  : nh_private_("~")
  , is_enabled_(true)
  , current_profile_("")
  , first_cloud_received_(false)
  , first_vel_received_(false)
  , is_stopped_(false)
  , has_goal_status_(false)
{
  ros::Time::waitForValid();

  if (!loadParams())
  {
    ROS_FATAL("Error during parameter loading. Shutting down.");
    return;
  }

  ROS_INFO("All parameters loaded. Launching.");

  setupTopics();

  if (!computeVelocityGrids())
  {
    ROS_FATAL("Error during velocity grid loading. Shutting down.");
    return;
  }
  velocity_grid_ = p_velocity_grid_map_[current_profile_];

  ros::Rate r(20.0);
  while (ros::ok())
  {
    publishTopics();
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
  loader.get_required("action_server", p_action_server_name_);
  loader.get_required("start_enabled", p_start_enabled_);
  is_enabled_ = p_start_enabled_;

  if (!loadLimitSetMap(p_limit_set_map_))
    return false;
  if (!isValidLimitSetMap(p_limit_set_map_))
    return false;
  limit_set_ = p_limit_set_map_[p_initial_limit_set_];
  current_profile_ = p_initial_limit_set_;
  
  current_operation_mode_ = "autonomous";
  is_safe_teleop_enabled_ = false;

  return loader.params_valid();
}

void VelocityLimiterNode::setupTopics()
{
  autonomous_velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel/autonomous", 1, &VelocityLimiterNode::onAutonomousVelocity, this);
  teleop_velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel/teleop", 1, &VelocityLimiterNode::onTeleopVelocity, this);
  cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/cloud", 1, &VelocityLimiterNode::onCloud, this);
  clicked_point_sub_ =
      nh_.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, &VelocityLimiterNode::onClickedPoint, this);
  std::string goal_status_topic = p_action_server_name_ + "/status";
  goal_status_sub_ = nh_.subscribe(goal_status_topic, 1, &VelocityLimiterNode::onActionStatus, this);

  velocity_limited_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel/capped", 1);
  velocity_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/velocity_grid", 1, true);
  velocity_frontiers_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("/velocity_frontiers", 1);
  merged_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud/persisted", 1);
  std::string goal_abort_topic = p_action_server_name_ + "/cancel";
  goal_abort_pub_ = nh_.advertise<actionlib_msgs::GoalID>(goal_abort_topic, 1);
  operation_mode_pub_ = nh_.advertise<velocity_limiter::OperationMode>("/operation_mode", 1);

  enable_srv_ = nh_.advertiseService("/enable_velocity_limiter", &VelocityLimiterNode::onEnableLimiter, this);
  switch_operation_mode_srv_ = nh_.advertiseService("/switch_operation_mode", &VelocityLimiterNode::onSwitchOperationMode, this);
  enable_safe_teleop_srv_ = nh_.advertiseService("/enable_safe_teleop", &VelocityLimiterNode::onEnableSafeTeleop, this);
  switch_limit_set_srv_ = nh_.advertiseService("/switch_limit_set", &VelocityLimiterNode::onSwitchLimitSet, this);
  publish_zones_srv_ = nh_.advertiseService("/publish_limit_zones", &VelocityLimiterNode::onPublishZones, this);
  publish_grid_srv_ = nh_.advertiseService("/publish_velocity_grid", &VelocityLimiterNode::onPublishGrid, this);

  updater_.setHardwareID("Velocity limiter");
  updater_.add("Node state", this, &VelocityLimiterNode::nodeState);
  updater_.add("Velocity limit enabled", this, &VelocityLimiterNode::limitEnabled);
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
    ROS_ERROR("Failed to read zone sets on param server");
    return false;
  }

  if (!checkType(limit_set_map_param, XmlRpc::XmlRpcValue::TypeStruct, "zone_sets"))
    return false;
  for (auto& kv : limit_set_map_param)
  {
    ROS_INFO_STREAM("Loading set " << kv.first);
    Set set;
    std::string path = "/velocity_limiter/zone_sets/" + static_cast<std::string>(kv.first);
    set = YamlUtils::required<Set>(path);
    if (!limiter_.buildLimitSet(set))
      return false;

    limit_set_map[kv.first] = set;
    ROS_INFO_STREAM("Loaded set " << kv.first);
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
    ROS_INFO_STREAM("Computing velocity grids for limit set: " << set.first);
    VelocityGrid velocity_grid;
    if (!velocity_grid.load(set.second.zone_list, p_grid_resolution_))
    {
      ROS_FATAL_STREAM("Failed to compute velocity grids for limit set: " << set.first);
      return false;
    }
    p_velocity_grid_map_.insert(std::pair<std::string, VelocityGrid>(set.first, velocity_grid));
  }

  ROS_INFO("Computed all velocity grids");
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
void VelocityLimiterNode::updateVelocityLimits(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  VelocityLimit velocity_limit_new;      // = velocity_grid_.getMaxVelocityLimit();
  VelocityLimit velocity_limit_current;  // = velocity_grid_.getMaxVelocityLimit();
  for (const auto& point : cloud.points)
  {
    if (!velocity_grid_.getVelocityLimit(point.x, point.y, velocity_limit_current))
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
  velocity_limit_ = velocity_limit_new;
}

bool VelocityLimiterNode::onPublishZones(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  resp.success = true;
  for (auto& zone : limit_set_.zone_list)
  {
    for (auto& frontier : zone.frontier_list)
    {
      velocity_frontiers_pub_.publish(limiter_.toPolygonMsg(frontier, p_base_frame_));
    }
  }
  return true;
}

bool VelocityLimiterNode::onPublishGrid(velocity_limiter::PublishGrid::Request& req,
                                        velocity_limiter::PublishGrid::Response& resp)
{
  resp.success = true;

  switch (req.zone_id)
  {
    case LINEAR_POSITIVE_X:
      velocity_grid_pub_.publish(velocity_grid_.toOccupancyGrid(LINEAR_POSITIVE_X, p_base_frame_));
      break;
    case LINEAR_NEGATIVE_X:
      velocity_grid_pub_.publish(velocity_grid_.toOccupancyGrid(LINEAR_NEGATIVE_X, p_base_frame_));
      break;
    case LINEAR_POSITIVE_Y:
      velocity_grid_pub_.publish(velocity_grid_.toOccupancyGrid(LINEAR_POSITIVE_Y, p_base_frame_));
      break;
    case LINEAR_NEGATIVE_Y:
      velocity_grid_pub_.publish(velocity_grid_.toOccupancyGrid(LINEAR_NEGATIVE_Y, p_base_frame_));
      break;
    case LINEAR_POSITIVE_Z:
      velocity_grid_pub_.publish(velocity_grid_.toOccupancyGrid(LINEAR_POSITIVE_Z, p_base_frame_));
      break;
    case LINEAR_NEGATIVE_Z:
      velocity_grid_pub_.publish(velocity_grid_.toOccupancyGrid(LINEAR_NEGATIVE_Z, p_base_frame_));
      break;
    case ANGULAR_POSITIVE_X:
      velocity_grid_pub_.publish(velocity_grid_.toOccupancyGrid(ANGULAR_POSITIVE_X, p_base_frame_));
      break;
    case ANGULAR_NEGATIVE_X:
      velocity_grid_pub_.publish(velocity_grid_.toOccupancyGrid(ANGULAR_NEGATIVE_X, p_base_frame_));
      break;
    case ANGULAR_POSITIVE_Y:
      velocity_grid_pub_.publish(velocity_grid_.toOccupancyGrid(ANGULAR_POSITIVE_Y, p_base_frame_));
      break;
    case ANGULAR_NEGATIVE_Y:
      velocity_grid_pub_.publish(velocity_grid_.toOccupancyGrid(ANGULAR_NEGATIVE_Y, p_base_frame_));
      break;
    case ANGULAR_POSITIVE_Z:
      velocity_grid_pub_.publish(velocity_grid_.toOccupancyGrid(ANGULAR_POSITIVE_Z, p_base_frame_));
      break;
    case ANGULAR_NEGATIVE_Z:
      velocity_grid_pub_.publish(velocity_grid_.toOccupancyGrid(ANGULAR_NEGATIVE_Z, p_base_frame_));
      break;
    case 12:
      velocity_grid_pub_.publish(velocity_grid_.toOccupancyGrid(12, p_base_frame_));
      break;
    default:
      resp.success = false;
      return false;
  }
  return true;
}

void VelocityLimiterNode::onActionStatus(actionlib_msgs::GoalStatusArray msg)
{
  has_goal_status_ = true;
  if (msg.status_list.size() > 0)
  {
    latest_goal_status_ = msg.status_list[0];
  }
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
  // ROS_INFO("dt: %.4f", (now - pcl_conversions::fromPCL(cloud.header).stamp).toSec());

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
    ROS_ERROR("%s", ex.what());
  }

  cloud_buffer_.push_back(cloud_merging_frame);

  cloud_buffer_.erase(std::remove_if(cloud_buffer_.begin(), cloud_buffer_.end(),
                                     boost::bind(&VelocityLimiterNode::isCloudOutdated, this, _1)),
                      cloud_buffer_.end());
}

void VelocityLimiterNode::onAutonomousVelocity(const geometry_msgs::Twist::ConstPtr& velocity)
{
  if (current_operation_mode_ != "autonomous")
    return;

  if (!first_vel_received_)
    first_vel_received_ = true;
  last_vel_time_ = ros::Time::now();

  geometry_msgs::Twist velocity_limited;
  limiter_.limitVelocity(*velocity, velocity_limited, velocity_limit_);
  if (is_enabled_)
  {
    velocity_limited_pub_.publish(velocity_limited);

    // check for stoppage
    double dx = fabs(velocity->linear.x - velocity_limited.linear.x);
    if (dx > 0.01 && fabs(velocity_limited.linear.x) < 1.0e-3)
    {
      if (!is_stopped_)
      {
        ROS_INFO("velo limiter stop");
        is_stopped_ = true;
        t_stopped_ = ros::Time::now();
      }
      else
      {
        double dt = (ros::Time::now() - t_stopped_).toSec();
        if (dt >= p_stop_timeout_)
        {
          ROS_INFO("we have been stopped for %5.2f s, aborting task", dt);
          if (has_goal_status_)
          {
            // should we change the stamp in the goal id?
            actionlib_msgs::GoalID goal_id = latest_goal_status_.goal_id;
            goal_abort_pub_.publish(goal_id);
          }
        }
      }
    }
    else
    {
      is_stopped_ = false;
    }
  }
  else
  {
    velocity_limited_pub_.publish(*velocity);
  }
}

void VelocityLimiterNode::onTeleopVelocity(const geometry_msgs::Twist::ConstPtr& velocity)
{
  if (current_operation_mode_ != "teleop")
    return;

  if (!first_vel_received_)
    first_vel_received_ = true;
  last_vel_time_ = ros::Time::now();

  geometry_msgs::Twist velocity_limited;
  limiter_.limitVelocity(*velocity, velocity_limited, velocity_limit_);
  if (is_enabled_)
  {
    velocity_limited_pub_.publish(velocity_limited);

    // check for stoppage
    double dx = fabs(velocity->linear.x - velocity_limited.linear.x);
    if (dx > 0.01 && fabs(velocity_limited.linear.x) < 1.0e-3)
    {
      if (!is_stopped_)
      {
        ROS_INFO("velo limiter stop");
        is_stopped_ = true;
        t_stopped_ = ros::Time::now();
      }
      else
      {
        double dt = (ros::Time::now() - t_stopped_).toSec();
        if (dt >= p_stop_timeout_)
        {
          ROS_INFO("we have been stopped for %5.2f s, aborting task", dt);
          if (has_goal_status_)
          {
            // should we change the stamp in the goal id?
            actionlib_msgs::GoalID goal_id = latest_goal_status_.goal_id;
            goal_abort_pub_.publish(goal_id);
          }
        }
      }
    }
    else
    {
      is_stopped_ = false;
    }
  }
  else
  {
    velocity_limited_pub_.publish(*velocity);
  }
}

void VelocityLimiterNode::onCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  if (!first_cloud_received_)
    first_cloud_received_ = true;
  last_cloud_time_ = ros::Time::now();

  if (cloud_msg->header.frame_id != p_base_frame_)
  {
    ROS_WARN("Discarding cloud due to wrong frame: %s (expected: %s)", cloud_msg->header.frame_id.c_str(),
             p_base_frame_.c_str());
    return;
  }

  // pcl::PCLPointCloud2 pcl_pc2;
  // pcl_conversions::toPCL(*cloud_msg, pcl_pc2);

  updateCloudBuffer(*cloud_msg);
  // ROS_INFO("Cloud buffer size: %lu", cloud_buffer_.size());

  sensor_msgs::PointCloud2 cloud_merged;
  for (const auto& cloud : cloud_buffer_)
  {
    pcl::concatenatePointCloud(cloud_merged, cloud, cloud_merged);
  }

  sensor_msgs::PointCloud2 cloud_base;
  try
  {
    pcl_ros::transformPointCloud(p_base_frame_, cloud_merged, cloud_base, tf_listener_);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  if (p_publish_pcl_)
    merged_cloud_pub_.publish(cloud_base);

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(cloud_base, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
  // ROS_INFO("Cloud size: %lu", cloud->points.size());

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  BoxHull boundary = velocity_grid_.getBoundary();
  pcl::CropBox<pcl::PointXYZ> boxFilter;
  boxFilter.setMin(Eigen::Vector4f(boundary.bottom_right.x(), boundary.bottom_right.y(), -10.0, 1.0));
  // boxFilter.setMin(Eigen::Vector4f(1., -1., -10.0, 1.0));
  boxFilter.setMax(Eigen::Vector4f(boundary.top_left.x(), boundary.top_left.y(), 10.0, 1.0));
  // boxFilter.setMax(Eigen::Vector4f(2., 0., 10.0, 1.0));
  boxFilter.setInputCloud(cloud);
  boxFilter.filter(*cloud_filtered);

  // ROS_INFO("Unfiltered size: %d, %d", cloud->width, cloud->height);
  // ROS_INFO("Filtered size: %d, %d", cloud_filtered->width, cloud_filtered->height);

  updateVelocityLimits(*cloud_filtered);
}

/**
 * Publish the velocity limit values of the clicked point.
 * see velocity_grid::getOrigin
 * see velocity_grid::getVelocityLimit
 */
void VelocityLimiterNode::onClickedPoint(const geometry_msgs::PointStamped::ConstPtr& point)
{
  VelocityLimit limit;
  ROS_INFO("Clicked on the point: (%.2f, %.2f)", point->point.x, point->point.y);

  double origin_x;
  double origin_y;
  velocity_grid_.getOrigin(origin_x, origin_y);
  velocity_grid_.getVelocityLimit(point->point.x + origin_x, point->point.y + origin_y, limit);
  if (limit.linear.positive.x < DBL_MAX)
    ROS_INFO("Linear positive: %.2f", limit.linear.positive.x);
  else
    ROS_INFO("Linear positive: DBL_MAX");
  if (limit.linear.negative.x < DBL_MAX)
    ROS_INFO("Linear negative: %.2f", limit.linear.negative.x);
  else
    ROS_INFO("Linear positive: DBL_MAX");
  if (limit.angular.positive.z < DBL_MAX)
    ROS_INFO("Angular positive: %.2f", limit.angular.positive.z);
  else
    ROS_INFO("Linear positive: DBL_MAX");
  if (limit.angular.negative.z < DBL_MAX)
    ROS_INFO("Angular negative: %.2f", limit.angular.negative.z);
  else
    ROS_INFO("Linear positive: DBL_MAX");
}

bool VelocityLimiterNode::onSwitchOperationMode(velocity_limiter::SwitchOperationMode::Request& req,
                                                velocity_limiter::SwitchOperationMode::Response& resp)
{
  std::string new_mode = req.operation_mode.data;
  if (new_mode == current_operation_mode_)
  {
    ROS_WARN("Ignoring request for switch to the current operation mode");
    resp.success = true;
    return true;
  }

  current_operation_mode_ = new_mode;
  resp.success = true;
  return true;
}

bool VelocityLimiterNode::onEnableSafeTeleop(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
{
  if (current_operation_mode_ != "teleop")
  {
    ROS_WARN("Ignoring request for enable safe teleop: current operation mode is not teleop");
    is_safe_teleop_enabled_ = false;
    resp.message = "Request ignored: current operation mode is not teleop";
    resp.success = false;
    return true;
  }

  if (req.data)
  {
    if (is_safe_teleop_enabled_)
    {
      ROS_WARN("Ignoring request for enable safe teleop: already enabled");
      resp.message = "Request ignored: already enabled";
      resp.success = true;
      return true;
    }

    if (switchLimitSet(p_safe_teleop_limit_set_))
    {
      ROS_INFO("Safe teleop enabled");
      resp.message = "Safe teleop enabled";
      is_safe_teleop_enabled_ = true;
      resp.success = true;
      return true;
    }
    else
    {
      ROS_WARN_STREAM("Frontier profile for safe teleop not found");
      resp.message = "Safe teleop cannot be enabled";
      resp.success = false;
      return false;
    }
  }
  else
  {
    if (!is_safe_teleop_enabled_)
    {
      ROS_WARN("Ignoring request for enable safe teleop: already disabled");
      resp.message = "Request ignored: already disabled";
      resp.success = true;
      return true;
    }

    if (switchLimitSet(p_initial_limit_set_))
    {
      ROS_INFO("Safe teleop disabled");
      resp.message = "Safe teleop disabled";
      is_safe_teleop_enabled_ = false;
      resp.success = true;
      return true;
    }
    else
    {
      ROS_WARN("Frontier profile for default not found");
      resp.message = "Safe teleop cannot be disabled";
      resp.success = false;
      return false;
    }
  }
}

bool VelocityLimiterNode::onSwitchLimitSet(velocity_limiter::SwitchLimitSet::Request& req,
                                           velocity_limiter::SwitchLimitSet::Response& resp)
{
  std::string new_profile = req.profile.data;
  if (new_profile == current_profile_)
  {
    ROS_WARN("Ignoring request for switch to the current limit set");
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
  ROS_INFO_STREAM("Set " << new_profile << " loaded");
  limit_set_ = p_limit_set_map_[new_profile];
  velocity_grid_ = p_velocity_grid_map_[new_profile];
  ROS_INFO("Velocity grid loaded");
  current_profile_ = new_profile;
  return true;
}

bool VelocityLimiterNode::onEnableLimiter(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
{
  if (req.data)
  {
    ROS_WARN("Velocity limiter enabled");
  }
  else
  {
    ROS_WARN("Velocity limiter disabled");
  }
  is_enabled_ = req.data;
  resp.success = true;
  return true;
}

void VelocityLimiterNode::publishOperationMode()
{
  velocity_limiter::OperationMode operation_mode_msg;
  operation_mode_msg.operation_mode.data = current_operation_mode_;
  operation_mode_msg.safe_teleop_enabled = is_safe_teleop_enabled_;
  operation_mode_pub_.publish(operation_mode_msg);
}

void VelocityLimiterNode::publishTopics()
{
  publishOperationMode();
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
  if (is_enabled_)
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Enabled");
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Not enabled");
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
  MovelLicense ml(8);
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
