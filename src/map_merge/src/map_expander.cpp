#include <map_expander/map_expander.h>

namespace map_expander
{

MapExpander::MapExpander()
  : nh_private_("~")
  , initial_robot_pose_acquired_(false)
{
  if (!loadParams())
  {
    ROS_FATAL("[map_expander] Error during parameter loading. Shutting down.");
    return;
  }

  ROS_INFO("[map_expander] All parameters loaded. Launching.");

  setupTopics();

  if (previous_map_available_ && previous_map_service_available_)
  {
    ROS_INFO("[map_expander] Static map service available; using service to load previous map.");
    loadStaticMap();
  }

  ros::Rate r(merging_rate_);
  while (ros::ok())
  {
    if (previous_map_available_)
    {
      if (initial_robot_pose_acquired_)
      {
        nav_msgs::OccupancyGridPtr merged_map;
        if (mergeMap(merged_map))
          merged_map_publisher_.publish(merged_map);
      }
      else
      {
        ROS_WARN("[map_expander] Robot pose not initialized, cannot merge map. Publishing previous map.");
        if (previous_map_.read_only_map)
          merged_map_publisher_.publish(previous_map_.read_only_map);
      }
    }
    else
    {
      if (current_map_.read_only_map)
        merged_map_publisher_.publish(current_map_.read_only_map);
    }

    ros::spinOnce();
    r.sleep();
  }
}

MapExpander::~MapExpander()
{
}

bool MapExpander::loadParams()
{
  ros_utils::ParamLoader loader(nh_private_);

  loader.get_optional("merging_rate", merging_rate_, 10.0);
  loader.get_optional("previous_map_available", previous_map_available_, true);
  loader.get_optional("previous_map_service_available", previous_map_service_available_, true);
  loader.get_optional("previous_map_service_name", previous_map_service_name_, std::string("static_map"));
  loader.get_optional("map_frame", map_frame_, std::string("map"));

  return loader.params_valid();
}

void MapExpander::setupTopics()
{
  merged_map_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("map/merged", 10, true);
  
  initial_robot_pose_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initial_pose", 1,
    &MapExpander::initialRobotPoseCallback, this
    );

  current_map_.map_sub = nh_.subscribe<nav_msgs::OccupancyGrid>("map/current", 10,
    [this](const nav_msgs::OccupancyGrid::ConstPtr& msg) {
      fullMapCallback(msg, current_map_);
    });
  
  current_map_.map_update_sub = nh_.subscribe<map_msgs::OccupancyGridUpdate>("map_updates/current", 10,
    [this](const map_msgs::OccupancyGridUpdate::ConstPtr& msg) {
      partialMapCallback(msg, current_map_);
    });

  if (!previous_map_service_available_)
  {
    previous_map_.map_sub = nh_.subscribe<nav_msgs::OccupancyGrid>("map/previous", 10,
      [this](const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        fullMapCallback(msg, previous_map_);
      });
  }
}

void MapExpander::loadStaticMap()
{
  previous_map_.map_client = nh_.serviceClient<nav_msgs::GetMap>(previous_map_service_name_);
  nav_msgs::GetMap get_map;
  
  ROS_INFO("[map_expander] Loading static map...");
  bool static_map_acquired = false;
  while (!static_map_acquired)
  {
    if (previous_map_.map_client.call(get_map))
    {
      previous_map_.read_only_map = boost::make_shared<nav_msgs::OccupancyGrid>(get_map.response.map);
      previous_map_.writable_map = nullptr;
      previous_map_.map_info = get_map.response.map.info;
      static_map_acquired = true;
    }
  }
  ROS_INFO("[map_expander] Static map acquired.");
}

void MapExpander::initialRobotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  initial_robot_pose_.translation.x = msg->pose.pose.position.x;
  initial_robot_pose_.translation.y = msg->pose.pose.position.y;
  initial_robot_pose_.translation.z = msg->pose.pose.position.z;
  initial_robot_pose_.rotation = msg->pose.pose.orientation;

  if (!initial_robot_pose_acquired_)
    initial_robot_pose_acquired_ = true;
}

void MapExpander::fullMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg, MapSource& map)
{
  std::lock_guard<std::mutex> lock(map.mutex);
  if (map.read_only_map && map.read_only_map->header.stamp > msg->header.stamp)
    return;
  
  map.read_only_map = msg;
  map.writable_map = nullptr;
  map.map_info = msg->info;
}

// https://github.com/hrnr/m-explore/blob/712bdd41027b645c9c876a4c0071478f090825ea/map_merge/src/map_merge.cpp#L222
void MapExpander::partialMapCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg, DynamicMapSource& map)
{
  if (msg->x < 0 || msg->y < 0)
  {
    ROS_ERROR("[map_expander] invalid partial map update: negative coordinates (%d, %d)", msg->x, msg->y);
    return;
  }

  size_t x0 = static_cast<size_t>(msg->x);
  size_t y0 = static_cast<size_t>(msg->y);
  size_t xn = msg->width + x0;
  size_t yn = msg->height + y0;

  nav_msgs::OccupancyGrid::Ptr map_temp;
  nav_msgs::OccupancyGrid::ConstPtr read_only_map;
  {
    std::lock_guard<std::mutex> lock(map.mutex);
    map_temp = map.writable_map;
    read_only_map = map.read_only_map;
  }

  if (!read_only_map)
  {
    ROS_WARN("[map expander] received partial map update, but don't have any full map to update. Skipping.");
    return;
  }

  // we don't have partial map to take update, we must copy read only map and update new writable map
  if (!map_temp)
    map_temp.reset(new nav_msgs::OccupancyGrid(*read_only_map));
  
  size_t grid_xn = map_temp->info.width;
  size_t grid_yn = map_temp->info.height;

  if (xn > grid_xn || x0 > grid_xn || yn > grid_yn || y0 > grid_yn) {
    ROS_WARN("[map_expander] received update doesn't fully fit into existing map_temp, only part will be copied."
             "Received [%lu, %lu], [%lu, %lu]; map_temp is [0, %lu], [0, %lu]", x0, xn, y0, yn, grid_xn, grid_yn);
  }
  
  // update map with data
  size_t i = 0;
  for (size_t y = y0; y < yn && y < grid_yn; ++y) {
    for (size_t x = x0; x < xn && x < grid_xn; ++x) {
      size_t idx = y * grid_xn + x;  // index to grid for this specified cell
      map_temp->data[idx] = msg->data[i];
      ++i;
    }
  }
  // update time stamp
  map_temp->header.stamp = msg->header.stamp;

  {
    // store back updated map
    std::lock_guard<std::mutex> lock(map.mutex);
    if (map.read_only_map && map.read_only_map->header.stamp > map_temp->header.stamp)
      return;

    map.writable_map = map_temp;
    map.read_only_map = map_temp;
    map.map_info = map_temp->info;
  }
}

bool MapExpander::mergeMap(nav_msgs::OccupancyGridPtr& merged_map)
{
  if (!previous_map_.read_only_map || !current_map_.read_only_map)
  {
    ROS_WARN("[map_expander] Either previous map or current map is not available. Skipping merge.");
    return false;
  }

  // get initial robot pose in cell units for map image transformation
  geometry_msgs::Transform initial_pose_cell_units = initial_robot_pose_;
  initial_pose_cell_units.translation.x /= previous_map_.map_info.resolution;
  initial_pose_cell_units.translation.y /= previous_map_.map_info.resolution;
  initial_pose_cell_units.translation.z /= previous_map_.map_info.resolution;

  // populating pipeline with maps and transforms
  std::vector<nav_msgs::OccupancyGridConstPtr> gridmaps;
  std::vector<geometry_msgs::Transform> transforms;
  gridmaps.reserve(2);
  gridmaps.push_back(previous_map_.read_only_map);
  gridmaps.push_back(current_map_.read_only_map);
  // current map origin relative to previous map
  transforms.push_back(initial_pose_cell_units);
  // current map origin relative to current map (i.e. zero)
  transforms.push_back(geometry_msgs::Transform());

  pipeline_.feed(gridmaps.begin(), gridmaps.end());
  pipeline_.setTransforms(transforms.begin(), transforms.end());

  // calculate merged map origin (previous map origin + inverse(initial robot pose))
  tf2::Transform previous_map_origin_transform, initial_robot_pose_transform, merged_map_origin_transform;
  tf2::fromMsg(previous_map_.map_info.origin, previous_map_origin_transform);
  tf2::fromMsg(initial_robot_pose_, initial_robot_pose_transform);
  merged_map_origin_transform = previous_map_origin_transform * initial_robot_pose_transform.inverse();
  
  geometry_msgs::Pose merged_map_origin = tf2::toMsg(merged_map_origin_transform, merged_map_origin);

  // execute pipeline
  merged_map = pipeline_.composeGrids(merged_map_origin);
  // merged_map = pipeline_.composeGrids();

  return true;
}

} // namespace map_expander

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_expander");
  map_expander::MapExpander map_expander;

  return 0;
}