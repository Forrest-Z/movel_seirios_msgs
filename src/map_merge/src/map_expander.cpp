#include <map_expander/map_expander.h>
#include <movel_hasp_vendor/license.h>

namespace map_expander
{

MapExpander::MapExpander()
  : nh_private_("~")
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
    if (previous_map_.read_only_map && current_map_.read_only_map)
    {
      nav_msgs::OccupancyGridPtr merged_map;
      if (mergeMap(merged_map))
        merged_map_publisher_.publish(merged_map);
    }
    else if (previous_map_.read_only_map)
    {
      ROS_WARN("[map_expander] No current map received yet, publishing previous map only.");
      merged_map_publisher_.publish(previous_map_.read_only_map);
    }
    else if (current_map_.read_only_map)
    {
      ROS_WARN("[map_expander] No information on previous map; this shouldn't happen. Publishing current map only.");
      merged_map_publisher_.publish(current_map_.read_only_map);
    }
    else
    {
      ROS_WARN("[map_expander] No information on both previous and current map. Skipping merge.");
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
  if (previous_map_.map_info.resolution != current_map_.map_info.resolution)
  {
    ROS_WARN("[map_expander] Resolution of current map and previous map must be the same.");
    return false;
  }

  // populate pipeline with maps
  std::vector<nav_msgs::OccupancyGridConstPtr> gridmaps;
  gridmaps.reserve(2);
  gridmaps.push_back(previous_map_.read_only_map);
  gridmaps.push_back(current_map_.read_only_map);
  
  // populate pipeline with transforms (in unit pixels)
  std::vector<geometry_msgs::Transform> transforms;

  // transform from (the origins of) previous map to current map
  tf2::Transform previous_map_origin_tf, current_map_origin_tf, map_transform_tf;
  tf2::fromMsg(previous_map_.map_info.origin, previous_map_origin_tf);
  tf2::fromMsg(current_map_.map_info.origin, current_map_origin_tf);
  map_transform_tf = previous_map_origin_tf.inverse() * current_map_origin_tf;
  geometry_msgs::Transform map_transform = tf2::toMsg(map_transform_tf);

  // convert inter-map transform to unit pixels (cells)
  map_transform.translation.x /= current_map_.map_info.resolution;
  map_transform.translation.y /= current_map_.map_info.resolution;
  map_transform.translation.z /= current_map_.map_info.resolution;

  // push transform to pipeline
  transforms.push_back(map_transform);

  // transform from current map to current map (i.e. zero)
  geometry_msgs::Transform zero;
  zero.rotation.w = 1;
  // push transform to pipeline
  transforms.push_back(zero);

  pipeline_.feed(gridmaps.begin(), gridmaps.end());
  pipeline_.setTransforms(transforms.begin(), transforms.end());

  // execute pipeline
  merged_map = pipeline_.composeGrids(current_map_.map_info.origin); // merged_map origin = current_map origin
  // merged_map = pipeline_.composeGrids();

  // populate rest of map metadata
  ros::Time now = ros::Time::now();
  merged_map->info.map_load_time = now;
  merged_map->header.stamp = now;
  merged_map->header.frame_id = map_frame_;

  return true;
}

} // namespace map_expander

int main(int argc, char** argv)
{
  #ifdef MOVEL_LICENSE
    MovelLicense ml(36);
    if (!ml.login())
      return 1;
  #endif

  ros::init(argc, argv, "map_expander");
  map_expander::MapExpander map_expander;

  #ifdef MOVEL_LICENSE
    ml.logout();
  #endif

  return 0;
}