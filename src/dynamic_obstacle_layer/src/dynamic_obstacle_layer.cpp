#include "dynamic_obstacle_layer/dynamic_obstacle_layer.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(dynamic_obstacle_layer::DynamicLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

namespace dynamic_obstacle_layer
{

DynamicLayer::DynamicLayer() {}

void DynamicLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;
  
  std::string map_topic;
  nh.param("map_topic", map_topic, std::string("map"));
  nh.param("first_map_only", first_map_only_, false);
  nh.param("track_unknown_space", track_unknown_space_, true);
  int temp_lethal_threshold, temp_unknown_cost_value;
  nh.param("lethal_cost_threshold", temp_lethal_threshold, int(100));
  nh.param("unknown_cost_value", temp_unknown_cost_value, int(-1));
  nh.param("trinary_costmap", trinary_costmap_, true);

  nh.param("map_tolerance", map_tolerance_, 0.2);
  nh.param("footprint_radius", footprint_radius_, 0.2);
  nh.param("range", range_, 1.0);

  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
  unknown_cost_value_ = temp_unknown_cost_value;
  matchSize();

  // Only resubscribe if topic has changed
  if (map_sub_.getTopic() != ros::names::resolve(map_topic))
  {
    // we'll subscribe to the latched topic that the map server uses
    ROS_INFO("Requesting the map...");
    map_sub_ = g_nh.subscribe(map_topic, 1, &DynamicLayer::incomingMap, this);
    map_received_ = false;

    ros::Rate r(10);
    while (!map_received_ && g_nh.ok())
    {
      ros::spinOnce();
      r.sleep();
    }

    ROS_INFO("Received a %d X %d map at %f m/pix", getSizeInCellsX(), getSizeInCellsY(), getResolution());
  }

  has_pose_ = false;
  pose_sub_ = g_nh.subscribe("/pose", 1, &DynamicLayer::getPose, this);

  dsrv_ = new dynamic_reconfigure::Server<dynamic_obstacle_layer::DynamicPluginConfig>(nh);
  dynamic_reconfigure::Server<dynamic_obstacle_layer::DynamicPluginConfig>::CallbackType cb = boost::bind(
      &DynamicLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}


void DynamicLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}


void DynamicLayer::reconfigureCB(dynamic_obstacle_layer::DynamicPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
  footprint_radius_ = config.footprint_radius;
  range_ = config.range;
}

void DynamicLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{}

void DynamicLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_ || !has_pose_ || !map_received_)
    return;

  costmap_2d::Costmap2D costmap_copy = master_grid;

  double world_min_x, world_max_x, world_min_y, world_max_y;
  world_min_x = pose_.position.x - range_;
  world_max_x = pose_.position.x + range_;
  world_min_y = pose_.position.y - range_;
  world_max_y = pose_.position.y + range_;

  unsigned int map_min_x, map_max_x, map_min_y, map_max_y;

  if (worldToMap(world_min_x, world_min_y, map_min_x, map_min_y) && 
      worldToMap(world_max_x, world_max_y, map_max_x, map_max_y))
  {

    for (unsigned int j = map_min_y; j < map_max_y; j++)
    {
      for (unsigned int i = map_min_x; i < map_max_x; i++)
      {
        if (costmap_copy.getCost(i, j) == LETHAL_OBSTACLE && getCost(i, j) != LETHAL_OBSTACLE)
          markFootprint(master_grid, i, j);
      }
    }

    prev_marked_pixels_.clear();
    prev_marked_pixels_ = marked_pixel_coordinates_;
    marked_pixel_coordinates_.clear();
  }  
}

void DynamicLayer::getOffsets(double radius, std::vector< std::vector<int> > &offsets)
{
  double res = layered_costmap_->getCostmap()->getResolution();
  int r = radius/res;

  for (int i = -1*r; i < r; i++)
  {
    for (int j = -1*r; j < r; j++)
    {
      if (i*i + j*j < r*r)
      {
        std::vector<int> offset;
        offset.push_back(i);
        offset.push_back(j);
        offsets.push_back(offset);
      }
    }
  }
}

void DynamicLayer::markFootprint(costmap_2d::Costmap2D& master_grid, unsigned int mark_i, unsigned int mark_j)
{
  if (std::find(marked_pixel_coordinates_.begin(), marked_pixel_coordinates_.end(), std::make_pair(mark_i, mark_j)) != marked_pixel_coordinates_.end())
    return;

  if (std::find(prev_marked_pixels_.begin(), prev_marked_pixels_.end(), std::make_pair(mark_i, mark_j)) != prev_marked_pixels_.end())
  {
    master_grid.setCost(mark_i, mark_j, LETHAL_OBSTACLE);
    if (std::find(marked_pixel_coordinates_.begin(), marked_pixel_coordinates_.end(), std::make_pair(mark_i, mark_j)) == marked_pixel_coordinates_.end())
      marked_pixel_coordinates_.push_back(std::make_pair(mark_i, mark_j));
    return;
  }

  std::vector< std::vector<int> > offsets;
  getOffsets(footprint_radius_, offsets);

  unsigned int max_i, max_j;
  max_i = layered_costmap_->getCostmap()->getSizeInCellsX();
  max_j = layered_costmap_->getCostmap()->getSizeInCellsY();

  for (int p = 0; p < offsets.size(); p++)
  {
    int di, dj;
    di = offsets[p][0];
    dj = offsets[p][1];

    if (mark_i < abs(di) || mark_j < abs(dj))
      continue;

    unsigned int mark_ii, mark_jj;
    mark_ii = mark_i + di;
    mark_jj = mark_j + dj;

    if (mark_ii >= max_i || mark_jj >= max_j)
      continue;

    if (master_grid.getCost(mark_ii, mark_jj) != NO_INFORMATION && master_grid.getCost(mark_ii, mark_jj) != LETHAL_OBSTACLE)
    {
      master_grid.setCost(mark_ii, mark_jj, LETHAL_OBSTACLE);
      if (std::find(marked_pixel_coordinates_.begin(), marked_pixel_coordinates_.end(), std::make_pair(mark_ii, mark_jj)) == marked_pixel_coordinates_.end())
        marked_pixel_coordinates_.push_back(std::make_pair(mark_ii, mark_jj));
    }
  }
}

void DynamicLayer::inflateMap(unsigned int mark_i, unsigned int mark_j)
{
  std::vector< std::vector<int> > offsets;
  getOffsets(map_tolerance_, offsets);

  unsigned int max_i, max_j;
  max_i = layered_costmap_->getCostmap()->getSizeInCellsX();
  max_j = layered_costmap_->getCostmap()->getSizeInCellsY();

  for (int p = 0; p < offsets.size(); p++)
  {
    int di, dj;
    di = offsets[p][0];
    dj = offsets[p][1];

    if (mark_i < abs(di) || mark_j < abs(dj))
      continue;

    int mark_ii, mark_jj;
    mark_ii = mark_i + di;
    mark_jj = mark_j + dj;

    if (mark_ii >= max_i || mark_jj >= max_j)
      continue;

    setCost(mark_ii, mark_jj, LETHAL_OBSTACLE);
  }
}

unsigned char DynamicLayer::interpretValue(unsigned char value)
{
  // check if the static value is above the unknown or lethal thresholds
  if (track_unknown_space_ && value == unknown_cost_value_)
    return NO_INFORMATION;
  else if (!track_unknown_space_ && value == unknown_cost_value_)
    return FREE_SPACE;
  else if (value >= lethal_threshold_)
    return LETHAL_OBSTACLE;
  else if (trinary_costmap_)
    return FREE_SPACE;

  double scale = (double) value / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

void DynamicLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
{
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

  ROS_DEBUG("Received a %d X %d map at %f m/pix", size_x, size_y, new_map->info.resolution);

  // resize costmap if size, resolution or origin do not match
  Costmap2D* master = layered_costmap_->getCostmap();
  if (!layered_costmap_->isRolling() &&
      (master->getSizeInCellsX() != size_x ||
       master->getSizeInCellsY() != size_y ||
       master->getResolution() != new_map->info.resolution ||
       master->getOriginX() != new_map->info.origin.position.x ||
       master->getOriginY() != new_map->info.origin.position.y))
  {
    // Update the size of the layered costmap (and all layers, including this one)
    ROS_INFO("Resizing costmap to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
    layered_costmap_->resizeMap(size_x, size_y, new_map->info.resolution, new_map->info.origin.position.x,
                                new_map->info.origin.position.y,
                                true );
  }
  else if (size_x_ != size_x || size_y_ != size_y ||
           resolution_ != new_map->info.resolution ||
           origin_x_ != new_map->info.origin.position.x ||
           origin_y_ != new_map->info.origin.position.y)
  {
    // only update the size of the costmap stored locally in this layer
    ROS_INFO("Resizing static layer to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
    resizeMap(size_x, size_y, new_map->info.resolution,
              new_map->info.origin.position.x, new_map->info.origin.position.y);
  }

  unsigned int index = 0;

  // initialize the costmap with static data
  for (unsigned int i = 0; i < size_y; ++i)
  {
    for (unsigned int j = 0; j < size_x; ++j)
    {
      unsigned char value = new_map->data[index];
      costmap_[index] = interpretValue(value);
      ++index;
    }
  }

  map_received_ = true;

  // shutdown the map subscrber if firt_map_only_ flag is on
  if (first_map_only_)
  {
    ROS_INFO("Shutting down the map subscriber. first_map_only flag is on");
    map_sub_.shutdown();
  }

  index = 0;
  costmap_2d::Costmap2D costmap_copy = *master;
  for (unsigned int i = 0; i < size_y; ++i)
  {
    for (unsigned int j = 0; j < size_x; ++j)
    {
      costmap_copy.setCost(j, i, costmap_[index]);
      ++index;
    }
  }

  for (unsigned int i = 0; i < size_y; i++)
  {
    for (unsigned int j = 0; j < size_x; j++)
    {
      if (costmap_copy.getCost(j, i) == LETHAL_OBSTACLE)
        inflateMap(j, i);
    }
  }
}

void DynamicLayer::getPose(const geometry_msgs::PoseConstPtr& pose)
{
  pose_ = *pose;
  has_pose_ = true;
}

} // end namespace
