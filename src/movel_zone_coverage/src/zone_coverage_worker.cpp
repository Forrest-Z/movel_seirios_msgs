#include <movel_zone_coverage/zone_coverage_worker.h>

#include <ros/ros.h>

#include <chrono>
#include <thread>

ZoneCoverageWorker::ZoneCoverageWorker(std::shared_ptr<costmap_2d::Costmap2DROS> map,
                                       std::shared_ptr<ZoneCoverageRedisClient> redis,
                                       std::shared_ptr<ros::Publisher> percentage_pub,
                                       std::shared_ptr<ros::Publisher> cell_update_pub,
                                       std::shared_ptr<ros::Publisher> worker_status_pub)
  : redis_client_(redis)
  , tf_listener_(tf_buffer_)
  , map_(map)
  , coverage_percentage_publisher_(percentage_pub)
  , coverage_cell_update_publisher_(cell_update_pub)
  , worker_status_publisher_(worker_status_pub)
{
  tf_buffer_.setUsingDedicatedThread(true);
  current_status_ = WorkerStatus::IDLE;
}

ZoneCoverageWorker::~ZoneCoverageWorker()
{
}

void ZoneCoverageWorker::threadLoop(double frequency)
{
  double desired_period = 1.0 / frequency;

  while (current_status_ != WorkerStatus::KILLED)
  {
    auto start_time = std::chrono::high_resolution_clock::now();

    publishWorkerStatus();

    if (current_status_ == WorkerStatus::IDLE)
      ;

    else if (current_status_ == WorkerStatus::PAUSING || current_status_ == WorkerStatus::STOPPING)
      current_status_ = WorkerStatus::IDLE;

    else if (current_status_ == WorkerStatus::RUNNING)
      updateZoneCoverage();

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> duration = end_time - start_time;
    double sleep_duration_secs = desired_period - duration.count();
    if (sleep_duration_secs > 0)
      std::this_thread::sleep_for(std::chrono::milliseconds((int)round(sleep_duration_secs * 1000)));
    else
      ROS_WARN("[ZoneCoverageWorker] thread loop missed the desired frequency of %.2f Hz", frequency);
  }

  for (std::map<std::string, ZoneCoverageTask>::iterator it = tasks_.begin(); it != tasks_.end(); ++it)
    redis_client_->deleteTask(it->first);
}

std::string ZoneCoverageWorker::getActiveTaskId()
{
  return active_task_id_;
}

WorkerStatus ZoneCoverageWorker::getStatus()
{
  return current_status_;
}

void ZoneCoverageWorker::updateZoneCoverage()
{
  if (!getRobotPose(current_pose_))
  {
    ROS_FATAL("[zone_coverage_worker] Failed to get robot pose");
    return;
  }

  ZoneCoverageTask& current_task = tasks_.at(active_task_id_);

  // get current occupied cells
  PointVector current_occupied_cells;

  if (current_task.use_footprint_radius)
  {
    geometry_msgs::Point footprint_center;
    footprint_center.x = current_pose_.position.x;
    footprint_center.y = current_pose_.position.y;
    if (!GetCurrentCoveredCellsCircular(footprint_center, current_task.robot_radius, current_task.zone,
                                        current_occupied_cells))
    {
      ROS_FATAL("[zone_coverage_worker] Failed to get occupied cells of circular footprint");
      return;
    }
  }
  else
  {
    while (!tf_buffer_.canTransform("map", "base_link", ros::Time(0)))
      ros::Duration(1 / 20).sleep();

    // transform footprint to world frame
    geometry_msgs::TransformStamped base_link_to_world = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
    PointVector footprint_world;

    for (PointVector::iterator it = current_task.robot_footprint.begin(); it != current_task.robot_footprint.end();
         ++it)
    {
      geometry_msgs::Point footprint_vertex_world;
      tf2::doTransform(*it, footprint_vertex_world, base_link_to_world);
      footprint_world.push_back(footprint_vertex_world);
    }

    if (!getCurrentCoveredCellsPolygon(footprint_world, current_task.zone, current_occupied_cells))
    {
      ROS_FATAL("[zone_coverage_worker] Failed to get occupied cells of polygonal footprint");
      return;
    }
  }

  // insert current occupied cells to visited cells
  for (PointVector::const_iterator it_current = current_occupied_cells.begin();
       it_current != current_occupied_cells.end(); ++it_current)
  {
    bool cell_already_exists = false;
    for (PointVector::const_iterator it_visited = current_task.visited_cells.begin();
         it_visited != current_task.visited_cells.end(); ++it_visited)
    {
      if (it_visited->x == it_current->x && it_visited->y == it_current->y)
      {
        cell_already_exists = true;
        break;
      }
    }
    if (cell_already_exists)
      continue;
    current_task.visited_cells.push_back(*it_current);
  }

  redis_client_->updateVisitedCells(current_task.task_id, current_occupied_cells);
  publishCurrentOccupiedCell(current_occupied_cells);

  current_task.area_coverage_percentage = current_task.visited_cells.size() / current_task.zone.area;
  redis_client_->setCoveragePercentage(current_task.task_id, current_task.area_coverage_percentage);
  publishCoveragePercentage(current_task.area_coverage_percentage);
}

bool ZoneCoverageWorker::startTask(std::string task_id, PointVector zone_polygon, PointVector footprint_polygon)
{
  if (!addNewTask(task_id, zone_polygon, footprint_polygon))
  {
    ROS_ERROR("[ZoneCoverageWorker] failed to create new task %s", task_id.c_str());
    return false;
  }
  active_task_id_ = task_id;

  if (redis_client_->taskExists(task_id))
    redis_client_->deleteTask(task_id);

  current_status_ = WorkerStatus::RUNNING;

  return true;
}

bool ZoneCoverageWorker::startTask(std::string task_id, PointVector zone_polygon, double footprint_radius)
{
  if (!addNewTask(task_id, zone_polygon, footprint_radius))
  {
    ROS_ERROR("[ZoneCoverageWorker] failed to create new task %s", task_id.c_str());
    return false;
  }
  active_task_id_ = task_id;

  if (redis_client_->taskExists(task_id))
    redis_client_->deleteTask(task_id);

  current_status_ = WorkerStatus::RUNNING;

  return true;
}

void ZoneCoverageWorker::stopCurrentTask()
{
  if (active_task_id_ == "")
  {
    ROS_WARN("[ZoneCoverageWorker] no active task running, nothing to stop");
    return;
  }

  current_status_ = WorkerStatus::STOPPING;

  while (current_status_ != WorkerStatus::IDLE)
    ;

  redis_client_->deleteTask(active_task_id_);
  tasks_.erase(active_task_id_);
  active_task_id_ = "";
}

bool ZoneCoverageWorker::resumeTask(std::string task_id)
{
  if (tasks_.find(task_id) == tasks_.end())
  {
    ROS_ERROR("[ZoneCoverageWorker] cannot resume task %s: task not found", task_id.c_str());
    return false;
  }

  active_task_id_ = task_id;
  current_status_ = WorkerStatus::RUNNING;
  return true;
}

void ZoneCoverageWorker::pauseCurrentTask()
{
  if (active_task_id_ == "")
  {
    ROS_WARN("[ZoneCoverageWorker] no active task running, nothing to pause");
    return;
  }

  current_status_ = WorkerStatus::PAUSING;

  while (current_status_ != WorkerStatus::IDLE)
    ;
}

void ZoneCoverageWorker::terminate()
{
  current_status_ = WorkerStatus::KILLED;
}

bool ZoneCoverageWorker::addNewTask(std::string task_id, PointVector zone_polygon, PointVector footprint_polygon)
{
  if (tasks_.find(task_id) != tasks_.end())
  {
    ROS_ERROR("[ZoneCoverageWorker] A task with the same taskId already exists.");
    return false;
  }

  MapLocationVector zone_polygon_map;
  double zone_area;
  std::vector<unsigned int> zone_cell_indices;
  if (!processZonePolygon(zone_polygon, zone_area, zone_cell_indices))
  {
    ROS_ERROR("[ZoneCoverageWorker] Failed to process zone polygon.");
    return false;
  }

  tasks_[task_id] =
      ZoneCoverageTask{ .task_id = task_id,
                        .zone = Zone{ .polygon = zone_polygon, .cell_indices = zone_cell_indices, .area = zone_area },
                        .use_footprint_radius = false,
                        .robot_footprint = footprint_polygon };

  return true;
}

bool ZoneCoverageWorker::addNewTask(std::string task_id, PointVector zone_polygon, double footprint_radius)
{
  if (tasks_.find(task_id) != tasks_.end())
  {
    ROS_ERROR("[ZoneCoverageWorker] A task with the same taskId already exists.");
    return false;
  }

  MapLocationVector zone_polygon_map;
  double zone_area;
  std::vector<unsigned int> zone_cell_indices;
  if (!processZonePolygon(zone_polygon, zone_area, zone_cell_indices))
  {
    ROS_ERROR("[ZoneCoverageWorker] Failed to process zone polygon.");
    return false;
  }

  tasks_[task_id] =
      ZoneCoverageTask{ .task_id = task_id,
                        .zone = Zone{ .polygon = zone_polygon, .cell_indices = zone_cell_indices, .area = zone_area },
                        .use_footprint_radius = true,
                        .robot_radius = footprint_radius };

  return true;
}

bool ZoneCoverageWorker::processZonePolygon(const PointVector& zone_polygon_world, double& zone_area,
                                            std::vector<unsigned int>& zone_cell_indices)
{
  MapLocationVector zone_polygon_map;
  for (PointVector::const_iterator it = zone_polygon_world.begin(); it != zone_polygon_world.end(); ++it)
  {
    costmap_2d::MapLocation vertex_map;
    if (!map_->getCostmap()->worldToMap(it->x, it->y, vertex_map.x, vertex_map.y))
      return false;
    zone_polygon_map.push_back(vertex_map);
  }

  MapLocationVector zone_cells_map;
  map_->getCostmap()->convexFillCells(zone_polygon_map, zone_cells_map);
  zone_cell_indices.clear();
  for (MapLocationVector::const_iterator it = zone_cells_map.begin(); it != zone_cells_map.end(); ++it)
    zone_cell_indices.push_back(map_->getCostmap()->getIndex(it->x, it->y));

  zone_area = (double)zone_cell_indices.size();

  return true;
}

bool ZoneCoverageWorker::getRobotPose(geometry_msgs::Pose& pose)
{
  geometry_msgs::PoseStamped pose_stamped;
  if (!map_->getRobotPose(pose_stamped))
    return false;

  pose = pose_stamped.pose;
  return true;
}

bool ZoneCoverageWorker::getCurrentCoveredCellsPolygon(const PointVector& polygon_world, const Zone& zone,
                                                       PointVector& cells_world)
{
  MapLocationVector polygon_cells;
  for (const geometry_msgs::Point& p : polygon_world)
  {
    costmap_2d::MapLocation footprint_map_cell;
    if (!map_->getCostmap()->worldToMap(p.x, p.y, footprint_map_cell.x, footprint_map_cell.y))
      return false;
    polygon_cells.push_back(footprint_map_cell);
  }

  cells_world.clear();
  MapLocationVector footprint_covered_cells;
  map_->getCostmap()->convexFillCells(polygon_cells, footprint_covered_cells);
  for (const costmap_2d::MapLocation& map_cell : footprint_covered_cells)
  {
    unsigned int p_idx = map_->getCostmap()->getIndex(map_cell.x, map_cell.y);
    if (std::find(zone.cell_indices.begin(), zone.cell_indices.end(), p_idx) != zone.cell_indices.end())
    {
      geometry_msgs::Point p;
      map_->getCostmap()->mapToWorld(map_cell.x, map_cell.y, p.x, p.y);
      cells_world.push_back(p);
    }
  }

  return true;
}

bool ZoneCoverageWorker::GetCurrentCoveredCellsCircular(const geometry_msgs::Point& center, double radius,
                                                        const Zone& zone, PointVector& cells_world)
{
  unsigned int radius_map = map_->getCostmap()->cellDistance(radius);
  costmap_2d::MapLocation center_map;
  if (!map_->getCostmap()->worldToMap(center.x, center.y, center_map.x, center_map.y))
    return false;
  unsigned int top = std::min(center_map.y + radius_map, map_->getCostmap()->getSizeInCellsY());
  unsigned int bottom = std::max(int(center_map.y) - int(radius_map), 0);
  unsigned int right = std::min(center_map.x + radius_map, map_->getCostmap()->getSizeInCellsX());
  unsigned int left = std::max(int(center_map.x) - int(radius_map), 0);
  MapLocationVector footprint_covered_cells;

  // get filling cells with flood fill algorithm
  std::function<void(costmap_2d::MapLocation)> f_flood_fill = [&](costmap_2d::MapLocation cell) -> void {
    if (cell.x < left || cell.x > right || cell.y < bottom || cell.y > top)
      return;

    for (MapLocationVector::const_iterator it = footprint_covered_cells.begin(); it != footprint_covered_cells.end();
         ++it)
      if (it->x == cell.x && it->y == cell.y)
        return;

    footprint_covered_cells.push_back(cell);

    costmap_2d::MapLocation cell_right = { .x = std::min(cell.x + 1, map_->getCostmap()->getSizeInCellsX()),
                                           .y = cell.y };
    costmap_2d::MapLocation cell_left = { .x = (unsigned int)std::max(int(cell.x) - 1, 0), .y = cell.y };
    costmap_2d::MapLocation cell_above = { .x = cell.x,
                                           .y = std::min(cell.y + 1, map_->getCostmap()->getSizeInCellsY()) };
    costmap_2d::MapLocation cell_below = { .x = cell.x, .y = (unsigned int)std::max(int(cell.y) - 1, 0) };

    f_flood_fill(cell_right);
    f_flood_fill(cell_left);
    f_flood_fill(cell_above);
    f_flood_fill(cell_below);
  };

  for (unsigned int x = left; x <= right; ++x)
  {
    for (unsigned int y = bottom; y <= top; ++y)
    {
      int dx = int(x) - int(center_map.x);
      int dy = int(y) - int(center_map.y);
      if (pow(dx, 2) + pow(dy, 2) <= pow(radius_map, 2))
      {
        costmap_2d::MapLocation cell = { .x = x, .y = y };
        f_flood_fill(cell);
      }
    }
  }

  cells_world.clear();
  for (const costmap_2d::MapLocation& map_cell : footprint_covered_cells)
  {
    unsigned int p_idx = map_->getCostmap()->getIndex(map_cell.x, map_cell.y);
    if (std::find(zone.cell_indices.begin(), zone.cell_indices.end(), p_idx) != zone.cell_indices.end())
    {
      geometry_msgs::Point p;
      map_->getCostmap()->mapToWorld(map_cell.x, map_cell.y, p.x, p.y);
      cells_world.push_back(p);
    }
  }

  return true;
}

void ZoneCoverageWorker::publishCurrentOccupiedCell(const PointVector& cells)
{
  movel_seirios_msgs::PointArray msg;
  msg.points = cells;
  coverage_cell_update_publisher_->publish(msg);
}

void ZoneCoverageWorker::publishCoveragePercentage(const double& percentage)
{
  std_msgs::Float64 msg;
  msg.data = percentage;
  coverage_percentage_publisher_->publish(msg);
}

void ZoneCoverageWorker::publishWorkerStatus()
{
  std_msgs::String msg;
  switch (current_status_)
  {
    case WorkerStatus::IDLE:
      msg.data = "idle";
      break;
    case WorkerStatus::RUNNING:
      msg.data = "running";
      break;
    case WorkerStatus::PAUSING:
      msg.data = "pausing";
      break;
    case WorkerStatus::STOPPING:
      msg.data = "stopping";
      break;
    case WorkerStatus::KILLED:
      msg.data = "killed";
      break;
  }

  worker_status_publisher_->publish(msg);
}

bool ZoneCoverageWorker::worldPointVecToMapLocationVec(const PointVector& world_points, MapLocationVector& map_points)
{
  map_points.clear();
  for (const geometry_msgs::Point& p : world_points)
  {
    costmap_2d::MapLocation map_point;
    if (!map_->getCostmap()->worldToMap(p.x, p.y, map_point.x, map_point.y))
      return false;
    map_points.push_back(map_point);
  }

  return true;
}

void ZoneCoverageWorker::mapLocationVecToWorldPointVec(const MapLocationVector& map_point, PointVector& world_points)
{
  world_points.clear();
  for (const costmap_2d::MapLocation& p : map_point)
  {
    geometry_msgs::Point world_point;
    map_->getCostmap()->mapToWorld(p.x, p.y, world_point.x, world_point.y);
    world_points.push_back(world_point);
  }
}