#ifndef ZONE_COVERAGE_WORKER_H_
#define ZONE_COVERAGE_WORKER_H_

#include <movel_zone_coverage/redis_client.h>
#include <sw/redis++/redis++.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <movel_seirios_msgs/PointArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include <string>
#include <atomic>
#include <vector>

using namespace sw::redis;

using MapLocationVector = std::vector<costmap_2d::MapLocation>;
using PointVector = std::vector<geometry_msgs::Point>;

enum WorkerStatus
{
  IDLE,
  RUNNING,
  PAUSING,
  STOPPING,
  KILLED
};

struct ZoneCoverageTask
{
  std::string task_id;

  // task state
  PointVector visited_cells;
  double area_coverage_percentage;

  // zone
  MapLocationVector zone_polygon;
  double zone_area;

  // footprint
  bool use_footprint_radius;
  double robot_radius;
  PointVector robot_footprint;
};

class ZoneCoverageWorker
{
public:
  ZoneCoverageWorker(std::shared_ptr<costmap_2d::Costmap2DROS> map, std::shared_ptr<ZoneCoverageRedisClient> redis,
                     std::shared_ptr<ros::Publisher> percentage_pub, std::shared_ptr<ros::Publisher> cell_update_pub,
                     std::shared_ptr<ros::Publisher> worker_status_pub);
  ~ZoneCoverageWorker();

  void threadFunction();

  std::string getActiveTaskId();
  WorkerStatus getStatus();

  bool startTask(std::string task_id, PointVector zone_polygon, PointVector footprint_polygon);
  bool startTask(std::string task_id, PointVector zone_polygon, double footprint_radius);
  void stopCurrentTask();
  void pauseCurrentTask();
  bool resumeTask(std::string task_id);

  void terminate();

private:
  bool addNewTask(std::string task_id, PointVector zone_polygon, PointVector footprint_polygon);
  bool addNewTask(std::string task_id, PointVector zone_polygon, double footprint_radius);

  bool processZonePolygon(const PointVector& zone_polygon_world, MapLocationVector& zone_polygon_map,
                          double& zone_area);
  bool getRobotPose(geometry_msgs::Pose& pose);
  bool getPolygonFillingCells(const PointVector& footprint, PointVector& cells);
  bool getCircleFillingCells(const geometry_msgs::Point& center, double radius, PointVector& cells);
  void publishCurrentOccupiedCell(const PointVector& cells);
  void publishCoveragePercentage(const double& percentage);
  void publishWorkerStatus();

private:
  // robot states
  geometry_msgs::Pose current_pose_;

  // worker state
  std::string active_task_id_;
  std::atomic<WorkerStatus> current_status_;

  // tasks
  std::map<std::string, ZoneCoverageTask> tasks_;

  // ros
  std::shared_ptr<ros::Publisher> coverage_percentage_publisher_;
  std::shared_ptr<ros::Publisher> coverage_cell_update_publisher_;
  std::shared_ptr<ros::Publisher> worker_status_publisher_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<costmap_2d::Costmap2DROS> map_;

  // redis
  std::shared_ptr<ZoneCoverageRedisClient> redis_client_;
};

#endif