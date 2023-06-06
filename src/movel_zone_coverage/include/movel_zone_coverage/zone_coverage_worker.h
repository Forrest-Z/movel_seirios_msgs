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
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include <string>
#include <atomic>
#include <vector>

using namespace sw::redis;

enum WorkerStatus
{
  INITIALIZED,
  RUNNING,
  PAUSED,
  STOPPED,
  FINISHED
};

class ZoneCoverageWorker
{
public:
  ZoneCoverageWorker(std::string task_id, std::vector<geometry_msgs::Point> zone_polygon,
                     std::shared_ptr<costmap_2d::Costmap2DROS> map, std::shared_ptr<ZoneCoverageRedisClient> redis,
                     std::shared_ptr<ros::Publisher> percentage_pub, std::shared_ptr<ros::Publisher> cell_update_pub);
  ~ZoneCoverageWorker();

  void setRobotFootprintPolygon(std::vector<geometry_msgs::Point> footprint_polygon);
  void setRobotFootprintRadius(double footprint_radius);

  std::string getTaskId();
  WorkerStatus getStatus();

  void startWorker();
  void pauseWorker();
  void stopWorker();

  void workerThread();

private:
  bool processZonePolygon(const std::vector<geometry_msgs::Point>& zone_polygon_world);
  bool getRobotPose(geometry_msgs::Pose& pose);
  bool getPolygonFillingCells(const std::vector<geometry_msgs::Point>& footprint,
                              std::vector<geometry_msgs::Point>& cells);
  bool getCircleFillingCells(const geometry_msgs::Point& center, double radius,
                             std::vector<geometry_msgs::Point>& cells);
  void publishCurrentOccupiedCell(const std::vector<geometry_msgs::Point>& cells);
  void publishCoveragePercentage(const double& percentage);

public:
  std::vector<geometry_msgs::Point> visited_cells;
  double area_coverage_percentage;

private:
  // zone
  std::vector<costmap_2d::MapLocation> zone_polygon_;
  unsigned int zone_area_;

  // footprint
  bool use_footprint_radius_;  // true if footprint_radius_ is used, false if footprint_polygon_ is used
  double robot_radius_;
  std::vector<geometry_msgs::Point> robot_footprint_;

  // robot states
  geometry_msgs::Pose current_pose_;

  // worker state
  std::string task_id_;
  std::atomic<WorkerStatus> current_status_;

  // ros
  std::shared_ptr<ros::Publisher> coverage_percentage_publisher_;
  std::shared_ptr<ros::Publisher> coverage_cell_update_publisher_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<costmap_2d::Costmap2DROS> map_;

  // redis
  std::shared_ptr<ZoneCoverageRedisClient> redis_client_;
};

#endif