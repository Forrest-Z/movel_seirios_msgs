#ifndef ZONE_COVERAGE_SERVER_H_
#define ZONE_COVERAGE_SERVER_H_

#include <movel_zone_coverage/zone_coverage_worker.h>
#include <movel_zone_coverage/redis_client.h>
#include <movel_seirios_msgs/StartZoneCoverageStats.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <map>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>
#include <thread>

using WorkerPtr = std::unique_ptr<ZoneCoverageWorker>;

class ZoneCoverageServer
{
public:
  ZoneCoverageServer(ros::NodeHandle nh);
  ~ZoneCoverageServer();

  // void main();
  void shutdownHandler();

private:
  bool loadParams();

  bool startServiceCb(movel_seirios_msgs::StartZoneCoverageStats::Request& req,
                      movel_seirios_msgs::StartZoneCoverageStats::Response& res);
  bool resumeServiceCb(movel_seirios_msgs::StartZoneCoverageStats::Request& req,
                       movel_seirios_msgs::StartZoneCoverageStats::Response& res);
  bool stopServiceCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool pauseServiceCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

private:
  std::unique_ptr<ZoneCoverageWorker> worker_;
  std::thread worker_thread_;
  std::vector<geometry_msgs::Point> robot_footprint_;

  // ros
  ros::NodeHandle nh_, nh_private_;
  ros::ServiceServer start_service_, resume_service_, stop_service_, pause_service_;
  std::shared_ptr<costmap_2d::Costmap2DROS> map_;
  std::shared_ptr<ros::Publisher> coverage_percentage_publisher_;
  std::shared_ptr<ros::Publisher> coverage_cell_update_publisher_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::Buffer tf_buffer_;

  // redis
  std::string redis_host_;
  int redis_port_;
  int redis_timeout_;
  std::shared_ptr<ZoneCoverageRedisClient> redis_client_;
};

#endif