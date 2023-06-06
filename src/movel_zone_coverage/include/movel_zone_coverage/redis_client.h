#ifndef ZONE_COVERAGE_REDIS_CLIENT_H_
#define ZONE_COVERAGE_REDIS_CLIENT_H_

#include <geometry_msgs/Point.h>
#include <sw/redis++/redis++.h>

using namespace sw::redis;

class ZoneCoverageRedisClient
{
public:
  ZoneCoverageRedisClient(std::string host, int port, int timeout);
  ~ZoneCoverageRedisClient();

  void setCoveragePercentage(const std::string& task_id, const double& percentage);
  void updateVisitedCells(const std::string& task_id, const std::vector<geometry_msgs::Point>& visited_cells);

  bool getCoveragePercentage(const std::string& task_id, double& percentage);
  bool getVisitedCells(const std::string& task_id, std::vector<geometry_msgs::Point>& visited_cells);

  void deleteTask(const std::string& task_id);

  bool taskExists(const std::string& task_id);

private:
  void coordinateStringToPoint(const std::string& in, geometry_msgs::Point& out);

private:
  ConnectionOptions opts_;
  std::unique_ptr<Redis> redis_;
};

#endif