#include <movel_zone_coverage/redis_client.h>
#include <optional>

ZoneCoverageRedisClient::ZoneCoverageRedisClient(std::string host, int port, int timeout)
{
  opts_.host = host;
  opts_.port = port;
  opts_.socket_timeout = std::chrono::milliseconds(timeout);
  redis_ = std::make_unique<Redis>(opts_);
}

ZoneCoverageRedisClient::~ZoneCoverageRedisClient()
{
}

void ZoneCoverageRedisClient::setCoveragePercentage(const std::string& task_id, const double& percentage)
{
  std::string key = "zone_coverage_task:" + task_id;
  redis_->hset(key, "percentage", std::to_string(percentage));
}

void ZoneCoverageRedisClient::updateVisitedCells(const std::string& task_id,
                                                 const std::vector<geometry_msgs::Point>& visited_cells)
{
  std::string key = "zone_coverage_task:" + task_id + ":cells";
  for (const geometry_msgs::Point& cell : visited_cells)
  {
    std::string cell_str = std::to_string(cell.x) + "," + std::to_string(cell.y);
    redis_->sadd(key, cell_str);
  }
}

bool ZoneCoverageRedisClient::getCoveragePercentage(const std::string& task_id, double& percentage)
{
  std::string key = "zone_coverage_task:" + task_id;

  if (!redis_->exists(key))
    return false;

  std::optional<std::string> percentage_str = redis_->hget(key, "percentage");
  if (percentage_str.has_value())
  {
    percentage = std::stod(percentage_str.value());
    return true;
  }
  else
  {
    percentage = 0;
    return false;
  }
}

bool ZoneCoverageRedisClient::getVisitedCells(const std::string& task_id,
                                              std::vector<geometry_msgs::Point>& visited_cells)
{
  std::string key = "zone_coverage_task:" + task_id;

  if (!redis_->exists(key))
    return false;

  key = "zone_coverage_task:" + task_id + ":cells";
  std::vector<std::string> visited_cells_str;
  redis_->smembers(key, std::back_inserter(visited_cells_str));

  for (const std::string& visited_cell_str : visited_cells_str)
  {
    geometry_msgs::Point visited_cell;
    coordinateStringToPoint(visited_cell_str, visited_cell);
    visited_cells.push_back(visited_cell);
  }

  return true;
}

void ZoneCoverageRedisClient::deleteTask(const std::string& task_id)
{
  std::string key = "zone_coverage_task:" + task_id;
  redis_->del(key);
}

bool ZoneCoverageRedisClient::taskExists(const std::string& task_id)
{
  std::string key = "zone_coverage_task:" + task_id;
  if (!redis_->exists(key))
    return false;
  return true;
}

void ZoneCoverageRedisClient::coordinateStringToPoint(const std::string& in, geometry_msgs::Point& out)
{
  size_t delimiter_pos = in.find(",");
  std::string x_str = in.substr(0, delimiter_pos);
  std::string y_str = in.substr(delimiter_pos + 1);

  out.x = std::stod(x_str);
  out.y = std::stod(y_str);
  out.z = 0.0;
}