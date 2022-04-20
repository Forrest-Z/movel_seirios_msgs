#include <ros/ros.h>
#include <ros_utils/ros_utils.h>
#include <movel_hasp_vendor/license.h>
#include <sw/redis++/redis++.h>


class Movel_Redis_Init 
{
public:
  Movel_Redis_Init();
  ~Movel_Redis_Init() = default;
  bool loadParams(std::map<std::string, std::string>& map);   
	bool initRedis();

	ros::NodeHandle nh_;
  std::string print_name_{"[Movel_Redis_Init]"};

  // loaded from rosparams
  std::map<std::string, std::string> redis_conn_vars_ { 
    {"redis_host", ""},
    {"redis_port", ""},
  };
  std::map<std::string, std::string> global_vars_ { 
    {"velo_smoother_enabled", ""},
    {"safe_teleop_enabled", ""},
    {"velo_limiter_enabled", ""},
  };	
};


Movel_Redis_Init::Movel_Redis_Init() : nh_("~")
{
  ros::Time::waitForValid();
  if(!loadParams(redis_conn_vars_)) {
    ROS_FATAL_STREAM(print_name_ << " Error loading redis connection params. Init failed... ");
    return;
  }
  if(!loadParams(global_vars_)) {
    ROS_FATAL_STREAM(print_name_ << " Error loading global variable params. Init failed... ");
    return;
  }
  if(!initRedis()) {
    ROS_FATAL_STREAM(print_name_ << " Error setting global variables in redis. Init failed... ");
    return;
  }
}


bool Movel_Redis_Init::loadParams(std::map<std::string, std::string>& map)
{
  ros_utils::ParamLoader param_loader(nh_);
  bool success = true;
  for (auto& pair : map) {
    // existance check
    if (!nh_.hasParam(pair.first)) {
      ROS_FATAL_STREAM(print_name_ << " param not found in config: " << pair.first);
      success = false;
      continue;
    }
    // load into map
    param_loader.get_required<std::string>(pair.first, pair.second);
  }
  return success;
}


bool Movel_Redis_Init::initRedis()
{
  // std::ostringstream redis_conn;
  sw::redis::ConnectionOptions opts1;
  opts1.host = redis_conn_vars_.at("redis_host");
  opts1.port = stoi(redis_conn_vars_.at("redis_port"));
  // redis_conn << redis_conn_vars_.at("redis_host") << ":" << redis_conn_vars_.at("redis_port");
  auto redis = sw::redis::Redis(opts1);
  bool success = true;
  for (const auto& pair : global_vars_) {
    try{  
      // TODO: improve error checking
      redis.set(pair.first, pair.second);
    } catch (const sw::redis::Error &e) {
      ROS_FATAL_STREAM(print_name_ << " redis could not set param: " << pair.first << " : " << pair.second);
      success = false;
    }
  }
  return success;
}



int main(int argc, char** argv)
{
  #ifdef MOVEL_LICENSE
    MovelLicense ml(10);
    if (!ml.login())
      return 1;
  #endif

  ros::init(argc, argv, "movel_redis_init");
  Movel_Redis_Init movel_redis_init_;

  #ifdef MOVEL_LICENSE
    ml.logout();
  #endif
  return 0;
}