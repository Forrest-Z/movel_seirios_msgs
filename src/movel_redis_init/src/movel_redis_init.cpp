#include <movel_redis_init/movel_redis_init.h>
using namespace sw::redis;
using namespace std;

Movel_Redis_Init::Movel_Redis_Init() : nh_("~")
{
  ros::Time::waitForValid();
  if(!loadParams())
  {
    ROS_FATAL("[Movel_Redis_Init] Error during parameter loading. Shutting down.");
  }
  if(!initRedis())
  {
    ROS_FATAL("[Movel_Redis_Init] Error during initialising global variable using redis");
  }

}

bool Movel_Redis_Init::initRedis()
{
  
  try{
      auto redis = Redis(p_connection_info_);
      redis.set(r_velo_smoother_enabled_,p_velo_smoother_enabled_);
      redis.set(r_safe_teleop_enabled_,p_safe_teleop_enabled_);
      redis.set(r_velo_limiter_enabled_,p_velo_limiter_enabled_);
      return true;
  }catch (const Error &e) {
    ROS_FATAL("[Movel_Redis_Init] Cannot set global variable using redis");
    return false;
  }
}

bool Movel_Redis_Init::loadParams()
{
  ros_utils::ParamLoader param_loader(nh_);
  param_loader.get_required("velo_smoother_enabled", p_velo_smoother_enabled_);
  param_loader.get_required("safe_teleop_enabled", p_safe_teleop_enabled_);
  param_loader.get_required("velo_limiter_enabled", p_velo_limiter_enabled_);
  param_loader.get_optional("redis_velo_smoother_enabled", r_velo_smoother_enabled_,std::string("velo_smoother_enabled"));
  param_loader.get_optional("redis_safe_teleop_enabled", r_safe_teleop_enabled_,std::string("safe_teleop_enabled"));
  param_loader.get_optional("redis_velo_limiter_enabled", r_velo_limiter_enabled_,std::string("velo_limiter_enabled"));
  param_loader.get_optional("connection_info_", p_connection_info_,std::string("tcp://127.0.0.1"));
	return param_loader.params_valid();
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