#ifndef MOVEL_REDIS_INIT_H
#define MOVEL_REDIS_INIT_H
#include <ros/ros.h>
#include <ros_utils/ros_utils.h>
#include <movel_hasp_vendor/license.h>
#include <sw/redis++/redis++.h>

class Movel_Redis_Init 
{
public:
  Movel_Redis_Init();
  ~Movel_Redis_Init(){}

private:
	ros::NodeHandle nh_;

	bool loadParams();   
	bool initRedis();

	std::string p_velo_smoother_enabled_;
	std::string p_safe_teleop_enabled_;
	std::string p_velo_limiter_enabled_;
	std::string p_connection_info_;
	std::string r_velo_smoother_enabled_;
	std::string r_safe_teleop_enabled_;
	std::string r_velo_limiter_enabled_;

};

#endif  	