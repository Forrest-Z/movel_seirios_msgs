#ifndef throttle_speed_hpp
#define throttle_speed_hpp

#include <ros/ros.h>
#include <movel_seirios_msgs/ThrottleSpeed.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>

class ThrottleSpeed {
  public:
    ThrottleSpeed();
    ~ThrottleSpeed(){};
    bool setupParams();
    bool setupTopics();

  private:
    ros::NodeHandle nh;
    ros::ServiceServer speed_limiter_serv_;

    bool should_limit_speed;
    bool onThrottleSpeed(movel_seirios_msgs::ThrottleSpeed::Request& req, movel_seirios_msgs::ThrottleSpeed::Response& res);

    ros::ServiceClient set_throttled_speed; // dynamic reconfigure speed
    bool reconfigureSpeed(bool should_reconfigure, double percentage=1.0);
    
    //double linear_speed;
    //double angular_speed;
    double linear_speed_default;
    double angular_speed_default;
    std::string local_planner;
    std::string move_base_name;
    std::string linear_topic;
    std::string angular_topic;

};
#endif