#ifndef plan_inspector_hpp
#define plan_inspector_hpp

#include <cmath>
#include <ros/ros.h>
#include <signal.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <rosgraph_msgs/Log.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <movel_seirios_msgs/ObstructionStatus.h>
#include <movel_seirios_msgs/GetTaskType.h>
#include <movel_seirios_msgs/StopReconfig.h>
#include <movel_seirios_msgs/ZonePolygon.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>

using std::string;
struct zonePoint
{
  float x;
  float y;
};
struct zonePolygon
{
  std::vector<zonePoint> zones_list;
  int label;
};

class StopAtObs
{
public:
  StopAtObs();
  ~StopAtObs(){};
  bool setupTopics();
  bool setupParams();

private:
  // infrastructure
  ros::NodeHandle nl;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_ear_;
  ros::Timer control_timer_;
  bool enable_check,dublicate_enable_check;
  std::string odom_topic_,costmap_topic_;
  bool reconfigure_triggered,inside_triggered,stop_feature,use_peb_;
  
  // subscribers
  ros::Subscriber odom_sub_;
  
  // publishers

  // services
  ros::ServiceClient set_pebble_params_;
  ros::ServiceServer enable_plan_;
  ros::ServiceServer stopzone;
  ros::ServiceServer  zone_polygon_;
  ros::ServiceServer stop_obstacle_checker;

  //datatype
  double control_frequency_;
  std::vector<zonePolygon> zonePolygonVector;
  std::vector<movel_seirios_msgs::Zones> zonelist_;
  
  // callbacks
  void odomCb(const ros::TimerEvent& msg);
  bool polygonCb(movel_seirios_msgs::ZonePolygon::Request &req, movel_seirios_msgs::ZonePolygon::Response &res);
  bool isZone();
  bool enableStopfeature(bool data);
  bool onSegment(zonePoint p, zonePoint q, zonePoint r);
  int orientation(zonePoint p, zonePoint q, zonePoint r);
  bool doIntersect(zonePoint p1, zonePoint q1, zonePoint p2, zonePoint q2);
  bool isInside(std::vector<zonePoint> polygon, int n, zonePoint p);
  bool enableCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  void saveParams();
  bool getRobotPose(geometry_msgs::PoseStamped& pose);
};

#endif
