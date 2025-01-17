#ifndef speed_limit_zones_hpp
#define speed_limit_zones_hpp

#include <ros/ros.h>
#include <ros_utils/ros_utils.h>
#include <movel_seirios_msgs/SpeedZones.h>
#include <movel_seirios_msgs/SpeedZone.h>
#include <movel_seirios_msgs/SetSpeed.h>
#include <movel_seirios_msgs/GetSpeed.h>
#include <std_srvs/Trigger.h>
#include <vector>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <geometry_msgs/PolygonStamped.h>

struct Point {
  double x; // Float64 = double; Float32 = float - robot Pose message is a double
  double y;
};


// change naming of Polygon to SpeedZone; don't confuse with geometry_msgs::Polygon
struct SpeedZone {
  std::vector<Point> zone_poly; // actual poly
  double linear;   // linear limit
  double angular;   // angular limit
};


class SpeedLimitZones 
{
  public:
    SpeedLimitZones();
    ~SpeedLimitZones(){};
    void setupTopics();

  private:
    ros::NodeHandle nh;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::vector<SpeedZone> speed_zones; // array of speed limit zones
    ros::ServiceServer draw_zones;
    ros::ServiceServer clear_zones;
    ros::Timer control_timer_;
    bool polygonCb(movel_seirios_msgs::SpeedZones::Request &req, movel_seirios_msgs::SpeedZones::Response &res);
    bool clearCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response& res);
    void odomCb(const ros::TimerEvent &msg);
    bool getRobotPose(geometry_msgs::PoseStamped &pose);
    void inZone();
    // functions to check if a pt is inside a zone
    bool isInside(std::vector<Point> polygon, int n, Point p); 
    bool doIntersect(Point p1, Point q1, Point p2, Point q2); 
    int orientation(Point p, Point q, Point r); 
    bool onSegment(Point p, Point q, Point r); 
    // utilities
    bool setSpeedUtil(double linear, double angular);
    bool setZoneSpeed(double linear, double angular);
    bool setSpeed();
    // speed cache and control
    ros::ServiceClient set_speed_client_;
    bool is_in_zone_ = false;
    int in_zone_idx_ = 0;

    ros::NodeHandle nh_private_;
    ros::Publisher display_pub_;

    // ROS param
    bool debug_;
    bool loadParams();

    // display zones in rviz
    void displayZones();
};

#endif