#ifndef speed_limit_zones_hpp
#define speed_limit_zones_hpp

#include <ros/ros.h>
#include <movel_seirios_msgs/ZonePolygon.h>
#include <movel_seirios_msgs/ThrottleSpeed.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//using std::string;
struct Point {
  float x;
  float y;
};

// change naming of Polygon to SpeedZone; don't confuse with geometry_msgs::Polygon
struct SpeedZone {
  std::vector<Point> zone_poly; // actual poly
  double percent; // % to cut speed by
};

class SpeedLimitZones {
  public:
    SpeedLimitZones();
    ~SpeedLimitZones(){};
    bool setupTopics();

  private:
    ros::NodeHandle nh;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::vector<SpeedZone> speed_zones; // array of speed limit zones
    ros::ServiceServer draw_zones;
    bool polygonCb(movel_seirios_msgs::ZonePolygon::Request &req, movel_seirios_msgs::ZonePolygon::Response &res);
    ros::Timer control_timer_;
    void odomCb(const ros::TimerEvent &msg);
    bool getRobotPose(geometry_msgs::PoseStamped &pose);
    bool inZone();
    // functions to check if a pt is inside a zone
    bool isInside(std::vector<Point> polygon, int n, Point p); 
    bool doIntersect(Point p1, Point q1, Point p2, Point q2); 
    int orientation(Point p, Point q, Point r); 
    bool onSegment(Point p, Point q, Point r); 
    ros::ServiceClient reduce_speed_client; // call limit_robot_speed service
    movel_seirios_msgs::ThrottleSpeed throttle_srv;
};

#endif