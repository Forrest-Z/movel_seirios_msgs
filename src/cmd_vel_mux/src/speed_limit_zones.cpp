#include <cstdlib>
#include <cmd_vel_mux/speed_limit_zones.h>
#include <movel_hasp_vendor/license.h>

SpeedLimitZones::SpeedLimitZones() : tf_listener_(tf_buffer_) {
  if (!setupTopics()) {
    ROS_ERROR("failed to setup topics");
    return;
  }
  control_timer_ = nl.createTimer(ros::Duration(0.5), &SpeedLimitZones::odomCb, this);
}

bool SpeedLimitZones::setupTopics() {
  draw_zones = nl.advertiseService("reduce_speed_zone", &SpeedLimitZones::polygonCb,this);
  return true;
}

void SpeedLimitZones::odomCb(const ros::TimerEvent &msg) {
  inZone();
}

// function to draw speed limit zones and set % to slow down speed by
bool SpeedLimitZones::polygonCb(movel_seirios_msgs::ZonePolygon::Request &req, movel_seirios_msgs::ZonePolygon::Response &res) {
  std::vector<Point> polygons; // a polygon is just a vector of points..
  
  for (int i=0; i < static_cast<int>(req.zone_data.size()); ++i) {
    polygons.clear();
    for (int j=0; j < static_cast<int>(req.zone_data[i].polygons.points.size()); ++j) {
      float p1 = req.zone_data[i].polygons.points[j].x;
      float p2 = req.zone_data[i].polygons.points[j].y;
      Point point1 = {p1, p2};
      polygons.push_back(point1);
      ROS_INFO("[callback] zone: %ld  points: %ld  polygon: %ld", req.zone_data.size(), req.zone_data[0].polygons.points.size(), polygons.size());
    }
    Polygon zone_polygon;
    zone_polygon = (Polygon){.zones_list = polygons, .percent = req.zone_data[i].percentage_reduction};
    
    speed_zones.push_back(zone_polygon); // multiple polygons
  }

  res.success = true;
  return true;
}

bool SpeedLimitZones::getRobotPose(geometry_msgs::PoseStamped& pose) {
  try
  {
    geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
    tf2::Transform position;
    tf2::fromMsg(transform.transform, position);
    pose.pose.position.x = position.getOrigin()[0];
    pose.pose.position.y = position.getOrigin()[1];
    pose.pose.orientation.x = position.getRotation()[0];
    pose.pose.orientation.y = position.getRotation()[1];
    pose.pose.orientation.z = position.getRotation()[2];
    pose.pose.orientation.w = position.getRotation()[3];

    pose.header.stamp = transform.header.stamp;
    pose.header.frame_id = "map";
    return true;
  }
  catch (tf2::TransformException &e)
  {
    return false;
  }
}

bool SpeedLimitZones::inZone() {
  /* function to check if robot is in speed limit zone
   * if true, call throttle speed by given %
   */
  return true;
}

int main(int argc, char **argv) {
  #ifdef MOVEL_LICENSE
    MovelLicense ml;
    if (!ml.login())
      return 1;
  #endif

  ros::init(argc, argv, "speed_limit_zones");
  
  SpeedLimitZones spd_zone;
  ros::spin();
 
  #ifdef MOVEL_LICENSE
    ml.logout();
  #endif

  return 0;
}