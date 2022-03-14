#include <cstdlib>
#include <cmd_vel_mux/speed_limit_zones.h>
#include <movel_hasp_vendor/license.h>

SpeedLimitZones::SpeedLimitZones() {
  if (!setupTopics()) {
    ROS_INFO("failed to setup topics");
    return;
  }
}

bool SpeedLimitZones::setupTopics() {
  draw_zones = nl.advertiseService("reduce_speed_zone", &SpeedLimitZones::polygonCb,this);
  return true;
}

// function to draw speed limit zones
bool SpeedLimitZones::polygonCb(movel_seirios_msgs::ZonePolygon::Request &req, movel_seirios_msgs::ZonePolygon::Response &res){
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