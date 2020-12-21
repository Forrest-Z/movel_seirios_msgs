#ifndef MAP_UPDATE_H_
#define MAP_UPDATE_H_

#include <ros/ros.h>
#include <stdint.h>
#include "nav_msgs/OccupancyGrid.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "nav_msgs/GetMap.h"
#include "std_srvs/Empty.h"

class MapUpdater
{
public:
  MapUpdater();

  void globalMapCallback(const nav_msgs::OccupancyGrid& globalmap_msg);
  void localMapCallback(const nav_msgs::OccupancyGrid& localmap_msg);
  bool restoreMap(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void updateMap();
  void readParams();

protected:
  ros::Publisher pub_updated_map;
  ros::Subscriber sub_global_map;
  ros::Subscriber sub_local_map;

  ros::ServiceClient client_map;
  ros::ServiceServer service_map_restore;

  nav_msgs::OccupancyGrid dynamicmap;
  nav_msgs::OccupancyGrid staticmap;
  nav_msgs::OccupancyGrid globalmap;
  nav_msgs::OccupancyGrid localmap;
  map_msgs::OccupancyGridUpdate updatemap;

  float dx, dy, nx, ny;
  int x_trav, y_trav;
  int id_start;
  int pro_loc, pro_glo, pro_sta;
  int new_width, new_height;
  bool initial_ = true;
  int trim_width, trim_height;

  bool map_once_;
  int loc_th_, glo_th_;
  float update_width_, update_height_;
};
#endif