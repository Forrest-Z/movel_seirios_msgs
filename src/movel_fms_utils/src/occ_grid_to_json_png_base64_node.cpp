#include <iostream>
#include <Magick++.h>
#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"


static ros::Subscriber map_sub;
static ros::Publisher map_string_pub;
static std::string map_string_json_latest;


std::string map_to_pgm_string(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  // write header
  std::ostringstream header;
  header << std::setprecision(3) <<
    "P5\n" <<
    "# CREATOR: Movel AI " << map->info.resolution << " m/pix\n" <<
    map->info.width << " " << map->info.height << '\n' <<
    "255\n";
  // prep string for map values
  std::string pgm = header.str();
  pgm.reserve( (map->info.height * map->info.width) + pgm.size() );
  // write map values
  unsigned char c_occupied = 000;
  unsigned char c_free = 254;
  unsigned char c_unknown = 205;
  for(unsigned int y = 0; y < map->info.height; y++) {
    for(unsigned int x = 0; x < map->info.width; x++) {
      unsigned int i = x + (map->info.height - y - 1) * map->info.width;
      // occupied
      if (map->data[i] == 100) { pgm += c_occupied; }
      // free 
      else if (map->data[i] == 0) { pgm += c_free; }
      // unknown
      else { pgm += c_unknown; }
    }
  }
  return pgm;
}


std::string pgm_as_png_base64(const std::string& pgm)
{
  // convret to png
  Magick::Image image(Magick::Blob(pgm.c_str(), pgm.size()));
  image.magick("png");   // change to png
  Magick::Blob blob; 
  image.write(&blob);
  std::cout << "pgm : " << pgm.size() << ", png : "  << blob.length() << '\n';
  return blob.base64();
}


std::string map_to_json(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  // occ grid map to png base64
  const std::string& pgm = map_to_pgm_string(map);
  const std::string& png = pgm_as_png_base64(pgm);
  // write json
  std::ostringstream json;
  json << std::setprecision(10) <<
    '{' << '\n' << 
    "  \"height\": " << map->info.height << ',' << '\n' << 
    "  \"width\": " << map->info.width << ',' << '\n' << 
    "  \"resolution\": " << map->info.resolution << ',' << '\n' << 
    "  \"origin\": {" << '\n' <<
    "    \"position_x\":" << map->info.origin.position.x << ',' << '\n' << 
    "    \"position_y\":" << map->info.origin.position.y << ',' << '\n' << 
    "    \"position_z\":" << map->info.origin.position.z << ',' << '\n' << 
    "    \"orientation_x\":" << map->info.origin.orientation.x << ',' << '\n' << 
    "    \"orientation_y\":" << map->info.origin.orientation.y << ',' << '\n' << 
    "    \"orientation_z\":" << map->info.origin.orientation.z << ',' << '\n' << 
    "    \"orientation_w\":" << map->info.origin.orientation.w << '\n' << 
    "  }," << '\n' << 
    "  \"uri\": \"data:image/png;base64," << png << '"' << '\n' << 
    '}';
  return json.str();
}


void mapCB(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  std_msgs::String msg;
  map_string_json_latest = map_to_json(map);  
  msg.data = map_string_json_latest;
  map_string_pub.publish(msg);
}


bool mapSrvCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  res.message = map_string_json_latest;
  res.success = true;  
  return true;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "occ_grid_to_json_png_base64_node");
  Magick::InitializeMagick(*argv);

  ros::NodeHandle nh_handler_;
  map_sub = nh_handler_.subscribe("/map", 1, mapCB);
  map_string_pub = nh_handler_.advertise<std_msgs::String>("/map/uri/json", 1, true);
  map_string_service_ = nh_.advertiseService("/map/uri/json", mapSrvCB);

  ros::spin();    
  return 0;
}
