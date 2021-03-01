#include <ros/ros.h>
#include <graphmap/graphmap.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "graph_map");
  ros::NodeHandle nh;

  GraphMap gm;
  gm.parseCsv("/home/ryan/graphmap_detail.csv");
  gm.printMap();

  std::vector<Vx> path;
  gm.findPath(0, 16, path);

  // planning between points not yet in the graph
  float x0, y0, z0, x1, y1, z1;
  path.clear();
  x0 = -3.0;
  y0 = 2.1;
  z0 = 0.0;
  x1 = -3.0;
  y1 = -2.1;
  z1 = 0.0;

  bool success = gm.findPath(x0, y0, z0, x1, y1, z1, path);
  std::cout << path.size() << std::endl;

  // for (int i = 0; i < path.size(); i++) 
  // {
  //   std::cout << path[i] << " ";
  // }
  // std::cout << std::endl;
  // gm.printMap();
  std::cout << "---" << std::endl;

  path.clear();
  success = gm.findPath(x1, y1, z1, x0, y0, z0, path);
  // for (int i = 0; i < path.size(); i++) 
  // {
  //   std::cout << path[i] << " ";
  // }
  // std::cout << std::endl;

  // gm.printMap();

  return 0;
}

