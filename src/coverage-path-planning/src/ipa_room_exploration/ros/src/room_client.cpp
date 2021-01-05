#include <iostream>
#include <ros/ros.h>
#include <cstdlib>
#include "ipa_room_exploration/RoomExplorationClient.h"
using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "room_client");

  ros::NodeHandle n;
  ros::ServiceClient client =
      n.serviceClient<ipa_room_exploration::RoomExplorationClient>("/room_exploration_client/start");

  ipa_room_exploration::RoomExplorationClient srv;
  //      srv.request.path_to_cropped_map = "/home/movel/coverage-path-planning/src/map_files/test_image.png";
  srv.request.path_to_cropped_map = "/home/movel/sim_map2_crop.png";

  srv.request.path_to_coordinates_txt = "/home/movel/coverage-path-planning/src/map_files/coordinates.txt";

  struct timeval start, end;

  gettimeofday(&start, NULL);

  if (client.call(srv))
  {
    std::cout << "executed" << std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call service ");
    return 1;
  }

  gettimeofday(&end, NULL);

  double time_taken;

  time_taken = (end.tv_sec - start.tv_sec) * 1e6;
  time_taken = (time_taken + (end.tv_usec - start.tv_usec)) * 1e-6;

  cout << "Time taken by program is : " << fixed << time_taken << setprecision(6);
  cout << " sec" << endl;
  return 0;
}
