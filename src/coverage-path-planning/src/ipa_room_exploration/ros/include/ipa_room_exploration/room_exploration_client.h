#include <ros/ros.h>
#include <ros/package.h>

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <ipa_building_msgs/RoomExplorationAction.h>
#include <ipa_room_exploration/dynamic_reconfigure_client.h>
#include <ipa_room_exploration/timer.h>
#include <Eigen/Dense>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include "ipa_room_exploration_msgs/RoomExplorationClient.h"

class ExplorationClient
{
protected:
  /**
   * @brief resolution
   * map resolution -- 0.05
   */
  float resolution;
  /**
   * @brief origin
   * origin of the map in map frame i.e (x, y, theta) [m]
   */
  std::vector<double> origin;
  /**
   * @brief robot_radius
   * half the width of the robot in metres [m]
   */
  double robot_radius;
  /**
   * @brief coverage_radius
   * distance between two waypoints when a robot turns and starts a new sweep [m]
   */
  double coverage_radius;
  /**
   * @brief start_pos
   * starting position of the robot in map frame (x, y, theta) [m]
   */
  std::vector<double> start_pos;
  std::string map_path;
  bool requested = false;
  actionlib::SimpleActionClient<ipa_building_msgs::RoomExplorationAction> ac;

public:
  ros::ServiceServer service;
  /**
   * constructor for initializin the map and robot related parameters
   * @brief ExplorationClient
   * @param res
   * @param origin
   * @param robot_radius
   * @param coverage_radius
   * @param start_pos
   */
  ExplorationClient(float res, std::vector<double> origin, double robot_radius, double coverage_radius,
                    std::vector<double> start_pos)
    : resolution(res)
    , origin(origin)
    , robot_radius(robot_radius)
    , coverage_radius(coverage_radius)
    , start_pos(start_pos)
    , ac("room_exploration_server", true)
  {
  }
  /**
   * @brief displayParams prints the map and robot related parameters
   *
   */

  void displayParams()
  {
    std::cout << "map resolution: " << resolution << "  origin of the map x:" << origin[0] << " y: " << origin[1]
              << " theta: " << origin[2] << "  robot radius:" << robot_radius
              << "  coverage radius: " << coverage_radius << std::endl;
    std::cout << "robot's starting position: x " << start_pos[0] << "  y:" << start_pos[1] << "  theta:" << start_pos[2]
              << std::endl;
  }

  /**
   * @brief pathPlan - service call where the room exploration client  receives map and text file containing map's
   * origin and robot's position
   * @param req
   * @param res
   * @return
   */

  bool pathPlan(ipa_room_exploration_msgs::RoomExplorationClient::Request& req,
                ipa_room_exploration_msgs::RoomExplorationClient::Response& res);

  /**
   * @brief get_resolution
   * @return  resolution
   */
  float get_resolution()

  {
    return resolution;
  }

  /**
   * @brief get_robot_radius
   * @return  robot's radius
   */
  float get_robot_radius()
  {
    return robot_radius;
  }
  /**
   * @brief get_coverage_radius
   * @return coverage radius
   */

  float get_coverage_radius()
  {
    return coverage_radius;
  }

  /**
   * @brief get_requested  if client is able to communicate to server, true is returned and a path is planned
   * @return  true/false
   */
  bool get_requested()
  {
    if (requested)
    {
      requested = false;
      return true;
    }
    else
      return false;
  }

  bool planPath();
};
