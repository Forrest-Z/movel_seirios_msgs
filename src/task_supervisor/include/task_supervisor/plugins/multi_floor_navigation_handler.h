#ifndef TASK_SUPERVISOR_MULTI_FLOOR_NAVIGATION_HANDLER_H
#define TASK_SUPERVISOR_MULTI_FLOOR_NAVIGATION_HANDLER_H

#include <ros/ros.h>
#include <task_supervisor/plugins/base/navigation_handler_base.h>

#include <nav_msgs/LoadMap.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Quaternion.h"
#include <fstream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <vector>
#include <iostream>
#include <filesystem>
#include <string>
#include "std_srvs/Empty.h"
#include <movel_seirios_msgs/StringTrigger.h>

using std::filesystem::directory_iterator;

namespace task_supervisor
{
class MultiFloorNavigationHandler : public NavigationHandlerBase
{
// private:
public:
  // ROS params
  std::string p_map_folder_path_;
  std::string p_map_nav_folder_path_;
  std::string p_graph_folder_path_;
  std::string p_transit_folder_path_;

  // variables
  std::vector<std::string> path_to_follow_;
  std::vector<std::string> map_nodes_;
  std::vector<std::vector<int>> graph_;
  std::ofstream graph_file_;
  std::string loc_map_path_;
  std::string nav_map_path_;

  // topics/services
  ros::ServiceClient map_change_client_;
  ros::ServiceClient map_nav_change_client_;
  ros::ServiceClient clear_costmap_client_;
  ros::Publisher initial_pose_pub_;
  ros::Publisher map_changed_pub_;
  ros::ServiceServer mfn_map_change_server_;

  
  bool loadParams();
  std::vector<float> getRobotPose();
  bool getNodeNames();
  bool pathGenerationHandle(std::string ,std::string ,std::vector<float> ,std::vector<float>);
  std::vector<int> DijkstraAlgo(int ,int , std::vector<std::vector<float>>);
  std::vector<int> getQueue(int[] ,int ,int );
  std::vector<float> getSwitchPoint(int ,int );
  float euclidDistance(float ,float ,float ,float );
  int miniDist(int[], bool[]);
  void printGraphConnections(std::vector<std::vector<std::string>>);
  void printGraph(std::vector<std::vector<int>>);
  std::vector<std::vector<std::string>> getTransitConnections();
  std::vector<std::string> getTransitFiles();
  void buildSaveGraph(std::vector<std::vector<std::string>> ,std::string);
  bool graphGenerationHandle();
  bool MFNChangeMapHandle(nav_msgs::LoadMap::Request& ,nav_msgs::LoadMap::Response& );
  bool clearCostmapFn();
  bool changeMapFn(std::string);
  

public:
   
   MultiFloorNavigationHandler();
  ~MultiFloorNavigationHandler(){};

   /**
     * @brief Method called by task_supervisor when a multi floor navigation task is received
     * @param task Relevant task passed to handler by task supervisor
     * @param error_message Error message returned by this handler if execution fails
     * @return ReturnCode which indicates failure, cancellation or success
     */
  virtual ReturnCode runTask(movel_seirios_msgs::Task& task, std::string& error_message);

  bool setupHandler();
  
};

}  // namespace task_supervisor

#endif
