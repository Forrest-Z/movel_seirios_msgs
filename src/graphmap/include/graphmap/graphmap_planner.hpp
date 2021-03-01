#ifndef graphmap_planner_h
#define graphmap_planner_h

#include <ros/ros.h>

#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <navfn/navfn_ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <movel_seirios_msgs/pathdistance.h>
#include <movel_seirios_msgs/edge_info.h>
#include <movel_seirios_msgs/get_graph_plan.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>
#include <movel_seirios_msgs/StringTrigger.h>

#include <graphmap/graphmap.hpp>


namespace graph_planner 
{

class GraphPlanner : public nav_core::BaseGlobalPlanner
{
public:
  GraphPlanner();
  ~GraphPlanner();
  GraphPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

  void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped &start,
                const geometry_msgs::PoseStamped &goal,
                std::vector<geometry_msgs::PoseStamped> &plan);

  bool makePlanServ(const geometry_msgs::PoseStamped &start,
                const geometry_msgs::PoseStamped &goal,
                std::vector<geometry_msgs::PoseStamped> &plan,std::list<Vx>& path_list_1);

  bool edge_display_cb(movel_seirios_msgs::edge_info::Request& req, movel_seirios_msgs::edge_info::Response& resp);

  bool make_plan_servicecb(movel_seirios_msgs::get_graph_plan::Request& req, movel_seirios_msgs::get_graph_plan::Response& resp);
  bool pathdist_servicecb(movel_seirios_msgs::pathdistance::Request& req, movel_seirios_msgs::pathdistance::Response& resp);
  bool change_graph_servicecb(movel_seirios_msgs::StringTrigger::Request& req, movel_seirios_msgs::StringTrigger::Response& resp);
  void odometryCb(const nav_msgs::Odometry msg);

  void initGraph();

  void visGraph();

  void visPath(std::vector<geometry_msgs::PoseStamped> &plan);

  bool checkLOS(geometry_msgs::PoseStamped &src,
                geometry_msgs::PoseStamped &tgt,
                float los_factor=1.5);

  bool addEdgeLOS(Vx src,bool start,const geometry_msgs::PoseStamped &pose,float los_factor=1.5);

  float getyawfromquaternion(const geometry_msgs::Quaternion msg);

private:
  ros::NodeHandle nh_;
  bool inited_;
  GraphMap gm_;
  ros::Publisher vis_graph_pub_;
  ros::Publisher vis_plan_pub_;
  float goal_threshold_;
  float k_connect_;
  std::vector<geometry_msgs::PoseStamped> active_plan_;
  std::list<Vx> path_list;
  nav_msgs::Odometry latest_odom_;
  bool have_active_goal_;
  bool serv_request;
  navfn::NavfnROS navfn_planner_;
  ros::ServiceServer make_plan_srv_;
  ros::ServiceServer path_distance_srv_;
  ros::ServiceServer edge_display_srv_;
  ros::ServiceServer change_graph_srv_;
  ros::Subscriber odom_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_ear_;
};

};

#endif
