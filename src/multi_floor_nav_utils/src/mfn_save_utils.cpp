#include "multi_floor_nav_utils/mfn_save_utils.hpp"

// Constructor
MFNSaveUtils::MFNSaveUtils(ros::NodeHandle* nodehandle):n_(*nodehandle){
    name_ = "mfn_save_utils";
    if (!loadParams()){
        ROS_FATAL("[%s] Error during parameter loading. Shutting down.", name_.c_str());
    }
    initializeServers();
}

void MFNSaveUtils::initializeServers(){
    save_transit_points_service_ = n_.advertiseService("/multi_floor_nav_utils/save_transit_points", &MFNSaveUtils::MFNSaveTransitPointsHandle, this);
    save_node_names_service_ = n_.advertiseService("/multi_floor_nav_utils/save_map_names", &MFNSaveUtils::MFNSaveNodeNamesHandle, this);
}

template <typename param_type>
bool MFNSaveUtils::load_param_util(string param_name, param_type& output)
{
  if (!n_.getParam(param_name, output)) {
    ROS_ERROR("[%s] Failed to load parameter: %s", name_.c_str(), param_name.c_str());
    return false;
  }
  else {  
    if (is_same<param_type, bool>::value) {   // bool
      ROS_INFO_STREAM("[" << name_ << "] " << param_name << ": " << output ? "true" : "false");
    }
    else {   // all others 
      ROS_INFO_STREAM("[" << name_ << "] " << param_name << ": " << output);
    }
    return true;
  }
}

bool MFNSaveUtils::loadParams(){
  ROS_WARN("[%s] Loading of plugin parameters by ros_utils has not been implemented. Loading directly from Parameter "
           "Server instead.", name_.c_str());
  if (!load_param_util("/task_supervisor/multi_floor_navigation_handler/mfn_graph_folder_path", p_graph_folder_path_)) { return false; }
  if (!load_param_util("/task_supervisor/multi_floor_navigation_handler/mfn_transit_folder_path", p_transit_folder_path_)) { return false; }
  return true;
}

bool MFNSaveUtils::MFNSaveTransitPointsHandle(movel_seirios_msgs::MFNSaveTransitPoint::Request& req,movel_seirios_msgs::MFNSaveTransitPoint::Response& res){
    try{
        string transit_file_name = p_transit_folder_path_ + "/" + req.from_map + "_" + req.to_map;
        transit_points_file_.open(transit_file_name);
        
        transit_points_file_ << req.pose.position.x << endl;
        transit_points_file_ << req.pose.position.y << endl;
        transit_points_file_ << req.pose.position.z << endl;
        transit_points_file_ << req.pose.orientation.x << endl;
        transit_points_file_ << req.pose.orientation.y << endl;
        transit_points_file_ << req.pose.orientation.z << endl;
        transit_points_file_ << req.pose.orientation.w << endl;

        transit_points_file_.close();
        res.transit_file_path = transit_file_name;
    }
    catch(...){
        ROS_ERROR("[%s] Failed to save transit point to file", name_.c_str());
        return false;
    }
    return true;
}

bool MFNSaveUtils::MFNSaveNodeNamesHandle(movel_seirios_msgs::MFNSaveMapNames::Request& req,movel_seirios_msgs::MFNSaveMapNames::Response& res){
    try{
        string nodes_file_name = p_graph_folder_path_ + "/node_names.txt";
        node_names_file_.open(nodes_file_name);

        for(int i = 0; i < req.map_names.size(); i++){
            node_names_file_ << req.map_names[i] << endl;
        }

        node_names_file_.close();
        res.node_names_file_path = nodes_file_name;
    }
    catch(...){
        ROS_ERROR("[%s] Failed to save node names to file", name_.c_str());
        return false;
    }
    return true;
}