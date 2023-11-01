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
    get_map_id_service_ = n_.advertiseService("/multi_floor_nav_utils/get_map_id", &MFNSaveUtils::getMapIdHandle, this);
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

string MFNSaveUtils::getMapIdFromTransitPoint(string& transit_point_file_path, string& map_id, geometry_msgs::Pose& transit_point_pose)
{
  if (map_id == "")
    return "";

  try
  {
    for (const auto& entry: fs::directory_iterator(transit_point_file_path))
    {
      string file_name = entry.path().filename().string();
      if (file_name.find(map_id) != string::npos)
      {
        ifstream transit_point_file;
        transit_point_file.open(entry.path());
        string line;
        geometry_msgs::Pose pose;
        int i = 0;
        while (getline(transit_point_file, line))
        {
          switch (i)
          {
            case 0:
              pose.position.x = stod(line);
              break;
            case 1:
              pose.position.y = stod(line);
              break;
            case 2:
              pose.position.z = stod(line);
              break;
            case 3:
              pose.orientation.x = stod(line);
              break;
            case 4:
              pose.orientation.y = stod(line);
              break;
            case 5:
              pose.orientation.z = stod(line);
              break;
            case 6:
              pose.orientation.w = stod(line);
              break;
            default:
              break;
          }
          i++;
        }
        transit_point_file.close();

        // since we are using double, it might be have some error when rounding the number. so we have to set the error tolerance
        double map_id_error_tolerance {0.005}; // set tolerance to 0.005
        n_.param("/task_supervisor/multi_floor_navigation_handler/get_map_id_error_tolerance", map_id_error_tolerance, 0.005); //change the value of map_id_error_tolerance in launch file if needed
        if (abs(transit_point_pose.position.x - pose.position.x) <= map_id_error_tolerance &&
          abs(transit_point_pose.position.y - pose.position.y) <= map_id_error_tolerance &&
          abs(transit_point_pose.position.z - pose.position.z) <= map_id_error_tolerance &&
          abs(transit_point_pose.orientation.x - pose.orientation.x) <= map_id_error_tolerance &&
          abs(transit_point_pose.orientation.y - pose.orientation.y) <= map_id_error_tolerance &&
          abs(transit_point_pose.orientation.z - pose.orientation.z) <= map_id_error_tolerance &&
          abs(transit_point_pose.orientation.w - pose.orientation.w) <= map_id_error_tolerance)
        {
          // e.g: file name in transit points folder: mapid1_mapid2 or mapid2_mapid1
          // we want to eliminate mapid1, 
          // so we find the position of mapid1 in the file name and then erase it
          size_t pos_id = file_name.find(map_id);
          if (pos_id != string::npos)
            file_name.erase(pos_id, map_id.length());
          else
            return "";
          
          // after that, we remove "_" in that file name
          size_t pos_separator = file_name.find("_");
          if (pos_separator != string::npos)
            file_name.erase(pos_separator,1);
          else
            return "";

          return file_name;
        }
      }
    }
  }
  catch(const exception& e) { ROS_ERROR("Exception: %s", e.what());}
    return "";
}

bool MFNSaveUtils::getMapIdHandle(movel_seirios_msgs::MFNGetMapId::Request& req, movel_seirios_msgs::MFNGetMapId::Response& res)
{
  string map_id = req.map;
  geometry_msgs::Pose transit_point_pose = req.transit_pose;
  string file_name = getMapIdFromTransitPoint(p_transit_folder_path_, map_id, transit_point_pose);
  res.map_id = file_name;

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