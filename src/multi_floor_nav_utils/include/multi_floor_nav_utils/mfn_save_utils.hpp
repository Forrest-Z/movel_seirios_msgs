#include <ros/ros.h>
#include <iostream>
#include <string>
#include <fstream>
#include <filesystem>
#include "movel_seirios_msgs/MFNSaveTransitPoint.h"
#include "movel_seirios_msgs/MFNSaveMapNames.h"
#include "movel_seirios_msgs/MFNGetMapId.h"

using namespace std;
namespace fs = std::filesystem;

class MFNSaveUtils{
    private:
    // ROS params
    string p_graph_folder_path_;
    string p_transit_folder_path_;

    // variables
    ofstream node_names_file_;
    ofstream transit_points_file_;
    string name_;

    public:
    ros::NodeHandle n_;
    ros::ServiceServer save_node_names_service_;
    ros::ServiceServer save_transit_points_service_;
    ros::ServiceServer get_map_id_service_; 

    MFNSaveUtils(ros::NodeHandle* nodehandle);

    template <typename param_type>
    bool load_param_util(std::string param_name, param_type& output);
    bool loadParams();
    void initializeServers();
    bool MFNSaveTransitPointsHandle(movel_seirios_msgs::MFNSaveTransitPoint::Request& ,movel_seirios_msgs::MFNSaveTransitPoint::Response& );
    bool MFNSaveNodeNamesHandle(movel_seirios_msgs::MFNSaveMapNames::Request& ,movel_seirios_msgs::MFNSaveMapNames::Response& );
    string getMapIdFromTransitPoint(string& transit_point_file_path, string& map_id, geometry_msgs::Pose& transit_point_pose);
    bool getMapIdHandle(movel_seirios_msgs::MFNGetMapId::Request& req, movel_seirios_msgs::MFNGetMapId::Response& res);
};