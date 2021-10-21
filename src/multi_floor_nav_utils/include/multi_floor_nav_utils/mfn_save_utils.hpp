#include <ros/ros.h>
#include <iostream>
#include <string>
#include <fstream>
#include "movel_seirios_msgs/MFNSaveTransitPoint.h"
#include "movel_seirios_msgs/MFNSaveMapNames.h"

using namespace std;

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

    MFNSaveUtils(ros::NodeHandle* nodehandle);

    template <typename param_type>
    bool load_param_util(std::string param_name, param_type& output);
    bool loadParams();
    void initializeServers();
    bool MFNSaveTransitPointsHandle(movel_seirios_msgs::MFNSaveTransitPoint::Request& ,movel_seirios_msgs::MFNSaveTransitPoint::Response& );
    bool MFNSaveNodeNamesHandle(movel_seirios_msgs::MFNSaveMapNames::Request& ,movel_seirios_msgs::MFNSaveMapNames::Response& );
};