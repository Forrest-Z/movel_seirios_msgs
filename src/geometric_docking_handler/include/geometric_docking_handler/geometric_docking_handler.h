#ifndef GEOMETRIC_DOCKING_HANDLER_H
#define GEOMETRIC_DOCKING_HANDLER_H

#include <ros/ros.h>
#include <task_supervisor/plugins/task_handler.h>
#include <task_supervisor/common.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <movel_seirios_msgs/Task.h>
#include <movel_seirios_msgs/Reports.h>
#include <ros_utils/ros_utils.h>

namespace geometric_docking_handler
{
class GeometricDockingHandler: public task_supervisor::TaskHandler
{
private:
    // Setup parameters and topics
    bool setupHandler();

    // Load parameters
    bool loadParams();

    // Function to launch docking launch file
    void startDocking();

    // Stop the docking by killing all nodes
    void stopAllLaunch();

    // Triggered when user cancelled the task and it will call dockCancelCb
    bool cancelDocking();

    // Callback of docking status
    void dockStatusCb(const std_msgs::UInt8ConstPtr& msg);

    // Pause Function
    bool pauseDocking();

    // Resume function
    bool resumeDocking();

    void healthReportCb(const movel_seirios_msgs::ReportsConstPtr &msg);
    bool healthCheck();

    // Variables
    unsigned int geometric_launch_id = 0;
    int dock_status_;
    double p_loop_rate_;
    std::string p_geometric_launch_package_;
    std::string p_geometric_launch_file_;            
    std::string p_geometric_launch_node_;

    ros::Subscriber health_report_sub_;

    bool loc_is_healthy_ = true;
    
    bool docking_;

public:
    GeometricDockingHandler();

    virtual task_supervisor::ReturnCode runTask(movel_seirios_msgs::Task& task, std::string& error_message);

    ros::ServiceClient start_dock;
    ros::ServiceClient stop_dock;
    ros::ServiceClient pause_dock;
    ros::ServiceClient resume_dock;
    ros::Subscriber dock_status;
};
}

#endif