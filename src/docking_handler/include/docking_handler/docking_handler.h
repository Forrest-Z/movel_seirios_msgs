#ifndef DOCKING_HANDLER_H
#define DOCKING_HANDLER_H

#include <ros/ros.h>
#include <task_supervisor/plugins/task_handler.h>
#include <task_supervisor/common.h>
#include <movel_seirios_msgs/Task.h>
#include <ros_utils/ros_utils.h>
#include <camera_lidar_docking_pallet/StartAutoDock.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <movel_seirios_msgs/Reports.h>

namespace docking_handler
{
    class DockingHandler: public task_supervisor::TaskHandler
    {
        private:
            //Initialize the parameters
            bool setupHandler();

            //Load Params
            bool loadParams();

            // Launch docking package
            bool runDocking();
            bool stopDocking();
            bool pauseDocking();
            bool resumeDocking();
            bool cancelDocking();
            
            // Callback function
            void dockStatusCb(const std_msgs::UInt8ConstPtr& msg);
            void locReportingCB(const movel_seirios_msgs::Reports::ConstPtr& msg);

            unsigned int docking_launch_id = 0;
            bool docking_;
            uint8_t dock_status_;
            bool isDockingHealthy_;
            bool isLocHealthy_;

            //ROS Params
            std::string p_docking_launch_package_;
            std::string p_docking_launch_file_;
            std::string p_docking_launch_node_;

            ros::ServiceClient start_dock;
            ros::Publisher pause_dock;
            ros::Publisher resume_dock;
            ros::Publisher cancel_dock;
            ros::Subscriber status_sub;

            ros::Subscriber loc_report_sub_;

        public:
            //Constructor
            DockingHandler();

            // Method called by task_supervisor when docking task is received
            virtual task_supervisor::ReturnCode runTask(movel_seirios_msgs::Task& Task, std::string& error_message);
            virtual bool healthCheck();



    };
}
#endif