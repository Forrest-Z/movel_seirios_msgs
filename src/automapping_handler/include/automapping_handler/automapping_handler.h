#ifndef AUTOMAPPING_HANDLER_H
#define AUTOMAPPING_HANDLER_H

#include <ros/ros.h>
#include <task_supervisor/plugins/task_handler.h>
#include <task_supervisor/common.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
#include <rosgraph_msgs/Log.h>
#include <geometry_msgs/Twist.h>
#include <movel_seirios_msgs/Task.h>
#include <movel_seirios_msgs/StringTrigger.h>
#include <ros_utils/ros_utils.h>
#include <movel_seirios_msgs/Reports.h>

namespace task_supervisor
{
class AutomappingHandler: public TaskHandler
{
private:
    /**
     * @brief Callback method for save_map service
     */
    bool onSaveServiceCall(movel_seirios_msgs::StringTrigger::Request& req, movel_seirios_msgs::StringTrigger::Response& res);

    /**
     * @brief Callback for checking if all launched nodes are ready
     */
    bool onStatus(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    /**
     * @brief Callback for async map saving, does not stop mapping
     */
    bool onAsyncSave(movel_seirios_msgs::StringTrigger::Request& req, movel_seirios_msgs::StringTrigger::Response& res);

    /**
     * @brief Save map using map_saver
     */
    bool saveMap(std::string map_name);

    /**
     * @brief Method to start automapping, which is automapping_handler's main purpose
     * @return Returns a boolean indicating success
     */
    bool runAutomapping();

    /**
     * @brief Setup handler method called by task_supervisor during initialization of plugin
     * @return Returns a boolean indicating success
     */
    bool setupHandler();

    /**
     * @brief Load parameters on setup of handler
     * @return Returns a boolean indicating success
     */
    bool loadParams();

    /**
     * @brief Listens to log messages from /explore node to determine if exploration ended
     */
    void logCB(const rosgraph_msgs::LogConstPtr& msg);

    ros::Publisher stopped_pub_;
    ros::Publisher vel_pub_;
    
    // ROS params
    double p_loop_rate_;
    double p_save_timeout_;
    std::string p_map_topic_;
    std::string p_automapping_launch_package_;
    std::string p_automapping_launch_file_;            
    
    // Internal variables
    bool saved_;
    unsigned int automapping_launch_id_ = 0;

public:
    AutomappingHandler();

    virtual ReturnCode runTask(movel_seirios_msgs::Task &Task, std::string &error_message);
    virtual bool healthCheck();
};
}

#endif
