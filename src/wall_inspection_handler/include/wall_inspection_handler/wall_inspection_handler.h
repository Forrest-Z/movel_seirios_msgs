#ifndef WALL_INSPECTION_HANDLER_H
#define WALL_INSPECTION_HANDLER_H

#include <ros/ros.h>
#include <task_supervisor/plugins/task_handler.h>
#include <task_supervisor/common.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
//#include <std_msgs/Bool.h>
//#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
//#include <rosgraph_msgs/Log.h>
//#include <geometry_msgs/Twist.h>
#include <movel_seirios_msgs/Task.h>
//#include <movel_seirios_msgs/StringTrigger.h>
#include <ros_utils/ros_utils.h>

namespace task_supervisor
{
class WallInspectionHandler: public TaskHandler
{
private:
    /**
     * @brief Callback for checking if all launched nodes are ready
     */
    bool onStatus(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

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
     * @brief Main loop for wall inspection
     * @return Returns true if task is completed successfully, false otherwise
     */
    bool runInspection();

    /**
     * @brief Callback for when wall inspection is stopped
     */
    void stoppedCB(const std_msgs::EmptyConstPtr& msg);

    /**
     * @brief Pause inspection through service call
     * @return Returns true if successfully paused, false otherwise
     */
    bool pauseInspection(bool pause);

    /**
     * @brief End inspection through service call
     * @return Returns true if successfully ended, false otherwise
     */
    bool endInspection();

    ros::Publisher stopped_pub_;
    ros::Publisher vel_pub_;
    ros::ServiceClient pause_client_;
    ros::ServiceClient end_client_;

    // ROS params
    double p_loop_rate_;
    std::string p_wall_inspection_launch_package_;
    std::string p_wall_inspection_launch_file_;            
    
    // Internal variables
    unsigned int wall_inspection_launch_id_ = 0;
    bool stopped_;

public:
    WallInspectionHandler();
    virtual ReturnCode runTask(movel_seirios_msgs::Task &Task, std::string &error_message);
};
}

#endif
