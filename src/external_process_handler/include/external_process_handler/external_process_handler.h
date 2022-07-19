#ifndef EXTERNAL_PROCESS_HANDLER_LAUNCH_H
#define EXTERNAL_PROCESS_HANDLER_LAUNCH_H

#include <ros/ros.h>
#include <future>
#include <task_supervisor/plugins/task_handler.h>
#include <task_supervisor/common.h>
#include <ros_utils/ros_utils.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <movel_seirios_msgs/StringTrigger.h>
#include <movel_seirios_msgs/Reports.h>
#include <movel_seirios_msgs/Task.h>
#include <thread>

namespace external_process_handler
{
    class ExternalProcessHandler: public task_supervisor::TaskHandler
    {

        private:
        /**
         * @brief Load parameters on setup of handler
         * @return Returns a boolean indicating success
         */
        bool loadParams();

        /**
         * @brief Setup handler method called by task_supervisor during initialization of plugin
         * @return Returns a boolean indicating success
         */
        bool setupHandler();
        
        /**
         * @brief Function called to start a launch file mentioned by client if user mentioned the launch req as true
         * @return Returns a boolean indicating success
         */
        bool runLaunch();

        /**
         * @brief Function called to stop the launch file and also call service stop if user set the service function true
         * @return Returns a boolean indicating success
         */
        bool stopProcess();

        /**
         * @brief Call back funtion to know the status of the service or function call, just update the process_status variable
         * @return returns none
         */
        void clientStatusCb(const std_msgs::UInt8ConstPtr& msg);

        /**
         * @brief Function called to cancel the process if the service get 
         * @return Returns a boolean indicating success
         */
        bool cancelProcess();

        /**
         * @brief Function called to check the health of the nodes 
         * @return Returns a boolean indicating success
         */
        bool healthCheck();

        /**
         * @brief Function called to check the health of the service 
         * @return Returns a boolean indicating success
         */
        void check_service_health_();


        // Subscribers
            ros::Subscriber status_sub;


        // Publishers
            ros::Publisher cancel_process_;


        // Services

            ros::ServiceClient start_process_srv_;
            ros::ServiceClient stop_process_srv_;

        // Internal Variables

            bool isProcessHealthy_;
            bool ins_hel_stop_=false;
            bool ins_serv_stop_=false;
            uint8_t process_status_;
            unsigned int process_launch_id = 0;
            std::string error_message_int;

        //ROS Params

            bool p_service_req_;
            std::string p_service_start_;
            std::string p_service_stop_;
            
            bool p_launch_req_;
            std::string p_launch_package_;
            std::string p_launch_file_;
            std::string p_launch_node_;

            double p_loop_rate_ = 0;

            bool p_topic_cancel_req;
            std::string p_topic_process_cancel;
            std::string p_service_status;
            std::string p_service_start_msg;
            std::string p_service_stop_msg;
            std::string srv_start_msg;
            std::string srv_stop_msg;

        public:
        
        //Constructor
            
            ExternalProcessHandler(){};
            ~ExternalProcessHandler(){};
            
        // Method called by task_supervisor when any extrenal process task is received
        
            virtual task_supervisor::ReturnCode runTask(movel_seirios_msgs::Task& Task, std::string& error_message);




    };
}
#endif