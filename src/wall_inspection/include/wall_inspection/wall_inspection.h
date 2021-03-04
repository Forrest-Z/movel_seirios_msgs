#ifndef WALL_INSPECTION_H
#define WALL_INSPECTION_H

#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros_utils/ros_utils.h>
#include <wall_inspection/target_detection.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Empty.h>
//#include <movel_api/Empty.h>
//#include <movel_api/SetBool.h>
//#include <movel_api/ActionStatus.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <move_base_msgs/MoveBaseActionResult.h>

#include <memory>

namespace wall_inspection
{
    //todo<yb>:to change to typedef
    #define SETUP 0
    #define PLAN 1
    #define PLANNED 2
    #define NAV 3
    #define SCANNING 4
    #define DONE 5
    
    class WallInspection
    {
        public:
            WallInspection();
            ~WallInspection() {};
        protected:
            //ros params
            ros::NodeHandle nh_;
            ros::NodeHandle nh_private_;
            ros::Publisher targets_pub_, targets_plan_pub_, inspection_points_pub_, inspection_plan_pub_,goal_pub_, vel_pub_, cancel_pub_, ended_pub_;
            ros::Subscriber goal_sub_,inspection_state_sub;
            ros::Timer timer_, validity_timer_;

            ros::ServiceServer pause_service_, skip_service_, terminate_service_;
            ros::ServiceClient make_plan_client_;
            tf::TransformListener listener_;
            //set up params
            bool vis_;
            double p_loop_rate_;
            double p_min_length_, p_max_gap_, p_threshold_;
            double p_radius_threshold_, p_rot_threshold_;
            double p_step_;
            double p_dist_;
            double p_inflation_;
            double p_angle_;
            double p_starvation_timeout_;
            bool   p_has_cam_;
            bool p_nearest_first_;
            std::vector<double> wall_distance_{1.0};
            std::string p_map_frame_, p_base_frame_, p_file_path_;
            //class attributes
            std::unique_ptr<TargetDetection> target_detector_;
            nav_msgs::Path plan_;
            geometry_msgs::PoseArray targets_;
            geometry_msgs::PoseArray inspection_points_;
            int t_idx_ = 0;
            int state_;
            tf::StampedTransform base_in_map_;
            ros::Time last_sent_;
            tf::StampedTransform prev_odom_;
            bool initialized_, paused_;
            double robot_radius_;
            geometry_msgs::PoseStamped current_goal_;

        private:
            bool loadParams();
            void onMainLoop(const ros::TimerEvent &main_event);
            void validateLoop(const ros::TimerEvent&);
            void getTarget();
            void reachedCallBack(const move_base_msgs::MoveBaseActionResult::ConstPtr& goal);
            void inspectionDoneCallBack(const std_msgs::Empty empty);
            void sort(nav_msgs::Path &inspection_plan, nav_msgs::Path &target_plan);

            //TODO
            //bool pause(movel_api::SetBool::Request& request, movel_api::SetBool::Response& response);
            //bool skip(movel_api::Empty::Request& request, movel_api::Empty::Response& response);
            bool pause(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
            bool skip(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);

            void endInspection();

            //TODO
            bool terminate(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);

            //\brief Converts point to geometery_msgs::Pose
            //\param point The point object to be converted
            geometry_msgs::Pose pointToPoseMsg(wall_inspection::Point point);


    };
}

#endif
