#ifndef TASK_DURATION_ESTIMATOR_H
#define TASK_DURATION_ESTIMATOR_H

#include <ros/ros.h>
#include <movel_seirios_msgs/GetTaskDurationEstimate.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Header.h>
#include <cmath>
#include <deque>
#include <iterator>
#include <movel_common_libs/json.hpp>
#include <movel_task_duration_estimator/DurationEstimatorConfig.h>
#include <movel_fms_utils/path_dist_utils.hpp>
#include <movel_seirios_msgs/RunTaskListActionGoal.h>
#include <movel_seirios_msgs/RunTaskListActionResult.h>
#include <movel_seirios_msgs/TaskDuration.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <dynamic_reconfigure/server.h>
#include <actionlib_msgs/GoalStatus.h>

#include <std_msgs/Int64.h>

/**
 * Wait for backend to call the service to estimate, if called, check inside the request.tasks
 * Task types : 
 * 3 = NavigationHandler / Waypoint
 * 4 = CleaningHandler / Zone
 * 6 = MultiPointHandler / Trail
 * Almost all starts with 3, so check if the length of request.tasks is more than 1.
 * Check for the points in the json payload. Parse the json payload.
 * Convert the points into geometry_msgs::PoseStamped message.
 * Put it in a deque target_poses (push_back).
 * (1) If task type is 3 and only 1 task :
 * Get plan for current pose to target_poses. Calculate the distance from current pose to first target_poses
 * from point to points inside the generated Path. Get time estimate by dividing distance by request.tasks[i].linear_velocity.
 * Pop/remove first element.
 * (2) If task type is 3 and next is 3 as well :
 * For all points, get plan for virtual_current_pos to target_poses. Calculate the distance, assign the
 * virtual_current_pos as the current target_poses. Get time estimate by dividing distance by request.tasks[i].linear_velocity.
 * Accumulate the time estimate. Pop/remove first element. Repeat until end.
 * (3) If task type is 3 and next is 4 :
 * Should be same as (2). The difference should be on how to get the target_poses.
 * (4) If task type is 3 and next is 6 :
 * For all points, calculate euclidian distance directly between virtual_current_pos to target_pos. Assign the
 * virtual_current_pos as the current target_poses. Get time estimate by dividing distance by request.tasks[i].linear_velocity.
 * Accumulate the time estimate. Pop/remove first element. Repeat until end.
 * 
 * Assign response.estimate_secs with the time estimate calculation obtained times the duration_estimate_factor.
 * Add duration_estimate_factor param to be reconfigureable.
 */

using json = nlohmann::json;
using GoalStatus = actionlib_msgs::GoalStatus;

class TaskDurationEstimator
{
public:
    TaskDurationEstimator(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);
    // ~TaskDurationEstimator() = default;
    
    float calculateDist(const nav_msgs::Path& path);
    float calculateEuclidianDist(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal);

    nav_msgs::Path getGlobalPlan(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal);

    void reconfCB(movel_task_duration_estimator::DurationEstimatorConfig&, uint32_t);

    void robotPoseCB(const geometry_msgs::Pose::ConstPtr& msg);

    void tsGoalCB(const movel_seirios_msgs::RunTaskListActionGoal::ConstPtr& msg);

    void tsResultCB(const movel_seirios_msgs::RunTaskListActionResult::ConstPtr& msg);

    void currentPlanCB(const nav_msgs::Path::ConstPtr& msg);

    void publishTaskDuration(const ros::TimerEvent& event);
    
    void moveBaseResultCB(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);

private:
    ros::NodeHandle& nh_;
    ros::NodeHandle& priv_nh_;
    ros::ServiceServer duration_estimator_server;
    ros::ServiceClient planner_request_client;
    ros::Subscriber robot_pose_sub_;
    ros::Subscriber current_plan_sub_;
    ros::Subscriber ts_goal_sub_;
    ros::Subscriber ts_result_sub_;
    ros::Subscriber move_base_result_sub_;
    
    ros::Publisher task_duration_pub_;
    ros::Publisher task_duration_only_pub_;

    ros::Timer publish_task_duration_timer_;

    float est_time_ = 0;
    std::deque<geometry_msgs::PoseStamped> target_poses_;
    std::deque<float> lin_vels_;
    std::deque<float> est_times_;

    nav_msgs::Path current_plan_;
    geometry_msgs::Pose robot_pose_;
    uint16_t curr_task_id_;

    bool is_navigating_ = false;
    bool is_plan_updated_ = false;
    
    std::shared_ptr< dynamic_reconfigure::Server<movel_task_duration_estimator::DurationEstimatorConfig> > dynamic_reconf_server_;
    dynamic_reconfigure::Server<movel_task_duration_estimator::DurationEstimatorConfig>::CallbackType dynamic_reconfigure_callback_;

    float multiplication_factor = 1;

};


#endif