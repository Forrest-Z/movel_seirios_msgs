#include <task_supervisor/plugins/multi_point_local_planner_navigation_handler.h>

#include <ros_utils/ros_utils.h>
#include <tf2/utils.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(task_supervisor::MultiPointLocalPlannerNavigationHandler, task_supervisor::TaskHandler);

namespace task_supervisor
{

void setAngle(geometry_msgs::PoseStamped* pose, double angle)
{
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, angle);
    tf2::convert(q, pose->pose.orientation);
}

MultiPointLocalPlannerNavigationHandler::MultiPointLocalPlannerNavigationHandler()
    : blp_loader_("nav_core", "nav_core::BaseLocalPlanner")
{
}

MultiPointLocalPlannerNavigationHandler::~MultiPointLocalPlannerNavigationHandler()
{
    local_planner_ptr_.reset();
}

bool MultiPointLocalPlannerNavigationHandler::setupHandler()
{
    ROS_INFO("[%s] Called setupHandler()", name_.c_str());

    setupDynamicReconfigure();

    MultiPointNavigationHandlerBase::setupHandler();

    ROS_INFO("[%s] Called setupHandler(): loading params", name_.c_str());
    if (!loadParams())
    {
        ROS_FATAL("[%s] Error during parameter loading. Shutting down.", name_.c_str());
        return false;
    }

    ROS_INFO("[%s] Called setupHandler(): loading local planner", name_.c_str());
    if (!loadLocalPlanner(local_planner_name_, tf_buffer_, costmap_ptr_.get()))
    {
        ROS_FATAL("[%s] Error during local planner loading. Shutting down.", name_.c_str());
        return false;
    }

    ROS_INFO("[%s] Completed setupHandler()", name_.c_str());

    return true;
}

void MultiPointLocalPlannerNavigationHandler::setupDynamicReconfigure()
{
    ros::NodeHandle nh("~" + name_);
    dynamic_reconfigure_srv_.reset(new dynamic_reconfigure::Server<multi_point_navigation::MultipointLocalPlannerConfig>(nh));
    dynamic_reconfigure_cb_ = boost::bind(&MultiPointLocalPlannerNavigationHandler::reconfigureCb, this, _1, _2);
    dynamic_reconfigure_srv_->setCallback(dynamic_reconfigure_cb_);
}

bool MultiPointLocalPlannerNavigationHandler::loadParams()
{
    ROS_INFO("[%s] Called loadParams()", name_.c_str());
    ros_utils::ParamLoader param_loader(nh_handler_);

    param_loader.get_required("base_local_planner", local_planner_name_);
    param_loader.get_required("controller_frequency", controller_frequency_);
    param_loader.get_required("controller_patience", controller_patience_);

    ROS_INFO("[%s] Completed loadParams()", name_.c_str());

    return true;
}

bool MultiPointLocalPlannerNavigationHandler::loadLocalPlanner(
    const std::string& planner, 
    tf2_ros::Buffer& tf_buffer, 
    costmap_2d::Costmap2DROS* costmap_ros)
{
    try
    {
        local_planner_ptr_ = blp_loader_.createInstance(planner);
        local_planner_ptr_->initialize(blp_loader_.getName(planner), &tf_buffer, costmap_ros);
    }
    catch (const pluginlib::PluginlibException& ex)
    {
        ROS_FATAL("[%s] Could not load local planner %s", name_.c_str(), planner.c_str());
        return false;
    }

    return true;
}

void MultiPointLocalPlannerNavigationHandler::publishZeroVelocity()
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub_.publish(cmd_vel);
}

bool MultiPointLocalPlannerNavigationHandler::navigateToPoint(
    const multi_point_navigation::Path& path,
    int goal_index)
{
    // Check for obstruction
    ros::Time obs_start_time;
    bool obs_timeout_started = false;
    ObstructionType obs_type;
    obstructed_ = checkForObstacle(path.points, goal_index, obs_type);
    ros::Time prev_obs_check_time = ros::Time::now();

    ROS_INFO("[%s] Navigating to point of index: %i", name_.c_str(), goal_index);
    
    // Get nearest point ahead on path
    multi_point_navigation::Point robot_position;
    robot_position.x = robot_pose_.position.x;
    robot_position.y = robot_pose_.position.y;
    int nearest_point_ahead_idx = path_generator.getIndexNearestPointAheadOnPath(path, robot_position);

    // Construct a path plan for the local planner to follow
    controller_plan_.clear();
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "map";
    pose_stamped.header.stamp = ros::Time::now();

    // Add the current robot's position
    pose_stamped.pose = robot_pose_;
    controller_plan_.push_back(pose_stamped);
    
    geometry_msgs::Point prev_position = pose_stamped.pose.position;

    // Add the points from the nearest point ahead to the one right before the goal
    for (int i = nearest_point_ahead_idx; i < goal_index; ++i)
    {
        pose_stamped.pose.position.x = path.points[i].x;
        pose_stamped.pose.position.y = path.points[i].y;

        // Set orientation to the next point
        double dx = path.points[i].x - prev_position.x;
        double dy = path.points[i].y - prev_position.y;
        double angle = atan2(dy, dx);
        setAngle(&pose_stamped, angle);

        controller_plan_.push_back(pose_stamped);

        prev_position = pose_stamped.pose.position;
    }

    // Add the goal_index regardless of the robot's current position 
    pose_stamped.pose.position.x = path.points[goal_index].x;
    pose_stamped.pose.position.y = path.points[goal_index].y;

    // Set orientation to the next point
    double dx = path.points[goal_index].x - prev_position.x;
    double dy = path.points[goal_index].y - prev_position.y;
    double angle = atan2(dy, dx);
    setAngle(&pose_stamped, angle);

    controller_plan_.push_back(pose_stamped);

    // Update local planner's plan
    ROS_INFO("[%s] Setting local planner plan", name_.c_str());
    if (!local_planner_ptr_->setPlan(controller_plan_))
    {
        ROS_ERROR("[%s] Failed to pass global plan to the local planner, aborting", name_.c_str());
        return false;
    }
    ROS_INFO("[%s] Local planner plan set", name_.c_str());

    ros::Rate r(controller_frequency_);

    while (!local_planner_ptr_->isGoalReached())
    {
        // If the task is cancelled, stop the robot
        if (task_cancelled_)
        {
            publishZeroVelocity();
            ROS_ERROR("[%s] Published stop command", name_.c_str());
            return false;
        }

        // Obstruction checks
        if (!obstructed_ && obs_timeout_started)
        {
            obs_timeout_started = false;
        }

        if (obstructed_ && !obs_timeout_started)
        {
            obs_timeout_started = true;
            obs_start_time = ros::Time::now();
        }

        if (obstructed_ && obs_timeout_started &&
            (ros::Time::now() - obs_start_time > ros::Duration(p_obstruction_timeout_)))
        {
            // Execute recovery behaviors if enabled
            if (p_recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size())
            {
                ROS_INFO("[%s] Executing recovery behavior %d", name_.c_str(), recovery_index_);
                recovery_behaviors_[recovery_index_]->runBehavior();

                // Check whether the recovery behavior worked
                obs_timeout_started = false;
                obstructed_ = checkForObstacle(path.points, goal_index, obs_type);
                prev_obs_check_time = ros::Time::now();

                if (obstructed_)
                {
                    obs_timeout_started = true;
                    obs_start_time = ros::Time::now();
                }

                // Update the recovery behavior index for next time
                recovery_index_++;
            }
            else
            {
                ROS_ERROR("[%s] Obstruction timeout reached. Cancelling task", name_.c_str());
                return false;
            }
        }

        // Update obstruction status
        if (ros::Time::now() - prev_obs_check_time > ros::Duration(obstacle_check_interval_))
        {
            obstructed_ = checkForObstacle(path.points, goal_index, obs_type);
            prev_obs_check_time = ros::Time::now();
            
            if (obstructed_)
            {
                ROS_WARN("[%s] Robot obstructed", name_.c_str());
            }
        }

        if (!obstructed_ && !isTaskPaused())
        {
            geometry_msgs::Twist cmd_vel;

            if (local_planner_ptr_->computeVelocityCommands(cmd_vel))
            {
                last_valid_control_ = ros::Time::now();

                cmd_vel_pub_.publish(cmd_vel);
            }
            else 
            {
                ROS_ERROR("[%s] Local planner failed to get a valid plan.", name_.c_str());
                // Local planner found no valid plan
                publishZeroVelocity();
                ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

                // Report failure if the time limit is exceeded
                if (ros::Time::now() > attempt_end)
                {
                    ROS_ERROR("[%s] Local planner's patience exceeded, aborting.", name_.c_str());
                    return false;
                }
            }
        }
        else
        {
            publishZeroVelocity();
        }

        r.sleep();
    }

    return true;
}

void MultiPointLocalPlannerNavigationHandler::stopNavigation()
{
    ROS_INFO("[%s] Called stopNavigation()", name_.c_str());
    publishZeroVelocity();
    ROS_INFO("[%s] Completed stopNavigation()", name_.c_str());
}

void MultiPointLocalPlannerNavigationHandler::reconfigureCb(
    multi_point_navigation::MultipointLocalPlannerConfig& config,
    uint32_t level)
{
    ROS_INFO("[%s] Called reconfigureCb(): %lf %d %lf %lf %lf %s %lf %lf %lf %lf",
            name_.c_str(), 
            config.points_distance, config.max_spline_bypass_degree,
            config.look_ahead_distance, config.goal_tolerance,
            config.angular_tolerance, 
            config.spline_enable ? "True" : "False", 
            config.obstacle_timeout, config.obst_check_freq,
            config.controller_frequency, config.controller_patience);

    path_generator_config_ptr_->point_generation_distance = config.points_distance;
    path_generator_config_ptr_->max_bypass_degree = config.max_spline_bypass_degree;

    p_look_ahead_dist_ = config.look_ahead_distance;
    p_goal_tolerance_ = config.goal_tolerance;
    p_angular_tolerance_ = config.angular_tolerance;
    p_spline_enable_ = config.spline_enable;
    p_obstruction_timeout_ = config.obstacle_timeout;
    p_obstacle_check_rate_ = config.obst_check_freq;

    controller_frequency_ = config.controller_frequency;
    controller_patience_ = config.controller_patience;

    setupDerivedValues();

    ROS_INFO("[%s] Completed reconfigureCb()", name_.c_str());
}

// For backup
// bool MultiPointLocalPlannerNavigationHandler::navigateToPoint(
//     const multi_point_navigation::Path& path,
//     int goal_index)
// {
//     // TODO: Might need to check for goal point obstruction
//     ROS_INFO("[%s] Navigating to point of index: %i", name_.c_str(), goal_index);
    
//     // Get nearest point ahead on path
//     multi_point_navigation::Point robot_position;
//     robot_position.x = robot_pose_.position.x;
//     robot_position.y = robot_pose_.position.y;
//     int nearest_point_ahead_idx = path_generator.getIndexNearestPointAheadOnPath(path, robot_position);
//     // ROS_INFO("[%s] Nearest point ahead index: %i", name_.c_str(), nearest_point_ahead_idx);

//     // Construct a path plan for the local planner to follow
//     controller_plan_.clear();
//     geometry_msgs::PoseStamped pose_stamped;
//     pose_stamped.header.frame_id = "map";
//     pose_stamped.header.stamp = ros::Time::now();

//     // Add the current robot's position
//     pose_stamped.pose = robot_pose_;
//     controller_plan_.push_back(pose_stamped);
    
//     geometry_msgs::Point prev_position = pose_stamped.pose.position;

//     for (int i = nearest_point_ahead_idx; i < goal_index; ++i)
//     {
//         pose_stamped.pose.position.x = path.points[i].x;
//         pose_stamped.pose.position.y = path.points[i].y;

//         // Set orientation to the next point
//         double dx = path.points[i].x - prev_position.x;
//         double dy = path.points[i].y - prev_position.y;
//         double angle = atan2(dy, dx);
//         setAngle(&pose_stamped, angle);

//         controller_plan_.push_back(pose_stamped);

//         prev_position = pose_stamped.pose.position;
//     }

//     // Add the goal_index regardless of the robot's current position 
//     pose_stamped.pose.position.x = path.points[goal_index].x;
//     pose_stamped.pose.position.y = path.points[goal_index].y;

//     // Set orientation to the next point
//     double dx = path.points[goal_index].x - prev_position.x;
//     double dy = path.points[goal_index].y - prev_position.y;
//     double angle = atan2(dy, dx);
//     setAngle(&pose_stamped, angle);

//     controller_plan_.push_back(pose_stamped);

//     // ROS_INFO("[%s] Controller plan:", name_.c_str());
//     // for (int i = 0; i < controller_plan_.size(); ++i)
//     // {
//     //     ROS_INFO("[%s] %i - (%.2lf,%.2lf)", name_.c_str(), i, controller_plan_[i].pose.position.x, controller_plan_[i].pose.position.y);
//     // }

//     // Update local planner's plan
//     ROS_INFO("[%s] Setting local planner plan", name_.c_str());
//     if (!local_planner_ptr_->setPlan(controller_plan_))
//     {
//         ROS_ERROR("[%s] Failed to pass global plan to the local planner, aborting.", name_.c_str());
//         return false;
//     }
//     ROS_INFO("[%s] Local planner plan set", name_.c_str());

//     ros::Rate r(controller_frequency_);

//     while (!local_planner_ptr_->isGoalReached())
//     {
//         geometry_msgs::Twist cmd_vel;

//         if (local_planner_ptr_->computeVelocityCommands(cmd_vel))
//         {
//             // ROS_INFO("[%s] Velocity command computed, x: %.2lf, yaw: %.2lf", name_.c_str(), cmd_vel.linear.x, cmd_vel.angular.z);
//             last_valid_control_ = ros::Time::now();

//             cmd_vel_pub_.publish(cmd_vel);
//         }
//         else 
//         {
//             ROS_ERROR("[%s] Local planner failed to get a valid plan.", name_.c_str());
//             // Local planner found no valid plan
//             publishZeroVelocity();
//             ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

//             // Report failure if the time limit is exceeded
//             if (ros::Time::now() > attempt_end)
//             {
//                 ROS_ERROR("[%s] Local planner's patience exceeded, aborting.", name_.c_str());
//                 return false;
//             }
//         }

//         r.sleep();
//     }

//     return true;
// }

}