#include <ros/ros.h>
#include <pure_pursuit_local_planner/pure_pursuit_planner.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(pure_pursuit_local_planner::PurePursuitPlanner, nav_core::BaseLocalPlanner)

geometry_msgs::Quaternion yawToQuaternion(double yaw)
{
    double pitch = 0;
    double roll = 0;
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    geometry_msgs::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

namespace pure_pursuit_local_planner 
{
    PurePursuitPlanner::PurePursuitPlanner() 
    {
        ROS_INFO("Pure Pursuit Constructed");
    }

    PurePursuitPlanner::~PurePursuitPlanner()
    {
    }

    void PurePursuitPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ros::NodeHandle private_nh("~/" + name);

        // Local Planner publisher
        local_plan_publisher_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
        loadParams();
        // Initialization
        costmap_ptr_ = costmap_ros;
        tf_buffer_ = tf;
        first_setPlan_ = true;
        rotate_to_global_plan_ = false;
        goal_reached_ = false;
        stand_at_goal_ = false;
        cmd_vel_angular_z_rotate_ = 0;
        cmd_vel_linear_x_ = 0;
        cmd_vel_angular_z_ = 0;
        name_ = name;

        // Parameter for dynamic reconfigure
        dsrv_ = new dynamic_reconfigure::Server<PurePursuitPlannerConfig>(private_nh);
        dynamic_reconfigure::Server<PurePursuitPlannerConfig>::CallbackType cb = boost::bind(&PurePursuitPlanner::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        ROS_INFO("PurePursuitPlanner Initialized");
    }

    void PurePursuitPlanner::loadParams()
    {
        ros::NodeHandle nl("~/" + name_);
        xy_tolerance_ = 0.30;
        if (nl.hasParam("xy_goal_tolerance"))
            nl.getParam("xy_goal_tolerance", xy_tolerance_);

        th_tolerance_ = 0.785;
        if (nl.hasParam("yaw_goal_tolerance"))
            nl.getParam("yaw_goal_tolerance", th_tolerance_);

        look_ahead_dist_ = 20;
        if (nl.hasParam("look_ahead_dist__dist"))
            nl.getParam("look_ahead_dist__dist", look_ahead_dist_);
    }

    void PurePursuitPlanner::reconfigureCB(PurePursuitPlannerConfig &config, uint32_t level)
    {
        if (config.restore_defaults)
        {
            config = default_config_;
            config.restore_defaults = false;
        }
        config_ = config;
    }

    bool PurePursuitPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        global_plan_.clear();
        global_plan_ = plan;

        //First start of the local plan. First global plan.
        bool first_use = false;
        if(first_setPlan_)
        {
            first_setPlan_ = false;
            pure_pursuit_local_planner::getXPose(*tf_buffer_,global_plan_, costmap_ptr_->getGlobalFrameID(),old_goal_pose_,global_plan_.size()-1);
            first_use = true;
        }

        pure_pursuit_local_planner::getXPose(*tf_buffer_,global_plan_, costmap_ptr_->getGlobalFrameID(),goal_pose_,global_plan_.size()-1);
        //Have the new global plan an new goal, reset. Else dont reset.
        if(std::abs(std::abs(old_goal_pose_.getOrigin().getX())-std::abs(goal_pose_.getOrigin().getX())) <= config_.position_accuracy &&
            std::abs(std::abs(old_goal_pose_.getOrigin().getY())-std::abs(goal_pose_.getOrigin().getY())) <= config_.position_accuracy && !first_use
            && std::abs(angles::shortest_angular_distance(tf::getYaw(old_goal_pose_.getRotation()), tf::getYaw(goal_pose_.getRotation()))) <= config_.rotation_accuracy)
        {
            ROS_DEBUG("FTCPlanner: Old Goal == new Goal.");
        }
        else
        {
            //Rotate to first global plan point.
            rotate_to_global_plan_ = true;
            goal_reached_ = false;
            stand_at_goal_ = false;
            ROS_INFO("FTCPlanner: New Goal. Start new routine.");
        }

        old_goal_pose_ = goal_pose_;

        return true;
    }

    bool PurePursuitPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        geometry_msgs::PoseStamped robotPose;
        costmap_ptr_->getRobotPose(robotPose);
        std::vector<double> localCoordinates;

        getGoalLocalCoordinates(localCoordinates, robotPose, look_ahead_dist_);

        geometry_msgs::Quaternion q = robotPose.pose.orientation;
        double yaw = tf::getYaw(q);
        setControls(localCoordinates, cmd_vel,yaw);

        double distanceToGoal = getEuclidianDistance(robotPose.pose.position.x,
                                                        robotPose.pose.position.y,
                                                        global_plan_[global_plan_.size()-1].pose.position.x,
                                                        global_plan_[global_plan_.size()-1].pose.position.y);

        auto quatToYaw = [] (geometry_msgs::Quaternion q){
            return atan2((2.0 * (q.w * q.z + q.y * q.x)), (1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
        };
        double err_th = fabs(quatToYaw(global_plan_[global_plan_.size()-1].pose.orientation) - quatToYaw(robotPose.pose.orientation));
        bool goal_check = distanceToGoal < xy_tolerance_ && 
                    err_th < th_tolerance_;

        std::cout<<distanceToGoal<<" ; "<<err_th<<std::endl;

        if(goal_check)
        {
            goal_reached_ = true;
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
        }

        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "map";

        std::vector<geometry_msgs::PoseStamped> poses;

        geometry_msgs::PoseStamped local_pose;
        local_pose.header.stamp = ros::Time::now();
        local_pose.header.frame_id = "base_link";
        local_pose.pose.position.x = localCoordinates[0];
        local_pose.pose.position.y = localCoordinates[1];
        local_pose.pose.orientation = yawToQuaternion(yaw);

        robotPose.header.frame_id = "map";
        robotPose.header.stamp = ros::Time::now();
        poses.push_back(robotPose);
        poses.push_back(local_pose);
        path.poses = poses;

        local_plan_publisher_.publish(path);
        return true;
    }


    bool PurePursuitPlanner::isGoalReached()
    {
        if(goal_reached_)
        {
            ROS_INFO("PurePursuitPlanner: Goal reached.");
            first_setPlan_ = true;
        }
        return goal_reached_;
    }

    void PurePursuitPlanner::getGoalLocalCoordinates(std::vector<double> &localCoordinates,
                                                        geometry_msgs::PoseStamped globalCoordinates,
                                                        double look_ahead_dist_) {
        double x_global = globalCoordinates.pose.position.x;
        double y_global = globalCoordinates.pose.position.y;
        double x_goal_global;
        double y_goal_global;
        if (look_ahead_dist_ < global_plan_.size() - 1) {
            x_goal_global = global_plan_[look_ahead_dist_].pose.position.x;
            y_goal_global = global_plan_[look_ahead_dist_].pose.position.y;
        } else {
            x_goal_global = global_plan_[global_plan_.size() - 1].pose.position.x;
            y_goal_global = global_plan_[global_plan_.size() - 1].pose.position.y;
        }

        double yaw = tf::getYaw(globalCoordinates.pose.orientation);
        
        localCoordinates.push_back((x_goal_global - x_global) * cos(-yaw) - (y_goal_global - y_global) * sin(-yaw));
        localCoordinates.push_back((x_goal_global - x_global) * sin(-yaw) + (y_goal_global - y_global) * cos(-yaw));

    }

    void PurePursuitPlanner::setControls(std::vector<double> look_ahead_dist_, geometry_msgs::Twist& cmd_vel, double yaw){

        double distance_square = look_ahead_dist_[0]*look_ahead_dist_[0] + look_ahead_dist_[1]*look_ahead_dist_[1];

        cmd_vel_linear_x_ = 0.5*(sqrt(distance_square));

        cmd_vel_angular_z_ = 0.5*((2*look_ahead_dist_[1]/(distance_square)));
        cmd_vel.angular.z = cmd_vel_angular_z_;
        cmd_vel.linear.x = cmd_vel_linear_x_;

    }

    double PurePursuitPlanner::getEuclidianDistance(const double x_init, const double y_init,
                                                    const double x_end, const double y_end) const {
        double x = (x_init - x_end);
        double y = (y_init - y_end);
        return sqrt(x*x + y*y);
    }

}

