// #include <algorithm>
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

double clamp(double x, double lower, double upper)
{
    return std::min(upper, std::max(x, lower));
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
        carrot_pose_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("lookahead_points",1);
        loadParams();
        // Initialization
        costmap_ptr_.reset(costmap_ros);
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
        if (nl.hasParam("look_ahead_dist_"))
            nl.getParam("look_ahead_dist_", look_ahead_dist_);

        max_angular_vel_ = 0.785;
        if (nl.hasParam("max_vel_theta"))
            nl.getParam("max_vel_theta", max_angular_vel_);

        max_linear_vel_ = 0.3;
        if (nl.hasParam("max_vel_x"))
            nl.getParam("max_vel_x", max_linear_vel_);

        regulated_linear_scaling_min_radius_ = 0.9;
        if (nl.hasParam("regulated_linear_scaling_min_radius"))
            nl.getParam("regulated_linear_scaling_min_radius", regulated_linear_scaling_min_radius_);

        use_regulated_linear_velocity_scaling_ = true;
        if (nl.hasParam("use_regulated_linear_velocity_scaling"))
            nl.getParam("use_regulated_linear_velocity_scaling", use_regulated_linear_velocity_scaling_);

        inflation_cost_scaling_factor_ = 3.0;
        if (nl.hasParam("inflation_cost_scaling_factor"))
            nl.getParam("inflation_cost_scaling_factor", inflation_cost_scaling_factor_);

        cost_scaling_dist_ = 0.3;
        if (nl.hasParam("cost_scaling_dist"))
            nl.getParam("cost_scaling_dist", cost_scaling_dist_);

        cost_scaling_gain_ = 1.0;
        if (nl.hasParam("cost_scaling_gain"))
            nl.getParam("cost_scaling_gain", cost_scaling_gain_);

        regulated_linear_scaling_min_speed_ = 0.25;
        if (nl.hasParam("regulated_linear_scaling_min_speed"))
            nl.getParam("regulated_linear_scaling_min_speed", regulated_linear_scaling_min_speed_);

        min_approach_linear_velocity_ = 0.05;
        if (nl.hasParam("min_approach_linear_velocity"))
            nl.getParam("min_approach_linear_velocity", min_approach_linear_velocity_);

        use_cost_regulated_linear_velocity_scaling_ = false;
        if (nl.hasParam("use_cost_regulated_linear_velocity_scaling"))
            nl.getParam("use_cost_regulated_linear_velocity_scaling", use_cost_regulated_linear_velocity_scaling_);
        
        lookahead_dist_ = 0.6;
        if (nl.hasParam("lookahead_dist"))
            nl.getParam("lookahead_dist", lookahead_dist_);

        transform_tolerance_ = 0.3;
        if (nl.hasParam("transform_tolerance"))
            nl.getParam("transform_tolerance", transform_tolerance_);

        rotate_to_heading_min_angle_ = 0.785;
        if (nl.hasParam("rotate_to_heading_min_angle"))
            nl.getParam("rotate_to_heading_min_angle", rotate_to_heading_min_angle_);

        rotate_to_heading_min_angle_ = true;
        if (nl.hasParam("rotate_to_heading_min_angle"))
            nl.getParam("rotate_to_heading_min_angle", rotate_to_heading_min_angle_);

        rotate_to_heading_angular_vel_ = 1.8;
        if (nl.hasParam("rotate_to_heading_angular_vel"))
            nl.getParam("rotate_to_heading_angular_vel", rotate_to_heading_angular_vel_);

        max_angular_accel_ = 3.2;
        if (nl.hasParam("max_angular_accel"))
            nl.getParam("max_angular_accel", max_angular_accel_);

        controler_frequency_ = 20.0;
        if (nl.hasParam("controler_frequency"))
            nl.getParam("controler_frequency", controler_frequency_);
    }

    void PurePursuitPlanner::reconfigureCB(PurePursuitPlannerConfig &config, uint32_t level)
    {
        ROS_INFO("[%s] New config for pure pursuit!", name_.c_str());
        if (config.restore_defaults)
        {
            config = default_config_;
            config.restore_defaults = false;
        }
        config_ = config;
        xy_tolerance_ = config.xy_tolerance;
        th_tolerance_ = config.th_tolerance;
        look_ahead_dist_ = config.look_ahead_dist;
        max_angular_vel_ = config.max_angular_vel;
        max_linear_vel_ = config.max_linear_vel;
        regulated_linear_scaling_min_radius_ = config.regulated_linear_scaling_min_radius;
        use_regulated_linear_velocity_scaling_ = config.use_regulated_linear_velocity_scaling;
        use_cost_regulated_linear_velocity_scaling_ = config.use_cost_regulated_linear_velocity_scaling;
        inflation_cost_scaling_factor_ = config.inflation_cost_scaling_factor;
        cost_scaling_dist_ = config.cost_scaling_dist;
        cost_scaling_gain_ = config.cost_scaling_gain;
        regulated_linear_scaling_min_speed_ = config.regulated_linear_scaling_min_speed;
        min_approach_linear_velocity_ = config.min_approach_linear_velocity;
        lookahead_dist_ = config.lookahead_dist;
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
        // std::lock_guard<std::mutex> lk(mutex_);

        // Get Robot Pose
        geometry_msgs::PoseStamped robotPose;
        costmap_ptr_->getRobotPose(robotPose);
        geometry_msgs::Quaternion q = robotPose.pose.orientation;
        double yaw = tf::getYaw(q);

        // Get paths in look ahead dist circle
        geometry_msgs::PoseStamped localCoordinates;
        geometry_msgs::PoseStamped goalCoordinates;
        getGoalLocalCoordinates(localCoordinates, robotPose, look_ahead_dist_, goalCoordinates);

        double dist_square =
                (localCoordinates.pose.position.x * localCoordinates.pose.position.x) +
                (localCoordinates.pose.position.y * localCoordinates.pose.position.y);


        std::cout<<"loc coord: "<<localCoordinates.pose.position.x<<";"<<localCoordinates.pose.position.y<<std::endl;
        // Find curvature
        double curvature = 0.0;
        if (dist_square > 0.001)
            curvature = 2.0 * localCoordinates.pose.position.y / dist_square;

        // Setting the velocity direction
        double sign = 1.0;
        double linear_vel, angular_vel;
        linear_vel = max_linear_vel_;
        std::cout<<"before proc: "<<linear_vel<<std::endl;
        // Make sure we're in complieance with basic constraints
        double angle_to_heading;
        if (shouldRotateToGoalHeading(localCoordinates)) 
        {
            std::cout<<"1"<<std::endl;
            double angle_to_goal = tf::getYaw(goalCoordinates.pose.orientation);
            rotateToHeading(linear_vel, angular_vel, angle_to_goal);
        } 
        else if (shouldRotateToPath(localCoordinates, angle_to_heading)) 
        {
            std::cout<<"2"<<std::endl;
            rotateToHeading(linear_vel, angular_vel, angle_to_heading);
        } 
        else {
            std::cout<<"3"<<std::endl;

            std::cout<<"Lookahead: "<<lookahead_dist_<<std::endl;
            applyConstraints(
                fabs(lookahead_dist_ - sqrt(dist_square)),
                lookahead_dist_, curvature,
                costAtPose(robotPose.pose.position.x, robotPose.pose.position.y), linear_vel, sign);

            // Apply curvature to angular velocity after constraining linear velocity
            angular_vel = linear_vel * curvature;
            angular_vel = clamp(angular_vel, -max_angular_vel_, max_angular_vel_);
        }
        // setControls(localCoordinates, cmd_vel, yaw, isLastN);

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
            cmd_vel.angular.z = 0;
        }

        cmd_vel.linear.x = linear_vel;
        cmd_vel.angular.z = angular_vel;


        // nav_msgs::Path path;
        // path.header.stamp = ros::Time::now();
        // path.header.frame_id = "map";

        // std::vector<geometry_msgs::PoseStamped> poses;

        // robotPose.header.frame_id = "map";
        // robotPose.header.stamp = ros::Time::now();
        // poses.push_back(robotPose);
        // poses.push_back(localCoordinates);
        // path.poses = poses;

        // local_plan_publisher_.publish(path);
        last_cmd_vel_ = cmd_vel;
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

    void PurePursuitPlanner::getGoalLocalCoordinates(geometry_msgs::PoseStamped &localCoordinates,
                                                        geometry_msgs::PoseStamped globalCoordinates,
                                                        double look_ahead_dist_,
                                                        geometry_msgs::PoseStamped &goalCoordinates) {
        double x_global = globalCoordinates.pose.position.x;
        double y_global = globalCoordinates.pose.position.y;
        double x_goal_global;
        double y_goal_global;
        geometry_msgs::Quaternion ori;
        if (look_ahead_dist_ < global_plan_.size() - 1) {
            x_goal_global = global_plan_[look_ahead_dist_].pose.position.x;
            y_goal_global = global_plan_[look_ahead_dist_].pose.position.y;
            ori = global_plan_[look_ahead_dist_].pose.orientation;
        } else {
            x_goal_global = global_plan_[global_plan_.size() - 1].pose.position.x;
            y_goal_global = global_plan_[global_plan_.size() - 1].pose.position.y;
            ori = global_plan_[global_plan_.size() - 1].pose.orientation;
        }

        double yaw = tf::getYaw(globalCoordinates.pose.orientation);

        localCoordinates.header.stamp = ros::Time::now();
        localCoordinates.header.frame_id = "map";
        localCoordinates.pose.position.x = x_goal_global;
        localCoordinates.pose.position.y = y_goal_global;
        localCoordinates.pose.orientation = ori;
        // tf_buffer_->canTransform("base_link", "map", ros::Time::now(),ros::Duration(0.5));

        goalCoordinates.header.stamp = ros::Time::now();
        goalCoordinates.header.frame_id = "map";
        goalCoordinates.pose.position.x = global_plan_[global_plan_.size() - 1].pose.position.x;
        goalCoordinates.pose.position.y = global_plan_[global_plan_.size() - 1].pose.position.y;
        goalCoordinates.pose.orientation = ori;

        localCoordinates = tf_buffer_->transform(localCoordinates, "base_link", ros::Duration(transform_tolerance_) );
        goalCoordinates = tf_buffer_->transform(goalCoordinates, "base_link", ros::Duration(transform_tolerance_) );
        carrot_pose_pub_.publish(localCoordinates);

    }

    void PurePursuitPlanner::setControls(std::vector<double> look_ahead_dist_, geometry_msgs::Twist& cmd_vel, double yaw, bool isLastN){

        if (look_ahead_dist_[0] > 0)
        {
            double distance_square = look_ahead_dist_[0]*look_ahead_dist_[0] + look_ahead_dist_[1]*look_ahead_dist_[1];

            if (isLastN)
                cmd_vel_linear_x_ = std::min(0.5*(sqrt(distance_square)), max_linear_vel_);
            else
                cmd_vel_linear_x_ = max_linear_vel_;

            cmd_vel.linear.x = cmd_vel_linear_x_;

            cmd_vel_angular_z_ = max_angular_vel_*((2*look_ahead_dist_[1]/(distance_square)));
        }
        else
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel_angular_z_ = max_angular_vel_;
        }
        cmd_vel.angular.z = std::max(-1.0 * fabs(cmd_vel_angular_z_), std::min(cmd_vel_angular_z_, fabs(max_angular_vel_)));
    }

    double PurePursuitPlanner::getEuclidianDistance(const double x_init, const double y_init,
                                                    const double x_end, const double y_end) const {
        double x = (x_init - x_end);
        double y = (y_init - y_end);
        return sqrt(x*x + y*y);
    }

    void PurePursuitPlanner::applyConstraints(
    const double & dist_error, const double & lookahead_dist,
    const double & curvature,
    const double & pose_cost, double & linear_vel, double & sign)
    {
        costmap_2d::Costmap2D* costmap = costmap_ptr_->getCostmap();

        double curvature_vel = linear_vel;
        double cost_vel = linear_vel;
        double approach_vel = linear_vel;

        // limit the linear velocity by curvature
        const double radius = fabs(1.0 / curvature);
        const double & min_rad = regulated_linear_scaling_min_radius_;
        if (use_regulated_linear_velocity_scaling_ && radius < min_rad) {
            curvature_vel *= 1.0 - (fabs(radius - min_rad) / min_rad);
        }
        std::cout<<"curv: "<<curvature<<" ; curvature_vel: "<<curvature_vel<<std::endl;
        // limit the linear velocity by proximity to obstacles
        if (use_cost_regulated_linear_velocity_scaling_ &&
            pose_cost != static_cast<double>(costmap_2d::NO_INFORMATION) &&
            pose_cost != static_cast<double>(costmap_2d::FREE_SPACE))
        {
            const double inscribed_radius = costmap_ptr_->getLayeredCostmap()->getInscribedRadius();
            const double min_distance_to_obstacle = (-1.0 / inflation_cost_scaling_factor_) *
            std::log(pose_cost / (costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1)) + inscribed_radius;

            if (min_distance_to_obstacle < cost_scaling_dist_) {
            cost_vel *= cost_scaling_gain_ * min_distance_to_obstacle / cost_scaling_dist_;
            }
        }

        // Use the lowest of the 2 constraint heuristics, but above the minimum translational speed
        linear_vel = std::min(cost_vel, curvature_vel);
        linear_vel = std::max(linear_vel, regulated_linear_scaling_min_speed_);

        std::cout<<"after lin_vel :"<<linear_vel<<std::endl;
        std::cout<<"dist_error: "<<dist_error<<std::endl;
        // if the actual lookahead distance is shorter than requested, that means we're at the
        // end of the path. We'll scale linear velocity by error to slow to a smooth stop.
        // This expression is eq. to (1) holding time to goal, t, constant using the theoretical
        // lookahead distance and proposed velocity and (2) using t with the actual lookahead
        // distance to scale the velocity (e.g. t = lookahead / velocity, v = carrot / t).
        if (dist_error > 2.0 * costmap->getResolution()) {
            double velocity_scaling = 1.0 - (dist_error / lookahead_dist);
            double unbounded_vel = approach_vel * velocity_scaling;
            if (unbounded_vel < min_approach_linear_velocity_) {
                approach_vel = min_approach_linear_velocity_;
            } else {
                approach_vel *= velocity_scaling;
            }
            std::cout<<"approach_vel: "<<approach_vel<<" ; velocity_scaling: "<<velocity_scaling<<std::endl;
            // Use the lowest velocity between approach and other constraints, if all overlapping
            linear_vel = std::min(linear_vel, approach_vel);
        }

        // Limit linear velocities to be valid
        linear_vel = clamp(fabs(linear_vel), 0.0, max_linear_vel_);
        linear_vel = sign * linear_vel;
    }

    double PurePursuitPlanner::costAtPose(const double & x, const double & y)
    {
        costmap_2d::Costmap2D* costmap = costmap_ptr_->getCostmap();
        unsigned int mx, my;

        if (!costmap->worldToMap(x, y, mx, my)) {
            ROS_ERROR(
            "The dimensions of the costmap is too small to fully include your robot's footprint, "
            "thusly the robot cannot proceed further");
            // throw nav2_core::PlannerException(
            //         "RegulatedPurePursuitController: Dimensions of the costmap are too small "
            //         "to encapsulate the robot footprint at current speeds!");
        }

        unsigned char cost = costmap->getCost(mx, my);
        return static_cast<double>(cost);
    }

    bool PurePursuitPlanner::shouldRotateToPath(
    const geometry_msgs::PoseStamped & carrot_pose, double & angle_to_path)
    {
        // Whether we should rotate robot to rough path heading
        angle_to_path = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
        return use_rotate_to_heading_ && fabs(angle_to_path) > rotate_to_heading_min_angle_;
    }
    
    bool PurePursuitPlanner::shouldRotateToGoalHeading(
    const geometry_msgs::PoseStamped & carrot_pose)
    {
        // Whether we should rotate robot to goal heading
        double dist_to_goal = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
        return use_rotate_to_heading_ && dist_to_goal < xy_tolerance_;
    }

    void PurePursuitPlanner::rotateToHeading(
    double & linear_vel, double & angular_vel,
    const double & angle_to_path)
    {
        // Rotate in place using max angular velocity / acceleration possible
        linear_vel = 0.0;
        const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
        angular_vel = sign * rotate_to_heading_angular_vel_;

        // const double & dt = 1.0/controler_frequency_;
        // const double min_feasible_angular_speed = last_cmd_vel_.angular.z - max_angular_accel_ * dt;
        // const double max_feasible_angular_speed = last_cmd_vel_.angular.z + max_angular_accel_ * dt;
        angular_vel = clamp(angular_vel, -max_angular_vel_, max_angular_vel_);
    }

    }

