#include <wall_inspection/wall_inspection.h>
#include <actionlib_msgs/GoalID.h>
#include <movel_hasp_vendor/license.h>

namespace wall_inspection
{
  WallInspection::WallInspection() : nh_private_("~"), paused_(false)
  {
      ros::Time::waitForValid();
      if (!loadParams())
      {
        ROS_FATAL("Error during parameter loading. Shutting down.");
        return;
      }
      ROS_INFO("All parameters loaded. Launching.");

      if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
      }

      target_detector_ = std::unique_ptr<TargetDetection>(new TargetDetection(robot_radius_, p_step_,p_inflation_,p_dist_));
      targets_pub_ = nh_private_.advertise<geometry_msgs::PoseArray>("targets", 1);
      inspection_points_pub_ = nh_private_.advertise<geometry_msgs::PoseArray>("inspection_points", 1);
      targets_plan_pub_ = nh_private_.advertise<nav_msgs::Path>("targets_plan", 1);
      inspection_plan_pub_ = nh_private_.advertise<nav_msgs::Path>("inspection_plan", 1);
      goal_pub_ = nh_private_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
      ended_pub_ = nh_private_.advertise<std_msgs::Empty>("/wallinspection_stopped", 1);
      vel_pub_ = nh_private_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/safety", 1);
      cancel_pub_ = nh_private_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
      goal_sub_ = nh_private_.subscribe("/move_base/result", 1, &WallInspection::reachedCallBack, this);
      if(p_has_cam_)
        inspection_state_sub = nh_private_.subscribe("/wall_scan_completed",1,&WallInspection::inspectionDoneCallBack,this);

      pause_service_ = nh_private_.advertiseService("pause", &WallInspection::pause, this);
      skip_service_ = nh_private_.advertiseService("skip", &WallInspection::skip, this);
      terminate_service_ = nh_private_.advertiseService("terminate", &WallInspection::terminate, this);
      make_plan_client_ = nh_private_.serviceClient<nav_msgs::GetPlan>("/move_base/GlobalPlanner/make_plan");
      plan_.header.frame_id = p_map_frame_;
      plan_.header.stamp = ros::Time::now();

      //restore targets from file (TODO)
      
      state_ = SETUP;
      ros::Rate r(p_loop_rate_);
      validity_timer_ = nh_private_.createTimer(ros::Duration(p_starvation_timeout_), &WallInspection::validateLoop, this);
      nav_msgs::Path target_plan , inspection_plan;
      
      while (ros::ok())
      {
        if (state_ == SETUP) 
        {  
          if(target_detector_->findInspectionPoints())
          {
            for(Point t:target_detector_->getInspectionPoints())
            {
             t.angle+= p_angle_; //adding compensation to the heading 
             geometry_msgs::Pose pose = pointToPoseMsg(t);
             inspection_points_.poses.push_back(pose);

             geometry_msgs::PoseStamped pose_stamped;
             pose_stamped.pose = pose;
             inspection_plan.poses.push_back(pose_stamped);
            }
            state_ = PLAN;
          }
          if(target_detector_->getTargets().size() > 0)
          {
            for(Point t:target_detector_->getTargets())
            {
             geometry_msgs::Pose pose = pointToPoseMsg(t);
             targets_.poses.push_back(pose);

             geometry_msgs::PoseStamped pose_stamped;
             pose_stamped.pose = pose;
             target_plan.poses.push_back(pose_stamped);
            }

          }
        }

        if(inspection_plan.poses.size() > 0)
          sort(inspection_plan, target_plan);

        inspection_plan.header.frame_id = p_map_frame_;
        inspection_plan.header.stamp = ros::Time::now();
        ROS_DEBUG_STREAM("Inspection pose " << t_idx_ << " / " << inspection_points_.poses.size()); 
        inspection_points_.header.frame_id = p_map_frame_;
        inspection_points_.header.stamp = ros::Time::now();
        inspection_points_pub_.publish(inspection_points_);
        inspection_plan_pub_.publish(inspection_plan);

        target_plan.header.frame_id = p_map_frame_;
        target_plan.header.stamp = ros::Time::now();
        ROS_DEBUG_STREAM("Total targets pose "  << targets_.poses.size()); 
        targets_.header.frame_id = p_map_frame_;
        targets_.header.stamp = ros::Time::now();
        targets_pub_.publish(targets_);
        targets_plan_pub_.publish(target_plan);

        if (paused_)
        {
          ROS_DEBUG("Currently paused.");
          ros::spinOnce();
          r.sleep();
          continue;
        }
        switch (state_)
        {
        case SETUP:
          ROS_DEBUG("State: Setting Up");
          break;
        case PLAN:
          ROS_DEBUG("State: PLAN");
          if (targets_.poses.size() > 0)
          {
            state_ = PLANNED;
          }
          
          break;
        case PLANNED:
          ROS_DEBUG("State: PLANNED");
          getTarget();
          break;
        case NAV:
          ROS_DEBUG("State: NAV");
          //TODO Time starvation
          break;
        case DONE:
          ROS_DEBUG("State: DONE");
          endInspection();
          break;  
        default:
          break;
        }

        ros::spinOnce();
        r.sleep();
      }
  }

  bool WallInspection::loadParams()
  {
    ros_utils::ParamLoader loader(nh_private_);

    loader.get_required("loop_rate", p_loop_rate_);
    loader.get_required("map_frame", p_map_frame_);
    loader.get_required("base_frame", p_base_frame_);
    loader.get_required("timeout", p_starvation_timeout_);
    loader.get_required("step", p_step_);
    loader.get_required("distance_to_wall", p_dist_);
    loader.get_required("angle", p_angle_);
    loader.get_required("file_path", p_file_path_);
    loader.get_required("has_cam", p_has_cam_);
    loader.get_required("inflation",p_inflation_);
    loader.get_required("robot_radius",robot_radius_);

    return loader.params_valid();
  }

  void WallInspection::validateLoop(const ros::TimerEvent&)
  {
    // get current pose
    tf::StampedTransform odom;
    try
    {
      ros::Time now = ros::Time::now();
      if (listener_.waitForTransform("odom", "base_link", now, ros::Duration(0.2)))
      {
        listener_.lookupTransform("odom", "base_link", now, odom);
      }
      else
      {
        ROS_WARN("Failed to get pose");
      }
    }
    catch (tf::TransformException e)
    {
      ROS_WARN("Failed to get pose");
      ROS_ERROR("tf exception: %s", e.what());
    }
    double trans_dist = odom.getOrigin().distance(prev_odom_.getOrigin());
    double rot_dist = tf::getYaw(odom.getRotation()) - tf::getYaw(prev_odom_.getRotation());
    
    if (trans_dist < 0.01 && rot_dist < 0.01)
    {
      if (state_ == NAV)
      {
        t_idx_++;
        state_ = PLANNED;
        ROS_DEBUG("Skipping current goal - timeout");
      }
      
    }
    prev_odom_ = odom;
  }

  void WallInspection::reachedCallBack(const move_base_msgs::MoveBaseActionResult::ConstPtr& goal)
  {
    if(goal->status.status == 3)
    {
      if(p_has_cam_)
      {
        state_ = SCANNING;
        ROS_DEBUG("[wall_inspection] Goal Reached. SCANNING");
      }
      else if( state_ == NAV)
      {
          t_idx_++;
          state_ = PLANNED;
          ROS_DEBUG("[wall_inspection] Goal reached. Next Point");
      }
    }
  }

  void WallInspection::inspectionDoneCallBack(const std_msgs::Empty empty)
  {
     ROS_DEBUG("[wall_inspection] Inspection Done. Moving to next point");
    if(state_==SCANNING)
      {
        t_idx_++;
        state_ = PLANNED;
      }
  }

  void WallInspection::getTarget()
  {
    if (t_idx_ >= inspection_points_.poses.size())
    {
      ROS_DEBUG("No more target points to inspect");
      state_ = DONE;
      return;
    }

    ROS_DEBUG("Sending new goal");
    
    try
    {
      bool state = listener_.waitForTransform(p_map_frame_, p_base_frame_, ros::Time(0), ros::Duration(0.5));
      listener_.lookupTransform(p_map_frame_, p_base_frame_, ros::Time(0), base_in_map_);
    }
    catch (tf::TransformException &e)
    {
      ROS_WARN("Failed to get transform now.");
      ROS_WARN("%s", e.what());
      return;
    }
    geometry_msgs::PoseStamped goal;
    //std::cout << targets_ << std::endl;
    goal.pose = *(&inspection_points_.poses.front() + t_idx_);
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = p_map_frame_;

    //Check if plan to goal is  possible
    nav_msgs::GetPlan get_plan;
    geometry_msgs::PoseStamped start_pose;
    start_pose.header.frame_id = p_map_frame_;
    start_pose.header.stamp = ros::Time::now();
    tf::pointTFToMsg(base_in_map_.getOrigin(), start_pose.pose.position);
    tf::quaternionTFToMsg(base_in_map_.getRotation(), start_pose.pose.orientation);
    std::cout << start_pose << std::endl;
    std::cout << goal <<std::endl;
    get_plan.request.start = start_pose;
    get_plan.request.goal = goal;
    
    if (make_plan_client_.call(get_plan))
    {
      if (get_plan.response.plan.poses.empty())
      {
        ROS_WARN("Received an empty plan, discarding");
        t_idx_++;
        return;
      }
    }
    else
    {
      ROS_WARN("Unable to make a plan");
      t_idx_++;
      return;
    }

    goal_pub_.publish(goal);
    current_goal_ = goal;
    last_sent_ = ros::Time::now();
    state_ = NAV;
    return;

  }

  bool WallInspection::pause(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
  {
    ROS_INFO("PAUSE/RESUME service called");
    paused_ = request.data;
    geometry_msgs::Twist zero;
    vel_pub_.publish(zero);
    //movel_api::ActionStatus status;
    //status.code = 0;

    if (state_ == NAV)
    {
      if (request.data)
        cancel_pub_.publish(actionlib_msgs::GoalID());
      else
        goal_pub_.publish(current_goal_);
    }
    response.success = true;
    return true;
  }

  bool WallInspection::skip(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
  {
    ROS_INFO("SKIP service called");
    if (state_ == NAV or state_ == PLANNED)
    {
      cancel_pub_.publish(actionlib_msgs::GoalID());
      state_ = PLAN;
    }
    //movel_api::ActionStatus status;
    //status.code = 0;
    response.success = true;
    return true;
  }

  void WallInspection::endInspection()
  {
    cancel_pub_.publish(actionlib_msgs::GoalID());
    state_ = DONE;
    ended_pub_.publish(std_msgs::Empty());
    //Save state, targets (TODO)
    
    //ros::shutdown();
  }

  bool WallInspection::terminate(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
  {
    ROS_INFO("TERMINATE service called");
    //movel_api::ActionStatus status;
    //status.code = 0;
    endInspection();
    response.success = true;
    return true;
  }

  geometry_msgs::Pose WallInspection::pointToPoseMsg(wall_inspection::Point point)
  {
    geometry_msgs::Pose pose;
    pose.position.x = point.x;
    pose.position.y = point.y;
    tf2::Quaternion q;
    q.setRPY(0,0,point.angle);
    q = q.normalize();
    tf2::convert(q, pose.orientation);

    return pose;
  }

  void WallInspection::sort(nav_msgs::Path &inspection_plan, nav_msgs::Path &target_plan)
  {
    bool strike1 = false, strike2 = false;
    bool assigned1 = false, assigned2 = false;
    for(size_t i = 0; i < inspection_plan.poses.size() - 1; i++)
    {
      if(assigned1 && assigned2)
        break;

      geometry_msgs::PoseStamped pose1 = inspection_plan.poses[i];
      geometry_msgs::PoseStamped pose2 = inspection_plan.poses[i+1];

      tf2::Quaternion q1_inv(pose1.pose.orientation.x, pose1.pose.orientation.y,
			     pose1.pose.orientation.z, -pose1.pose.orientation.w);
      tf2::Matrix3x3 m(q1_inv);
      double r, p, y;
      m.getRPY(r, p, y);
      if(fabs(y) > M_PI/2.0)
        continue;

      geometry_msgs::TransformStamped tf;
      tf.header.stamp = ros::Time::now();
      tf.header.frame_id = "map";
      tf.transform.translation.x = - pose1.pose.position.x*cos(y) + pose1.pose.position.y*sin(y);
      tf.transform.translation.y = - pose1.pose.position.x*sin(y) - pose1.pose.position.y*cos(y);
      tf.transform.translation.z = 0;
      tf.transform.rotation.x = pose1.pose.orientation.x;
      tf.transform.rotation.y = pose1.pose.orientation.y;
      tf.transform.rotation.z = pose1.pose.orientation.z;
      tf.transform.rotation.w = -pose1.pose.orientation.w;

      geometry_msgs::PoseStamped pose_r;
      tf2::doTransform(pose2, pose_r, tf);
      if(pose_r.pose.position.x < 0)
      {
        if(!strike1)
          strike1 = true;
        else
          strike2 = true;
      }

      if(!assigned1)
        assigned1 = true;
      else
        assigned2 = true;
    }

    if(strike1 && strike2)
    {
      std::reverse(inspection_plan.poses.begin(), inspection_plan.poses.end());
      std::reverse(target_plan.poses.begin(), target_plan.poses.end());
      std::reverse(inspection_points_.poses.begin(), inspection_points_.poses.end());
      std::reverse(targets_.poses.begin(), targets_.poses.begin());
    }
  }
}

int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE
  MovelLicense ml;
  if (!ml.login())
    return 1;
#endif

  ros::init(argc, argv, "wall_inspection");
  wall_inspection::WallInspection w;

#ifdef MOVEL_LICENSE
  ml.logout();
#endif
  return(0);
}
