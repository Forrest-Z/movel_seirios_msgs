/**
 * @file /src/velocity_smoother_nodelet.cpp
 *
 * @brief Velocity smoother implementation.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_velocity_smoother/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <dynamic_reconfigure/server.h>
#include <yocs_velocity_smoother/paramsConfig.h>

#include <ecl/threads/thread.hpp>

#include "yocs_velocity_smoother/velocity_smoother_nodelet.hpp"

/*****************************************************************************
 ** Preprocessing
 *****************************************************************************/

#define PERIOD_RECORD_SIZE    5
#define ZERO_VEL_COMMAND      geometry_msgs::Twist();
#define IS_ZERO_VEOCITY(a)   ((a.linear.x == 0.0) && (a.angular.z == 0.0))

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace yocs_velocity_smoother {

/*********************
** Implementation
**********************/

VelocitySmoother::VelocitySmoother(const std::string &name)
: name(name)
, quiet(false)
, shutdown_req(false)
, input_active(false)
, pr_next(0)
, dynamic_reconfigure_server(NULL)
{
};

void VelocitySmoother::reconfigCB(yocs_velocity_smoother::paramsConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request : %f %f %f %f %f %f %f %f",
           config.speed_lim_vx, config.speed_lim_vy, config.speed_lim_w, config.accel_lim_vx, config.accel_lim_vy, config.accel_lim_w, config.decel_lim_vx, config.decel_lim_vy, config.decel_lim_w);

  locker.lock();
  velo_sm_on_ = config.velocity_smoother_on;
  speed_lim_vx  = config.speed_lim_vx;
  speed_lim_vx  = config.speed_lim_vy;
  speed_lim_w  = config.speed_lim_w;
  accel_lim_vx  = config.accel_lim_vx;
  accel_lim_vy  = config.accel_lim_vy;
  accel_lim_w  = config.accel_lim_w;
  decel_lim_vx  = config.decel_lim_vx;
  decel_lim_vy  = config.decel_lim_vy;
  decel_lim_w  = config.decel_lim_w;
  //decel_factor = config.decel_factor;
  //decel_lim_vx  = decel_factor*accel_lim_vx;
  //decel_lim_w  = decel_factor*accel_lim_w;
  locker.unlock();
}

void VelocitySmoother::velocityCB(const geometry_msgs::Twist::ConstPtr& msg)
{
  // Estimate commands frequency; we do continuously as it can be very different depending on the
  // publisher type, and we don't want to impose extra constraints to keep this package flexible
  if (period_record.size() < PERIOD_RECORD_SIZE)
  {
    period_record.push_back((ros::Time::now() - last_cb_time).toSec());
  }
  else
  {
    period_record[pr_next] = (ros::Time::now() - last_cb_time).toSec();
  }

  pr_next++;
  pr_next %= period_record.size();
  last_cb_time = ros::Time::now();

  if (period_record.size() <= PERIOD_RECORD_SIZE/2)
  {
    // wait until we have some values; make a reasonable assumption (10 Hz) meanwhile
    cb_avg_time = 0.1;
  }
  else
  {
    // enough; recalculate with the latest input
    cb_avg_time = median(period_record);
  }

  input_active = true;

  // Bound speed with the maximum values
  locker.lock();
  target_vel.linear.x  =
      msg->linear.x  > 0.0 ? std::min(msg->linear.x,  speed_lim_vx) : std::max(msg->linear.x,  -speed_lim_vx);
  target_vel.linear.y  =
      msg->linear.y  > 0.0 ? std::min(msg->linear.y,  speed_lim_vy) : std::max(msg->linear.y,  -speed_lim_vy);
  target_vel.angular.z =
      msg->angular.z > 0.0 ? std::min(msg->angular.z, speed_lim_w) : std::max(msg->angular.z, -speed_lim_w);
  locker.unlock();
}

void VelocitySmoother::odometryCB(const nav_msgs::Odometry::ConstPtr& msg)
{
  if (robot_feedback == ODOMETRY)
    current_vel = msg->twist.twist;

  // ignore otherwise
}

void VelocitySmoother::robotVelCB(const geometry_msgs::Twist::ConstPtr& msg)
{
  if (robot_feedback == COMMANDS)
    current_vel = *msg;

  // ignore otherwise
}

bool VelocitySmoother::onVeloSmoothOnServiceCall(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  velo_sm_on_ = req.data;
  res.success = true;
  return true;
}

bool VelocitySmoother::statusService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  res.success = velo_sm_on_;
  if(velo_sm_on_)
    res.message = "On";
  else
    res.message = "Off";
  return true;
}

void VelocitySmoother::spin()
{
  double period = 1.0/frequency;
  ros::Rate spin_rate(frequency);

  while (! shutdown_req && ros::ok())
  {
    
    if (!velo_sm_on_ && input_active == true)
    {
      smooth_vel_pub.publish(target_vel);
    }
    else 
    {
    
      locker.lock();
      double accel_lim_vx_(accel_lim_vx);
      double accel_lim_vy_(accel_lim_vy);
      double accel_lim_w_(accel_lim_w);
      // double decel_factor(decel_factor);
      double decel_lim_vx_(decel_lim_vx);
      double decel_lim_vy_(decel_lim_vy);
      double decel_lim_w_(decel_lim_w);
      locker.unlock();
      
      if ((input_active == true) && (cb_avg_time > 0.0) &&
          ((ros::Time::now() - last_cb_time).toSec() > std::min(3.0*cb_avg_time, 0.5)))
      {
        // Velocity input no active anymore; normally last command is a zero-velocity one, but reassure
        // this, just in case something went wrong with our input, or he just forgot good manners...
        // Issue #2, extra check in case cb_avg_time is very big, for example with several atomic commands
        // The cb_avg_time > 0 check is required to deal with low-rate simulated time, that can make that
        // several messages arrive with the same time and so lead to a zero median
        input_active = false;
        if (IS_ZERO_VEOCITY(target_vel) == false)
        {
          ROS_WARN_STREAM("Velocity Smoother : input got inactive leaving us a non-zero target velocity ("
                << target_vel.linear.x << ", " << target_vel.angular.z << "), zeroing...[" << name << "]");
          target_vel = ZERO_VEL_COMMAND;
        }
      }

      //check if the feedback is off from what we expect
      //don't care about min / max velocities here, just for rough checking
      double period_buffer = 2.0;

      double vx_deviation_lower_bound = last_cmd_vel.linear.x - decel_lim_vx_ * period * period_buffer;
      double vx_deviation_upper_bound = last_cmd_vel.linear.x + accel_lim_vx_ * period * period_buffer;

      double vy_deviation_lower_bound = last_cmd_vel.linear.y - decel_lim_vy_ * period * period_buffer;
      double vy_deviation_upper_bound = last_cmd_vel.linear.y + accel_lim_vy_ * period * period_buffer;

      double w_deviation_lower_bound = last_cmd_vel.angular.z - decel_lim_w_ * period * period_buffer;
      double angular_max_deviation = last_cmd_vel.angular.z + accel_lim_w_ * period * period_buffer;

      bool vx_different_from_feedback = current_vel.linear.x < vx_deviation_lower_bound || current_vel.linear.x > vx_deviation_upper_bound;
      bool vy_different_from_feedback = current_vel.linear.y < vy_deviation_lower_bound || current_vel.linear.y > vy_deviation_upper_bound;
      bool w_different_from_feedback = current_vel.angular.z < w_deviation_lower_bound || current_vel.angular.z > angular_max_deviation;

      if ((robot_feedback != NONE) && (input_active == true) && (cb_avg_time > 0.0) &&
          (((ros::Time::now() - last_cb_time).toSec() > 5.0*cb_avg_time)     || // 5 missing msgs
              vx_different_from_feedback || vy_different_from_feedback || w_different_from_feedback))
      {
        // If the publisher has been inactive for a while, or if our current commanding differs a lot
        // from robot velocity feedback, we cannot trust the former; relay on robot's feedback instead
        // This might not work super well using the odometry if it has a high delay
        if ( !quiet ) {
          // this condition can be unavoidable due to preemption of current velocity control on
          // velocity multiplexer so be quiet if we're instructed to do so
          ROS_WARN_STREAM("Velocity Smoother : using robot velocity feedback " <<
                          std::string(robot_feedback == ODOMETRY ? "odometry" : "end commands") <<
                          " instead of last command: " <<
                          (ros::Time::now() - last_cb_time).toSec() << ", " <<
                          current_vel.linear.x  - last_cmd_vel.linear.x << ", " <<
                          current_vel.linear.y  - last_cmd_vel.linear.y << ", " <<
                          current_vel.angular.z - last_cmd_vel.angular.z << ", [" << name << "]"
                          );
        }
        last_cmd_vel = current_vel;
      }

      geometry_msgs::TwistPtr cmd_vel;

      if ((target_vel.linear.x  != last_cmd_vel.linear.x) ||
          (target_vel.linear.y  != last_cmd_vel.linear.y) ||
          (target_vel.angular.z != last_cmd_vel.angular.z))
      {
        // Try to reach target velocity ensuring that we don't exceed the acceleration limits
        cmd_vel.reset(new geometry_msgs::Twist(target_vel));

        double vx_inc, vy_inc, w_inc, max_vx_inc, max_vy_inc, max_w_inc;

        vx_inc = target_vel.linear.x - last_cmd_vel.linear.x;
        if ((robot_feedback == ODOMETRY) && (current_vel.linear.x*target_vel.linear.x < 0.0))
        {
          // countermarch (on robots with significant inertia; requires odometry feedback to be detected)
          max_vx_inc = decel_lim_vx_*period;
        }
        else
        {
          max_vx_inc = ((vx_inc*target_vel.linear.x > 0.0)?accel_lim_vx:decel_lim_vx_)*period;
        }

        vy_inc = target_vel.linear.y - last_cmd_vel.linear.y;
        if ((robot_feedback == ODOMETRY) && (current_vel.linear.y*target_vel.linear.y < 0.0))
        {
          // countermarch (on robots with significant inertia; requires odometry feedback to be detected)
          max_vy_inc = decel_lim_vy_*period;
        }
        else
        {
          max_vy_inc = ((vy_inc*target_vel.linear.y > 0.0)?accel_lim_vy:decel_lim_vy_)*period;
        }

        w_inc = target_vel.angular.z - last_cmd_vel.angular.z;
        if ((robot_feedback == ODOMETRY) && (current_vel.angular.z*target_vel.angular.z < 0.0))
        {
          // countermarch (on robots with significant inertia; requires odometry feedback to be detected)
          max_w_inc = decel_lim_w_*period;
        }
        else
        {
          max_w_inc = ((w_inc*target_vel.angular.z > 0.0)?accel_lim_w_:decel_lim_w_)*period;
        }

        // Calculate and normalise vectors A (desired velocity increment) and B (maximum velocity increment),
        // where v acts as coordinate x and w as coordinate y; the sign of the angle from A to B determines
        // which velocity (v or w) must be overconstrained to keep the direction provided as command
        double MA_x = sqrt(    vx_inc *     vx_inc +     w_inc *     w_inc);
        double MB_x = sqrt(max_vx_inc * max_vx_inc + max_w_inc * max_w_inc);

        double Av_x = std::abs(vx_inc) / MA_x;
        double Aw_x = std::abs(w_inc) / MA_x;
        double Bv_x = max_vx_inc / MB_x;
        double Bw_x = max_w_inc / MB_x;
        double theta_x = atan2(Bw_x, Bv_x) - atan2(Aw_x, Av_x);

        double MA_y = sqrt(    vy_inc *     vy_inc +     w_inc *     w_inc);
        double MB_y = sqrt(max_vy_inc * max_vy_inc + max_w_inc * max_w_inc);

        double Av_y = std::abs(vy_inc) / MA_y;
        double Aw_y = std::abs(w_inc) / MA_y;
        double Bv_y = max_vy_inc / MB_y;
        double Bw_y = max_w_inc / MB_y;
        double theta_y = atan2(Bw_y, Bv_y) - atan2(Aw_y, Av_y);

        if (theta_x < 0)
        {
          // overconstrain linear velocity
          max_vx_inc = (max_w_inc*std::abs(vx_inc))/std::abs(w_inc);
        }
        else
        {
          // overconstrain angular velocity
          max_w_inc = (max_vx_inc*std::abs(w_inc))/std::abs(vx_inc);
        }

        if (theta_y < 0)
        {
          // overconstrain linear velocity
          max_vy_inc = (max_w_inc*std::abs(vy_inc))/std::abs(w_inc);
        }
        else
        {
          // overconstrain angular velocity
          max_w_inc = (max_vy_inc*std::abs(w_inc))/std::abs(vy_inc);
        }

        if (std::abs(vx_inc) > max_vx_inc)
        {
          // we must limit linear velocity
          cmd_vel->linear.x  = last_cmd_vel.linear.x  + sign(vx_inc)*max_vx_inc;
        }

        if (std::abs(vy_inc) > max_vy_inc)
        {
          // we must limit linear velocity
          cmd_vel->linear.y  = last_cmd_vel.linear.y  + sign(vy_inc)*max_vy_inc;
        }

        if (std::abs(w_inc) > max_w_inc)
        {
          // we must limit angular velocity
          cmd_vel->angular.z = last_cmd_vel.angular.z + sign(w_inc)*max_w_inc;
        }
        smooth_vel_pub.publish(cmd_vel);
        last_cmd_vel = *cmd_vel;
      }
      else if (input_active == true)
      {
        // We already reached target velocity; just keep resending last command while input is active
        cmd_vel.reset(new geometry_msgs::Twist(last_cmd_vel));
        smooth_vel_pub.publish(cmd_vel);
      }
    }
    spin_rate.sleep();
  }
}

/**
 * Initialise from a nodelet's private nodehandle.
 * @param nh : private nodehandle
 * @return bool : success or failure
 */
bool VelocitySmoother::init(ros::NodeHandle& nh)
{
  // Dynamic Reconfigure
  dynamic_reconfigure_callback = boost::bind(&VelocitySmoother::reconfigCB, this, _1, _2);

  dynamic_reconfigure_server = new dynamic_reconfigure::Server<yocs_velocity_smoother::paramsConfig>(nh);
  dynamic_reconfigure_server->setCallback(dynamic_reconfigure_callback);

  // Optional parameters
  int feedback;
  nh.param("frequency",      frequency,     20.0);
  nh.param("quiet",          quiet,         quiet);
  //nh.param("decel_factor",   decel_factor,   1.0);
  nh.param("robot_feedback", feedback, (int)NONE);

  if ((int(feedback) < NONE) || (int(feedback) > COMMANDS))
  {
    ROS_WARN("Invalid robot feedback type (%d). Valid options are 0 (NONE, default), 1 (ODOMETRY) and 2 (COMMANDS)",
             feedback);
    feedback = NONE;
  }

  robot_feedback = static_cast<RobotFeedbackType>(feedback);

  // Mandatory parameters

  if (nh.hasParam("velocity_smoother_on"))
    nh.getParam("velocity_smoother_on", velo_sm_on_);

  if ((nh.getParam("speed_lim_vx", speed_lim_vx) == false) ||
      (nh.getParam("speed_lim_vy", speed_lim_vy) == false) ||
      (nh.getParam("speed_lim_w", speed_lim_w) == false))
  {
    ROS_ERROR("Missing velocity limit parameter(s)");
    return false;
  }

  if ((nh.getParam("accel_lim_vx", accel_lim_vx) == false) ||
      (nh.getParam("accel_lim_vy", accel_lim_vy) == false) ||
      (nh.getParam("accel_lim_w", accel_lim_w) == false))
  {
    ROS_ERROR("Missing acceleration limit parameter(s)");
    return false;
  }

  if ((nh.getParam("decel_lim_vx", decel_lim_vx) == false) ||
      (nh.getParam("decel_lim_vy", decel_lim_vx) == false) ||
      (nh.getParam("decel_lim_w", decel_lim_w) == false))
  {
    ROS_ERROR("Missing deceleration limit parameter(s)");
    return false;
  }
  // Deceleration can be more aggressive, if necessary
  //decel_lim_vx = decel_factor*accel_lim_vx;
  //decel_lim_w = decel_factor*accel_lim_w;

  // Publishers and subscribers
  odometry_sub    = nh.subscribe("odometry",      1, &VelocitySmoother::odometryCB, this);
  current_vel_sub = nh.subscribe("robot_cmd_vel", 1, &VelocitySmoother::robotVelCB, this);
  raw_in_vel_sub  = nh.subscribe("raw_cmd_vel",   1, &VelocitySmoother::velocityCB, this);
  smooth_vel_pub  = nh.advertise <geometry_msgs::Twist> ("smooth_cmd_vel", 1);
  velocity_smoother_on_ = nh.advertiseService("velocity_smoother_on", &VelocitySmoother::onVeloSmoothOnServiceCall, this);
  status_srv_ = nh.advertiseService("get_status", &VelocitySmoother::statusService, this);
  return true;
}


/*********************
** Nodelet
**********************/

class VelocitySmootherNodelet : public nodelet::Nodelet
{
public:
  VelocitySmootherNodelet()  { }
  ~VelocitySmootherNodelet()
  {
    NODELET_DEBUG("Velocity Smoother : waiting for worker thread to finish...");
    vel_smoother_->shutdown();
    worker_thread_.join();
  }

  std::string unresolvedName(const std::string &name) const {
    size_t pos = name.find_last_of('/');
    return name.substr(pos + 1);
  }


  virtual void onInit()
  {
    ros::NodeHandle ph = getPrivateNodeHandle();
    std::string resolved_name = ph.getUnresolvedNamespace(); // this always returns like /robosem/goo_arm - why not unresolved?
    std::string name = unresolvedName(resolved_name); // unresolve it ourselves
    NODELET_DEBUG_STREAM("Velocity Smoother : initialising nodelet...[" << name << "]");
    vel_smoother_.reset(new VelocitySmoother(name));
    if (vel_smoother_->init(ph))
    {
      NODELET_DEBUG_STREAM("Velocity Smoother : nodelet initialised [" << name << "]");
      worker_thread_.start(&VelocitySmoother::spin, *vel_smoother_);
    }
    else
    {
      NODELET_ERROR_STREAM("Velocity Smoother : nodelet initialisation failed [" << name << "]");
    }
  }

private:
  boost::shared_ptr<VelocitySmoother> vel_smoother_;
  ecl::Thread                        worker_thread_;
};

} // namespace yocs_velocity_smoother

PLUGINLIB_EXPORT_CLASS(yocs_velocity_smoother::VelocitySmootherNodelet, nodelet::Nodelet);
