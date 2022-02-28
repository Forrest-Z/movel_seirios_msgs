#include <curb_edges_detection/max_velocity_adjuster.h>
#include <movel_hasp_vendor/license.h>

VelocityAdjuster::VelocityAdjuster(): nh_("~")
{
    setupParams();
    setupTopics();
}

void VelocityAdjuster::setupTopics()
{  	
    vel_client_ = nh_.serviceClient<movel_seirios_msgs::SetSpeed>("velocity_setter/set_speed");
    cmd_vel_sub_ = nh_.subscribe(cmd_vel_topic_, 1, &VelocityAdjuster::cmdVelCB, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 1, &VelocityAdjuster::odomCB, this);
    status_sub_ = nh_.subscribe(status_topic_, 1, &VelocityAdjuster::statusCB, this);
}

void VelocityAdjuster::setupParams()
{    
    ros::param::param<std::string>("~cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");    
    ros::param::param<std::string>("~odom_topic", odom_topic_, "/odom");   
    ros::param::param<std::string>("~status_topic", status_topic_, "/move_base/status");       
    ros::param::param<double>("~max_vel_x", max_vel_x_, 0.3);    
    ros::param::param<double>("~max_vel_x_backwards", max_vel_x_backwards_, 6.0);    
    ros::param::param<double>("~acc_lim_x", acc_lim_x_, 0.5);    
    ros::param::param<double>("~vel_x_thresh", vel_x_thresh_, 0.1);
    ros::param::param<double>("~max_vel_theta", max_vel_theta_, 0.7);    
}

void VelocityAdjuster::cmdVelCB(const geometry_msgs::Twist::ConstPtr& twistmsg)
{
    current_vel_ = *twistmsg;
}

void VelocityAdjuster::odomCB(const nav_msgs::Odometry::ConstPtr& odommsg)
{
    actual_vel_ = odommsg->twist.twist;
    tf::Quaternion q(
        odommsg->pose.pose.orientation.x,
        odommsg->pose.pose.orientation.y,
        odommsg->pose.pose.orientation.z,
        odommsg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll_, pitch_, yaw_);
}

bool VelocityAdjuster::check()
{

    if (yaw_ > 0.15) //~8.59 degrees
    {
        if (current_vel_.linear.x - actual_vel_.linear.x > vel_x_thresh_)
        {
            ROS_WARN("robot is slipping \n");
            movel_seirios_msgs::SetSpeed srv; 
            srv.request.linear = 1.5*max_vel_x_;
            srv.request.angular = max_vel_theta_;
            if (vel_client_.call(srv))
            {
            ROS_INFO("Reconfigured linear speed successfully to %f.\n", srv.request.linear);
            }
            else
            {
                ROS_WARN("Velocity adjuster failed.\n");
            }
            return true;
        }
    }
    return false;
}

void VelocityAdjuster::statusCB(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
    int status = msg->status_list[0].status;
    if (status == 1) //active
    {
        bool climbing_ramp = check();
    }
}

int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE
  MovelLicense ml;
  if (!ml.login())
    return 1;
#endif
    ros::init(argc, argv, "movel_velocity_adjuster");
    VelocityAdjuster velocity_adjuster;
    ros::spin();
#ifdef MOVEL_LICENSE
  ml.logout();
#endif
    return 0;
}
