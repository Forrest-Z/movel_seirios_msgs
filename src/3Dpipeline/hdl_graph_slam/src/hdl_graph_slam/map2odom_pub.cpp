#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>

class Map2OdomPublisher
{
public:
    Map2OdomPublisher(): odom_frame_("odom"), has_odom_(false)
    {
        ros::NodeHandle private_nh("~");
        hdl_odom_sub_ = private_nh.subscribe("/hdl_graph_slam/odom2pub",1, &Map2OdomPublisher::odomCallback, this);
        odom_tf_timer_ = private_nh.createTimer(ros::Duration(1.0 / 10.0), &Map2OdomPublisher::onOdomTimer, this);


        getParams();
    }

    ~Map2OdomPublisher(){};

    void getParams()
    {
        ros::NodeHandle private_nh("~");
        private_nh.getParam("odom_frame", odom_frame_);
    }

    void odomCallback(const geometry_msgs::TransformStamped &msg)
    {
        odom_msg = msg;
        has_odom_ = true;
    }
    
    void onOdomTimer(const ros::TimerEvent& timer_event)
    {
        if (!has_odom_)
        {
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
            tf::Quaternion q;
            q.setRPY(0.0, 0.0, 0.0);
            transform.setRotation(q);
            odom_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", odom_frame_));
            return;
        }
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(odom_msg.transform.translation.x, odom_msg.transform.translation.y, odom_msg.transform.translation.z) );
        tf::Quaternion q(odom_msg.transform.rotation.x, odom_msg.transform.rotation.y, odom_msg.transform.rotation.z, odom_msg.transform.rotation.w);
        transform.setRotation(q);
        odom_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", odom_frame_));
    }

protected:
    ros::Subscriber hdl_odom_sub_;
    ros::Timer odom_tf_timer_;

    tf::TransformBroadcaster odom_broadcaster;

    std::string odom_frame_;
    geometry_msgs::TransformStamped odom_msg;
    bool has_odom_;

};

int main(int argc, char** argv)
{

  ros::init(argc, argv, "map2odom_pub");
  ros::NodeHandle nh("~");
  Map2OdomPublisher map2odom_pub;

  ros::spin();

  return 0;
}
