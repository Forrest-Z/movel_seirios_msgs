#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <movel_seirios_msgs/LaserScanWithTF.h>


static ros::Subscriber scan_sub;
static ros::Publisher scan_with_tf_pub;
static tf2_ros::Buffer tf_buffer;


void scanCB(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  try{
    geometry_msgs::TransformStamped&& transform = tf_buffer.lookupTransform(
      scan->header.frame_id,
      "map", 
      scan->header.stamp,
      ros::Duration(0.2)
    );
    movel_seirios_msgs::LaserScanWithTF scan_with_tf;
    scan_with_tf.laserscan = *scan;
    scan_with_tf.transform = transform;
    scan_with_tf_pub.publish(scan_with_tf);
  }
  catch (tf2::TransformException ex) {
    ROS_ERROR("%s", ex.what());
  }
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_with_tf_node");
  ros::NodeHandle nh_handler_{"~"};
  tf2_ros::TransformListener tf_listener(tf_buffer, nh_handler_);
  
  using LaserWithTF = movel_seirios_msgs::LaserScanWithTF;
  scan_sub = nh_handler_.subscribe("/scan", 1, scanCB);
  scan_with_tf_pub = nh_handler_.advertise<LaserWithTF>("/scan/with_tf", 1, false);

  ros::spin();    
  return 0;
}