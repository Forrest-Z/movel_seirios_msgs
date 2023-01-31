#ifndef CART_FOOTPRINT_PUBLISHER_H_
#define CART_FOOTPRINT_PUBLISHER_H_

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

struct Cart
{
  geometry_msgs::Polygon footprint;
  geometry_msgs::Point32 attach_point;
}; // Cart

class CartFootprintPublisher
{
public:
  CartFootprintPublisher(ros::NodeHandle& nh);
  ~CartFootprintPublisher();

private:
  bool loadParams();
  void setupConnections();

  void gripperAngleCb(const std_msgs::Float64::ConstPtr& msg);
  void gripperAttachCb(const std_msgs::Bool::ConstPtr& msg);
  void selectCartTypeCb(const std_msgs::String::ConstPtr& msg);
  void onPublishFootprintTimerEvent(const ros::TimerEvent& event);
  void publishFootprint(const geometry_msgs::Polygon& footprint);

  bool checkInclusion(const geometry_msgs::Point32& point, const geometry_msgs::Polygon& polygon);
  void polygonUnion(const geometry_msgs::Polygon& poly1_in, const geometry_msgs::Polygon& poly2_in, geometry_msgs::Polygon& poly_out);
  void transformPolygon(const geometry_msgs::Polygon& poly_in, geometry_msgs::Polygon& poly_out, const geometry_msgs::TransformStamped transform);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Timer publish_footprint_timer_;
  ros::Subscriber gripper_angle_sub_;
  ros::Subscriber attach_cart_sub_;
  ros::Subscriber cart_type_sub_;
  ros::Publisher cart_attached_status_pub_;
  ros::Publisher current_cart_type_pub_;
  std::vector<ros::ServiceClient> reconfigure_clients_;
  // std::vector<ros::Publisher> footprint_pubs_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::map<std::string, Cart> carts_;
  geometry_msgs::Polygon unattached_robot_footprint_;
  // std::vector<std::vector<float>> unattached_robot_footprint_;

  // bookkeeping
  std::string current_cart_type_;
  bool is_cart_attached_;
  double gripper_angle_;
  geometry_msgs::Polygon current_footprint_;
  geometry_msgs::TransformStamped gripper_to_robot_;
  geometry_msgs::TransformStamped cart_to_gripper_;

  // parameters
  std::string p_gripper_angle_topic_;
  std::string p_gripper_attach_topic_;
  std::vector<std::string> p_costmap_namespaces_;
  float p_tf_sampling_rate_;
  float p_publish_footprint_rate_;
  std::string p_gripper_tf_frame_;
  bool p_publish_cart_tf_;
  bool p_publish_gripper_tf_;
  bool p_gripper_has_tf_;
  double p_gripper_angle_at_zero_;
  std::vector<float> p_gripper_offset_at_zero_;
  std::string p_robot_frame_;
  std::string p_gripper_frame_;
  std::string p_cart_frame_;
}; // CartFootprintPublisher

#endif