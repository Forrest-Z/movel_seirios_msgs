#include <cart_footprint_publisher/cart_footprint_publisher.h>

CartFootprintPublisher::CartFootprintPublisher(ros::NodeHandle& nh)
  : is_cart_attached_(false), gripper_angle_(0.0), tf_listener_(tf_buffer_), nh_private_("~")
{
  ros::Time::waitForValid();

  // do not deepcopy global nodehandler
  nh_ = nh;

  if (!loadParams())
  {
    ROS_FATAL("[cart_footprint_publisher] Error during parameter loading. Shutting down.");
    return;
  }
  ROS_INFO("[cart_footprint_publisher] All parameters loaded. Launching.");
  setupConnections();

  // initialise transforms
  gripper_to_robot_.header.frame_id = p_robot_frame_;
  gripper_to_robot_.child_frame_id = p_gripper_frame_;
  gripper_to_robot_.transform.translation.x = p_gripper_offset_at_zero_[0];
  gripper_to_robot_.transform.translation.y = p_gripper_offset_at_zero_[1];
  tf2::Quaternion gripper_to_robot_rot;
  gripper_to_robot_rot.setRPY(0.0, 0.0, p_gripper_angle_at_zero_);
  gripper_to_robot_rot.normalize();
  gripper_to_robot_.transform.rotation = tf2::toMsg(gripper_to_robot_rot);
  if (p_gripper_has_tf_)
  {
    if (tf_buffer_.canTransform(p_robot_frame_, p_gripper_frame_, ros::Time(0), ros::Duration(10.0)))
    {
      gripper_to_robot_ = tf_buffer_.lookupTransform(p_robot_frame_, p_gripper_frame_, ros::Time(0));
    }
    else
    {
      ROS_WARN("[cart_footprint_publisher] cannot find transform from robot frame to gripper frame.");
    }
  }

  cart_to_gripper_.header.frame_id = p_gripper_frame_;
  cart_to_gripper_.child_frame_id = p_cart_frame_;
  cart_to_gripper_.transform.translation.x = -carts_[current_cart_type_].attach_point.x;
  cart_to_gripper_.transform.translation.y = -carts_[current_cart_type_].attach_point.y;
  cart_to_gripper_.transform.rotation.w = 1.0;

  current_footprint_ = unattached_robot_footprint_;

  publish_footprint_timer_ = nh_.createTimer(ros::Duration(1.0 / p_publish_footprint_rate_),
                                             &CartFootprintPublisher::onPublishFootprintTimerEvent, this, true, false);
}

CartFootprintPublisher::~CartFootprintPublisher()
{
}

bool CartFootprintPublisher::loadParams()
{
  // test
  current_cart_type_ = "cart0";
  Cart cart0;
  std::vector<std::vector<float>> fp = {{0.5, 0.3}, {-0.5, 0.3}, {-0.5, -0.3}, {0.5, -0.3}};
  for (int i = 0; i < fp.size(); ++i)
  {
    geometry_msgs::Point32 pt;
    pt.x = fp[i][0];
    pt.y = fp[i][1];
    cart0.footprint.points.push_back(pt);
  }
  geometry_msgs::Point32 attach_pt;
  attach_pt.x = 0.5;
  attach_pt.y = 0.0;
  cart0.attach_point = attach_pt;
  carts_[current_cart_type_] = cart0;

  std::vector<std::vector<float>> robot_fp = {{0.135, 0.135}, {0.135, -0.135}, {-0.135, -0.135}, {-0.135,0.135}};
  for (int i = 0; i < robot_fp.size(); ++i)
  {
    geometry_msgs::Point32 pt;
    pt.x = fp[i][0];
    pt.y = fp[i][1];
    unattached_robot_footprint_.points.push_back(pt);
  }

  p_gripper_has_tf_ = false;
  p_gripper_angle_at_zero_ = 0.0;
  p_robot_frame_ = "base_link";
  p_gripper_frame_ = "gripper";
  p_cart_frame_ = "cart";
  is_cart_attached_ = true;

  return true;
}

void CartFootprintPublisher::setupConnections()
{
  gripper_angle_sub_ =
      nh_.subscribe<std_msgs::Float64>(p_gripper_angle_topic_, 1, &CartFootprintPublisher::gripperAngleCb, this);

  for (std::vector<std::string>::iterator it = p_footprint_topics_.begin(); it < p_footprint_topics_.end(); ++it)
  {
    ros::Publisher pub = nh_.advertise<geometry_msgs::Polygon>(*it, 1);
    footprint_pubs_.push_back(pub);
  }
}

void CartFootprintPublisher::gripperAngleCb(const std_msgs::Float64::ConstPtr& msg)
{
  double relative_angle = msg->data;
  gripper_angle_ = p_gripper_angle_at_zero_ + relative_angle;

  tf2::Quaternion rot;
  rot.setRPY(0.0, 0.0, gripper_angle_);
  rot.normalize();

  gripper_to_robot_.transform.rotation = tf2::toMsg(rot);
}

void CartFootprintPublisher::onPublishFootprintTimerEvent(const ros::TimerEvent& event)
{
  geometry_msgs::Polygon new_footprint;

  if (!is_cart_attached_)
  {
    new_footprint.points = unattached_robot_footprint_.points;
  }
  else
  {
    if (p_gripper_has_tf_)
    {
      if (tf_buffer_.canTransform(p_robot_frame_, p_gripper_frame_, ros::Time(0), ros::Duration(10.0)))
      {
        gripper_to_robot_ = tf_buffer_.lookupTransform(p_robot_frame_, p_gripper_frame_, ros::Time(0));
      }
      else
      {
        ROS_WARN("[cart_footprint_publisher] cannot find transform from robot frame to gripper frame.");
      }
    }
    Cart& current_cart = carts_[current_cart_type_];

    geometry_msgs::Polygon cart_pts_gripper;
    // tf2::doTransform(carts_[current_cart_type_].footprint, cart_pts_gripper, cart_to_gripper_);
    transformPolygon(current_cart.footprint, cart_pts_gripper, cart_to_gripper_);
    geometry_msgs::Polygon cart_pts_robot;
    // tf2::doTransform(cart_pts_gripper, cart_pts_robot, gripper_to_robot_);
    transformPolygon(cart_pts_gripper, cart_pts_robot, gripper_to_robot_);

    // check inclusion
    for (int i = 0; i < cart_pts_robot.points.size(); ++i)
    {
      if (!checkInclusion(cart_pts_robot.points[i], unattached_robot_footprint_))
        new_footprint.points.push_back(cart_pts_robot.points[i]);
    }
    for (int i = 0; i < unattached_robot_footprint_.points.size(); ++i)
    {
      if (!checkInclusion(unattached_robot_footprint_.points[i], cart_pts_robot))
        new_footprint.points.push_back(unattached_robot_footprint_.points[i]);
    }
    // sort positively-oriented (clockwise starting from closest to 0) with gripper coordinate as origin
    geometry_msgs::Point32 zero, gripper_robot;
    tf2::Transform gripper_to_robot_tf;
    tf2::fromMsg(gripper_to_robot_.transform, gripper_to_robot_tf);
    tf2::Vector3 vec_zero(zero.x, zero.y, zero.z);
    tf2::Vector3 vec_gripper_robot = gripper_to_robot_tf * vec_zero;
    gripper_robot.x = vec_gripper_robot.getX();
    gripper_robot.y = vec_gripper_robot.getY();
    gripper_robot.z = vec_gripper_robot.getZ();
    // tf2::doTransform(zero, gripper_robot, gripper_to_robot_);
    std::sort(new_footprint.points.begin(), new_footprint.points.end(), [gripper_robot](const geometry_msgs::Point32& lhs, const geometry_msgs::Point32& rhs) {
      geometry_msgs::Point32 lhs_translated, rhs_translated;
      lhs_translated.x = lhs.x - gripper_robot.x;
      lhs_translated.y = lhs.y - gripper_robot.y;
      lhs_translated.z = lhs.z - gripper_robot.z;
      rhs_translated.x = rhs.x - gripper_robot.x;
      rhs_translated.y = rhs.y - gripper_robot.y;
      rhs_translated.z = rhs.z - gripper_robot.z;
      return (atan2(lhs_translated.y, lhs_translated.x) < atan2(rhs_translated.y, rhs_translated.x));
    });
  }

  if (new_footprint != current_footprint_)
  {
    current_footprint_ = new_footprint;
    publishFootprint(current_footprint_);
  }

  // testing
  ros::Time now = ros::Time::now();
  gripper_to_robot_.header.stamp = now;
  cart_to_gripper_.header.stamp = now;
  tf_broadcaster_.sendTransform(gripper_to_robot_);
  tf_broadcaster_.sendTransform(cart_to_gripper_);
}

void CartFootprintPublisher::publishFootprint(const geometry_msgs::Polygon& footprint)
{
  for (std::vector<ros::Publisher>::iterator pub_it = footprint_pubs_.begin(); pub_it < footprint_pubs_.end(); ++pub_it)
    pub_it->publish(footprint);
}

bool CartFootprintPublisher::checkInclusion(const geometry_msgs::Point32& point, const geometry_msgs::Polygon& polygon)
{
  std::vector<float> AB = { polygon.points[1].x - polygon.points[0].x, polygon.points[1].y - polygon.points[0].y };
  std::vector<float> AP = { point.x - polygon.points[0].x, point.y - polygon.points[0].y };
  std::vector<float> BC = { polygon.points[2].x - polygon.points[1].x, polygon.points[2].y - polygon.points[1].y };
  std::vector<float> BP = { point.x - polygon.points[1].x, point.y - polygon.points[1].y };

  auto dot = [](const std::vector<float>& p1, const std::vector<float>& p2) -> float {
    return p1[0] * p2[1] + p1[1] * p2[0];
  };

  return ((0 <= dot(AB, AP) && dot(AB, AP) <= dot(AB, AB)) && (0 <= dot(BC, BP) && dot(BC, BP) <= dot(BC, BC)));
}

void CartFootprintPublisher::transformPolygon(const geometry_msgs::Polygon& poly_in, geometry_msgs::Polygon& poly_out, const geometry_msgs::TransformStamped transform)
{
  tf2::Transform t;
  tf2::fromMsg(transform.transform, t);
  for (int i = 0; i < poly_in.points.size(); ++i)
  {
    tf2::Vector3 vec_in(poly_in.points[i].x, poly_in.points[i].y, poly_in.points[i].z);
    tf2::Vector3 vec_out = t * vec_in;
    geometry_msgs::Point32 pt_32_out;
    pt_32_out.x = vec_out.getX();
    pt_32_out.y = vec_out.getY();
    pt_32_out.z = vec_out.getZ();
    poly_out.points.push_back(pt_32_out);
  }
}





int main(int argc, char** argv)
{
// #ifdef MOVEL_LICENSE
//   MovelLicense ml;
//   if (!ml.login())
//     return 1;
// #endif

  ros::init(argc, argv, "cart_footprint_publisher");
  ros::NodeHandle nh;
  CartFootprintPublisher cfp(nh);
// #ifdef MOVEL_LICENSE
//   ml.logout();
// #endif

  return (0);
}