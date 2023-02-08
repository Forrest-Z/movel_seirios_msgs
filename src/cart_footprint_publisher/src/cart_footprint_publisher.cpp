#include <cart_footprint_publisher/cart_footprint_publisher.h>
#include <cart_footprint_publisher/common.h>

CartFootprintPublisher::CartFootprintPublisher(ros::NodeHandle& nh)
  : is_cart_attached_(false), gripper_angle_(0.0), tf_listener_(tf_buffer_), nh_private_("~")
{
  ros::Time::waitForValid();

  // do not deepcopy global nodehandle
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
  gripper_to_robot_rot.setRPY(0.0, 0.0, 0.0);
  gripper_to_robot_rot.normalize();
  gripper_to_robot_.transform.rotation = tf2::toMsg(gripper_to_robot_rot);
  if (p_use_gripper_tf_)
  {
    if (tf_buffer_.canTransform(p_robot_frame_, p_gripper_frame_, ros::Time(0), ros::Duration(10.0)))
    {
      gripper_to_robot_ = tf_buffer_.lookupTransform(p_robot_frame_, p_gripper_frame_, ros::Time(0));
    }
    else
    {
      ROS_WARN("[cart_footprint_publisher] cannot find transform from gripper frame to robot frame.");
    }
  }

  cart_to_gripper_.header.frame_id = p_gripper_frame_;
  cart_to_gripper_.child_frame_id = p_cart_frame_;
  cart_to_gripper_.transform.translation.x = -carts_[current_cart_type_].attach_point.x;
  cart_to_gripper_.transform.translation.y = -carts_[current_cart_type_].attach_point.y;
  cart_to_gripper_.transform.rotation.w = 1.0;

  current_footprint_ = unattached_robot_footprint_;
  publishFootprint(unattached_robot_footprint_);

  publish_footprint_timer_ = nh_.createTimer(ros::Duration(1.0 / p_publish_footprint_rate_),
                                             &CartFootprintPublisher::onPublishFootprintTimerEvent, this, false, false);
  publish_footprint_timer_.start();
}

CartFootprintPublisher::~CartFootprintPublisher()
{
}

void CartFootprintPublisher::main()
{
}

void CartFootprintPublisher::shutdownHandler()
{
  publishFootprint(unattached_robot_footprint_);
}

bool CartFootprintPublisher::loadParams()
{
  // test
  // current_cart_type_ = "cart0";
  // Cart cart0;
  // std::vector<std::vector<float>> fp = {{0.5, 0.3}, {-0.5, 0.3}, {-0.5, -0.3}, {0.5, -0.3}};
  // for (int i = 0; i < fp.size(); ++i)
  // {
  //   geometry_msgs::Point32 pt;
  //   pt.x = fp[i][0];
  //   pt.y = fp[i][1];
  //   cart0.footprint.points.push_back(pt);
  // }
  // geometry_msgs::Point32 attach_pt;
  // attach_pt.x = 0.5;
  // attach_pt.y = 0.0;
  // cart0.attach_point = attach_pt;
  // carts_[current_cart_type_] = cart0;

  // std::vector<std::vector<float>> robot_fp = {{0.135, 0.135}, {-0.135, 0.135}, {-0.135, -0.135}, {0.135,-0.135}};
  // for (int i = 0; i < robot_fp.size(); ++i)
  // {
  //   geometry_msgs::Point32 pt;
  //   pt.x = robot_fp[i][0];
  //   pt.y = robot_fp[i][1];
  //   unattached_robot_footprint_.points.push_back(pt);
  // }

  is_enabled_ = false;
  if (nh_private_.hasParam("enable_cart_footprint_publisher"))
    nh_private_.getParam("enable_cart_footprint_publisher", is_enabled_);

  p_publish_footprint_rate_ = 10.0;
  if (nh_private_.hasParam("publish_footprint_rate"))
    nh_private_.getParam("publish_footprint_rate", p_publish_footprint_rate_);

  p_gripper_attach_topic_ = "/gripper/attach";
  if (nh_private_.hasParam("gripper_attach_topic"))
    nh_private_.getParam("gripper_attach_topic", p_gripper_attach_topic_);

  p_use_gripper_tf_ = true;
  if (nh_private_.hasParam("use_gripper_tf"))
    nh_private_.getParam("use_gripper_tf", p_use_gripper_tf_);

  p_gripper_angle_topic_ = "gripper/angle";
  if (!nh_private_.hasParam("gripper_angle_topic") && !p_use_gripper_tf_)
  {
    ROS_FATAL("[cart_footprint_publisher] param gripper_angle_topic is mandatory if param use_gripper_tf==false.");
    return false;
  }
  else
  {
    nh_private_.getParam("gripper_angle_topic", p_gripper_angle_topic_);
  }

  p_gripper_angle_at_zero_ = 0.0;
  if (!nh_private_.hasParam("gripper_angle_at_zero") && !p_use_gripper_tf_)
  {
    ROS_FATAL("[cart_footprint_publisher] param gripper_angle_at_zero is mandatory if param use_gripper_tf==false.");
    return false;
  }
  else
  {
    nh_private_.getParam("gripper_angle_at_zero", p_gripper_angle_at_zero_);
  }

  p_gripper_offset_at_zero_ = { 0.0, 0.0 };
  if (!nh_private_.hasParam("gripper_offset_at_zero") && !p_use_gripper_tf_)
  {
    ROS_FATAL("[cart_footprint_publisher] param gripper_offset_at_zero is mandatory if param use_gripper_tf==false.");
    return false;
  }
  else
  {
    nh_private_.getParam("gripper_offset_at_zero", p_gripper_offset_at_zero_);
  }

  p_robot_frame_ = "base_link";
  if (nh_private_.hasParam("robot_frame"))
    nh_private_.getParam("robot_frame", p_robot_frame_);

  p_gripper_frame_ = "gripper";
  if (nh_private_.hasParam("gripper_frame"))
    nh_private_.getParam("gripper_frame", p_gripper_frame_);

  p_cart_frame_ = "cart";
  if (nh_private_.hasParam("cart_frame"))
    nh_private_.getParam("cart_frame", p_cart_frame_);

  p_publish_cart_tf_ = true;
  if (nh_private_.hasParam("publish_cart_tf"))
    nh_private_.getParam("publish_cart_tf", p_publish_cart_tf_);

  if (!nh_private_.hasParam("costmap_namespaces"))
  {
    ROS_FATAL("[cart_footprint_publisher] param costmap_namespaces is mandatory.");
    return false;
  }
  else
  {
    nh_private_.getParam("costmap_namespaces", p_costmap_namespaces_);
  }

  if (!loadCartFootprints(carts_))
    return false;

  if (!loadUnattachedRobotFootprint(unattached_robot_footprint_))
    return false;

  std::string default_cart_type;
  if (!nh_private_.hasParam("default_cart_type"))
  {
    ROS_FATAL("[cart_footprint_publisher] param default_cart_type is mandatory.");
    return false;
  }
  else
  {
    nh_private_.getParam("default_cart_type", default_cart_type);
    if (!selectCartType(default_cart_type))
    {
      ROS_FATAL("[cart_footprint_publisher] cannot select default cart type.");
      return false;
    }
  }

  return true;
}

bool CartFootprintPublisher::loadCartFootprints(std::map<std::string, Cart>& carts)
{
  XmlRpc::XmlRpcValue cart_footprint_param;

  if (!nh_private_.getParam("cart_footprints", cart_footprint_param))
  {
    ROS_FATAL("[cart_footprint_publisher] Failed to read cart footprints on param server");
    return false;
  }

  if (cart_footprint_param.getType() == XmlRpc::XmlRpcValue::Type::TypeStruct)
  {
    for (auto& kv : cart_footprint_param)
    {
      Cart new_cart;
      geometry_msgs::Point32 attach_point;
      attach_point.x = (float)double(kv.second["attach_point"][0]);
      attach_point.y = (float)double(kv.second["attach_point"][1]);
      geometry_msgs::Polygon footprint;
      for (int i = 0; i < kv.second["footprint"].size(); ++i)
      {
        geometry_msgs::Point32 p;
        p.x = (float)double(kv.second["footprint"][i][0]);
        p.y = (float)double(kv.second["footprint"][i][1]);
        footprint.points.push_back(p);
      }
      std::sort(footprint.points.begin(), footprint.points.end(),
                [](const geometry_msgs::Point32& lhs, const geometry_msgs::Point32& rhs) {
                  return (atan2(lhs.y, lhs.x) < atan2(rhs.y, rhs.x));
                });
      new_cart.attach_point = attach_point;
      new_cart.footprint = footprint;
      carts[kv.first] = new_cart;
    }
  }
  else
  {
    ROS_FATAL("[cart_footprint_publisher] cart_footprints parameter type is different from what is expected");
    return false;
  }

  return true;
}

bool CartFootprintPublisher::loadUnattachedRobotFootprint(geometry_msgs::Polygon& footprint)
{
  XmlRpc::XmlRpcValue robot_footprint_param;

  if (!nh_private_.getParam("robot_footprint", robot_footprint_param))
  {
    ROS_FATAL("[cart_footprint_publisher] Failed to read cart footprints on param server");
    return false;
  }

  if (robot_footprint_param.getType() == XmlRpc::XmlRpcValue::Type::TypeString)
  {
    std::string e;
    std::vector<std::vector<float>> footprint_vec = parseVVF(robot_footprint_param, e);
    for (int i = 0; i < footprint_vec.size(); ++i)
    {
      geometry_msgs::Point32 p;
      p.x = (float)double(footprint_vec[i][0]);
      p.y = (float)double(footprint_vec[i][1]);
      footprint.points.push_back(p);
    }
    std::sort(footprint.points.begin(), footprint.points.end(),
              [](const geometry_msgs::Point32& lhs, const geometry_msgs::Point32& rhs) {
                return (atan2(lhs.y, lhs.x) < atan2(rhs.y, rhs.x));
              });
  }
  else if (robot_footprint_param.getType() == XmlRpc::XmlRpcValue::Type::TypeArray)
  {
    for (int i = 0; i < robot_footprint_param.size(); ++i)
    {
      geometry_msgs::Point32 p;
      p.x = (float)double(robot_footprint_param[i][0]);
      p.y = (float)double(robot_footprint_param[i][1]);
      footprint.points.push_back(p);
    }
    std::sort(footprint.points.begin(), footprint.points.end(),
              [](const geometry_msgs::Point32& lhs, const geometry_msgs::Point32& rhs) {
                return (atan2(lhs.y, lhs.x) < atan2(rhs.y, rhs.x));
              });
  }
  else
  {
    ROS_FATAL("[cart_footprint_publisher] robot_footprint parameter type is different from what is expected");
    return false;
  }

  return true;
}

void CartFootprintPublisher::setupConnections()
{
  gripper_angle_sub_ = nh_private_.subscribe<std_msgs::Float64>(p_gripper_angle_topic_, 1,
                                                                &CartFootprintPublisher::gripperAngleCb, this);
  attach_cart_sub_ =
      nh_private_.subscribe<std_msgs::Bool>(p_gripper_attach_topic_, 1, &CartFootprintPublisher::gripperAttachCb, this);
  cart_type_sub_ =
      nh_private_.subscribe<std_msgs::String>("cart_type/select", 1, &CartFootprintPublisher::selectCartTypeCb, this);

  cart_attached_status_pub_ = nh_private_.advertise<std_msgs::Bool>("gripper/status", 1);
  current_cart_type_pub_ = nh_private_.advertise<std_msgs::String>("cart_type/current", 1);

  for (std::vector<std::string>::iterator it = p_costmap_namespaces_.begin(); it < p_costmap_namespaces_.end(); ++it)
  {
    ros::ServiceClient client = nh_.serviceClient<dynamic_reconfigure::Reconfigure>(*it);
    reconfigure_clients_.push_back(client);
  }

  enable_srv_ = nh_private_.advertiseService("enable", &CartFootprintPublisher::onEnableFeature, this);
}

bool CartFootprintPublisher::onEnableFeature(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
{
  if (req.data)
  {
    ROS_INFO("[cart_footprint_publisher] cart footprint publisher enabled");
    resp.message = "cart footprint publisher enabled";
    resp.success = true;
  }
  else
  {
    ROS_INFO("[cart_footprint_publisher] cart footprint publisher disabled, reverting to original footprint");
    current_footprint_ = unattached_robot_footprint_;
    publishFootprint(unattached_robot_footprint_);
    resp.message = "cart footprint publisher disabled";
    resp.success = true;
  }

  is_enabled_ = req.data;
  return true;
}

void CartFootprintPublisher::gripperAngleCb(const std_msgs::Float64::ConstPtr& msg)
{
  if (!p_use_gripper_tf_)
  {
    double relative_angle = msg->data;
    gripper_angle_ = p_gripper_angle_at_zero_ + relative_angle;

    tf2::Quaternion rot;
    rot.setRPY(0.0, 0.0, gripper_angle_);
    rot.normalize();

    // gripper_to_robot_.transform.rotation = tf2::toMsg(rot);
    tf2::Transform gripper_to_robot_tf, tf_new;
    tf_new.setRotation(rot);
    tf2::fromMsg(gripper_to_robot_.transform, gripper_to_robot_tf);
    gripper_to_robot_tf.setOrigin(tf2::Vector3(p_gripper_offset_at_zero_[0], p_gripper_offset_at_zero_[1], 0.0));
    gripper_to_robot_tf.setRotation(tf2::Quaternion(0.0, 0.0, 0.0));
    gripper_to_robot_tf = tf_new * gripper_to_robot_tf;
    gripper_to_robot_.transform = tf2::toMsg(gripper_to_robot_tf);
  }
}

void CartFootprintPublisher::gripperAttachCb(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data)
  {
    ROS_INFO("[cart_footprint_publisher] Gripper attached to cart %s", current_cart_type_.c_str());
    is_cart_attached_ = true;
  }
  else
  {
    ROS_INFO("[cart_footprint_publisher] Gripper released");
    is_cart_attached_ = false;
  }
}

void CartFootprintPublisher::selectCartTypeCb(const std_msgs::String::ConstPtr& msg)
{
  bool success = selectCartType(msg->data);
}

void CartFootprintPublisher::onPublishFootprintTimerEvent(const ros::TimerEvent& event)
{
  if (!is_enabled_)
    return;

  ros::Time start = ros::Time::now();

  // publish status messages
  std_msgs::Bool attached_status_msg;
  attached_status_msg.data = is_cart_attached_;
  std_msgs::String cart_type_msg;
  cart_type_msg.data = current_cart_type_;
  cart_attached_status_pub_.publish(attached_status_msg);
  current_cart_type_pub_.publish(cart_type_msg);

  geometry_msgs::Polygon new_footprint;

  if (!is_cart_attached_)
  {
    new_footprint.points = unattached_robot_footprint_.points;
  }
  else
  {
    if (p_use_gripper_tf_)
    {
      if (tf_buffer_.canTransform(p_robot_frame_, p_gripper_frame_, ros::Time(0), ros::Duration(10.0)))
      {
        gripper_to_robot_ = tf_buffer_.lookupTransform(p_robot_frame_, p_gripper_frame_, ros::Time(0));
      }
      else
      {
        ROS_WARN("[cart_footprint_publisher] cannot find transform from gripper frame to robot frame.");
      }
    }
    Cart& current_cart = carts_[current_cart_type_];

    geometry_msgs::Polygon cart_pts_gripper;
    transformPolygon(current_cart.footprint, cart_pts_gripper, cart_to_gripper_);
    geometry_msgs::Polygon cart_pts_robot;
    transformPolygon(cart_pts_gripper, cart_pts_robot, gripper_to_robot_);

    // check inclusion
    // for (int i = 0; i < cart_pts_robot.points.size(); ++i)
    // {
    //   if (!checkInclusion(cart_pts_robot.points[i], unattached_robot_footprint_))
    //     new_footprint.points.push_back(cart_pts_robot.points[i]);
    // }
    // for (int i = 0; i < unattached_robot_footprint_.points.size(); ++i)
    // {
    //   if (!checkInclusion(unattached_robot_footprint_.points[i], cart_pts_robot))
    //     new_footprint.points.push_back(unattached_robot_footprint_.points[i]);
    // }
    polygonUnion(unattached_robot_footprint_, cart_pts_robot, new_footprint);

    // sort positively-oriented (clockwise starting from closest to 0) with gripper coordinate as origin
    geometry_msgs::Point32 zero, gripper_robot;
    tf2::Transform gripper_to_robot_tf;
    tf2::fromMsg(gripper_to_robot_.transform, gripper_to_robot_tf);
    tf2::Vector3 vec_zero(zero.x, zero.y, zero.z);
    tf2::Vector3 vec_gripper_robot = gripper_to_robot_tf * vec_zero;
    gripper_robot.x = vec_gripper_robot.getX();
    gripper_robot.y = vec_gripper_robot.getY();
    gripper_robot.z = vec_gripper_robot.getZ();
    std::sort(new_footprint.points.begin(), new_footprint.points.end(),
              [gripper_robot](const geometry_msgs::Point32& lhs, const geometry_msgs::Point32& rhs) {
                geometry_msgs::Point32 lhs_translated, rhs_translated;
                lhs_translated.x = lhs.x - gripper_robot.x;
                lhs_translated.y = lhs.y - gripper_robot.y;
                rhs_translated.x = rhs.x - gripper_robot.x;
                rhs_translated.y = rhs.y - gripper_robot.y;
                return (atan2(lhs_translated.y, lhs_translated.x) < atan2(rhs_translated.y, rhs_translated.x));
              });

    ros::Time now = ros::Time::now();
    gripper_to_robot_.header.stamp = now;
    cart_to_gripper_.header.stamp = now;
    if (p_publish_cart_tf_)
    {
      if (!tf_buffer_.canTransform(p_robot_frame_, p_gripper_frame_, ros::Time::now()))
        tf_broadcaster_.sendTransform(gripper_to_robot_);
      tf_broadcaster_.sendTransform(cart_to_gripper_);
    }
  }

  current_footprint_ = new_footprint;
  publishFootprint(new_footprint);

  ros::Time end = ros::Time::now();
  double time_elapsed = (end - start).toSec();
  // debug
  // std::cout << "Taking " << time_elapsed << " secs to complete the loop, compared to " << 1.0 /
  // p_publish_footprint_rate_ << " secs desired." << std::endl;
}

void CartFootprintPublisher::publishFootprint(const geometry_msgs::Polygon& footprint)
{
  std::stringstream footprint_str;
  footprint_str << "[";
  for (int i = 0; i < footprint.points.size(); ++i)
  {
    footprint_str << "[" << footprint.points[i].x << ", " << footprint.points[i].y << "]";
    if (i < footprint.points.size() - 1)
      footprint_str << ", ";
    else
      footprint_str << "]";
  }

  dynamic_reconfigure::Reconfigure reconfigure_params;
  dynamic_reconfigure::StrParameter set_footprint;
  set_footprint.name = "footprint";
  set_footprint.value = footprint_str.str();
  reconfigure_params.request.config.strs.push_back(set_footprint);
  for (std::vector<ros::ServiceClient>::iterator client_it = reconfigure_clients_.begin();
       client_it < reconfigure_clients_.end(); ++client_it)
  {
    if (!client_it->call(reconfigure_params))
      ROS_WARN("[cart_footprint_publisher] cannot call reconfigure service to %s", client_it->getService().c_str());
  }
}

bool CartFootprintPublisher::selectCartType(const std::string& cart_type)
{
  if (carts_.find(cart_type) == carts_.end())
  {
    ROS_WARN("[cart_footprint_publisher] cart type %s doesn't exist", cart_type.c_str());
    return false;
  }
  else
  {
    current_cart_type_ = cart_type;
    ROS_INFO("[cart_footprint_publisher] selected cart type: %s", cart_type.c_str());
    return true;
  }
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

void CartFootprintPublisher::polygonUnion(const geometry_msgs::Polygon& poly1_in,
                                          const geometry_msgs::Polygon& poly2_in, geometry_msgs::Polygon& poly_out)
{
  // TODO: implement proper polygon union algorithm.
  // Since polygon union is not a simple operation, convex hull will do for now.
  std::vector<geometry_msgs::Point32> all;
  all.reserve(poly1_in.points.size() + poly2_in.points.size());
  all.insert(all.end(), poly1_in.points.begin(), poly1_in.points.end());
  all.insert(all.end(), poly2_in.points.begin(), poly2_in.points.end());

  if (all.size() < 3)
  {
    poly_out.points = all;
    return;
  }

  // solving convex hull using quick hull
  std::function<void(const std::vector<geometry_msgs::Point32>&, const std::vector<geometry_msgs::Point32>&,
                     const int& pos)>
      f_quick_hull;
  f_quick_hull = [&](const std::vector<geometry_msgs::Point32>& polygon_points,
                     const std::vector<geometry_msgs::Point32>& line, const int& pos) mutable -> void {
    geometry_msgs::Point32 point_left = line[0];
    geometry_msgs::Point32 point_right = line[1];

    auto f_find_position = [](const std::vector<geometry_msgs::Point32>& l, geometry_msgs::Point32 p) -> int {
      float side = (p.y - l[0].y) * (l[1].x - l[0].x) - (l[1].y - l[0].y) * (p.x - l[0].x);
      if (side > 0)
        return 1;
      else if (side < 0)
        return -1;
      else
        return 0;
    };

    int max_idx = -1;
    float max_dist = 0;
    for (int i = 0; i < polygon_points.size(); ++i)
    {
      float temp_dist = abs((polygon_points[i].y - line[0].y) * (line[1].x - line[0].x) -
                            (line[1].y - line[0].y) * (polygon_points[i].x - line[0].x));
      if (f_find_position(line, polygon_points[i]) == pos && temp_dist > max_dist)
      {
        max_idx = i;
        max_dist = temp_dist;
      }
    }

    if (max_idx == -1)
    {
      if (std::find(poly_out.points.begin(), poly_out.points.end(), point_left) == poly_out.points.end())
      {
        poly_out.points.push_back(point_left);
      }
      if (std::find(poly_out.points.begin(), poly_out.points.end(), point_right) == poly_out.points.end())
      {
        poly_out.points.push_back(point_right);
      }
      return;
    }
    else
    {
      std::vector<geometry_msgs::Point32> line_left = { polygon_points[max_idx], point_left };
      std::vector<geometry_msgs::Point32> line_right = { polygon_points[max_idx], point_right };
      f_quick_hull(polygon_points, line_left, -f_find_position(line_left, point_right));
      f_quick_hull(polygon_points, line_right, -f_find_position(line_right, point_left));
    }
  };

  int min_x = 0;
  int max_x = 0;
  for (int i = 0; i < all.size(); ++i)
  {
    if (all[i].x < all[min_x].x)
    {
      min_x = i;
    }
    if (all[i].x > all[max_x].x)
    {
      max_x = i;
    }
  }
  std::vector<geometry_msgs::Point32> baseline = { all[min_x], all[max_x] };
  f_quick_hull(all, baseline, -1);
  f_quick_hull(all, baseline, 1);
}

void CartFootprintPublisher::transformPolygon(const geometry_msgs::Polygon& poly_in, geometry_msgs::Polygon& poly_out,
                                              const geometry_msgs::TransformStamped transform)
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