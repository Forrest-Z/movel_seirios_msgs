#include <cstdlib>
#include <speed_limit_zone/speed_limit_zones.h>
#include <movel_hasp_vendor/license.h>

#define INF 10000


SpeedLimitZones::SpeedLimitZones() : tf_listener_(tf_buffer_), nh_private_("~") {
  if (!loadParams()) {
    ROS_FATAL("[speed_limit_zones] Error during parameter loading. Shutting down.");
    return;
  }
  setupTopics();
  // continously calls inZone() function to check if robot is inside a speed limit zone
  control_timer_ = nh.createTimer(ros::Duration(0.5), &SpeedLimitZones::odomCb, this);
}

// Load ROS params
bool SpeedLimitZones::loadParams()
{
  ros_utils::ParamLoader loader(nh_private_);
  loader.get_required("debug", debug_);
  return loader.params_valid();
}

void SpeedLimitZones::setupTopics() {
  // mark zones on the map where speed must be reduced
  draw_zones = nh.advertiseService("reduce_speed_zone", &SpeedLimitZones::polygonCb,this);
  clear_zones = nh.advertiseService("clear_speed_zone", &SpeedLimitZones::clearCb, this);
  // reduce_speed_client = nh.serviceClient<movel_seirios_msgs::ThrottleSpeed>("limit_robot_speed");
  set_speed_client_ = nh.serviceClient<movel_seirios_msgs::SetSpeed>("/velocity_setter_node/set_speed");
  get_speed_client_ = nh.serviceClient<movel_seirios_msgs::GetSpeed>("/velocity_setter_node/get_speed");
  if(debug_)
    display_pub_ = nh_private_.advertise<jsk_recognition_msgs::PolygonArray>("display", 1);
}


void SpeedLimitZones::odomCb(const ros::TimerEvent &msg) {
  inZone();
  if(debug_)
    displayZones();
}


// Addon: Clear zones
bool SpeedLimitZones::clearCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response& res) {
  int len =  speed_zones.size();

  if (len == 0) {
    res.success = false;
    res.message = "No speed limit zones to clear!";
  }
  else {
    speed_zones.clear();
    ROS_WARN("Num of zones %ld ", speed_zones.size());
    res.success = true;
    res.message = "Speed zones cleared";
  }
  return true;
}


// Main functionality #1: draw the zones
// function to draw speed limit zones and set % to slow down speed by
bool SpeedLimitZones::polygonCb(movel_seirios_msgs::SpeedZones::Request &req, 
                                movel_seirios_msgs::SpeedZones::Response &res) {  
  speed_zones.clear();   // clear all zones before creating
  for (int i=0; i < static_cast<int>(req.zone_data.size()); ++i) {
    std::vector<Point> polygons; // a polygon is just a vector of points..
    // precess one speed zone
    movel_seirios_msgs::SpeedZone req_speed_zone = req.zone_data[i];
    int n_points = static_cast<int>(req_speed_zone.polygons.points.size());
    // sanity check for polygon
    if (n_points <= 2) {
      ROS_ERROR("[speed_limit_zones] Speed limit zone requires at least 3 points! Zones creation failed");
      res.success = false;
      speed_zones.clear();
      return true;
    }
    // append to speed zones
    for (int j=0; j < static_cast<int>(req_speed_zone.polygons.points.size()); ++j) {
      float p1 = req_speed_zone.polygons.points[j].x;
      float p2 = req_speed_zone.polygons.points[j].y;
      Point point1 = {p1, p2};
      polygons.push_back(point1);
    }
    SpeedZone speed_zone;
    speed_zone.zone_poly = std::move(polygons);
    speed_zone.linear = req_speed_zone.linear;
    speed_zone.angular = req_speed_zone.angular;
    speed_zones.push_back(std::move(speed_zone)); // multiple polygons
    ROS_INFO("[speed_limit_zones] new zone: %ld  points: %ld", speed_zones.size()-1, speed_zones.back().zone_poly.size());
  }
  // reset zone tracking
  is_in_zone_ = false;
  in_zone_idx_ = 0;

  res.success = true;
  return true;
}


bool SpeedLimitZones::getRobotPose(geometry_msgs::PoseStamped& pose) {
  try
  {
    geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
    tf2::Transform position;
    tf2::fromMsg(transform.transform, position);
    pose.pose.position.x = position.getOrigin()[0];
    pose.pose.position.y = position.getOrigin()[1];
    pose.pose.orientation.x = position.getRotation()[0];
    pose.pose.orientation.y = position.getRotation()[1];
    pose.pose.orientation.z = position.getRotation()[2];
    pose.pose.orientation.w = position.getRotation()[3];

    pose.header.stamp = transform.header.stamp;
    pose.header.frame_id = "map";
    return true;
  }
  catch (tf2::TransformException &e)
  {
    return false;
  }
}


bool SpeedLimitZones::onSegment(Point p, Point q, Point r) {

  if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) && q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y)) {
    return true;
  }
	return false;
}


// Check orientation
// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are collinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int SpeedLimitZones::orientation(Point p, Point q, Point r) {
  int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
	if (val == 0) {
    return 0;
  } // collinear

	return (val > 0)? 1: 2; // clock or counterclock wise
}


// Check intersection
// The function that returns true if line segment 'p1q1' && 'p2q2' intersect.
bool SpeedLimitZones::doIntersect(Point p1, Point q1, Point p2, Point q2) {
  // Find the four orientations needed for general and
	// special cases
	int o1 = orientation(p1, q1, p2);
	int o2 = orientation(p1, q1, q2);
	int o3 = orientation(p2, q2, p1);
	int o4 = orientation(p2, q2, q1);

	// General case
	if (o1 != o2 && o3 != o4) {
    return true;
  }
	// Special Cases
	// p1, q1 and p2 are collinear and p2 lies on segment p1q1
	if (o1 == 0 && onSegment(p1, p2, q1)) {
    return true;
  }
	// p1, q1 and p2 are collinear and q2 lies on segment p1q1
	if (o2 == 0 && onSegment(p1, q2, q1)) {
    return true;
  }
	// p2, q2 and p1 are collinear and p1 lies on segment p2q2
	if (o3 == 0 && onSegment(p2, p1, q2)) {
    return true;
  }
	// p2, q2 and q1 are collinear and q1 lies on segment p2q2
	if (o4 == 0 && onSegment(p2, q1, q2)) {
    return true;
  }
  //ROS_INFO("no intersection checked");
	return false; // Doesn't fall in any of the above cases
}


// Returns true if the point p lies inside the polygon[] with n vertices
bool SpeedLimitZones::isInside(std::vector<Point> polygon, int n, Point p) {
  // Create a point for line segment from p to infinite
	Point extreme = {INF, p.y};
  // Count intersections of the above line with sides of polygon
	int count = 0, i = 0;
	do {
		int next = (i+1)%n;
		// Check if the line segment from 'p' to 'extreme' intersects
		// with the line segment from 'polygon[i]' to 'polygon[next]'
		if (doIntersect(polygon[i], polygon[next], p, extreme)) {
			// If the point 'p' is collinear with line segment 'i-next',
			// then check if it lies on segment. If it lies, return true,
			// otherwise false
			if (orientation(polygon[i], p, polygon[next]) == 0) {
        return onSegment(polygon[i], p, polygon[next]);
      }  
			count++; // false?
		}
		i = next;
	} while (i != 0);
	// Return true if count is odd, false otherwise
	return count&1; // Same as (count%2 == 1)
}


bool SpeedLimitZones::setSpeedUtil(double linear, double angular) 
{
  movel_seirios_msgs::SetSpeed srv_set;
  srv_set.request.linear = linear;
  srv_set.request.angular = angular;
  bool success = set_speed_client_.call(srv_set);
  return success;
}


bool SpeedLimitZones::setZoneSpeed(double linear, double angular) 
{
  if (!setSpeedUtil(linear, angular)) {
    ROS_ERROR("[speed_limit_zones] Could not set zone speed");
    return false;
  }
  ROS_INFO("[speed_limit_zones] Zone speed linear: %f", linear);
  ROS_INFO("[speed_limit_zones] Zone speed angular: %f", angular);
  return true;
}


bool SpeedLimitZones::setSpeed(double linear, double angular) 
{
  if (!setSpeedUtil(linear, angular)) {
    ROS_ERROR("[speed_limit_zones] Could not set speed");
    return false;
  }
  ROS_INFO("[speed_limit_zones] Reverted speed linear: %f", linear);
  ROS_INFO("[speed_limit_zones] Reverted speed angular: %f", angular);
  return true;
}


// Main functionality #2: check if robot is inside speed limit zone
// if true, call service to throttle speed
void SpeedLimitZones::inZone() 
{
  geometry_msgs::PoseStamped robot_pose;
  getRobotPose(robot_pose);
  Point robot_point = {robot_pose.pose.position.x, robot_pose.pose.position.y};
  for(int i = 0; i < static_cast<int>(speed_zones.size()); ++i) {
    // for each zone inside speed_zones, get the area and reduce_percent
    std::vector<Point> this_polygon = speed_zones[i].zone_poly;
    double zone_linear = speed_zones[i].linear; 
    double zone_angular = speed_zones[i].angular; 
    int n = this_polygon.size();
    // inside zone
    if (isInside(this_polygon, n, robot_point)) {
      // just entered zone
      if (!is_in_zone_) {
        // get current speed
        movel_seirios_msgs::GetSpeed srv_get;
        if (!get_speed_client_.call(srv_get)) {
          ROS_ERROR("[speed_limit_zones] Could not get current speed");
          return;
        }
        double orig_linear = srv_get.response.linear;
        double orig_angular = srv_get.response.angular;
        // set reduced speed
        if(!setZoneSpeed(zone_linear, zone_angular))
          return;
        // start zone tracking 
        is_in_zone_ = true;
        in_zone_idx_ = i;
        speed_linear_ = orig_linear;   // cache original speed
        speed_angular_ = orig_angular;   // cache original speed
      }
      // previously already inside zone
      else {
        // same zone
        if (in_zone_idx_ == i)
          return;
        // in different zone
        else {
          // set reduced speed
          if(!setZoneSpeed(zone_linear, zone_angular))
            return;
          // update zone tracking 
          in_zone_idx_ = i;
        }
      }
      // exit loop after processing inside zone
      return;
    }
  }
  // checked all zones, not inside any
  // just exited zone
  if (is_in_zone_) {
    if(!setSpeed(speed_linear_, speed_angular_))
      return;
    is_in_zone_ = false;
    in_zone_idx_ = 0;
    speed_linear_ = 0.0;
    speed_angular_ = 0.0;
  }
}

void SpeedLimitZones::displayZones()
{
  jsk_recognition_msgs::PolygonArray polygons;
  polygons.header.frame_id = "map";
  for(size_t i = 0; i < speed_zones.size(); i++)
  {
    geometry_msgs::PolygonStamped polygon;
    polygon.header.frame_id = "map";
    for(size_t j = 0; j < speed_zones[i].zone_poly.size(); j++)
    {
      geometry_msgs::Point32 vertex;
      vertex.x = speed_zones[i].zone_poly[j].x;
      vertex.y = speed_zones[i].zone_poly[j].y;
      vertex.z = 0;
      polygon.polygon.points.push_back(vertex);
    }
    polygons.polygons.push_back(polygon);
  }
  display_pub_.publish(polygons);
}

int main(int argc, char **argv) {
  #ifdef MOVEL_LICENSE
    MovelLicense ml;
    if (!ml.login())
      return 1;
  #endif

  ros::init(argc, argv, "speed_limit_zones");
  
  SpeedLimitZones spd_zone;
  ros::spin();
 
  #ifdef MOVEL_LICENSE
    ml.logout();
  #endif

  return 0;
}