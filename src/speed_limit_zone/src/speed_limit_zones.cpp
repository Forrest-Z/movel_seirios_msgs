#include <cstdlib>
#include <speed_limit_zone/speed_limit_zones.h>
#include <movel_hasp_vendor/license.h>

#define INF 10000

SpeedLimitZones::SpeedLimitZones() : tf_listener_(tf_buffer_) {
  if (!setupTopics()) {
    ROS_ERROR("failed to setup topics");
    return;
  }
  // continously calls inZone() function to check if robot is inside a speed limit zone
  control_timer_ = nh.createTimer(ros::Duration(0.5), &SpeedLimitZones::odomCb, this);
}

bool SpeedLimitZones::setupTopics() {
  // mark zones on the map where speed must be reduced
  draw_zones = nh.advertiseService("reduce_speed_zone", &SpeedLimitZones::polygonCb,this);
  reduce_speed_client = nh.serviceClient<movel_seirios_msgs::ThrottleSpeed>("limit_robot_speed");
  clear_zones = nh.advertiseService("clear_speed_zone", &SpeedLimitZones::clearCb, this);
  return true;
}

void SpeedLimitZones::odomCb(const ros::TimerEvent &msg) {
  inZone();
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
bool SpeedLimitZones::polygonCb(movel_seirios_msgs::ZonePolygon::Request &req, movel_seirios_msgs::ZonePolygon::Response &res) {
  std::vector<Point> polygons; // a polygon is just a vector of points..
  
  for (int i=0; i < static_cast<int>(req.zone_data.size()); ++i) {
    polygons.clear();
    for (int j=0; j < static_cast<int>(req.zone_data[i].polygons.points.size()); ++j) {
      float p1 = req.zone_data[i].polygons.points[j].x;
      float p2 = req.zone_data[i].polygons.points[j].y;
      Point point1 = {p1, p2};
      polygons.push_back(point1);
      ROS_INFO("[callback] zone: %ld  points: %ld  polygon: %ld", req.zone_data.size(), req.zone_data[0].polygons.points.size(), polygons.size());
    }
    SpeedZone zone_polygon;
    zone_polygon = (SpeedZone){.zone_poly = polygons, .percent = req.zone_data[i].percentage_reduction};
    
    speed_zones.push_back(zone_polygon); // multiple polygons
  }

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

// Main functionality #2: check if robot is inside speed limit zone
// if true, call service to throttle speed
bool SpeedLimitZones::inZone() {
  geometry_msgs::PoseStamped robot_pose;
  getRobotPose(robot_pose);
  
  std::vector<Point> this_polygon; // polygon can also be represented as a vector of points
  for(int i = 0; i < static_cast<int>(speed_zones.size()); ++i){
    this_polygon.clear();
    // for each zone inside speed_zones, get the area and reduce_percent
    this_polygon = speed_zones[i].zone_poly;
    double reduce_percent = speed_zones[i].percent;
    int n = this_polygon.size();
    //ROS_INFO("this_polygon size: %d  ", n);

    // check this_polygon.size() > 2 because area needs at least 3 points
    if(n > 2) { 
      Point robot_point = {robot_pose.pose.position.x, robot_pose.pose.position.y};
      if(isInside(this_polygon, n, robot_point)) {
        //ROS_WARN("Entered speed limit zone");
        throttle_srv.request.set_throttle = true;
        throttle_srv.request.percentage = reduce_percent;
        //reduce_speed_client.waitForExistence();
        if(reduce_speed_client.call(throttle_srv)) {
          ROS_WARN("[speed_limit_zones] Robot inside zone. Reduce robot speed by: %f", throttle_srv.request.percentage);
        }
        else {
          ROS_ERROR("Error calling limit_robot_speed service");
        }
      }
      else {
        // if robot not inside zone
        throttle_srv.request.set_throttle = false;
        throttle_srv.request.percentage = 1.0; // probably not needed, but just to be safe.
        // if(reduce_speed_client.call(throttle_srv)) {
        //   ROS_INFO("[speed_limit_zones] Robot not inside zone");
        // }
        // else {
        //   ROS_ERROR("Error calling limit_robot_speed service");
        // }
      }
    }
    else {
      ROS_ERROR("Speed limit zone requires at least 3 points");
    }
  }
  return true;
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