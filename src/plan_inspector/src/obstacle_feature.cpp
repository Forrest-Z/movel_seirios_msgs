
#include <cstdlib>

#include <plan_inspector/stop_at_obs.hpp>
#include <movel_hasp_vendor/license.h>
#define INF 10000
using namespace std;


StopAtObs::StopAtObs()
:tf_ear_(tf_buffer_)
{

  if (!setupParams())
  {
    ROS_INFO("bad parameters");
    return;
  }

  if (!setupTopics())
  {
    ROS_INFO("failed to setup topics");
    return;
  }
  control_timer_ = nl.createTimer(ros::Duration(0.5),
                                   &StopAtObs::odomCb, this);

  
  set_pebble_params_.waitForExistence();
  

}
bool StopAtObs::setupParams()
{
  

  control_frequency_ = 20.0;
  if (nl.hasParam("control_frequency"))
    nl.getParam("control_frequency", control_frequency_);
  
  odom_topic_ = "/odom";
  if (nl.hasParam("odom_topic"))
    nl.getParam("odom_topic", odom_topic_);
  costmap_topic_ = "/move_base/local_costmap/costmap";
  if (nl.hasParam("costmap_topic"))
    nl.getParam("costmap_topic", costmap_topic_);
  use_peb_ = false;
  saveParams();
  reconfigure_triggered = false;
  inside_triggered = false;
  duplicate_enable_check = false;
  zonePolygonVector.clear();
  enable_check = false;
  return true;
}

bool StopAtObs::setupTopics()
{
  // ros::NodeHandle nh_;
  // ros::NodeHandle nl("~");
  // odom_sub_ = nl.subscribe(costmap_topic_, 1, &StopAtObs::odomCb, this);
  zone_polygon_ = nl.advertiseService("nostop_zone",&StopAtObs::polygonCb,this);
  enable_plan_ = nl.advertiseService("enable_plan_inspector",&StopAtObs::enableCb,this);
  // stopzone = nl.advertiseService("no_stop_zones",&StopAtObs::stopzoneCb,this);
  if(use_peb_){
    set_pebble_params_ = nl.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/PebbleLocalPlanner/set_parameters");
  }
  return true;
}

void StopAtObs::odomCb(const ros::TimerEvent& msg){
  // ROS_INFO("odom");

  iszone();

}

bool StopAtObs::polygonCb(movel_seirios_msgs::Zonepolygon::Request &req, movel_seirios_msgs::Zonepolygon::Response &res)
{
  // std::vector<geometry_msgs::Polygon> zone = req.
  
  // req.polygons[0].points[0].x;
  // std::map< std::vector<zonePoint> ,int> zoneMap;
  
  // zonelist_.clear();
  // for (int i = 0; i < static_cast<int>(req.zone_data.size()); i++){
  //   movel_seirios_msgs::zones zone_[] = req.zone_data;
  //     movel_seirios_msgs::zones zon = req.zone_data[0];
  // zonelist_.push_back(movel_seirios_msgs::zones());
  // zonelist_.back().polygons = req.zone_data[0].polygons;
  // zonelist_.back().labels = req.zone_data[0].labels;
  // waypoints_.back().pose.orientation.w = 1.0;
    // zonelist_.push_back(zon) ;
  // }
  
  // zoneslist.clear();
  
  std::vector<zonePoint> polygons;
  for (int i = 0; i < static_cast<int>(req.zone_data.size()); i++)
  { polygons.clear();
    for (int j = 0; j < static_cast<int>(req.zone_data[i].polygons.points.size()); j++){
      
      float X = req.zone_data[i].polygons.points[j].x;
      float Y = req.zone_data[i].polygons.points[j].y;
      zonePoint s1 = {X,Y};
      // s1.x = float (req.zone_data[i].polygons.points[j].x);
      // s1.y = req.zone_data[i].polygons.points[j].y.data;
      // s1 = (zonePoint){.x = req.zone_data[i].polygons.points[j].x, .y = req.zone_data[i].polygons.points[j].y};
      polygons.push_back(s1);
      ROS_INFO("[callback] zone: %d  points: %d  polygon: %d",req.zone_data.size(), req.zone_data[0].polygons.points.size(),polygons.size());
    }
    
    // ROS_INFO("[callback]size: %d  ",polygons.size());
    // zonePoint s2;
    // s2 = polygons[0];
    // ROS_INFO("Data x: %f  Data Y: %f",s2.x,s2.y);
    zonePolygon zonePolygon_;
    zonePolygon_ = (zonePolygon){.zoneslist= polygons, .label = req.zone_data[i].labels };
   zonePolygonVector.push_back(zonePolygon_);
   
  }
  
  zonePoint s2;
  s2 = zonePolygonVector[0].zoneslist[0];
  ROS_INFO("Data x: %f  Data Y: %f",s2.x,s2.y);
  
  // if(req.data )
    res.message = "stop_feature  enabled";
  // else if(!req.data)
    res.message = "stop_feature  disabled";
  
  // enableStopfeature(req.data);

  res.success = true;
  return true;

} 
bool StopAtObs::enableCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  if(req.data )
    res.message = "stop_feature  enabled";
  else if(!req.data)
    res.message = "stop_feature  disabled";
  
  enableStopfeature(req.data);

  res.success = true;
  return true;

} 

bool StopAtObs::enableStopfeature(auto data)
{
  dynamic_reconfigure::Reconfigure reconfigure_common;
  dynamic_reconfigure::BoolParameter set_obs_check;
  set_obs_check.name = "enable_obsctacle_check";
  set_obs_check.value = data ;
  reconfigure_common.request.config.bools.push_back(set_obs_check);
  if(use_peb_){
    if(set_pebble_params_.call(reconfigure_common))
      { ROS_INFO("Pebble plan set params..."); }
    else{
      ROS_ERROR("Failed Pebble plan set params...");
      return false;
    }
  }
  return true;
}
bool StopAtObs::onSegment(zonePoint p, zonePoint q, zonePoint r)
{
	if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
			q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
		return true;
	return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are collinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int StopAtObs::orientation(zonePoint p, zonePoint q, zonePoint r)
{
	int val = (q.y - p.y) * (r.x - q.x) -
			(q.x - p.x) * (r.y - q.y);

	if (val == 0) return 0; // collinear
	return (val > 0)? 1: 2; // clock or counterclock wise
}

// The function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool StopAtObs::doIntersect(zonePoint p1, zonePoint q1, zonePoint p2, zonePoint q2)
{
	// Find the four orientations needed for general and
	// special cases
	int o1 = orientation(p1, q1, p2);
	int o2 = orientation(p1, q1, q2);
	int o3 = orientation(p2, q2, p1);
	int o4 = orientation(p2, q2, q1);

	// General case
	if (o1 != o2 && o3 != o4)
		return true;

	// Special Cases
	// p1, q1 and p2 are collinear and p2 lies on segment p1q1
	if (o1 == 0 && onSegment(p1, p2, q1)) return true;

	// p1, q1 and p2 are collinear and q2 lies on segment p1q1
	if (o2 == 0 && onSegment(p1, q2, q1)) return true;

	// p2, q2 and p1 are collinear and p1 lies on segment p2q2
	if (o3 == 0 && onSegment(p2, p1, q2)) return true;

	// p2, q2 and q1 are collinear and q1 lies on segment p2q2
	if (o4 == 0 && onSegment(p2, q1, q2)) return true;

	return false; // Doesn't fall in any of the above cases
}

// Returns true if the point p lies inside the polygon[] with n vertices
bool StopAtObs::isInside(std::vector<zonePoint> polygon, int n, zonePoint p)
{
  for (size_t i = 0; i < 4; i++)
  {
   ROS_INFO("polygon x: %f  polygon Y: %f",polygon[i].x,polygon[i].y);

  }
  
	// There must be at least 3 vertices in polygon[]
	if (n < 3) return false;

	// Create a point for line segment from p to infinite
	zonePoint extreme = {INF, p.y};

	// Count intersections of the above line with sides of polygon
	int count = 0, i = 0;
	do
	{
		int next = (i+1)%n;

		// Check if the line segment from 'p' to 'extreme' intersects
		// with the line segment from 'polygon[i]' to 'polygon[next]'
		if (doIntersect(polygon[i], polygon[next], p, extreme))
		{
			// If the point 'p' is collinear with line segment 'i-next',
			// then check if it lies on segment. If it lies, return true,
			// otherwise false
			if (orientation(polygon[i], p, polygon[next]) == 0)
			return onSegment(polygon[i], p, polygon[next]);

			count++;
		}
		i = next;
	} while (i != 0);

	// Return true if count is odd, false otherwise
	return count&1; // Same as (count%2 == 1)
}


bool StopAtObs::iszone(){

  std::vector<zonePoint> coordinate;
  std::vector<float> point_x;
  std::vector<float> point_y;
  geometry_msgs::PoseStamped robot_pose1;
  getRobotPose(robot_pose1);
  if (zonePolygonVector.empty()){
    return true;
  }
  std::vector<zonePoint> polygons;
  // ROS_INFO("[iszone]size: %d  ",zonePolygonVector.size());
  for (int i = 0; i < static_cast<int>(zonePolygonVector.size()); i++){
    polygons.clear();
    polygons = zonePolygonVector[i].zoneslist;
    // ROS_INFO("Data x: %f  Data Y: %f",polygons[0].x,polygons[0].y);
    // for (int j = 0; i < static_cast<int>(zonelist_[i].polygons.points.size()); i++){
      // zonePoint s1;
      // s1 = (zonePoint){.x = zonelist_[i].polygons.points[j].x, .y = zonelist_[i].polygons.points[j].y};
      // polygons.push_back(s1);
      // polygons
      // zonePoint s2;
      // s2 = polygons[0];
    
    zonePoint polygon1[] = {{3, -3},{-4, -4}, {-4, 3}, {3, 3}};
    ROS_INFO("[iszone]polygon size: %d  ",polygons.size());
    // int n = sizeof(polygon1)/sizeof(polygon1[0]);
    int n = polygons.size();
    if (polygons.size() > 2){
      
      zonePoint p = {robot_pose1.pose.position.x, robot_pose1.pose.position.y};
      ROS_INFO("Is inside %d \n",isInside(polygons, n, p));
      ROS_INFO("enable_check  %d  duplicate check %d",enable_check,duplicate_enable_check);
      saveParams();
      if(isInside(polygons, n, p)){

        // if(!reconfigure_triggered){
            
            if(enable_check){
              enableStopfeature(false);
            }
            inside_triggered = true;
            // reconfigure_triggered = false;

        // }
      return true;
      }
      if(inside_triggered){
        enableStopfeature(duplicate_enable_check);
        inside_triggered = false;
      }
      
      duplicate_enable_check = enable_check;
      return false;
    }   
  } 
  return true;
}
bool StopAtObs::getRobotPose(geometry_msgs::PoseStamped& pose)
{
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

void StopAtObs::saveParams()
{
    std::string local_planner;
    ros::NodeHandle nh("~");
    // ROS_INFO("save params enable_check  %d  ",enable_check);
   
    nh.getParam("/move_base/base_local_planner", local_planner);
    if (local_planner == "obstacle_pebble_planner/PebbleLocalPlanner")
    {
      nh.getParam("/move_base/PebbleLocalPlanner/enable_obsctacle_check", enable_check);
      use_peb_ = true;
      // nl.getParam("/move_base/TebLocalPlannerROS/weight_obstacle", weight_obstacle_temp_);
    }

}


int main(int argc, char **argv)
{
  #ifdef MOVEL_LICENSE
    MovelLicense ml;
    if (!ml.login())
      return 1;
  #endif

  ros::init(argc, argv, "obstacle");
  
  StopAtObs stopatobs;
  ros::spin();
 

  #ifdef MOVEL_LICENSE
    ml.logout();
  #endif
   return 0;
}