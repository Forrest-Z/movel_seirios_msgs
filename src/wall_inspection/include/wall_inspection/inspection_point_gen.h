#ifndef INSPECTION_POINT_GEN_H
#define INSPECTION_POINT_GEN_H
#include <wall_inspection/point.h>

#include <nav_msgs/OccupancyGrid.h>

namespace wall_inspection
{

//\brief To generate the actual waypoints for inspection, based on the target points generated.
class Inspection_Point_Gen
{

	typedef Point Vector;

	double dist_to_wall_;  //\brief The distance from the wall where the robot should inspect the wall
	double inflation_;	   //\brief The inflation distance, from the wall, used to generate the target points 
	double target_to_inspection_;//\brief The distance from a target to inspection point
	double footprint_size_; //\brief A circular footprint of the inspection robot. Used to check if the inspection point is possible for the robot
	std::vector<Point> target_points_; //\brief The list of points on the wall to perform inspection. Points are based on a inflated wall.
	std::vector<Point> inspection_points_; //\brief The list of waypoints for the robot to perfrom inspection from.
	std::vector<Point> invalid_target_points_; //\brief The list of target points without a valid inspection point.
	nav_msgs::OccupancyGrid map_; //\brief The occupancy map
	

	//\brief Check occupancy of point on the map
	//\param point The point to check for occupancy 
	//return 1 if  point is occupided , 0 if free, and -1 if unknown
	int isOccupiedAt(Point point);

	//\brief generate a projection vector based on the distance and and angle
	//\param distance The magnitute of the vector in meters
	//\param angle The angle of the vector
	//\param The resultant projection vector
	void projectionVector(double distance, double angle, Vector& projection);

	//\brief Check if footprint is in collision at a point in map
	//param  pose the position of the footprint on the map
    //return True is footprint is in collision
    bool hasFootprintCollided(Point point);

public:
	//\brief Constructor
	//\param footprint_size A circular footprint of the inspection robot in radius(M). Used to check if the inspection point is possible for the robot
	//\param dist_to_wall 	The distance from the wall where the robot should inspect the wall
	//\param inflation 		The inflation distance, from the wall, used to generate the target points
	//\param map 			The occupancy map  
	
	Inspection_Point_Gen(double footprint_size, double dist_to_wall, double inflation,nav_msgs::OccupancyGrid::ConstPtr map );	

	//\brief Generate the list of inspection points based on the given target points and distance to wall requirement. As well as the list of target points without a valid inspection points	
	//\param target_points  The list of points on the wall to perform inspection. Points are based on a inflated wall. 
	//\return False if there are target points without a valid inspection poitns
	bool genInspectionPoints(std::vector<Point> target_points);


	std::vector<Point> getInspectionPoints();
	std::vector<Point> getInvalidTargetPoints();
	

};



}

#endif
