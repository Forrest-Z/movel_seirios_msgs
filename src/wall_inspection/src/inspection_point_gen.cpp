#include <wall_inspection/inspection_point_gen.h>

#include <cmath>

using namespace wall_inspection;

Inspection_Point_Gen::Inspection_Point_Gen(double footprint_size,double dist_to_wall, double inflation, nav_msgs::OccupancyGrid::ConstPtr map):
footprint_size_(footprint_size),
dist_to_wall_(dist_to_wall),
inflation_(inflation),
target_to_inspection_(dist_to_wall- inflation),
map_(*map)
{
}	

bool Inspection_Point_Gen::genInspectionPoints(std::vector<Point> target_points)
{

	
	for(Point target : target_points)
	{
		
		
		double halfCell = map_.info.resolution/2;

		Vector projection;
		bool blocked = false;

		Point inspection_point;
		projectionVector(target_to_inspection_,target.angle,projection);
		inspection_point = target - projection;
		inspection_point.angle = target.angle;

		//check if the inspection point is valid with the robot's footprint
		if(hasFootprintCollided(inspection_point))
			continue;

		//seeking steps by step backwards, based on the angle to see if there is any obstruction to reach inspection point.
		double current_dist = halfCell +  map_.info.resolution;
		for(double steps = 0; current_dist < target_to_inspection_ && !blocked; steps++)
		{
			projectionVector(current_dist,target.angle,projection);
			if(isOccupiedAt(target-projection)!=0)
			{	
				blocked = true;
				break;
			}
			current_dist = halfCell + (steps * map_.info.resolution);
		}
		
		if(blocked)	invalid_target_points_.push_back(target);
		else
		{
			inspection_points_.push_back(inspection_point);
		} 

	}

	if(invalid_target_points_.size() !=0) return false;

	return true;

}

int Inspection_Point_Gen::isOccupiedAt(Point point)
{
	//convert to cell coordinate
	point.x = point.x - map_.info.origin.position.x;
	point.y = point.y - map_.info.origin.position.y;
	int cell_x = std::ceil(point.x/map_.info.resolution);
	int cell_y = std::ceil(point.y/map_.info.resolution);

	int data_point = (cell_y-1)*map_.info.width + (cell_x-1);
	
	if(data_point > map_.data.size())
		printf("point is out of bound\n");
	if(map_.data.at(data_point)>0) return 1;
    if(map_.data.at(data_point) == 0) return 0;
	return -1;

	//todo<yb>:to us exception to hangled out of bound
}


void Inspection_Point_Gen::projectionVector(double distance, double angle, Vector& projection)
{
	projection.x = cos(angle) * distance;
	projection.y = sin(angle) * distance;

}

bool Inspection_Point_Gen::hasFootprintCollided(Point point)
{
	const double MAX_ANGLE = 2 * M_PI;
	double check_radius = footprint_size_;

	if(Inspection_Point_Gen::isOccupiedAt(point)!=0)
		return true;

	while(check_radius >0)
	{

		for(double angle = 0; angle < MAX_ANGLE; angle+=0.0017)
		{
			Vector check_px;
			projectionVector(check_radius,angle,check_px);

			if(Inspection_Point_Gen::isOccupiedAt(point+check_px)>0)
				return true;
		}

		check_radius -= map_.info.resolution;
	}
	return false;
}

std::vector<Point> Inspection_Point_Gen::getInspectionPoints()
{
	return inspection_points_;
}

std::vector<Point> Inspection_Point_Gen::getInvalidTargetPoints()
{	
	return invalid_target_points_;
}

