#ifndef KERNAL_WALKER_H
#define KERNAL_WALKER_H
#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/ximgproc.hpp"

namespace wall_inspection
{

class Walker
{
	public:
	//\brief Data struct to store the current pixel position and well as the number of steps taken to reach 	
	struct Position {
        cv::Point point;
        int step;
        Position(cv::Point p, int s){
            point = p;
            step = s;
        };
    };

    typedef cv::Mat_<cv::Vec2f> CostKernal;
    typedef cv::Vec2f AxisCost;
	typedef cv::Vec2f CellCost;

	protected:
	
	

	const int AXIS_X = 0;
	const int AXIS_Y = 1;

	cv::Mat& map_;
  	Position point_;
  	CostKernal cost_;
  	cv::Mat local_occu_;

  	bool stopped = false;

  	AxisCost genCost();
  	void clearNbh();

  	

	public:
	Walker(cv::Mat& map, Position point,const CostKernal& cost);
	virtual Walker* walk()=0;
	bool isStopped();
	Position getPosition();
	CostKernal getCostKernal();

}; 

class UpWalker: public Walker
{
	public:
 	UpWalker(cv::Mat& map, Position point,const CostKernal&  cost);
	
	Walker* walk();
}; 

class RightWalker: public Walker
{
	public:
	RightWalker(cv::Mat& map, Position point,const CostKernal&  cost);
	
	Walker* walk();
};


class LeftWalker: public Walker
{
	public:
	LeftWalker(cv::Mat& map, Position point,const CostKernal&  cost);
	
	Walker* walk();
};

class DownWalker: public Walker
{
	public:
	DownWalker(cv::Mat& map, Position point,const CostKernal&  cost);
  	
  	Walker* walk();
};	

//\brief To travels a loop path in a binary opencv mat and generate waypoints at a fixed interval
// while walking the path
class KernalWalker
{
	int totalSteps; //\brief the total steps taken for the last search
	public:

	//\brief Constructs the object with the fix kernal cost:
	//\      (-0.5,0.5), (0.0,1.0), (0.5,0.5)
	//\      (-0.5,0.0), (0.0,0.0), (0.5,0.0)
	//\      (-0.5,-0.5), (0.0,-1.0), (0.5,-0.5)
	//\param intervals The intervals of pixels to generate the waypoint
	KernalWalker(int intervals);

	//\brief Constructs the object
	//\param cost_kernal A 3x3 mat of vectors, storing the cost used for determining the direction to walk
	//\param intervals The intervals of pixels to generate the waypoint
	KernalWalker(Walker::CostKernal& cost_kernal, int intervals);

	//\brief Generate waypoints on the loop path
	//\param The binary map with the loop path.
	//\param The point on the loop path to start walking
	//\return false if there is error in generating waypoint 
	void genWaypoints(cv::Mat& map, cv::Point pose,int inital_step=0);

	//\brief Get the total steps taken for the last search, inclusive of the steps taken after the last target point
    //\return Total steps taken
    int getTotalSteps();

	std::vector<cv::Point> getWayPoints();

	private:
	Walker::CostKernal cost_kernal_; //\brief for searching
	int intervals_; //\brief The intervals of pixels to generate the waypoint
	std::vector<cv::Point> waypoints; //\brief The list of waypoints generated
    
    //brief To move throught the loop path in the binary map, based on the cost kernal
    //\param map The current state of the binary map
    //\point the point to walk onto 
    //\ return false if have issue walking
    bool walk(cv::Mat& map, Walker::Position point);
	
};


}


#endif