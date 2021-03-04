#ifndef Wall_INSPECTION_POINT_H
#define Wall_INSPECTION_POINT_H


namespace wall_inspection
{
//\brief Data structure used by wall inspection to store pose 
struct Point
{
    float x;		//\param x X - coordinate
    float y;		//\param y Y- coordinate
    float angle;	//\param angle The heading of this point in rad

    //\brief operator+ Overloads the operator + to perfrom coordinate level addition
    //\param rh right hand side of the operator
    Point operator +(Point rh)
    {
    	Point result;

    	result.x = this->x+rh.x;
    	result.y = this->y+rh.y;

    	return result; 
    }

    //\brief operator- Overloads the operator - to perfrom coordinate level subtraction
    //\param rh right hand side of the operator
    Point operator -(Point rh)
    {
    	Point result;

    	result.x = this->x-rh.x;
    	result.y = this->y-rh.y;

    	return result; 
    }
};
}

#endif