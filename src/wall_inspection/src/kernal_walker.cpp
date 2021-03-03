#include <wall_inspection/kernal_walker.h>

using namespace wall_inspection;


Walker::Walker(cv::Mat& map, Position point,const CostKernal& cost):
map_(map),
point_(point)
{
    //get local occupancy
    cv::Rect roi(point_.point.x-1, point_.point.y-1, 3, 3);
    local_occu_ = map_(roi);
    cost.copyTo(cost_);
}

UpWalker::UpWalker(cv::Mat& map, Position point,const CostKernal& cost):
Walker(map,point,cost) 
{}

RightWalker::RightWalker(cv::Mat& map, Position point,const CostKernal& cost):
Walker(map,point,cost) {}

LeftWalker::LeftWalker(cv::Mat& map, Position point,const CostKernal& cost):
Walker(map,point,cost) {}

DownWalker::DownWalker(cv::Mat& map, Position point,const CostKernal& cost):
Walker(map,point,cost) 
{}

Walker::AxisCost Walker::genCost()
{
    //apply cost kernal to current point
    AxisCost axis_cost(0,0);
    
    //for debug
    // cv::namedWindow( "local", cv::WINDOW_NORMAL);
    // cv::imshow("local",local_occu_);
    // cv::waitKey(0);
    //

    for(int y = 0 ; y < local_occu_.rows ; y++)
    {
        for(int x = 0; x < local_occu_.cols;x++)
        {
            int occ = local_occu_.at<uchar>(y,x);
            if(local_occu_.at<uchar>(y,x)>0)
            {
                axis_cost+=cost_.at<cv::Vec2f>(x,y);
            }
        }
    }   
    return axis_cost; 
}

void Walker::clearNbh()
{
    //set the cell arond walker to be visted
     
    for(auto cell = local_occu_.begin<uchar>(); cell != local_occu_.end<uchar>(); cell++)
    {
        *cell = 0;
    }
}
Walker* UpWalker::walk()
{
    AxisCost axis_cost = genCost();

    clearNbh();

    //determine next cell to go and move
    Position new_point = point_;
    new_point.step++;

    if(axis_cost[AXIS_X]==0)
    {
        if(axis_cost[AXIS_Y] ==0)
        {    
            stopped = true;
            return this;
        }
        
        else if(axis_cost[AXIS_Y] >0) //move up
        {
            new_point.point.y--;
            return new UpWalker(map_,new_point,cost_);
        }

        else //move down
        {
            new_point.point.y++;
            //update orentation of cost kernal
            cv::Mat new_cost;
            cv::rotate(cost_,new_cost,cv::ROTATE_180);
            return new DownWalker(map_,new_point,new_cost);
        }
    }
    else if (axis_cost[AXIS_Y] ==0)
    {
        if(axis_cost[AXIS_X] > 0) // move right
        {
            new_point.point.x++;
            cv::Mat new_cost;
            cv::rotate(cost_,new_cost,cv::ROTATE_90_COUNTERCLOCKWISE);
            return new RightWalker(map_,new_point,new_cost);
        }
        else                    //move left
        {
            new_point.point.x--;
            cv::Mat new_cost;
            cv::rotate(cost_,new_cost,cv::ROTATE_90_CLOCKWISE);
            return new LeftWalker(map_,new_point,new_cost);
        }
    }
    else //diagonal movement
    {
        if(axis_cost[AXIS_Y] > 0)
        {
            if(axis_cost[AXIS_X] > 0)    //move up right
            {
                new_point.point.y--;
                new_point.point.x++;
                cv::Mat new_cost;
                cv::rotate(cost_,new_cost,cv::ROTATE_90_COUNTERCLOCKWISE);
                return new RightWalker(map_,new_point,new_cost);
            }

            else
            {
                new_point.point.y--;    //move up left
                new_point.point.x--;
                CostKernal new_cost;
                cv::rotate(cost_,new_cost,cv::ROTATE_90_CLOCKWISE);
                return new LeftWalker(map_,new_point,new_cost);
            }
        }

        else
        {
            if(axis_cost[AXIS_X] > 0)   //move down right
            {
                new_point.point.y++;
                new_point.point.x++;
                CostKernal new_cost;
                cv::rotate(cost_,new_cost,cv::ROTATE_90_COUNTERCLOCKWISE);
                return new RightWalker(map_,new_point,new_cost);
            }

            else                    //move down left
            {
                new_point.point.y++;
                new_point.point.x--;
                CostKernal new_cost;
                cv::rotate(cost_,new_cost,cv::ROTATE_90_CLOCKWISE);
                return new LeftWalker(map_,new_point,new_cost);
            }
        }

    }

}
Walker* RightWalker::walk()
{
    AxisCost axis_cost = genCost();

    clearNbh();

    //determine next cell to go and move
    Position new_point = point_;
    new_point.step++;

    if(axis_cost[AXIS_X]==0)
    {
        if(axis_cost[AXIS_Y] ==0)
        {    
            stopped = true;
            return this;
        }
        
        else if(axis_cost[AXIS_Y] >0) //move up
        {
            new_point.point.x++;
            return new RightWalker(map_,new_point,cost_);
        }

        else //move down
        {
            new_point.point.x--;
            //update orentation of cost kernal
            cv::Mat new_cost;
            cv::rotate(cost_,new_cost,cv::ROTATE_180);
            return new LeftWalker(map_,new_point,new_cost);
        }
    }
    else if (axis_cost[AXIS_Y] ==0)
    {
        if(axis_cost[AXIS_X] > 0) // move right
        {
            new_point.point.y++;
            cv::Mat new_cost;
            cv::rotate(cost_,new_cost,cv::ROTATE_90_COUNTERCLOCKWISE);
            return new DownWalker(map_,new_point,new_cost);
        }
        else
        {
            new_point.point.y--; // move left
            cv::Mat new_cost;
            cv::rotate(cost_,new_cost,cv::ROTATE_90_CLOCKWISE);
            return new UpWalker(map_,new_point,new_cost);
        }
    }
    else //diagonal movement
    {
        if(axis_cost[AXIS_Y] > 0)
        {
            if(axis_cost[AXIS_X] > 0)  //move up right
            {
                new_point.point.y++;
                new_point.point.x++;
                cv::Mat new_cost;
                cv::rotate(cost_,new_cost,cv::ROTATE_90_COUNTERCLOCKWISE);
                return new DownWalker(map_,new_point,new_cost);
            }

            else                     //moe up left
            {
                new_point.point.y--;
                new_point.point.x++;
                CostKernal new_cost;
                cv::rotate(cost_,new_cost,cv::ROTATE_90_CLOCKWISE);
                return new UpWalker(map_,new_point,new_cost);
            }
        }

        else
        {
            if(axis_cost[AXIS_X] > 0)    //move down right
            {
                new_point.point.y++;
                new_point.point.x--;
                CostKernal new_cost;
                cv::rotate(cost_,new_cost,cv::ROTATE_90_COUNTERCLOCKWISE);
                return new DownWalker(map_,new_point,new_cost);
            }

            else                           //move down left
            {
                new_point.point.y--;
                new_point.point.x--;
                CostKernal new_cost;
                cv::rotate(cost_,new_cost,cv::ROTATE_90_CLOCKWISE);
                return new LeftWalker(map_,new_point,new_cost);
            }
        }

    }

}

Walker* DownWalker::walk()
{
    AxisCost axis_cost = genCost();

    clearNbh();

    //determine next cell to go and move
    Position new_point = point_;
    new_point.step++;

    if(axis_cost[AXIS_X]==0)
    {
        if(axis_cost[AXIS_Y] ==0)
        {    
            stopped = true;
            return this;
        }
        
        else if(axis_cost[AXIS_Y] >0) //move up
        {
            new_point.point.y++;
            return new DownWalker(map_,new_point,cost_);
        }

        else //move down
        {
            new_point.point.y--;
            //update orentation of cost kernal
            cv::Mat new_cost;
            cv::rotate(cost_,new_cost,cv::ROTATE_180);
            return new UpWalker(map_,new_point,new_cost);
        }
    }
    else if (axis_cost[AXIS_Y] ==0)
    {
        if(axis_cost[AXIS_X] > 0) // move right
        {
            new_point.point.x--;
            cv::Mat new_cost;
            cv::rotate(cost_,new_cost,cv::ROTATE_90_COUNTERCLOCKWISE);
            return new LeftWalker(map_,new_point,new_cost);
        }
        else                    //move left
        {
            new_point.point.x++;
            cv::Mat new_cost;
            cv::rotate(cost_,new_cost,cv::ROTATE_90_CLOCKWISE);
            return new RightWalker(map_,new_point,new_cost);
        }
    }
    else //diagonal movement
    {
        if(axis_cost[AXIS_Y] > 0)
        {
            if(axis_cost[AXIS_X] > 0)    //move up right
            {
                new_point.point.y++;
                new_point.point.x--;
                cv::Mat new_cost;
                cv::rotate(cost_,new_cost,cv::ROTATE_90_COUNTERCLOCKWISE);
                return new LeftWalker(map_,new_point,new_cost);
            }

            else
            {
                new_point.point.y++;    //move up left
                new_point.point.x++;
                CostKernal new_cost;
                cv::rotate(cost_,new_cost,cv::ROTATE_90_CLOCKWISE);
                return new RightWalker(map_,new_point,new_cost);
            }
        }

        else
        {
            if(axis_cost[AXIS_X] > 0)   //move down right
            {
                new_point.point.y--;
                new_point.point.x--;
                CostKernal new_cost;
                cv::rotate(cost_,new_cost,cv::ROTATE_90_COUNTERCLOCKWISE);
                return new LeftWalker(map_,new_point,new_cost);
            }

            else                    //move down left
            {
                new_point.point.y--;
                new_point.point.x++;
                CostKernal new_cost;
                cv::rotate(cost_,new_cost,cv::ROTATE_90_CLOCKWISE);
                return new RightWalker(map_,new_point,new_cost);
            }
        }

    }

}

Walker* LeftWalker::walk()
{
    AxisCost axis_cost = genCost();

    clearNbh();

    //determine next cell to go and move
    Position new_point = point_;
    new_point.step++;

    if(axis_cost[AXIS_X]==0)
    {
        if(axis_cost[AXIS_Y] ==0)
        {    
            stopped = true;
            return this;
        }
        
        else if(axis_cost[AXIS_Y] >0) //move up
        {
            new_point.point.x--;
            return new LeftWalker(map_,new_point,cost_);
        }

        else //move down
        {
            new_point.point.x++;
            //update orentation of cost kernal
            cv::Mat new_cost;
            cv::rotate(cost_,new_cost,cv::ROTATE_180);
            return new RightWalker(map_,new_point,new_cost);
        }
    }
    else if (axis_cost[AXIS_Y] ==0)
    {
        if(axis_cost[AXIS_X] > 0) // move right
        {
            new_point.point.y--;
            cv::Mat new_cost;
            cv::rotate(cost_,new_cost,cv::ROTATE_90_COUNTERCLOCKWISE);
            return new UpWalker(map_,new_point,new_cost);
        }
        else                    //move left
        {
            new_point.point.y++;
            cv::Mat new_cost;
            cv::rotate(cost_,new_cost,cv::ROTATE_90_CLOCKWISE);
            return new DownWalker(map_,new_point,new_cost);
        }
    }
    else //diagonal movement
    {
        if(axis_cost[AXIS_Y] > 0)
        {
            if(axis_cost[AXIS_X] > 0)    //move up right
            {
                new_point.point.y--;
                new_point.point.x--;
                cv::Mat new_cost;
                cv::rotate(cost_,new_cost,cv::ROTATE_90_COUNTERCLOCKWISE);
                return new UpWalker(map_,new_point,new_cost);
            }

            else
            {
                new_point.point.y++;    //move up left
                new_point.point.x--;
                CostKernal new_cost;
                cv::rotate(cost_,new_cost,cv::ROTATE_90_CLOCKWISE);
                return new DownWalker(map_,new_point,new_cost);
            }
        }

        else
        {
            if(axis_cost[AXIS_X] > 0)   //move down right
            {
                new_point.point.y--;
                new_point.point.x++;
                CostKernal new_cost;
                cv::rotate(cost_,new_cost,cv::ROTATE_90_COUNTERCLOCKWISE);
                return new UpWalker(map_,new_point,new_cost);
            }

            else                    //move down left
            {
                new_point.point.y++;
                new_point.point.x++;
                CostKernal new_cost;
                cv::rotate(cost_,new_cost,cv::ROTATE_90_CLOCKWISE);
                return new DownWalker(map_,new_point,new_cost);
            }
        }

    }

}



bool Walker::isStopped()
{
    return stopped;
}

Walker::Position Walker::getPosition()
{
    return point_;
}

Walker::CostKernal Walker::getCostKernal()
{
    return cost_;
}

KernalWalker::KernalWalker(int intervals):
intervals_(intervals)
{
    Walker::CostKernal default_cost(3,3);

    default_cost.at<Walker::CellCost>(0,0) = Walker::CellCost(-0.5,0.5);
    default_cost.at<Walker::CellCost>(1,0) = Walker::CellCost(0,1.0);
    default_cost.at<Walker::CellCost>(2,0) = Walker::CellCost(0.5,0.5);
    default_cost.at<Walker::CellCost>(0,1) = Walker::CellCost(-0.5,0);
    default_cost.at<Walker::CellCost>(1,1) = Walker::CellCost(0.0);
    default_cost.at<Walker::CellCost>(2,1) = Walker::CellCost(0.5,0);
    default_cost.at<Walker::CellCost>(0,2) = Walker::CellCost(-0.5,-0.5);
    default_cost.at<Walker::CellCost>(1,2) = Walker::CellCost(0,-1.0);
    default_cost.at<Walker::CellCost>(2,2) = Walker::CellCost(0.5,-0.5);

    default_cost.copyTo(cost_kernal_);
}
KernalWalker::KernalWalker(Walker::CostKernal& cost_kernal, int intervals):
intervals_(intervals)
{
   cost_kernal.copyTo(cost_kernal_);

}


int KernalWalker::getTotalSteps()
{
    return totalSteps;
} 

std::vector<cv::Point> KernalWalker::getWayPoints()
{
    return waypoints;
}

void KernalWalker::genWaypoints(cv::Mat& map, cv::Point pose, int inital_step)
{
    Walker* walker_ptr  = new UpWalker(map,Walker::Position(pose,inital_step),cost_kernal_);
    waypoints.clear();

    while(!walker_ptr->isStopped())
    {
        if(walker_ptr->getPosition().step % intervals_ == 0){
            waypoints.push_back(walker_ptr->getPosition().point);
        }

        //debug used
        // cv::Mat dst = cv::Mat::zeros(map.rows, map.cols, CV_8UC3);
        // cv::cvtColor(map,dst,CV_GRAY2BGR);

        // dst.at<cv::Vec3b>(walker_ptr->getPosition().point.y,walker_ptr->getPosition().point.x) =0X000ff;    

        // cv::namedWindow( "map", cv::WINDOW_NORMAL);
        // cv::imshow("map",dst);
        // cv::waitKey(0);
        // //    

        walker_ptr = walker_ptr->walk();    
    }

    totalSteps = walker_ptr->getPosition().step;
}




