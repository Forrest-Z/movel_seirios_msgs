#include <wall_inspection/target_detection.h>
#include <wall_inspection/inspection_point_gen.h>
#include <wall_inspection/kernal_walker.h>
#include <cmath>

#define DEBUG true

namespace wall_inspection
{

TargetDetection::TargetDetection(double robot_radius,double step, double length,double dist_from_wall) : nh_private_("~"),
    vis_(true),
    has_robot_pose(false),
    has_map(false),
    dist_from_wall_(dist_from_wall),
    robot_radius_(robot_radius)
{
    step_ = step;
    length_ = length;
    //todo<yb>: look into probablly using a service request
    map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &TargetDetection::mapCallBack, this);
    robot_pose_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose",1, &TargetDetection::robotPoseCallBack, this);

}

TargetDetection::TargetDetection(double robot_radius,double step, double length,double dist_from_wall,double angle_step,double angle_ksize) : nh_private_("~"),
    vis_(true),
    has_robot_pose(false),
    has_map(false),
    dist_from_wall_(dist_from_wall),
    robot_radius_(robot_radius)
{
    step_ = step;
    length_ = length;
    angle_step = angle_step;
    angle_ksize = angle_ksize;
    //todo<yb>: look into probablly using a service request
    map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &TargetDetection::mapCallBack, this);
    robot_pose_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose",1, &TargetDetection::robotPoseCallBack, this);

}

TargetDetection::BinaryMaps::BinaryMaps(nav_msgs::MapMetaData info):info(info)
{
    m_occupiedMap = cv::Mat(info.height, info.width, CV_8UC1);
    m_freeMap= cv::Mat(info.height, info.width, CV_8UC1);
}

std::vector<cv::Point> TargetDetection::neighbors(cv::Mat& map, cv::Point pose, int state)
{
    std::vector<cv::Point> pts;
    
    if ((int)map.at<uchar>(pose.y + 1, pose.x) == state)
    {
        cv::Point c = cv::Point(pose.x, pose.y + 1);
        pts.push_back(c);
    }
    if ((int)map.at<uchar>(pose.y, pose.x + 1) == state)
    {
        cv::Point c = cv::Point(pose.x + 1, pose.y);
        pts.push_back(c);
    }
    if ((int)map.at<uchar>(pose.y - 1, pose.x) == state)
    {
        cv::Point c = cv::Point(pose.x, pose.y - 1);
        pts.push_back(c);
    }
    
    if ((int)map.at<uchar>(pose.y, pose.x - 1) == state)
    {
        cv::Point c = cv::Point(pose.x - 1, pose.y);
        pts.push_back(c);
    }
    if ((int)map.at<uchar>(pose.y - 1, pose.x - 1) == state)
    {
        cv::Point c = cv::Point(pose.x - 1, pose.y - 1);
        pts.push_back(c);
    }
    if ((int)map.at<uchar>(pose.y + 1, pose.x + 1) == state)
    {
        cv::Point c = cv::Point(pose.x + 1, pose.y + 1);
        pts.push_back(c);
    }
    if ((int)map.at<uchar>(pose.y - 1, pose.x + 1) == state)
    {
        cv::Point c = cv::Point(pose.x + 1, pose.y - 1);
        pts.push_back(c);
    }
    if ((int)map.at<uchar>(pose.y + 1, pose.x - 1) == state)
    {
        cv::Point c = cv::Point(pose.x - 1, pose.y + 1);
        pts.push_back(c);
    }
    return pts;
}

void TargetDetection::lookup(cv::Mat& map, std::vector<cv::Point>& targets, cv::Point pose, int step)
{
    map.at<uchar>(pose.y,pose.x) = 0; 
    std::vector<cv::Point> nbrs = neighbors(map,pose,255);
    if (step % 10 == 0)
    {
        targets.push_back(pose);
    }
    for(cv::Point nb: nbrs)
    {
        lookup(map, targets, nbrs[0], (step+1) % 10); 
    }
   
    return;
}

//Breadth first search
void TargetDetection::lookupBFS(cv::Mat& map, std::vector<cv::Point>& targets, cv::Point pose, int s)
{  
    struct Position {
        cv::Point point;
        int step;
        Position(cv::Point p, int s){
            point = p;
            step = s;
        };
    };
    std::list<Position> queue;
    int step;
    Position p(pose,0);

    queue.push_back(p);
    while (!queue.empty()) 
    {
        //get current position info
        pose = queue.front().point;
        step = queue.front().step;
        queue.pop_front();
       
        //set current position as visited
        map.at<uchar>(pose.y,pose.x) = 0;
         
        if (step % s == 0) //steps interval for each target
        {
          targets.push_back(pose);
        }

        std::vector<cv::Point> nbrs = neighbors(map,pose,255);
        //clear and queue neighbour pixels
        for(cv::Point nb: nbrs)
        {
          map.at<uchar>(nb.y,nb.x) = 0; 
          Position n(nb,step+1);
          queue.push_back(n);
        } 
    }
   
    return;
}


void TargetDetection::mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& grid_map)
{
    //Step 1: Transform grid map to cv Mat
    map_ptr = new BinaryMaps(grid_map->info);

    std::vector<signed char>::const_iterator mapDataIter = grid_map->data.begin();

    unsigned char map_occ_thres = 65;
    unsigned char map_free_thres = 0; 

    // iterate over map, store in image
    // (0,0) is lower left corner of OccupancyGrid
    for( int i = grid_map->info.height-1; i >= 0 ; --i)
    {
        for( int j = 0; j < grid_map->info.width; ++j)
        {
            try{
            if (*mapDataIter > map_occ_thres)
                {
                    map_ptr->m_occupiedMap.at<uchar>(i,j) = 255;
                    map_ptr->m_freeMap.at<uchar>(i,j) = 0;
                } else if (*mapDataIter >= map_free_thres)
                {
                    map_ptr->m_occupiedMap.at<uchar>(i,j) = 0;
                    map_ptr->m_freeMap.at<uchar>(i,j) = 255;
                } else
                {
                    map_ptr->m_occupiedMap.at<uchar>(i,j) = 0;
                    map_ptr->m_freeMap.at<uchar>(i,j) = 0;
                }
            } catch(const std::exception& e){
                ROS_WARN("%s", e.what());
            }
            
            ++mapDataIter;
        }
    }

    //Note: CV's y axis is top down while map's y axis is bottom up
    //Finding the pixel coordinate of the origin from the bottom left edge
    //see: http://wiki.ros.org/map_server#YAML_format on origin
    cv::Point origin_pixel_from_bottom_left
    (
        (-map_ptr->info.origin.position.x)/map_ptr->info.resolution,
        -(-map_ptr->info.origin.position.y)/map_ptr->info.resolution
    );
    
    map_ptr->m_map_origin_pixel = cv::Point
    (
        origin_pixel_from_bottom_left.x, 
        map_ptr->info.height + origin_pixel_from_bottom_left.y 
    );

   

    has_map = true;

    inspection_pt_gen_ptr = new Inspection_Point_Gen(robot_radius_,dist_from_wall_,length_,grid_map);
    
}

void TargetDetection::robotPoseCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
    robot_pose_ = *pose;

    has_robot_pose = true;
}

bool TargetDetection::findTargets()
{
    if(!has_map)
    {
        ROS_WARN("[target_detection] Waiting for Map!");
        return false;
    }

    if(!has_robot_pose)
    {
        ROS_WARN("[target_detection] Waiting for Robot Pose!");
        return false;
    }
   
    cv::Point robot_pixel_point = mapPointToImgPoint(robot_pose_.pose.pose.position);
    Point robot_point = imgPointToMapPoint(robot_pixel_point);

    int length_px = length_ / map_ptr->info.resolution;
    int step_px = step_ / map_ptr->info.resolution;
    cv::imwrite("./actual_map.jpg",map_ptr->m_occupiedMap);
    cv::Mat dst = cv::Mat::zeros(map_ptr->info.width, map_ptr->info.height, CV_8UC3);
    cv::cvtColor(map_ptr->m_freeMap,dst,cv::COLOR_GRAY2BGR);
    //cv::imshow("map",map_ptr->m_occupiedMap);
    //cv::waitKey(1.0);
    
    //TODO: finish up filtering (apply median filtering) 
    //Dilation by 2 by2 kernel(connect dots)
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size( 3 , 3 ),cv::Point( 1, 1));
    cv::Mat m_dilate_1;
    cv::dilate(map_ptr->m_occupiedMap,m_dilate_1, element);

    cv::Mat filtered_map;
    cv::medianBlur(m_dilate_1,filtered_map,3); 
    cv::imwrite("./filtered_map.jpg",filtered_map);

    cv::Mat m_canny, m_dilate;
    element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size( 2 * length_px + 1, 2 * length_px + 1 ),cv::Point( length_px , length_px ));
    cv::dilate(filtered_map,m_dilate, element);

    cv::Canny(m_dilate,m_canny,30,70);



    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours( m_dilate, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE );

    //Mask unkown and occupied cells
    cv::bitwise_and(m_canny,map_ptr->m_freeMap,m_canny);
    cv::Mat m_canny_copy = m_canny.clone();
    cv::imwrite("./contours.jpg",m_canny);
    cv::imwrite("./dilate.jpg",m_dilate);
    
    //---generating coutour map message------------//
    nav_msgs::OccupancyGrid contours_map;
    contours_map.info = map_ptr->info;

    //Note: CV's y axis is top down while map's y axis is bottom up
    //Finding the pixel coordinate of the origin from the bottom left edge
    //see: http://wiki.ros.org/map_server#YAML_format on origin

    for(int y = m_canny.rows-1;y >=0 ; y-- )
    {
        for(int x = 0; x < m_canny.cols; x++)
            contours_map.data.push_back(m_canny.at<uchar>(y,x));
    }

    ros::Publisher contours_map_pub = nh_.advertise<nav_msgs::OccupancyGrid>("/wall_inspection/contours",1,true);
    contours_map_pub.publish(contours_map);

    //----generating target points from contour---------// 
    std::vector<cv::Point> targets;
    
    cv::Mat locations;   // output, locations of non-zero pixels 
    cv::findNonZero(m_canny, locations);
    int i = 0;
    ROS_INFO("lookup started");
    cv::Point c;
    cv::Point c_pose = mapPointToImgPoint(robot_pose_.pose.pose.position);
    int initial_step = 0;
    c = findClosestPoint(c_pose,locations);
    while(locations.size[0] > 0)
    {
        i++;
       
        Walker::CostKernal cost_kernal(3,3);
        //TODO<YB>: create a param to allow flexible kernal cost
        //setup cost kernal
        cost_kernal.at<Walker::CellCost>(0,0) = Walker::CellCost(-0.5,0.5);
        cost_kernal.at<Walker::CellCost>(1,0) = Walker::CellCost(0.0,1.0);
        cost_kernal.at<Walker::CellCost>(2,0) = Walker::CellCost(0.8,0.5);
        cost_kernal.at<Walker::CellCost>(0,1) = Walker::CellCost(-0.5,0.0);
        cost_kernal.at<Walker::CellCost>(1,1) = Walker::CellCost(0.0,0.0);
        cost_kernal.at<Walker::CellCost>(2,1) = Walker::CellCost(0.8,0.0);
        cost_kernal.at<Walker::CellCost>(0,2) = Walker::CellCost(-0.5,-0.8);
        cost_kernal.at<Walker::CellCost>(1,2) = Walker::CellCost(0,-1.5);
        cost_kernal.at<Walker::CellCost>(2,2) = Walker::CellCost(0.8,-0.8);
        //

        KernalWalker target_walker(cost_kernal,step_px);
        target_walker.genWaypoints(m_canny,c,initial_step);
        std::vector<cv::Point> new_targets = target_walker.getWayPoints();
        targets.insert(std::end(targets),std::begin(new_targets),std::end(new_targets));

        cv::findNonZero(m_canny, locations);
        if (targets.size() > 0)
        {
            c_pose = targets[targets.size() - 1];
            c = findClosestPoint(c_pose,locations);
            initial_step = target_walker.getTotalSteps();
            if(initial_step % step_px == 0) initial_step++; //to deal with the situation where the search fails right after a target is queued 
        }
          
        //debug

        // if (targets.size() > 0)
        // {
        //     c_pose = targets[targets.size() - 1];
        // }
        // cv::Mat start_mat = cv::Mat::zeros(map_ptr->info.width, map_ptr->info.height, CV_8UC1);
        // cv::line( start_mat, c, c, 255, 3, 1);

        // cv::Mat end_mat = cv::Mat::zeros(map_ptr->info.width, map_ptr->info.height, CV_8UC1);
        // cv::line( end_mat, c_pose, c_pose, 255, 3, 1);

        // std::vector<cv::Mat> array_to_merge;

        // array_to_merge.push_back(start_mat); //blue
        // array_to_merge.push_back(end_mat);   //green
        // array_to_merge.push_back(m_canny);   //red

        // cv::Mat color;

        // cv::merge(array_to_merge, color);
        // cv::imwrite("./map"+ std::to_string(i)+".jpg",color);
    }

    ROS_INFO("lookup ended %zu", targets.size());

    //------------------------------------------------//
    
    //geting the direction of the target pose 
    
    //use contours and check
    //cv::Mat gxx, gyy, angles;
    //cv::Sobel(m_dilate,gxx,CV_32FC1,1,0,5); //  x derivative
    //cv::Sobel(m_dilate,gyy,CV_32FC1,0,1,5); //  y derivative
    //cv::phase(gxx,gyy,angles,false);
    //
    //cv::imwrite("./Sobel_x.jpg",gxx);
    //cv::imwrite("./Sobel_y.jpg",gyy);
    //cv::imwrite("./angles.jpg",angles);

    targets_.clear();
    //buidling target queues in map coodinates
    int index = 0;
    if(DEBUG){
        //imshow("Display window", m_canny);
        cv::imwrite("./contours2.jpg",m_canny_copy);
        /*for (int j = 0; j < m_canny_copy.rows; j++)
        {
            for(int i = 0; i < m_canny_copy.cols; i++){
                uchar k = m_canny_copy.at<uchar>(j,i);
                if(k!=0){
                    ROS_INFO("%d",k);
                }
            }
        }*/
    }   


    for(cv::Point p:targets)
    {
        //temp limit targets to 2
        // if(targets_.size()>7)
        //     break;
        cv::Mat m_canny_debug = m_canny_copy.clone();
        Point pt = imgPointToMapPoint(p);
        try{
            
            cv::Point pinv(p.y,p.x);
            cv::Point p_x = p;
            cv::Point p_x2 = p;
            cv::Point wall_point = findClosestBorderPoint(pinv,filtered_map);
            uchar left_val = m_canny_copy.at<uchar>(p.x,p.y);
            if(left_val==0) {
                cv::Mat locations;   // output, locations of non-zero pixels 
                cv::findNonZero(m_canny_copy, locations);
                p_x = findClosestPoint(p,locations);
                p_x2 = findClosestBorderPoint(pinv,m_canny_copy);
            }
            if(DEBUG){
                m_canny_debug.at<uchar>(p_x2.x,p_x2.y) =63;
                m_canny_debug.at<uchar>(p_x2.x,p_x2.y) =63;
                cv::imwrite("./debug.jpg",m_canny_debug);
            }

            if(p_x2.x != -1 && p_x2.y != -1){
                pt.angle = findAngle(p_x2,m_canny_copy);
            }

            float angle =0.0;
            /*if(wall_point.y != wall_point.y){
                angle = atan(float(-wall_point.x+pinv.x)/float(wall_point.y - pinv.y));
            }else{
                if (wall_point.y>pinv.y){
                     angle = M_PI/2;
                } else{
                     angle = -M_PI/2;
                }
            }*/

            angle = atan2(float(-wall_point.x+pinv.x),float(wall_point.y-pinv.y));

            if(abs(angle-pt.angle)>M_PI/2){
                pt.angle = pt.angle + M_PI;
            }
        }
        catch(const std::exception& e){
            ROS_WARN("%s", e.what());
        }
        //pt.angle = 2*M_PI - pt.angle; //reflect over x axis due t0 CV's y axis is top down while map's y axis is bottom up
        targets_.push_back(pt);
        if (vis_)
        {
            cv::line( dst, p, p, cv::Scalar(255,0,0), 3, 1);
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << pt.angle;
            std::string ang = ss.str();
            cv::putText(dst, ang, p, 1, 0.4,cv::Scalar(0,0,255));
        }    
    }

    

    if(targets_.size() == 0)
    {
        ROS_WARN("[target_detection] No targets found!");
         return false;
    }

    
    return true;
}
//.x is row .y is column
float TargetDetection::findAngle(cv::Point p,cv::Mat &map){

    // find left movst value that is angle_step away
    uchar left_val = map.at<uchar>(p.x,p.y);
    cv::Point left_point = p;
    for(int j= 0;j<angle_step;j++){
        left_point = leftMost(left_point,map);
    }
    cv::Point right_point = p;
    for(int j = 0;j<angle_step;j++){
        right_point = rightMost(right_point,map);
    }

    float angle =0.0;
    if(right_point.x != left_point.x){
        angle = atan(float(right_point.y-left_point.y)/float(right_point.x - left_point.x));
    }else{
        if (right_point.y>left_point.y){
            angle = M_PI/2;
        } else{
            angle = -M_PI/2;
        }
    }
    //angle = atan2(float(right_point.y - left_point.y),float(right_point.x - left_point.x));
    return angle; 
}

cv::Point TargetDetection::rightMost(cv::Point p,cv::Mat &map){
    
    for(int k = 0;k<=angle_ksize/2;k++){
        int x = 0,y = 0;

        x = p.x-angle_ksize/2; 
        if(x< 0){
            x = 0;
        }

        y = p.y + k;
        if(y > map.cols){
            y = map.cols;
        }
        uchar left_val = map.at<uchar>(x,y);
        if(left_val!=0){
            cv::Point left_p(x,y);
            return left_p;
        } 
            
    }   
    
    for(int j = 0;j<angle_ksize;j++){
        int x = 0,y = 0;

        x = p.x-angle_ksize/2+j; 
        if(x< 0){
            x = 0;
        }

        y = p.y + angle_ksize/2;
        if(y >= map.cols){
            y = map.cols - 1;
        }
        uchar left_val = map.at<uchar>(x,y);
        if(left_val!=0){
            cv::Point left_p(x,y);
            return left_p;
        } 
    }
    return p;
        
}

cv::Point TargetDetection::leftMost(cv::Point p,cv::Mat &map){
    for(int k = 0;k<=angle_ksize/2;k++){
        long int x = 0,y = 0;

        x = p.x + angle_ksize/2; 
        if(x >= map.rows){
            x = map.rows - 1;
        }

        y = p.y - k;
        if(y < 0){
            y = 0;
        }

        uchar left_val = map.at<uchar>(x,y);
        if(left_val!=0){
            cv::Point right_p(x,y);
            return right_p;
        } 
            
    }   
    
    for(int j = 0;j<angle_ksize;j++){
        long int x = 0,y = 0;

        x = p.x+angle_ksize/2 - j; 
        if(x >= map.rows){
            x = map.rows - 1;
        }

        y = p.y - angle_ksize/2;
        if(y < 0){
            y = 0;
        }

        uchar left_val = map.at<uchar>(x,y);
        if(left_val!=0){
            cv::Point right_p(x,y);
            return right_p;
        } 
    }
    return p;
      
}

bool TargetDetection::findInspectionPoints()
{
    if(!findTargets()) return false;
    
    inspection_pt_gen_ptr->genInspectionPoints(targets_);   
    inspection_points_ = inspection_pt_gen_ptr->getInspectionPoints();

    if(inspection_points_.size() == 0) 
    {
        ROS_WARN("[target_detection] No inspection_points_ found!");
        return false;
    }

    return true;
    
}

cv::Point TargetDetection::mapPointToImgPoint(geometry_msgs::Point map_point)
{

    cv::Point map_point_pixel
    (
       map_ptr->m_map_origin_pixel.x + (map_point.x/map_ptr->info.resolution),
       map_ptr->m_map_origin_pixel.y + -(map_point.y/map_ptr->info.resolution)
    );

    return map_point_pixel;
}

Point TargetDetection::imgPointToMapPoint(cv::Point img_point)
{
    //Note: CV's y axis is top down while map's y axis is bottom up
    cv::Point img_point_from_map_origin_pixel
    (
         img_point.x - map_ptr->m_map_origin_pixel.x,
         -(img_point.y - map_ptr->m_map_origin_pixel.y)
    );

   Point map_point;

    map_point.x = (img_point_from_map_origin_pixel.x * map_ptr->info.resolution);

    map_point.y = (img_point_from_map_origin_pixel.y * map_ptr->info.resolution);

    return map_point;
}

cv::Point TargetDetection::findClosestPoint(cv::Point point,const cv::Mat& map)
{
    double min = 10000;
    cv::Point min_p;
        
    //find the point closest location point to c_pose 
    for (int j = 0; j < map.size[0]; j++)
    {
        double dist = sqrt(pow(map.at<cv::Point>(j).y - point.y,2) + pow(map.at<cv::Point>(j).x - point.x,2));
        if (dist < min)
        {
            min = dist;
            min_p = map.at<cv::Point>(j);
        }
    }

    return min_p;
}


cv::Point TargetDetection::findClosestBorderPoint(cv::Point point,const cv::Mat& map)
{
     // A border point on a canny filtered image (brofders as 1 and free space as 0)
    double min = 10000;
    cv::Point min_p;
    bool flag = true;
    std::queue<cv::Point> n_points;
    n_points.push(point);    
    bool *isVisited = new bool[map.rows*map.cols];
    for(int z = 0;z<map.rows*map.cols;z++){
        isVisited[z] = false;
    }
    int ctr = 0;
    //find the point closest location point to c_pose 
    while (!n_points.empty())
    {
        cv::Point frontier_point = n_points.front();
        n_points.pop();
        int value = map.at<int>(frontier_point.x,frontier_point.y);
        isVisited[frontier_point.x*map.rows+frontier_point.y] = true;

        uchar left_val = map.at<uchar>(frontier_point.x,frontier_point.y);
        if(left_val!=0){
            ROS_INFO(" Elements added : %d", ctr);
            delete isVisited;
            return frontier_point;
        }
        
        cv::Point left(frontier_point.x,frontier_point.y-1);
        cv::Point right(frontier_point.x,frontier_point.y+1);
        cv::Point up(frontier_point.x-1,frontier_point.y);
        cv::Point down(frontier_point.x+1,frontier_point.y); 
        if (left.y>=0 && !isVisited[left.x*map.rows+left.y]) {
            
            n_points.push(left);
            isVisited[left.x*map.rows+left.y] = true;
            ctr++;
        }
        if (up.x>=0 &&!isVisited[up.x*map.rows+up.y]) {
            n_points.push(up);
            isVisited[up.x*map.rows+up.y] = true;
            ctr++;
        }
        if (right.y<map.cols && !isVisited[right.x*map.rows+right.y]) {
            n_points.push(right);
            isVisited[right.x*map.rows+right.y] = true;
            ctr++;
        }
        if (down.x <map.rows && !isVisited[down.x*map.rows+down.y]) {
            n_points.push(down);
            isVisited[down.x*map.rows+down.y] = true;
            ctr++;
        }
       
    }
    ROS_INFO(" Elements added : %d", ctr);
    delete isVisited;
    return cv::Point(-1,-1);
}


std::vector<Point> TargetDetection::getTargets()
{
    return targets_;
}



std::vector<Point> TargetDetection::getInspectionPoints()
{
    return inspection_points_;
}


} //end namespace    




