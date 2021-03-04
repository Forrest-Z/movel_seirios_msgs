#ifndef TARGET_DETECTION_H
#define TARGET_DETECTION_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include <wall_inspection/point.h>
#include <wall_inspection/inspection_point_gen.h>
#include <queue>
namespace wall_inspection
{


class TargetDetection
{
    protected:
        //ros params
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Subscriber map_sub_;
        ros::Subscriber robot_pose_sub_;

        //set up params
        bool vis_;
        double robot_radius_, step_ = 5, length_ = 6, dist_from_wall_;
        int angle_step = 4;
        int angle_ksize = 3;
        bool has_robot_pose;
        bool has_map;
        //class attributes
        std::vector<Point> targets_;
        std::vector<Point> inspection_points_;

        //robot's current pose
        geometry_msgs::PoseWithCovarianceStamped robot_pose_;

        int step_px; //steps in pixel
        

        //\brief struct storing maps in cv::mat type
        //\param m_binaryMap Binary map storing the occupied spaces as white pixel
        //\param m_freeMap   Binary map storing the free space as white pixel
        //\param m_map_origin_pixel Pixel coordinate of the map origin
        struct BinaryMaps
        {
            nav_msgs::MapMetaData info;
            cv::Mat m_occupiedMap;
            cv::Mat m_freeMap;
            cv::Point m_map_origin_pixel;

            BinaryMaps(nav_msgs::MapMetaData info);
        };

        

        BinaryMaps* map_ptr;
        Inspection_Point_Gen* inspection_pt_gen_ptr;
        
    public:
        TargetDetection();
        TargetDetection(double robot_radius,double threshold, double length, double dist_from_wall);
        TargetDetection(double robot_radius,double step, double length,double dist_from_wall,double angle_step,double angle_ksize);
        ~TargetDetection() {};
        bool findTargets();
        bool findInspectionPoints();
        std::vector<Point> getTargets();
        std::vector<Point> getInspectionPoints();

    private:
        std::vector<cv::Point> neighbors(cv::Mat& map, cv::Point pose,int state);
        void lookup(cv::Mat& map, std::vector<cv::Point>& targets, cv::Point pose, int step);
        void lookupBFS(cv::Mat& map, std::vector<cv::Point>& targets, cv::Point pose, int step);
        void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& map); //recieves the map as a pointcloud and detect lines
        void robotPoseCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);
        cv::Point mapPointToImgPoint(geometry_msgs::Point map_point);
        Point imgPointToMapPoint(cv::Point img_point);
        //\brief Find the closest non zero point in given cv::mat map
        //\param point The point to find the closest point from
        //\param map The cv::mat to search the closest point in.
        //\return the closest point
        cv::Point findClosestPoint(cv::Point point,const cv::Mat& map);
        float findAngle(cv::Point p,cv::Mat &map);
        cv::Point leftMost(cv::Point p,cv::Mat &map);
        cv::Point rightMost(cv::Point p,cv::Mat &map);
        cv::Point findClosestBorderPoint(cv::Point p,const cv::Mat& map);

};


} //end namespace

#endif