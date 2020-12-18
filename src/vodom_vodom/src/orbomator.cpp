#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

cv::Ptr<cv::ORB> g_detector;

void imgCb(const sensor_msgs::ImageConstPtr msg);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odomator");
    ros::NodeHandle nh;

    g_detector = cv::ORB::create(512, 1.2, 8, 31, 0, 2, (cv::ORB::ScoreType)cv::ORB::HARRIS_SCORE, 31, 20);

    ros::Subscriber img_sub = nh.subscribe("/camera/color/image_raw", 1, imgCb);

    ros::spin();
}

void imgCb(const sensor_msgs::ImageConstPtr msg)
{
    ROS_INFO("new image");

    cv_bridge::CvImagePtr rgb_ptr = cv_bridge::toCvCopy(msg);
    std::vector<cv::KeyPoint> kps;
    cv::Mat descs;
    g_detector->detectAndCompute(rgb_ptr->image, cv::Mat(), kps, descs);

    ROS_INFO("%lu keypoints", kps.size());
}