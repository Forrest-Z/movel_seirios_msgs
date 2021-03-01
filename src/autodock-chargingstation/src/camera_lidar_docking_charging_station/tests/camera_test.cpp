#include <iostream>
#include <math.h>
#include "gtest/gtest.h"
#include<pthread.h>
#include<camera_docking_/camera_docking.h>

using namespace std;

struct FiducialsNodeTest : public testing::Test, public FiducialsNode{
protected:
    FiducialsNodeTest()
    {

    }

};


TEST_F(FiducialsNodeTest, dist)
{
    const cv::Point2f p1(10,20);
    const cv::Point2f p2(10, 20);
    double distance = 0;
    EXPECT_EQ (distance, dist(p1, p2));       // equating disctance
    EXPECT_GE (dist(p1, p2), 0);              // distance should be positive

}


TEST_F(FiducialsNodeTest, area) {

    std::vector<cv::Point2f> pts;
    const cv::Point2f p1(10,10);
    const cv::Point2f p2(10,0);
    const cv::Point2f p3(0,10);
    const cv::Point2f p4(0,0);
    pts.push_back(p1);
    pts.push_back(p2);
    pts.push_back(p3);
    pts.push_back(p4);
    EXPECT_GT (calcFiducialArea(pts), 0);    // area should be greater than 0

}


TEST_F(FiducialsNodeTest, computeImageSide)
{

    cv::Mat image (1920, 1080, CV_8UC3, cv::Scalar(0,0, 100));
    vector<vector<cv::Point2f> > corners;
    std::vector<cv::Point2f> pts;
    const cv::Point2f p1(10,10);
    const cv::Point2f p2(10,0);
    const cv::Point2f p3(0,10);
    const cv::Point2f p4(0,0);
    pts.push_back(p1);
    pts.push_back(p2);
    pts.push_back(p3);
    pts.push_back(p4);
    corners.push_back(pts);

    vector<int>ids {24};

    EXPECT_GT (computeImageSide(image, corners, ids), -1*image.cols);
    EXPECT_LT (computeImageSide(image, corners, ids), image.cols);

}



int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "crop_map_test");
    return RUN_ALL_TESTS();
}

