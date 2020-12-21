
#include <iostream>
#include <crop_map/crop_map.h>
#include <math.h>
#include "gtest/gtest.h"
#include <pthread.h>
#include <ros/ros.h>

struct CropMapTest : public testing::Test, public CropMap
{
protected:
  CropMapTest()
  {
    cv::Mat image(500, 1000, CV_8UC3, cv::Scalar(0, 0, 100));
    image_ = image;
    setMapOrigin(-50.0, -50.0);
    setResolution(0.05);
    crop_points_ = { { 10, 20 }, { 40, 100 }, { 5, 300 }, { 300, 5 } };
  }
};

TEST_F(CropMapTest, check_params)
{
  EXPECT_GT(image_.rows, 0);
  EXPECT_GT(image_.cols, 0);
  EXPECT_GT(resolution_, 0);
  EXPECT_GE(map_origin_x_, -50);
  EXPECT_GE(map_origin_y_, -50);
}

TEST_F(CropMapTest, points_in_map_frame)
{
  cv::Point_<double> pts_output;
  cv::Point_<double> pts_input = { 250, 200 };
  pts_output.x = -37.5;
  pts_output.y = -35;
  cv::Point_<double> pts = getPointsInMapFrame(pts_input);
  EXPECT_FLOAT_EQ(pts_output.x, pts.x);
  EXPECT_FLOAT_EQ(pts_output.y, pts.y);
}

TEST_F(CropMapTest, compute_origin)
{
  cv::Point origin_output = { 5, 300 };
  try
  {
    EXPECT_EQ(origin_output, computeOrigin());
  }
  catch (...)
  {
    ADD_FAILURE() << "Uncaught exception";
  }
}

TEST_F(CropMapTest, maskedImage)
{
  try
  {
    cv::Mat masked_image = maskImage();
    EXPECT_GT(masked_image.rows, 0);
    EXPECT_GT(masked_image.cols, 0);
  }
  catch (...)
  {
    ADD_FAILURE() << "Uncaught exception";
  }
}

TEST_F(CropMapTest, cropped_image)
{
  try
  {
    cropImage(maskImage());
    EXPECT_GT(cropped_image_.rows, 0);
    EXPECT_GT(cropped_image_.cols, 0);
  }
  catch (...)
  {
    ADD_FAILURE() << "Uncaught exception";
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "crop_map_test");
  return RUN_ALL_TESTS();
}
