#include <gtest/gtest.h>
#include "human_detection/human_detection.h"

class TestHumanDetection : public ::testing::Test
{
  protected:
  TestHumanDetection()
  {}
};

TEST_F(TestHumanDetection, testHistoryAveraging)
{
  HumanDetection detect;
  detect.frames_tracked_ = 5;

  double average;
  EXPECT_EQ(detect.historyAveraging(1, average), false);
}

TEST_F(TestHumanDetection, testCropMap)
{
  HumanDetection detect;
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;

  EXPECT_EQ(detect.getCroppedMap(pose), 0);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
