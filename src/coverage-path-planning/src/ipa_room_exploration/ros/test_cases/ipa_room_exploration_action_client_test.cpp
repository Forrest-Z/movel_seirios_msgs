
#include <iostream>
#include <math.h>
#include "gtest/gtest.h"
#include <pthread.h>
#include <ipa_room_exploration/room_exploration_client.h>

using namespace std;

struct ExplorationClient_test : public testing::Test
{
  ExplorationClient* obj;
  ExplorationClient_test()
  {
    obj = new ExplorationClient(0.05, { 0, 0, 0 }, 0.117, 0.25, { 0, 0, 0 });
  }
};

TEST_F(ExplorationClient_test, check_coverage_radius)
{
  EXPECT_GT(obj->get_resolution(), 0);
  EXPECT_GT(obj->get_coverage_radius(), 0);
  EXPECT_GT(obj->get_robot_radius(), 0);
}

TEST_F(ExplorationClient_test, check_map_image)
{
  string map_image = "/home/movel/sim_map2.png";
  cv::Mat image = cv::imread(map_image);

  EXPECT_GT(image.rows, 0);
  EXPECT_GT(image.cols, 0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
