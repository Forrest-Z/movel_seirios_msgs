#include <velocity_setter/velocity_setter.h>
#include <ros/ros.h>
#include <gtest/gtest.h>

class VelocitySetterTest : public ::testing::Test
{

};

// setSpeed not reconfigured by client
TEST_F(VelocitySetterTest, testSpeedNotReconfigured)
{
  VelocitySetter vs;
  bool test_set_speed = vs.setSpeed(1, 1);
  EXPECT_FALSE(test_set_speed);
}

// setVelocity not reconfigured by client
TEST_F(VelocitySetterTest, testVelocityNotReconfigured)
{
  VelocitySetter vs;
  bool test_set_velocity = vs.setVelocity(1);
  EXPECT_FALSE(test_set_velocity);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "velocity_setter");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}