#include <gtest/gtest.h>
#include <ros/ros.h>
#include <move_base_state/move_base_state.h>

class MoveBaseStateTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    state_sub_ = nh_.subscribe("/move_base_state/state", 1, &MoveBaseStateTest::onState, this);
  }

  virtual void TearDown()
  {
  }

  ros::NodeHandle nh_;
  ros::Subscriber state_sub_;

  MoveBaseStateTest()
  {
    state_ = 0;
  }

public:
  uint8_t state_;

  void onState(const std_msgs::UInt8::ConstPtr& msg)
  {
    state_ = msg->data;
  }
};

TEST_F(MoveBaseStateTest, testFailedState)
{
  ROS_ERROR("Failed to get a plan.");
  ros::Duration(0.5).sleep();
  EXPECT_EQ(state_, 1);
  ros::Duration(2.0).sleep();
  EXPECT_EQ(state_, 0);
}

TEST_F(MoveBaseStateTest, testRecoveryState)
{
  ROS_WARN("Rotate recovery behavior started.");
  ros::Duration(0.5).sleep();
  EXPECT_EQ(state_, 2);
  ros::Duration(2.0).sleep();
  EXPECT_EQ(state_, 0);
}

TEST_F(MoveBaseStateTest, testAbortedState)
{
  ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
  ros::Duration(0.5).sleep();
  EXPECT_EQ(state_, 3);
  ros::Duration(2.0).sleep();
  EXPECT_EQ(state_, 0);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "/move_base");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
