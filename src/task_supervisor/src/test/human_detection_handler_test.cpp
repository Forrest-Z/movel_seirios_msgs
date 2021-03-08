#include <ros/ros.h>
#include <ros/package.h>
#include <gtest/gtest.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <movel_seirios_msgs/Task.h>
#include <movel_seirios_msgs/RunTaskListActionResult.h>
#include <movel_seirios_msgs/RunTaskListActionGoal.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <movel_seirios_msgs/StringTrigger.h>

#include <thread>
#include <chrono>

// Task supervisor needs to be started
class HumanDetectionFixture : public ::testing::Test
{
protected:
  ros::NodeHandle nh;
  int status = 0;
  bool result = false;
  ros::Publisher goal_pub;
  ros::Subscriber status_sub, result_sub;

  void statusCB(const actionlib_msgs::GoalStatusArrayConstPtr& msg)
  {
    if (!msg->status_list.empty())
    {
      status = msg->status_list[0].status;
    }
  }

  void resultCB(const movel_seirios_msgs::RunTaskListActionResultConstPtr& msg)
  {
    result = msg->result.success;
  }

  virtual void SetUp()
  {
    goal_pub = nh.advertise<movel_seirios_msgs::RunTaskListActionGoal>("/task_supervisor/goal", 1, true);
    status_sub = nh.subscribe("/task_supervisor/status", 1, &HumanDetectionFixture::statusCB, this);
    result_sub = nh.subscribe("/task_supervisor/result", 1, &HumanDetectionFixture::resultCB, this);
  }

  virtual ~HumanDetectionFixture()
  {
  }
};

// Start stop services exposed
TEST_F(HumanDetectionFixture, StartStopServices)
{
  EXPECT_TRUE(ros::service::exists("/task_supervisor/human_detection_handler/start", false));
  EXPECT_TRUE(ros::service::exists("/task_supervisor/human_detection_handler/stop", false));
  EXPECT_TRUE(ros::service::exists("/task_supervisor/human_detection_handler/status", false));
}

// Stop before starting service, response should be false
TEST_F(HumanDetectionFixture, StopBeforeStart)
{
  ros::ServiceClient stop_client = nh.serviceClient<std_srvs::Trigger>("/task_supervisor/human_detection_handler/stop");
  std_srvs::Trigger stop_item;
  bool called = stop_client.call(stop_item);

  EXPECT_TRUE(called);
  EXPECT_FALSE(stop_item.response.success);
}

// Start service success
TEST_F(HumanDetectionFixture, StartWithService)
{
  ros::ServiceClient start_client =
      nh.serviceClient<movel_seirios_msgs::StringTrigger>("/task_supervisor/human_detection_handler/start");
  ros::ServiceClient status_client = nh.serviceClient<std_srvs::Trigger>("/task_supervisor/human_detection_handler/status");

  movel_seirios_msgs::StringTrigger start_item;
  bool call_success = start_client.call(start_item);

  std_srvs::Trigger status_item;

  // Wait for relevant launches to start
  auto start_time = ros::Time::now();
  while (!status_client.call(status_item))
  {
    if (ros::Time::now() - start_time > ros::Duration(30.0))
      FAIL() << "Human detection start with service timed out";
  }

  EXPECT_TRUE(call_success);
  EXPECT_TRUE(start_item.response.success);
  EXPECT_TRUE(status_item.response.success);
}

// Stop service success
TEST_F(HumanDetectionFixture, StopWithService)
{
  ros::ServiceClient stop_client = nh.serviceClient<std_srvs::Trigger>("/task_supervisor/human_detection_handler/stop");
  ros::ServiceClient status_client = nh.serviceClient<std_srvs::Trigger>("/task_supervisor/human_detection_handler/status");

  std_srvs::Trigger stop_item;
  bool call_success = stop_client.call(stop_item);

  std_srvs::Trigger status_item;

  // Wait for relevant launches to start
  auto start_time = ros::Time::now();
  while (!status_client.call(status_item))
  {
    if (ros::Time::now() - start_time > ros::Duration(30.0))
      FAIL() << "Human detection stop with service timed out";
  }

  EXPECT_TRUE(call_success);
  EXPECT_TRUE(stop_item.response.success);
  EXPECT_FALSE(status_item.response.success);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "human_detection_handler_test_node");
  ros::NodeHandle nh;
  ::testing::InitGoogleTest(&argc, argv);

  std::thread t([] {
    while (ros::ok())
      ros::spin();
  });
  int result = -1;

  bool srv_exists = ros::service::waitForService("/launch_manager/start_launch", ros::Duration(10.0));
  if (srv_exists)
    result = RUN_ALL_TESTS();

  ros::shutdown();
  t.join();
  return result;
}
