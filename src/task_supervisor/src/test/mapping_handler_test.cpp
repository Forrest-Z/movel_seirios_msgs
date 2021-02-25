#include <ros/ros.h>
#include <gtest/gtest.h>
#include <movel_seirios_msgs/RunTaskListActionGoal.h>
#include <movel_seirios_msgs/Task.h>
#include <movel_seirios_msgs/RunTaskListActionResult.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalID.h>
#include <movel_seirios_msgs/StringTrigger.h>
#include <nav_msgs/OccupancyGrid.h>

#include <thread>
#include <chrono>

// Task supervisor needs to be started
class MappingFixture : public ::testing::Test
{
protected:
  ros::NodeHandle nh;
  int status = 0, result = false;
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
    status_sub = nh.subscribe("/task_supervisor/status", 1, &MappingFixture::statusCB, this);
    result_sub = nh.subscribe("/task_supervisor/result", 1, &MappingFixture::resultCB, this);
  }

  virtual ~MappingFixture()
  {
  }
};

// Test if start mapping succeeded
TEST_F(MappingFixture, StartSuccess)
{
  movel_seirios_msgs::Task task_item;
  movel_seirios_msgs::RunTaskListActionGoal goal_item;

  task_item.type = 2;
  goal_item.goal.task_list.tasks.push_back(task_item);
  goal_pub.publish(goal_item);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  EXPECT_EQ(status, 1);
}

// Test if save service is exposed when mapping has started
TEST_F(MappingFixture, SaveExists)
{
  bool save_exists = ros::service::exists("/task_supervisor/mapping_handler/save_map", false);
  ASSERT_TRUE(save_exists);
}

// Test if save timeout works
TEST_F(MappingFixture, SaveTimeout)
{
  ros::ServiceClient save_client =
      nh.serviceClient<movel_seirios_msgs::StringTrigger>("/task_supervisor/mapping_handler/save_map_async");

  movel_seirios_msgs::StringTrigger save_item;
  bool call_success = save_client.call(save_item);

  EXPECT_TRUE(call_success);
  EXPECT_FALSE(save_item.response.success);
}

// Test if save succeeded and mapping ends after saving, publish mock /map
TEST_F(MappingFixture, SaveSuccess_TaskFinished)
{
  ros::ServiceClient save_client =
      nh.serviceClient<movel_seirios_msgs::StringTrigger>("/task_supervisor/mapping_handler/save_map");
  ros::Publisher empty_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);

  nav_msgs::OccupancyGrid empty_map;
  movel_seirios_msgs::StringTrigger save_item;

  empty_map_pub.publish(empty_map);
  bool call_success = save_client.call(save_item);

  // Wait for save to finish
  auto start_time = ros::Time::now();
  while (status != 3)
  {
    if (ros::Time::now() - start_time > ros::Duration(30.0))
      FAIL() << "Status update timed out";
  }

  EXPECT_TRUE(call_success);
  EXPECT_TRUE(save_item.response.success);
  EXPECT_EQ(status, 3);
  EXPECT_TRUE(result);
}

// Test if save map service ends after saving
TEST_F(MappingFixture, SaveStopped)
{
  bool save_exists = ros::service::exists("/task_supervisor/mapping_handler/save_map", false);
  EXPECT_FALSE(save_exists);
}

// Test if cancelling goal when mapping is running succeeds
TEST_F(MappingFixture, CancelTest)
{
  movel_seirios_msgs::Task task_item;
  movel_seirios_msgs::RunTaskListActionGoal goal_item;

  task_item.type = 2;
  goal_item.goal.task_list.tasks.push_back(task_item);
  goal_pub.publish(goal_item);

  std::this_thread::sleep_for(std::chrono::seconds(1));

  ros::Publisher cancel_pub = nh.advertise<actionlib_msgs::GoalID>("/task_supervisor/cancel", 1, true);
  actionlib_msgs::GoalID cancel_item;

  cancel_pub.publish(cancel_item);

  std::this_thread::sleep_for(std::chrono::seconds(1));

  EXPECT_FALSE(result);
}

// TODO Check if map is actually saved

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mapping_handler_test_node");
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
