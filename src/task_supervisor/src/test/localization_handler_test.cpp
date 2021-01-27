#include <ros/ros.h>
#include <ros/package.h>
#include <gtest/gtest.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <movel_seirios_msgs/Task.h>
#include <movel_seirios_msgs/RunTaskListActionResult.h>
#include <movel_seirios_msgs/RunTaskListActionGoal.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <movel_seirios_msgs/StringTrigger.h>

#include <thread>
#include <chrono>

// Task supervisor needs to be started
class LocalizationFixture : public ::testing::Test
{
protected:
  ros::NodeHandle nh;
  int status = 0;
  bool result = false;
  bool localizing = true;
  ros::Publisher goal_pub;
  ros::Subscriber status_sub, result_sub, localizing_sub;

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

  void locCB(const std_msgs::BoolConstPtr& msg)
  {
    localizing = msg->data;
  }

  virtual void SetUp()
  {
    goal_pub = nh.advertise<movel_seirios_msgs::RunTaskListActionGoal>("/task_supervisor/goal", 1, true);
    status_sub = nh.subscribe("/task_supervisor/status", 1, &LocalizationFixture::statusCB, this);
    result_sub = nh.subscribe("/task_supervisor/result", 1, &LocalizationFixture::resultCB, this);
    localizing_sub =
        nh.subscribe("/task_supervisor/localization_handler/localizing", 1, &LocalizationFixture::locCB, this);
  }

  virtual ~LocalizationFixture()
  {
  }
};

// Start stop services exposed
TEST_F(LocalizationFixture, StartStopServices)
{
  EXPECT_TRUE(ros::service::exists("/task_supervisor/localization_handler/start", false));
  EXPECT_TRUE(ros::service::exists("/task_supervisor/localization_handler/stop", false));
}

// Localizing topic publishing
TEST_F(LocalizationFixture, LocalizingTopic)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  EXPECT_FALSE(localizing);
}

// Subscribed to map?
TEST_F(LocalizationFixture, MapSubscribed)
{
  ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  EXPECT_EQ(map_pub.getNumSubscribers(), 1);
}

// Stop before starting service, response should be false
TEST_F(LocalizationFixture, StopBeforeStart)
{
  ros::ServiceClient stop_client = nh.serviceClient<std_srvs::Trigger>("/task_supervisor/localization_handler/stop");
  std_srvs::Trigger stop_item;
  bool called = stop_client.call(stop_item);

  EXPECT_TRUE(called);
  EXPECT_FALSE(stop_item.response.success);
}

// Topic wrong payload
TEST_F(LocalizationFixture, WrongPayloadTopic)
{
  std::vector<std::string> wrong_args;
  wrong_args.push_back("staht");
  wrong_args.push_back(".lp0?1");
  wrong_args.push_back("stop1");
  wrong_args.push_back("start aa ab");
  wrong_args.push_back("stop aer");
  wrong_args.push_back("start map.yaml");

  while (wrong_args.size())
  {
    movel_seirios_msgs::Task task_item;
    movel_seirios_msgs::RunTaskListActionGoal goal_item;

    task_item.type = 1;
    task_item.payload = wrong_args.back();
    goal_item.goal.task_list.tasks.push_back(task_item);
    goal_pub.publish(goal_item);

    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    EXPECT_FALSE(localizing) << task_item.payload;
    wrong_args.pop_back();
  }
}

// Wrong payload start service
TEST_F(LocalizationFixture, WrongPayloadService)
{
  std::vector<std::string> wrong_args;
  wrong_args.push_back("staht");
  wrong_args.push_back(".lp0?1");
  wrong_args.push_back("stop1");
  wrong_args.push_back("start aa ab");
  wrong_args.push_back("/homze4/map.yaml");
  wrong_args.push_back("map.yaml");

  while (wrong_args.size())
  {
    ros::ServiceClient start_client =
        nh.serviceClient<movel_seirios_msgs::StringTrigger>("/task_supervisor/localization_handler/start");
    movel_seirios_msgs::StringTrigger start_item;

    start_item.request.input = wrong_args.back();
    bool call_success = start_client.call(start_item);

    EXPECT_TRUE(call_success);
    EXPECT_FALSE(start_item.response.success) << start_item.request.input;
    wrong_args.pop_back();
  }
}

// Start service with map successs
TEST_F(LocalizationFixture, StartWithServiceWithMap)
{
  localizing = false;
  ros::ServiceClient start_client =
      nh.serviceClient<movel_seirios_msgs::StringTrigger>("/task_supervisor/localization_handler/start");

  // Find task_supervisor package
  std::string task_supervisor_path = ros::package::getPath("task_supervisor");

  movel_seirios_msgs::StringTrigger start_item;

  start_item.request.input = task_supervisor_path + "/test_files/test.yaml";
  bool call_success = start_client.call(start_item);

  // Wait for relevant launches to start
  auto start_time = ros::Time::now();
  while (!localizing)
  {
    if (ros::Time::now() - start_time > ros::Duration(30.0))
      FAIL() << "Localizing start with service with map timed out";
  }

  EXPECT_TRUE(call_success);
  EXPECT_TRUE(start_item.response.success);
  EXPECT_TRUE(localizing);
}

// Stop service success
TEST_F(LocalizationFixture, StopWithService)
{
  localizing = true;
  ros::ServiceClient stop_client = nh.serviceClient<std_srvs::Trigger>("/task_supervisor/localization_handler/stop");

  std_srvs::Trigger stop_item;
  bool call_success = stop_client.call(stop_item);

  // Wait for relevant launches to start
  auto start_time = ros::Time::now();
  while (localizing)
  {
    if (ros::Time::now() - start_time > ros::Duration(30.0))
      FAIL() << "Localizing stop with service timed out";
  }

  EXPECT_TRUE(call_success);
  EXPECT_TRUE(stop_item.response.success);
  EXPECT_FALSE(localizing);
}

// Start service without map success
TEST_F(LocalizationFixture, StartWithService)
{
  localizing = false;
  ros::ServiceClient start_client =
      nh.serviceClient<movel_seirios_msgs::StringTrigger>("/task_supervisor/localization_handler/start");

  movel_seirios_msgs::StringTrigger start_item;
  bool call_success = start_client.call(start_item);

  // Wait for relevant launches to start
  auto start_time = ros::Time::now();
  while (!localizing)
  {
    if (ros::Time::now() - start_time > ros::Duration(30.0))
      FAIL() << "Localizing start with service timed out";
  }

  EXPECT_TRUE(call_success);
  EXPECT_TRUE(start_item.response.success);
  EXPECT_TRUE(localizing);
}

// Start without map success
TEST_F(LocalizationFixture, StartByTopic)
{
  localizing = false;
  movel_seirios_msgs::Task task_item;
  movel_seirios_msgs::RunTaskListActionGoal goal_item;

  task_item.type = 1;
  task_item.payload = "start";
  goal_item.goal.task_list.tasks.push_back(task_item);
  goal_pub.publish(goal_item);

  // Wait for relevant launches to start
  auto start_time = ros::Time::now();
  while (!localizing || !result)
  {
    if (ros::Time::now() - start_time > ros::Duration(30.0))
      FAIL() << "Localizing start timed out";
  }

  EXPECT_TRUE(localizing);
  EXPECT_TRUE(result);
}

// Stop success
TEST_F(LocalizationFixture, StopByTopic)
{
  localizing = true;
  movel_seirios_msgs::Task task_item;
  movel_seirios_msgs::RunTaskListActionGoal goal_item;

  task_item.type = 1;
  task_item.payload = "stop";
  goal_item.goal.task_list.tasks.push_back(task_item);
  goal_pub.publish(goal_item);

  // Wait for relevant launches to start
  auto start_time = ros::Time::now();
  while (localizing || !result)
  {
    if (ros::Time::now() - start_time > ros::Duration(30.0))
      FAIL() << "Localizing stop timed out";
  }

  EXPECT_FALSE(localizing);
  EXPECT_TRUE(result);
}

// Start with map successs
TEST_F(LocalizationFixture, StartWithMapByTopic)
{
  localizing = false;
  movel_seirios_msgs::Task task_item;
  movel_seirios_msgs::RunTaskListActionGoal goal_item;

  task_item.type = 1;

  // Find task_supervisor package
  std::string task_supervisor_path = ros::package::getPath("task_supervisor");

  task_item.payload = std::string("start") + " " + task_supervisor_path + "/test_files/test.yaml";
  goal_item.goal.task_list.tasks.push_back(task_item);
  goal_pub.publish(goal_item);

  // Wait for relevant launches to start
  auto start_time = ros::Time::now();
  while (!localizing || !result)
  {
    if (ros::Time::now() - start_time > ros::Duration(30.0))
      FAIL() << "Localizing start with map timed out";
  }

  EXPECT_TRUE(localizing);
  EXPECT_TRUE(result);
}

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
