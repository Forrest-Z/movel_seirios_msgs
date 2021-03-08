#include <ros/ros.h>
#include <gtest/gtest.h>
#include <movel_seirios_msgs/LaunchExists.h>
#include <movel_seirios_msgs/StartLaunch.h>
#include <movel_seirios_msgs/StopLaunch.h>

unsigned int test_id = 0;

class LaunchFixture : public ::testing::Test
{
protected:
  ros::NodeHandle nh;
  ros::ServiceClient exists_client = nh.serviceClient<movel_seirios_msgs::LaunchExists>("/launch_manager/launch_exists");
  ros::ServiceClient start_client = nh.serviceClient<movel_seirios_msgs::StartLaunch>("/launch_manager/start_launch");
  ros::ServiceClient stop_client = nh.serviceClient<movel_seirios_msgs::StopLaunch>("/launch_manager/stop_launch");
};

TEST_F(LaunchFixture, StartSuccessWithArgs)
{
  movel_seirios_msgs::StartLaunch start_item;
  start_item.request.package = "task_supervisor";
  start_item.request.launch_file = "map_saver.launch";
  start_item.request.args = "map_topic:=/map";
  start_client.call(start_item);

  EXPECT_GT(start_item.response.launch_id, 0);
}

TEST_F(LaunchFixture, StartSuccessNoArgs)
{
  movel_seirios_msgs::StartLaunch start_item;
  start_item.request.package = "task_supervisor";
  start_item.request.launch_file = "map_saver.launch";
  start_item.request.args = "";
  start_client.call(start_item);

  test_id = start_item.response.launch_id;

  EXPECT_GT(test_id, 0);
}

TEST_F(LaunchFixture, StartFailNoLaunch)
{
  movel_seirios_msgs::StartLaunch start_item;
  start_item.request.package = "task_supervisor";
  start_item.request.launch_file = "xxxx";
  start_client.call(start_item);

  EXPECT_EQ(start_item.response.launch_id, 0);
}

TEST_F(LaunchFixture, StartFailNoPackage)
{
  movel_seirios_msgs::StartLaunch start_item;
  start_item.request.package = "xxxx";
  start_item.request.launch_file = "map_saver.launch";
  start_client.call(start_item);

  EXPECT_EQ(start_item.response.launch_id, 0);
}

TEST_F(LaunchFixture, StartFailInvalidArgsSpace)
{
  movel_seirios_msgs::StartLaunch start_item;
  start_item.request.package = "task_supervisor";
  start_item.request.launch_file = "map_saver.launch";
  start_item.request.args = "test  test arg:= extraSpace";
  start_client.call(start_item);

  EXPECT_EQ(start_item.response.launch_id, 0);
}

// TODO launch_manager is unable to check for random symbols in args, roslaunch in command line does not do this
// either if the arguments are enclosed in ''

// TEST_F(LaunchFixture, StartFailInvalidArgsSymbol){
//	movel_seirios_msgs::StartLaunch start_item;
//	start_item.request.package = "task_supervisor";
//	start_item.request.launch_file = "map_saver.launch";
//	start_item.request.args = "map_topic;:=/map";
//	start_client.call(start_item);
//
//	EXPECT_EQ(start_item.response.launch_id, 0);
//}

TEST_F(LaunchFixture, LaunchExistTrue)
{
  movel_seirios_msgs::LaunchExists exists_item;
  exists_item.request.launch_id = test_id;
  exists_client.call(exists_item);

  EXPECT_TRUE(exists_item.response.exists);
}

TEST_F(LaunchFixture, LaunchExistFalse)
{
  movel_seirios_msgs::LaunchExists exists_item;
  exists_item.request.launch_id = 0;
  exists_client.call(exists_item);

  EXPECT_FALSE(exists_item.response.exists);
}

TEST_F(LaunchFixture, StopSuccess)
{
  movel_seirios_msgs::StopLaunch stop_item;
  stop_item.request.launch_id = test_id;
  stop_client.call(stop_item);

  EXPECT_TRUE(stop_item.response.success);
}

TEST_F(LaunchFixture, StopFail)
{
  movel_seirios_msgs::StopLaunch stop_item;
  stop_item.request.launch_id = 0;
  stop_client.call(stop_item);

  EXPECT_FALSE(stop_item.response.success);
}

// Start then stop immediately
TEST_F(LaunchFixture, StartStop)
{
  movel_seirios_msgs::StartLaunch start_item;
  movel_seirios_msgs::StopLaunch stop_item;
  start_item.request.package = "task_supervisor";
  start_item.request.launch_file = "map_saver.launch";
  start_item.request.args = "";

  start_client.call(start_item);
  stop_item.request.launch_id = start_item.response.launch_id;
  stop_client.call(stop_item);

  EXPECT_TRUE(stop_item.response.success);
}

// Start then check existence immediately
TEST_F(LaunchFixture, StartCheck)
{
  movel_seirios_msgs::StartLaunch start_item;
  movel_seirios_msgs::LaunchExists check_item;
  start_item.request.package = "task_supervisor";
  start_item.request.launch_file = "map_saver.launch";
  start_item.request.args = "";

  start_client.call(start_item);
  check_item.request.launch_id = start_item.response.launch_id;
  exists_client.call(check_item);

  EXPECT_TRUE(check_item.response.exists);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "launch_manager_test_node");
  ros::NodeHandle nh;
  ::testing::InitGoogleTest(&argc, argv);
  bool srvExists = ros::service::waitForService("/launch_manager/start_launch", ros::Duration(10.0));

  if (srvExists)
    return RUN_ALL_TESTS();

  else
  {
    ROS_ERROR("Launch manager service not found");
    return -1;
  }
}
