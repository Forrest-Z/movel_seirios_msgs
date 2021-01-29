#include <ros/ros.h>
#include <ros/package.h>
#include <gtest/gtest.h>
#include <std_msgs/Bool.h>
#include <movel_seirios_msgs/RunTaskListActionResult.h>
#include <movel_seirios_msgs/RunTaskListActionGoal.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <movel_seirios_msgs/Task.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>

#include <thread>
#include <chrono>
#include <stdio.h>

// Task supervisor needs to be started
class CleaningFixture : public ::testing::Test
{
protected:
  ros::NodeHandle nh;
  int status = 0;
  bool result = false;
  std::string task_supervisor_path;
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

  void setParam(std::string param, std::string val)
  {
    while (!ros::param::has(param))
      ;

    ros::param::set(param, val);
  }

  virtual void SetUp()
  {
    goal_pub = nh.advertise<movel_seirios_msgs::RunTaskListActionGoal>("/task_supervisor/goal", 1, true);
    status_sub = nh.subscribe("/task_supervisor/status", 1, &CleaningFixture::statusCB, this);
    result_sub = nh.subscribe("/task_supervisor/result", 1, &CleaningFixture::resultCB, this);

    // Set cleaning_handler params to test directory
    task_supervisor_path = ros::package::getPath("task_supervisor");

    setParam("/task_supervisor/cleaning_handler/big_map_path", task_supervisor_path + "/test_files/test.pgm");
    setParam("/task_supervisor/cleaning_handler/cropped_coordinates_path", task_supervisor_path + "/test_files/"
                                                                                                  "coordinates.txt");
    setParam("/task_supervisor/cleaning_handler/cropped_map_path", task_supervisor_path + "/test_files/cropped.png");
    setParam("/task_supervisor/cleaning_handler/yaml_path", task_supervisor_path + "/test_files/");
  }

  virtual ~CleaningFixture()
  {
  }
};

TEST_F(CleaningFixture, TestParamSet)
{
  std::string big_map_path, cropped_coords_path, cropped_path;
  ros::param::get("/task_supervisor/cleaning_handler/big_map_path", big_map_path);
  ros::param::get("/task_supervisor/cleaning_handler/cropped_coordinates_path", cropped_coords_path);
  ros::param::get("/task_supervisor/cleaning_handler/cropped_map_path", cropped_path);

  EXPECT_EQ(big_map_path, task_supervisor_path + "/test_files/test.pgm");
  EXPECT_EQ(cropped_coords_path, task_supervisor_path + "/test_files/coordinates.txt");
  EXPECT_EQ(cropped_path, task_supervisor_path + "/test_files/cropped.png");
}

// Test if starting successful
TEST_F(CleaningFixture, StartByTopic)
{
  movel_seirios_msgs::Task task_item;
  movel_seirios_msgs::RunTaskListActionGoal goal_item;

  task_item.type = 3;
  task_item.payload = task_supervisor_path + "/test_files/polygon.txt";
  goal_item.goal.task_list.tasks.push_back(task_item);
  goal_pub.publish(goal_item);

  // Wait for relevant launches to start
  auto start_time = ros::Time::now();
  while (status != 1)
  {
    if (ros::Time::now() - start_time > ros::Duration(30.0))
      FAIL() << "Start by topic timed out";
  }

  EXPECT_EQ(status, 1);
}

// Test if cropping produced image
TEST_F(CleaningFixture, ImageCropped)
{
  FILE* cropped_file;
  FILE* coord_file;
  auto start_time = ros::Time::now();
  do
  {
    cropped_file = fopen(std::string(task_supervisor_path + "/test_files/cropped.png").c_str(), "r");
    if (ros::Time::now() - start_time > ros::Duration(30.0))
      FAIL() << "Crop map file timed out";
  } while (cropped_file == NULL);

  start_time = ros::Time::now();
  do
  {
    coord_file = fopen(std::string(task_supervisor_path + "/test_files/coordinates.txt").c_str(), "r");
    if (ros::Time::now() - start_time > ros::Duration(30.0))
      FAIL() << "Coordinates text file timed out";
  } while (coord_file == NULL);

  fclose(cropped_file);
  fclose(coord_file);

  EXPECT_TRUE(coord_file != NULL);
  EXPECT_TRUE(cropped_file != NULL);
}

// Test if cancellable
TEST_F(CleaningFixture, CancelTest)
{
  result = true;
  ros::Publisher cancel_pub = nh.advertise<actionlib_msgs::GoalID>("/task_supervisor/cancel", 1, true);
  actionlib_msgs::GoalID cancel_item;

  cancel_pub.publish(cancel_item);

  auto start_time = ros::Time::now();
  while (result || status != 2)
  {
    if (ros::Time::now() - start_time > ros::Duration(30.0))
      FAIL() << "Cleaning cancel test timed out";
  }

  EXPECT_EQ(status, 2);
  EXPECT_FALSE(result);
}

// Test if planning timeout works, set timeout param then unset
TEST_F(CleaningFixture, PlanningTimeout)
{
  result = true;
  ros::param::set("/task_supervisor/cleaning_handler/planning_timeout", 0.01);

  movel_seirios_msgs::Task task_item;
  movel_seirios_msgs::RunTaskListActionGoal goal_item;

  task_item.type = 3;
  task_item.payload = task_supervisor_path + "/test_files/polygon.txt";
  goal_item.goal.task_list.tasks.push_back(task_item);
  goal_pub.publish(goal_item);

  // Wait for relevant launches to start
  auto start_time = ros::Time::now();
  while (status != 4 || result)
  {
    if (ros::Time::now() - start_time > ros::Duration(30.0))
      FAIL() << "Planning time out failed";
  }

  EXPECT_EQ(status, 4);
  EXPECT_FALSE(result);
  ros::param::set("/task_supervisor/cleaning_handler/planning_timeout", 15.0);
}

// Test if path received, /room_exploration_server/coverage_path
TEST_F(CleaningFixture, PathReceived)
{
  movel_seirios_msgs::Task task_item;
  movel_seirios_msgs::RunTaskListActionGoal goal_item;

  task_item.type = 3;
  task_item.payload = task_supervisor_path + "/test_files/polygon.txt";
  goal_item.goal.task_list.tasks.push_back(task_item);
  goal_pub.publish(goal_item);

  boost::shared_ptr<nav_msgs::Path const> path;
  path = ros::topic::waitForMessage<nav_msgs::Path>("/room_exploration_server/coverage_path", ros::Duration(30.0));

  EXPECT_TRUE(path != NULL);
}

// Test if path saver saved path
TEST_F(CleaningFixture, PathSaved)
{
  FILE* saved_path_file;
  auto start_time = ros::Time::now();
  do
  {
    saved_path_file = fopen(std::string(task_supervisor_path + "/test_files/coverage_path.yaml").c_str(), "r");
    if (ros::Time::now() - start_time > ros::Duration(30.0))
      FAIL() << "Timeout";
  } while (saved_path_file == NULL);

  fclose(saved_path_file);
  EXPECT_TRUE(saved_path_file != NULL);
}

// Test if robot distance too far
TEST_F(CleaningFixture, RobotTooFar)
{
  result = true;
  ros::param::set("/task_supervisor/cleaning_handler/start_distance_thresh", 0.5);
  movel_seirios_msgs::Task task_item;
  movel_seirios_msgs::RunTaskListActionGoal goal_item;

  task_item.type = 3;
  task_item.payload = task_supervisor_path + "/test_files/polygon.txt";
  goal_item.goal.task_list.tasks.push_back(task_item);
  goal_pub.publish(goal_item);

  // Wait for relevant launches to start
  auto start_time = ros::Time::now();
  while (status != 4 || result)
  {
    if (ros::Time::now() - start_time > ros::Duration(30.0))
      FAIL() << "Timeout";
  }

  EXPECT_EQ(status, 4);
  EXPECT_FALSE(result);
}

// Test if robot distance not too far
TEST_F(CleaningFixture, RobotNotTooFar)
{
  ros::param::set("/task_supervisor/cleaning_handler/start_distance_thresh", 10000.0);
  movel_seirios_msgs::Task task_item;
  movel_seirios_msgs::RunTaskListActionGoal goal_item;

  task_item.type = 3;
  task_item.payload = task_supervisor_path + "/test_files/polygon.txt";
  goal_item.goal.task_list.tasks.push_back(task_item);
  goal_pub.publish(goal_item);

  // Wait for result to succeed
  auto start_time = ros::Time::now();
  while (status != 1)
  {
    if (ros::Time::now() - start_time > ros::Duration(30.0))
      FAIL() << "Start by topic timed out";
  }

  EXPECT_EQ(status, 1);
}

// TODO Test if move_base is being triggered. Hard to test, need move_base/GlobalPlanner/make_plan service

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mapping_handler_test_node");
  ros::NodeHandle nh;
  ::testing::InitGoogleTest(&argc, argv);

  // Publish mock pose for tests to use
  geometry_msgs::Pose mock_pose;
  ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose>("/pose", 1, true);
  mock_pose.position.x = 0;
  mock_pose.position.y = 0;
  mock_pose.position.z = 0;
  pose_pub.publish(mock_pose);

  std::thread t([] {
    while (ros::ok())
      ros::spin();
  });
  int result = -1;

  bool srv_exists = ros::service::waitForService("/launch_manager/start_launch", ros::Duration(10.0));
  if (srv_exists)
    result = RUN_ALL_TESTS();

  std::string task_supervisor_path = ros::package::getPath("task_supervisor");
  std::remove(std::string(task_supervisor_path + "/test_files/cropped.png").c_str());
  std::remove(std::string(task_supervisor_path + "/test_files/coordinates.txt").c_str());
  std::remove(std::string(task_supervisor_path + "/test_files/coverage_path.yaml").c_str());

  ros::shutdown();
  t.join();
  return result;
}
