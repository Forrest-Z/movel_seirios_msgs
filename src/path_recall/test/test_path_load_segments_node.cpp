#include "gtest/gtest.h"
#include "ros/ros.h"
#include <path_recall/PathName.h>
#include <path_recall/SavePath.h>
#include <path_recall/PathInfo.h>
#include <path_recall/PathCheck.h>
#include <nav_msgs/GetPlan.h>
#include <actionlib_msgs/GoalID.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Trigger.h>

auto createPose = [](double px, double py, double pz, double ox, double oy, double oz, double ow) {
  geometry_msgs::Pose pose;
  pose.position.x = px;
  pose.position.y = py;
  pose.position.z = pz;
  pose.orientation.x = ox;
  pose.orientation.y = oy;
  pose.orientation.z = oz;
  pose.orientation.w = ow;
  return pose;
};

auto createPath = [](geometry_msgs::Pose pose1) {
  nav_msgs::Path path;
  path.header.frame_id = "map";
  geometry_msgs::PoseStamped Pose;
  Pose.header.frame_id = "map";
  Pose.pose = pose1;
  path.poses.push_back(Pose);
  return path;
};

class LoaderNodeTest : public testing::Test
{
protected:
  virtual void SetUp()
  {
    cancel = false;
    goal_sent = false;
    name = "loader_test";
    pose1 = createPose(1, 2, 3, 1, 2, 3, 4);
    pose2 = createPose(3, 2, 1, 4, 3, 2, 1);
    path1 = createPath(pose1);
    path2 = createPath(pose2);

    display_sub_ = nh_.subscribe("/path_load/display", 1, &LoaderNodeTest::onDisplay, this);
    info_sub_ = nh_.subscribe("/path_load/path_info", 1, &LoaderNodeTest::onInfo, this);
    cancel_sub_ = nh_.subscribe("/move_base/cancel", 1, &LoaderNodeTest::onCancel, this);

    load_client_ = nh_.serviceClient<path_recall::PathName>("/path_load/load");
    pause_client_ = nh_.serviceClient<std_srvs::Trigger>("/path_load/pause");
    resume_client_ = nh_.serviceClient<std_srvs::Trigger>("/path_load/resume");
    cancel_client_ = nh_.serviceClient<std_srvs::Trigger>("/path_load/cancel");
    path_client_ = nh_.serviceClient<path_recall::SavePath>("/path_load/path_input");
    check_client_ = nh_.serviceClient<path_recall::PathCheck>("/path_load/check");

    plan_srv_ = nh_.advertiseService("/move_base/GlobalPlanner/make_plan", &LoaderNodeTest::onPlan, this);
  }

  virtual void TearDown()
  {
  }

  ros::NodeHandle nh_;
  ros::Subscriber display_sub_, info_sub_, cancel_sub_;
  ros::ServiceClient load_client_, pause_client_, resume_client_, cancel_client_, path_client_, check_client_;
  ros::ServiceServer plan_srv_;

  nav_msgs::Path path1, path2;
  geometry_msgs::Pose pose1, pose2;
  std::string name;
  bool cancel, goal_sent;
  path_recall::PathInfo path_info;
  nav_msgs::Path path_display;

public:
  void onDisplay(const nav_msgs::Path::ConstPtr& msg)
  {
    path_display = *msg;
  }

  void onInfo(const path_recall::PathInfo::ConstPtr& msg)
  {
    path_info = *msg;
  }

  void onCancel(const actionlib_msgs::GoalID::ConstPtr& msg)
  {
    cancel = true;
  }

  bool onPlan(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res)
  {
    goal_sent = true;
    nav_msgs::Path plan;
    res.plan = plan;
    return true;
  }

  nav_msgs::Path getDisplay()
  {
    return path_display;
  }

  path_recall::PathInfo getInfo()
  {
    return path_info;
  }

  bool getCancel()
  {
    return cancel;
  }

  bool getPlan()
  {
    return goal_sent;
  }

  void resetCancel()
  {
    cancel = false;
  }
};

TEST_F(LoaderNodeTest, test_path_load_segments_node)
{
  ros::Rate loop_rate(0.5);

  path_recall::PathName load_srv;
  load_srv.request.name = name;
  loop_rate.sleep();
  load_client_.call(load_srv);
  loop_rate.sleep();
  EXPECT_EQ(load_srv.response.success, false);

  std_srvs::Trigger pause_srv;
  loop_rate.sleep();
  pause_client_.call(pause_srv);
  loop_rate.sleep();
  EXPECT_EQ(pause_srv.response.success, true);
  EXPECT_EQ((this->getCancel()), true);
  resetCancel();

  std_srvs::Trigger resume_srv;
  loop_rate.sleep();
  resume_client_.call(resume_srv);
  loop_rate.sleep();
  EXPECT_EQ(resume_srv.response.success, false);

  std_srvs::Trigger cancel_srv;
  loop_rate.sleep();
  cancel_client_.call(cancel_srv);
  loop_rate.sleep();
  EXPECT_EQ(cancel_srv.response.success, true);
  EXPECT_EQ((this->getCancel()), true);

  path_recall::PathCheck check_srv;
  check_srv.request.name = name;
  loop_rate.sleep();
  check_client_.call(check_srv);
  loop_rate.sleep();
  EXPECT_EQ(check_srv.response.pass, false);

  path_recall::SavePath path_srv;
  path_srv.request.name = name;
  path_srv.request.path = path1;
  loop_rate.sleep();
  path_client_.call(path_srv);
  loop_rate.sleep();
  EXPECT_EQ(path_srv.response.success, true);
  EXPECT_EQ((this->getDisplay()).poses.size(), 1);
  EXPECT_EQ((this->getInfo()).name, name);
  EXPECT_EQ((this->getInfo()).path.poses.size(), 1);
  EXPECT_EQ((this->getPlan()), true);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "path_load_segments_node_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
