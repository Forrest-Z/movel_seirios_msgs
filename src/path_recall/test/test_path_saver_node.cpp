#include "gtest/gtest.h"
#include "ros/ros.h"
#include <path_recall/PathName.h>
#include <path_recall/SavePath.h>
#include <path_recall/PathInfo.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Trigger.h>
#include <sys/stat.h>
#include <sys/types.h>

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

class SaverNodeTest : public testing::Test
{
protected:
  virtual void SetUp()
  {
    name = "saver_test";
    pose1 = createPose(1, 2, 3, 1, 2, 3, 4);
    pose2 = createPose(3, 2, 1, 4, 3, 2, 1);
    path1 = createPath(pose1);
    path2 = createPath(pose2);

    pose_pub_ = nh_.advertise<geometry_msgs::Pose>("/pose", 1);

    start_client_ = nh_.serviceClient<path_recall::PathName>("/path_saver/start");
    stop_client_ = nh_.serviceClient<std_srvs::Trigger>("/path_saver/stop");
    save_client_ = nh_.serviceClient<path_recall::SavePath>("/path_saver/save");
  }

  virtual void TearDown()
  {
  }

  ros::NodeHandle nh_;
  ros::Publisher pose_pub_;
  ros::ServiceClient start_client_, stop_client_, save_client_;

  nav_msgs::Path path1, path2;
  geometry_msgs::Pose pose1, pose2;
  std::string name;
};

TEST_F(SaverNodeTest, test_path_saver_node)
{
  struct stat st;
  if (stat("saver_test", &st) == -1)
  {
    mkdir("saver_test", 0777);
  }

  ros::Rate loop_rate(0.5);

  path_recall::PathName start_srv;
  start_srv.request.name = name;
  loop_rate.sleep();
  start_client_.call(start_srv);
  loop_rate.sleep();
  EXPECT_EQ(start_srv.response.success, true);

  pose_pub_.publish(pose1);
  loop_rate.sleep();
  pose_pub_.publish(pose2);
  loop_rate.sleep();

  std_srvs::Trigger stop_srv;
  loop_rate.sleep();
  stop_client_.call(stop_srv);
  loop_rate.sleep();
  EXPECT_EQ(stop_srv.response.success, true);

  path_recall::SavePath save_srv;
  save_srv.request.name = name;
  save_srv.request.path = path1;
  loop_rate.sleep();
  save_client_.call(save_srv);
  loop_rate.sleep();
  EXPECT_EQ(save_srv.response.success, true);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "path_saver_node_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
