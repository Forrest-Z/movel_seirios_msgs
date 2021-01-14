#include "gtest/gtest.h"
#include "ros/ros.h"
#include <path_recall/PathName.h>
#include <path_recall/SavePath.h>
#include <path_recall/PathInfo.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>

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

class RecoveryNodeTest : public testing::Test
{
protected:
  virtual void SetUp()
  {
    name = "recovery_test";
    pose1 = createPose(1, 2, 3, 1, 2, 3, 4);
    pose2 = createPose(3, 2, 1, 4, 3, 2, 1);
    path1 = createPath(pose1);
    path2 = createPath(pose2);

    start_pub_ = nh_.advertise<std_msgs::Bool>("/path_load/start", 1);
    pose_pub_ = nh_.advertise<geometry_msgs::Pose>("/pose", 1);
    info_pub_ = nh_.advertise<path_recall::PathInfo>("/path_load/path_info", 1);
    recovery_client_ = nh_.serviceClient<path_recall::PathName>("/path_recovery/recovery");

    display_sub_ = nh_.subscribe("/path_recovery/display", 1, &RecoveryNodeTest::getDisplay, this);
    save_srv_ = nh_.advertiseService("/path_saver/save", &RecoveryNodeTest::onSave, this);
    load_srv_ = nh_.advertiseService("/path_load/load", &RecoveryNodeTest::onLoad, this);
  }

  virtual void TearDown()
  {
  }

  ros::NodeHandle nh_;
  ros::Publisher start_pub_, pose_pub_, info_pub_;
  ros::Subscriber display_sub_;
  ros::ServiceClient recovery_client_;
  ros::ServiceServer save_srv_, load_srv_;

  nav_msgs::Path path1, path2;
  geometry_msgs::Pose pose1, pose2;
  nav_msgs::Path path_saved, path_display;
  std::string name;
  std::string name_saved, name_loaded;

public:
  void getDisplay(const nav_msgs::Path::ConstPtr& msg)
  {
    path_display = *msg;
  }

  bool onSave(path_recall::SavePath::Request& req, path_recall::SavePath::Response& res)
  {
    name_saved = req.name;
    path_saved = req.path;
    res.success = true;
    return true;
  }

  bool onLoad(path_recall::PathName::Request& req, path_recall::PathName::Response& res)
  {
    name_loaded = req.name;
    res.success = true;
    return true;
  }

  std::string getSavedName()
  {
    return name_saved;
  }

  std::string getLoadedName()
  {
    return name_loaded;
  }

  nav_msgs::Path getSavedPath()
  {
    return path_saved;
  }

  nav_msgs::Path getDisplayPath()
  {
    return path_display;
  }
};

TEST_F(RecoveryNodeTest, test_path_recovery_node)
{
  ros::Rate loop_rate(0.5);

  path_recall::PathInfo info;
  info.name = name;
  info.path = path1;
  loop_rate.sleep();
  info_pub_.publish(info);
  loop_rate.sleep();

  std_msgs::Bool boolean;
  boolean.data = true;
  start_pub_.publish(boolean);
  loop_rate.sleep();

  pose_pub_.publish(pose2);
  loop_rate.sleep();

  boolean.data = false;
  start_pub_.publish(boolean);
  loop_rate.sleep();

  EXPECT_EQ((this->getDisplayPath()).poses.size(), 1);
  EXPECT_EQ((this->getSavedPath()).poses.size(), 1);
  EXPECT_EQ((this->getSavedName()), name + "(recovery)");

  path_recall::PathName srv;
  srv.request.name = name;
  loop_rate.sleep();
  recovery_client_.call(srv);
  loop_rate.sleep();
  EXPECT_EQ(srv.response.success, true);
  EXPECT_EQ((this->getLoadedName()), name + "(recovery)");
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "path_recovery_node_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
