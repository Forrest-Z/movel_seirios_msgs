#include <path_recall/path_saver.h>
#include <path_recall/path_load_segments.h>
#include <path_recall/path_recovery.h>
#include <ros/ros.h>
#include <gtest/gtest.h>

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

auto createPath = [](geometry_msgs::Pose pose1, geometry_msgs::Pose pose2) {
  nav_msgs::Path path;
  path.header.frame_id = "map";
  geometry_msgs::PoseStamped Pose;
  Pose.header.frame_id = "map";
  Pose.pose = pose1;
  path.poses.push_back(Pose);
  Pose.pose = pose2;
  path.poses.push_back(Pose);
  return path;
};

class PathRecallTest : public ::testing::Test
{
protected:
  std::string name;
  geometry_msgs::Pose pose1, pose2;
  nav_msgs::Path path;

  PathRecallTest()
  {
    name = "unit_test";
    pose1 = createPose(1, 2, 3, 1, 2, 3, 4);
    pose2 = createPose(3, 2, 1, 4, 3, 2, 1);
    path = createPath(pose1, pose2);
  }
};

/*TEST_F(PathRecallTest, test_path)
{
  bool save_pass, load_pass;
  nav_msgs::Path output;
  PathSaver saver;
  PathLoadSegments loader;
  save_pass = saver.writePath(name, path);
  load_pass = loader.loadYAML(name, output);

  EXPECT_EQ(save_pass, true);
  EXPECT_EQ(load_pass, true);
  ASSERT_GT(output.poses.size(), 0);
  EXPECT_EQ(output.poses[0].pose.position.x, path.poses[0].pose.position.x);
  EXPECT_EQ(output.poses[0].pose.position.y, path.poses[0].pose.position.y);
  EXPECT_EQ(output.poses[0].pose.position.z, path.poses[0].pose.position.z);
  EXPECT_EQ(output.poses[0].pose.orientation.x, path.poses[0].pose.orientation.x);
  EXPECT_EQ(output.poses[0].pose.orientation.y, path.poses[0].pose.orientation.y);
  EXPECT_EQ(output.poses[0].pose.orientation.z, path.poses[0].pose.orientation.z);
  EXPECT_EQ(output.poses[0].pose.orientation.w, path.poses[0].pose.orientation.w);

  EXPECT_EQ(output.poses[1].pose.position.x, path.poses[1].pose.position.x);
  EXPECT_EQ(output.poses[1].pose.position.y, path.poses[1].pose.position.y);
  EXPECT_EQ(output.poses[1].pose.position.z, path.poses[1].pose.position.z);
  EXPECT_EQ(output.poses[1].pose.orientation.x, path.poses[1].pose.orientation.x);
  EXPECT_EQ(output.poses[1].pose.orientation.y, path.poses[1].pose.orientation.y);
  EXPECT_EQ(output.poses[1].pose.orientation.z, path.poses[1].pose.orientation.z);
  EXPECT_EQ(output.poses[1].pose.orientation.w, path.poses[1].pose.orientation.w);
}

TEST_F(PathRecallTest, test_empty)
{
  bool save_pass, load_pass;
  nav_msgs::Path empty, output;
  PathSaver saver;
  PathLoadSegments loader;
  save_pass = saver.writePath(name, empty);
  load_pass = loader.loadYAML(name, output);

  EXPECT_EQ(save_pass, false);
  EXPECT_EQ(load_pass, false);
}*/

TEST_F(PathRecallTest, test_recovery)
{
  PathRecovery recovery;
  recovery.tolerance = 0.1;
  nav_msgs::Path recovery_path;
  bool recovery_pass = recovery.Recovery(name);
  recovery_path = recovery.comparePath(path, path);

  EXPECT_EQ(recovery_pass, false);
  EXPECT_EQ(recovery_path.poses.size(), 0);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
