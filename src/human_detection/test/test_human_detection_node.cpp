#include <gtest/gtest.h>
#include "ros/ros.h"
#include "human_detection/human_detection.h"

class HumanDetectionNodeTest : public ::testing::Test
{
  protected:
    virtual void SetUp() 
    {
      pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 1);
      pose_pub_ = nh_.advertise<geometry_msgs::Pose>("/pose", 1);

      detection_sub_ = nh_.subscribe("/human_detection/detection", 1, &HumanDetectionNodeTest::onDetection, this);
      map_sub_ = nh_.subscribe("/human_detection/map_cloud", 1, &HumanDetectionNodeTest::onMap, this);
      cloud_sub_ = nh_.subscribe("/human_detection/filtered_cloud", 1, &HumanDetectionNodeTest::onCloud, this);
      //pose_array_sub_ = nh_.subscribe("/human_detection/poses", 1, &HumanDetectionNodeTest::onPoses, this);
      //cluster_array_sub_ = nh_.subscribe("/human_detection/clusters", 1, &HumanDetectionNodeTest::onClusters, this);
    }

    virtual void TearDown() 
    {
    }

    ros::NodeHandle nh_;
    ros::Publisher pc_pub_, pose_pub_;
    ros::Subscriber detection_sub_, map_sub_, cloud_sub_, pose_array_sub_, cluster_array_sub_;

    std::stringstream ss;

  HumanDetectionNodeTest() : cloud(new pcl::PointCloud<pcl::PointXYZI>)
  {
    detection_received = false;
    map_received = false;
    cloud_received = false;
    //poses_received = false;
    //clusters_received = false;
    //poses_size = 0;
    //clusters_size = 0;
    
    ss << "/home/user/movel/mov/src/movel_perception/people_detection/input/cloud.pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (ss.str(), *cloud) == -1) //* load the file
    {
      ROS_ERROR("Couldn't read file test_pcd.pcd");
    }
  }

  public:
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    bool detection_received, map_received, cloud_received, poses_received, clusters_received;
    size_t poses_size, clusters_size;

    void onDetection(const std_msgs::Float64::ConstPtr& msg)
    {
      detection_received = true;
    }

    void onMap(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
      map_received = true;
    }

    void onCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
      cloud_received = true;
    }

    /*void onPoses(const geometry_msgs::PoseArray::ConstPtr& msg)
    {
      poses_received = true;
      poses_size = msg->poses.size();
    }

    void onClusters(const human_detection::ClusterArray::ConstPtr& msg)
    {
      clusters_received = true;
      clusters_size = msg->clusters.size();
    }*/
};

TEST_F(HumanDetectionNodeTest, testTopics)
{
  ros::Rate loop_rate(0.1);

  ros::Duration(1.0).sleep();
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose_pub_.publish(pose);

  ros::Duration(1.0).sleep();
  EXPECT_EQ(map_received, true);

  sensor_msgs::PointCloud2 cloud_input;
  pcl::toROSMsg(*cloud, cloud_input);
  cloud_input.header.frame_id = "map";
  pc_pub_.publish(cloud_input);

  ros::Duration(1.0).sleep();
  EXPECT_EQ(cloud_received, true);
  EXPECT_EQ(detection_received, 0);
  //EXPECT_EQ(poses_received, true);
  //EXPECT_EQ(clusters_received, true);
  //EXPECT_EQ(poses_size, 1);
  //EXPECT_EQ(clusters_size, 1);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "human_detection_node_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
