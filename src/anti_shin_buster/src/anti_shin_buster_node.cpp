#include <anti_shin_buster/anti_shin_buster_node.hpp>

AntiShinBusterNode::AntiShinBusterNode()
{
  ros::Subscriber cloudSub = nh_.subscribe("camera/depth/color/points", 1, &AntiShinBusterNode::pointcloudCb, this);
  obstacleCloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>("obstacles_cloud", 1);
  surfNormalPub_ = nh_.advertise<sensor_msgs::PointCloud2>("normals_cloud", 1);
  ros::param::param<float>("~min_z", min_z_, 0.05);
  ros::param::param<float>("~max_z", max_z_, 10.0);
  ros::param::param<float>("~r_search", r_search_, 0.05);
  ros::param::param<float>("~leaf_size", leaf_size_, 0.025);
  ros::param::param<int>("~downsample", downsample_, 4);

  bool status;
  ROS_INFO("wait for camera_link to base_link");
  status = tfEar_.waitForTransform("base_link", "camera_link", ros::Time(0.0), ros::Duration(10.0));
  ROS_INFO("status %d", status);

  ROS_INFO("wait for camera_depth_optical_frame to base_link");
  status = tfEar_.waitForTransform("base_link", "camera_depth_optical_frame", ros::Time(0.0), ros::Duration(10.0));
  ROS_INFO("status %d", status);
  ROS_INFO("Anti Shin Buster ready!");

  ros::spin();
}

void AntiShinBusterNode::pointcloudCb(const sensor_msgs::PointCloud2ConstPtr cloud)
{
  // ROS_INFO("new cloud! w: %lu, h: %lu, N: %lu", cloud->height, cloud->width, cloud->data.size());

  // tranform to base_link frame
  sensor_msgs::PointCloud2 obstacle_cloud;

  // pcl_ros::transformPointCloud("base_link", *cloud, obstacle_cloud, tfEar_);
  tf::StampedTransform transform;
  bool status;
  status = tfEar_.waitForTransform("base_link", cloud->header.frame_id, ros::Time(0.0), ros::Duration(10.0));
  tfEar_.lookupTransform("base_link", cloud->header.frame_id, ros::Time(0.0), transform);
  pcl_ros::transformPointCloud("base_link", transform, *cloud, obstacle_cloud);

  // ROS_INFO("obstacle cloud! w: %lu, h: %lu, N: %lu", obstacle_cloud.height, obstacle_cloud.width,
  // obstacle_cloud.data.size());

  Cloud::Ptr pcl_obs_cloud(new Cloud);
  pcl::fromROSMsg(obstacle_cloud, *pcl_obs_cloud);

  // ROS_INFO("Conversion good. Pre filter size %lu", pcl_obs_cloud->points.size());
  Cloud::Ptr pcl_ds_cloud(new Cloud());
  for (int i = 0; i < pcl_obs_cloud->points.size(); i += downsample_)
  {
    pcl_ds_cloud->points.push_back(pcl_obs_cloud->points[i]);
  }
  pcl_obs_cloud->points = pcl_ds_cloud->points;
  // ROS_INFO("post filter size %lu", pcl_obs_cloud->points.size());

  // filter floor points and out of range points
  pcl::PassThrough<Pt> passthrough;
  passthrough.setInputCloud(pcl_obs_cloud);
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(min_z_, max_z_);
  passthrough.filter(*pcl_obs_cloud);
  // ROS_INFO("filtering good");

  // reduce point cloud density
  // pcl::VoxelGrid<Pt> ve;
  // ve.setInputCloud(pcl_obs_cloud);
  // ve.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  // ve.setMinimumPointsNumberPerVoxel(10);
  // ve.filter(*pcl_obs_cloud);

  // force unorganised pointcloud
  pcl_obs_cloud->width = pcl_obs_cloud->points.size();
  pcl_obs_cloud->height = 1;
  // ROS_INFO("cloud to publish, w: %lu, h: %lu, N: %lu", pcl_obs_cloud->width, pcl_obs_cloud->height,
  // pcl_obs_cloud->points.size());
  // publish obstacle cloud
  pcl::toROSMsg(*pcl_obs_cloud, obstacle_cloud);
  obstacleCloudPub_.publish(obstacle_cloud);
  // ROS_INFO("publishing good");
}

void AntiShinBusterNode::calcCloudNormal(Cloud::Ptr& in_cloud, CloudNormal::Ptr& cloud_normal)
{
  pcl::NormalEstimation<Pt, pcl::Normal> ne;
  pcl::search::KdTree<Pt>::Ptr kdtree(new pcl::search::KdTree<Pt>());

  ne.setInputCloud(in_cloud);
  // ROS_INFO("input good");
  ne.setSearchMethod(kdtree);
  // ROS_INFO("search tree good");
  ne.setRadiusSearch(r_search_);
  // ROS_INFO("search radius good");

  ne.compute(*cloud_normal);
  // ROS_INFO("normal calc good");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "anti_shin_buster");
  AntiShinBusterNode ASBN;

  return 0;
}
