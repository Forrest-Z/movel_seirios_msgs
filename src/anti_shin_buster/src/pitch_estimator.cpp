#include <anti_shin_buster/pitch_estimator.hpp>

PitchEstimator::PitchEstimator() : nh_("~"), N_support_(100), leaf_size_(0.10), r_support_(0.50)
{
  ROS_INFO("Begin pitch estimator");

  // setup parameters
  if (nh_.hasParam("N_support"))
    nh_.getParam("N_support", N_support_);
  if (nh_.hasParam("leaf_size"))
    nh_.getParam("leaf_size", leaf_size_);
  if (nh_.hasParam("r_support"))
    nh_.getParam("r_support", r_support_);

  ROS_INFO("voxel grid leaf size %5.2f", leaf_size_);
  ROS_INFO("normal calculation support radius %5.2f", r_support_);

  // setup topics
  ros::Subscriber cloud_sub = nh_.subscribe("/camera/depth/color/points", 1, &PitchEstimator::cloudCb, this);
  support_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/support_cloud", 1);
  normal_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/central_normal", 1);
  sparse_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sparse_cloud", 1);

  ros::spin();
}

PitchEstimator::~PitchEstimator()
{
}

float PitchEstimator::xyDistance(float x, float y, Pt& pt)
{
  float dx, dy;
  dx = x - pt.x;
  dy = y - pt.y;
  return sqrt(dx * dx + dy * dy);
}

size_t PitchEstimator::getMinXYDistance(float x, float y, Cloud::Ptr& cloud)
{
  Pt pt_i = cloud->points[0];
  size_t min_idx = 0;
  float min_d = xyDistance(x, y, pt_i);

  for (size_t i = 1; i < cloud->points.size(); i++)
  {
    pt_i = cloud->points[i];
    float d_i = xyDistance(x, y, pt_i);

    if (d_i < min_d)
    {
      min_idx = i;
      min_d = d_i;
    }
  }

  return min_idx;
}

void PitchEstimator::cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  // ROS_INFO("new cloud, of size (%d, %d)", cloud->width, cloud->height);

  Cloud::Ptr pcl_cloud(new Cloud);
  pcl::fromROSMsg(*cloud, *pcl_cloud);

  // filter cloud, voxel grid
  Cloud::Ptr pcl_filtered_cloud(new Cloud);
  pcl::VoxelGrid<Pt> vg;
  vg.setInputCloud(pcl_cloud);
  vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  vg.filter(*pcl_filtered_cloud);

  // publish sparse cloud (faster to visualise)
  sensor_msgs::PointCloud2 sparse_cloud;
  pcl::toROSMsg(*pcl_filtered_cloud, sparse_cloud);
  sparse_cloud.header.frame_id = cloud->header.frame_id;
  sparse_cloud.header.stamp = cloud->header.stamp;
  sparse_cloud_pub_.publish(sparse_cloud);

  // find index closest to centre
  size_t min_idx = getMinXYDistance(0.0, 0.0, pcl_filtered_cloud);
  // ROS_INFO("most central point in index %lu, (%f, %f)", min_idx,
  //          pcl_filtered_cloud->points[min_idx].x, pcl_filtered_cloud->points[min_idx].y);

  // get support cloud
  pcl::search::KdTree<Pt>::Ptr tree(new pcl::search::KdTree<Pt>());
  tree->setInputCloud(pcl_filtered_cloud);
  std::vector<int> indices;
  std::vector<float> sqr_distances;

  int N_neighbours = tree->radiusSearch(min_idx, r_support_, indices, sqr_distances, N_support_);
  // ROS_INFO("there are %d points in the support area", N_neighbours);

  // calculate normal
  pcl::NormalEstimation<Pt, Nm> ne;
  float nx, ny, nz, cc;
  ne.computePointNormal(*pcl_filtered_cloud, indices, nx, ny, nz, cc);

  // print, visualise result
  float th_roll = acos(nz);
  float th_pitch_base = th_roll - 0.5 * M_PI;
  float cam_height = sin(th_pitch_base) * pcl_filtered_cloud->points[min_idx].z;
  ROS_INFO("cam roll %5.2f, base pitch is %5.2f, height is %5.2f", 180.0 * th_roll / M_PI, 180.0 * th_pitch_base / M_PI,
           cam_height);
  ROS_INFO("tf to base %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f", 0.0, 0.0, cam_height, 0.0, sin(0.5 * th_pitch_base),
           0.0, cos(0.5 * th_pitch_base));
  geometry_msgs::PoseStamped normal_pose;
  normal_pose.header.stamp = cloud->header.stamp;
  normal_pose.header.frame_id = cloud->header.frame_id;
  normal_pose.pose.position.x = pcl_filtered_cloud->points[min_idx].x;
  normal_pose.pose.position.y = pcl_filtered_cloud->points[min_idx].y;
  normal_pose.pose.position.z = pcl_filtered_cloud->points[min_idx].z;

  normal_pose.pose.orientation.x = sin(th_roll / 2.0);
  normal_pose.pose.orientation.y = 0.0;
  normal_pose.pose.orientation.z = 0.0;
  normal_pose.pose.orientation.w = cos(th_roll / 2.0);
  normal_pose_pub_.publish(normal_pose);
  // ROS_INFO("normal published");

  Cloud::Ptr pcl_support_cloud(new Cloud());
  boost::shared_ptr<std::vector<int> > indices_ptr = boost::make_shared<std::vector<int> >(indices);

  pcl::ExtractIndices<Pt> ei;
  ei.setInputCloud(pcl_filtered_cloud);
  ei.setIndices(indices_ptr);
  ei.setNegative(false);
  ei.filter(*pcl_support_cloud);

  sensor_msgs::PointCloud2 support_cloud;
  pcl::toROSMsg(*pcl_support_cloud, support_cloud);
  support_cloud.header.frame_id = cloud->header.frame_id;
  support_cloud.header.stamp = cloud->header.stamp;
  support_cloud_pub_.publish(support_cloud);
}