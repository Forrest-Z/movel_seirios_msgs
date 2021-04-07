#include <point_cloud_merger/point_cloud_merger.h>

PointCloudMerger::PointCloudMerger(int mode)
  : nh_private_("~")
{
  std::string node_name;
  if (mode == 0)
    node_name = "scan_cloud_merger";
  else if (mode == 1)
    node_name = "point_cloud_merger";

  ros::Time::waitForValid();

  if (!loadParams())
  {
    ROS_FATAL("[%s] Error during parameter loading. Shutting down.", node_name.c_str());
    return;
  }
  ROS_INFO("[%s] All parameters loaded. Launching.", node_name.c_str());

  setupTopics(mode);

  ROS_INFO("[%s] Point Cloud Merger ready.", node_name.c_str());

  ros::Rate r(20.0);
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
}

PointCloudMerger::~PointCloudMerger()
{
}

bool PointCloudMerger::loadParams()
{
  ros_utils::ParamLoader loader(nh_private_);
  loader.get_required("target_frame", p_target_frame_);

  return loader.params_valid();
}

void PointCloudMerger::setupTopics(int mode)
{
  if (mode == 0)
  {
    laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "/scan", 1);
    point_cloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/cloud", 1);
    sensors_sync_ = new message_filters::Synchronizer<sync_pol>(sync_pol(10), *laser_scan_sub_, *point_cloud_sub_);
    sensors_sync_->registerCallback(boost::bind(&PointCloudMerger::onNewData, this, _1, _2));
  }

  else if (mode == 1)
  {
    point_cloud_1_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/cloud_1", 1);
    point_cloud_2_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/cloud_2", 1);
    pc_sync_ = new message_filters::Synchronizer<sync_pc2>(sync_pc2(10), *point_cloud_1_sub_, *point_cloud_2_sub_);
    pc_sync_->registerCallback(boost::bind(&PointCloudMerger::onNewClouds, this, _1, _2));
  }

  merged_point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud/merged", 1);
}

void PointCloudMerger::onNewData(const sensor_msgs::LaserScan::ConstPtr& scan_in,
                                     const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
{
  // transform incoming LaserScan data to PointCloud2
  if(!tf_listener_.waitForTransform(
        scan_in->header.frame_id,
        p_target_frame_,
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(3.0))) {
    ROS_WARN_STREAM("Transform from origin frame " << scan_in->header.frame_id
                    << " to target frame " << p_target_frame_ << " not found");
    return;
  }
  sensor_msgs::PointCloud2 cloud_from_scan;
  projector_.transformLaserScanToPointCloud(p_target_frame_, *scan_in, cloud_from_scan, tf_listener_,
                                            -1.0, laser_geometry::channel_option::None);

  // transform incoming PointCloud2 frame to target frame
  sensor_msgs::PointCloud2 cloud_base;
  try
  {
    pcl_ros::transformPointCloud(p_target_frame_, *cloud_in, cloud_base, tf_listener_);
  }
  catch(tf::TransformException e)
  {
    ROS_ERROR("%s", e.what());
  }

  sensor_msgs::PointCloud2 cloud_xyz;
  stripUnimportantFields(cloud_base, cloud_xyz);

  // merge cloud from scan and transformed cloud_in
  sensor_msgs::PointCloud2 merged;
  pcl::concatenatePointCloud(cloud_from_scan, cloud_xyz, merged);
  cloud_merged_ = merged;

  merged_point_cloud_pub_.publish(merged);
}

void PointCloudMerger::onNewClouds(const sensor_msgs::PointCloud2::ConstPtr& cloud1,
                                     const sensor_msgs::PointCloud2::ConstPtr& cloud2)
{
  // transform incoming PointCloud2 frame to target frame
  sensor_msgs::PointCloud2 cloud_base1, cloud_base2;
  try
  {
    pcl_ros::transformPointCloud(p_target_frame_, *cloud1, cloud_base1, tf_listener_);
    pcl_ros::transformPointCloud(p_target_frame_, *cloud2, cloud_base2, tf_listener_);
  }
  catch(tf::TransformException e)
  {
    ROS_ERROR("%s", e.what());
  }

  sensor_msgs::PointCloud2 cloud_xyz1, cloud_xyz2;
  stripUnimportantFields(cloud_base1, cloud_xyz1);
  stripUnimportantFields(cloud_base2, cloud_xyz2);

  // merge cloud from both point cloud sources
  sensor_msgs::PointCloud2 merged;
  pcl::concatenatePointCloud(cloud_xyz1, cloud_xyz2, merged);
  cloud_merged_ = merged;

  merged_point_cloud_pub_.publish(merged);
}

void PointCloudMerger::stripUnimportantFields(const sensor_msgs::PointCloud2& cloud_in, sensor_msgs::PointCloud2& cloud_out)
{
  uint32_t point_step = cloud_in.point_step;
  uint32_t row_step = cloud_in.row_step;

  cloud_out.header = cloud_in.header;

  // currently always assume that data_in is little endian
  cloud_out.is_bigendian = false;
  cloud_out.is_dense = cloud_in.is_dense;
  
  cloud_out.height = 1;
  cloud_out.width = cloud_in.height * cloud_in.width;

  cloud_out.point_step = 12;
  cloud_out.row_step = cloud_out.width * cloud_out.point_step;

  cloud_out.fields.resize(3);

  cloud_out.fields[0].name = "x";
  cloud_out.fields[0].offset = 0;
  cloud_out.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_out.fields[0].count = 1;

  cloud_out.fields[1].name = "y";
  cloud_out.fields[1].offset = 4;
  cloud_out.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_out.fields[1].count = 1;

  cloud_out.fields[2].name = "z";
  cloud_out.fields[2].offset = 8;
  cloud_out.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_out.fields[2].count = 1;

  for (size_t i = 0; i < cloud_in.width * cloud_in.height; i++)
  {
    for (size_t j = 0; j < cloud_out.point_step; j++)
    {
      int current_idx = (i * cloud_in.point_step) + j;
      cloud_out.data.push_back(cloud_in.data[current_idx]);
    }
  }
}
