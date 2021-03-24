#include <point_cloud_merger/point_cloud_merger_node.h>
#include <movel_hasp_vendor/license.h>

PointCloudMergerNode::PointCloudMergerNode()
  : nh_private_("~")
{
  ros::Time::waitForValid();

  if (!loadParams())
  {
    ROS_FATAL("Error during parameter loading. Shutting down.");
    return;
  }
  ROS_INFO("All parameters loaded. Launching.");

  setupTopics();

  ros::Rate r(20.0);
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
}

PointCloudMergerNode::~PointCloudMergerNode()
{
}

bool PointCloudMergerNode::loadParams()
{
  ros_utils::ParamLoader loader(nh_private_);
  loader.get_required("target_frame", p_target_frame_);

  return loader.params_valid();
}

void PointCloudMergerNode::setupTopics()
{
  laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "/scan", 1);
  point_cloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/cloud", 1);
  sensors_sync_ = new message_filters::Synchronizer<sync_pol>(sync_pol(10), *laser_scan_sub_, *point_cloud_sub_);
  sensors_sync_->registerCallback(boost::bind(&PointCloudMergerNode::onNewData, this, _1, _2));

  merged_point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud/merged", 1);
}

void PointCloudMergerNode::onNewData(const sensor_msgs::LaserScan::ConstPtr& scan_in,
                                     const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
{
  // transform incoming LaserScan data to PointCloud2
  if(!tf_listener_.waitForTransform(
        scan_in->header.frame_id,
        p_target_frame_,
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0))) {
    ROS_WARN_STREAM("Transform from origin frame " << scan_in->header.frame_id
                    << " to target frame " << p_target_frame_ << " not found");
    return;
  }
  sensor_msgs::PointCloud2 cloud_from_scan;
  projector_.transformLaserScanToPointCloud(p_target_frame_, *scan_in, cloud_from_scan, tf_listener_,
                                            -1.0, laser_geometry::channel_option::None);

  std::cout << "cloud_from_scan frame id " << cloud_from_scan.header.frame_id << " size " << cloud_from_scan.data.size() << " pt step " << cloud_from_scan.point_step << std::endl;
  if (cloud_from_scan.is_bigendian)
    std::cout << "bigendian true" << std::endl;
  if (!cloud_from_scan.is_bigendian)
    std::cout << "bigendian false" << std::endl;
  for (size_t i = 0; i < cloud_from_scan.point_step; i++)
  {
    if (i == 0)
      std::cout << "x part " << (int)cloud_from_scan.data[i] << " ";
    if (i > 0 && i < 4)
      std::cout << (int)cloud_from_scan.data[i] << " ";
    if (i == 4)
      std::cout << "\ny part " << (int)cloud_from_scan.data[i] << " ";
    if (i > 4 && i < 8)
      std::cout << (int)cloud_from_scan.data[i] << " ";
    if (i == 8)
      std::cout << "\nz part " << (int)cloud_from_scan.data[i] << " ";
    if (i > 8)
      std::cout << (int)cloud_from_scan.data[i] << " ";
  }
  std::cout << std::endl;
  for (size_t i = 0; i < cloud_from_scan.fields.size(); i++)
  {
    std::cout << "field " << i << ": " << cloud_from_scan.fields[i].name << " offset " << cloud_from_scan.fields[i].offset << std::endl;
  }

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

  std::cout << "cloud_base frame id " << cloud_base.header.frame_id << " size " << cloud_base.data.size() << " pt step " << cloud_base.point_step << std::endl;
  if (cloud_base.is_bigendian)
    std::cout << "bigendian true" << std::endl;
  if (!cloud_base.is_bigendian)
    std::cout << "bigendian false" << std::endl;
  for (size_t i = 0; i < cloud_base.fields[3].offset; i++)
  {
    if (i == 0)
      std::cout << "x part " << (int)cloud_base.data[i] << " ";
    if (i > 0 && i < 4)
      std::cout << (int)cloud_base.data[i] << " ";
    if (i == 4)
      std::cout << "\ny part " << (int)cloud_base.data[i] << " ";
    if (i > 4 && i < 8)
      std::cout << (int)cloud_base.data[i] << " ";
    if (i == 8)
      std::cout << "\nz part " << (int)cloud_base.data[i] << " ";
    if (i > 8)
      std::cout << (int)cloud_base.data[i] << " ";
  }
  std::cout << std::endl;
  for (size_t i = 0; i < cloud_base.fields.size(); i++)
  {
    std::cout << "field " << i << ": " << cloud_base.fields[i].name << " offset " << cloud_base.fields[i].offset << std::endl;
  }

  /* sensor_msgs::PointCloud2 cloud_xyz;
  stripUnimportantFields(cloud_base, cloud_xyz);
  std::cout << "cloud_xyz frame id " << cloud_xyz.header.frame_id << " size " << cloud_xyz.data.size() << " pt step " << cloud_xyz.point_step << std::endl;
  if (cloud_xyz.is_bigendian)
    std::cout << "bigendian true" << std::endl;
  if (!cloud_xyz.is_bigendian)
    std::cout << "bigendian false" << std::endl;
  for (size_t i = 0; i < cloud_xyz.point_step; i++)
  {
    if (i == 0)
      std::cout << "x part " << (int)cloud_xyz.data[i] << " ";
    if (i > 0 && i < 4)
      std::cout << (int)cloud_xyz.data[i] << " ";
    if (i == 4)
      std::cout << "\ny part " << (int)cloud_xyz.data[i] << " ";
    if (i > 4 && i < 8)
      std::cout << (int)cloud_xyz.data[i] << " ";
    if (i == 8)
      std::cout << "\nz part " << (int)cloud_xyz.data[i] << " ";
    if (i > 8)
      std::cout << (int)cloud_xyz.data[i] << " ";
  }
  std::cout << std::endl;
  for (size_t i = 0; i < cloud_xyz.fields.size(); i++)
  {
    std::cout << "field " << i << ": " << cloud_xyz.fields[i].name << " offset " << cloud_xyz.fields[i].offset << std::endl;
  } */

/*   pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*cloud_in, pcl_pc2);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_in(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2, *pcl_in);

  // strip RGB
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in_stripped(new pcl::PointCloud<pcl::PointXYZ>);
  pcl_in_stripped->points.resize(pcl_in->points.size());
  for (size_t i = 0; i < pcl_in->points.size(); i++)
  {
    pcl_in_stripped->points[i].x = pcl_in->points[i].x;
    pcl_in_stripped->points[i].y = pcl_in->points[i].y;
    pcl_in_stripped->points[i].z = pcl_in->points[i].z;
  }

  sensor_msgs::PointCloud2 cloud_xyz;
  pcl::toROSMsg(*pcl_in_stripped, cloud_xyz);
  cloud_xyz.header = cloud_base.header;

  // std::cout << "cloud_xyz has " << cloud_xyz.fields.size() << " fields " << cloud_xyz.data.size() << " " << cloud_xyz.point_step << std::endl;
  std::cout << "cloud_xyz frame id " << cloud_xyz.header.frame_id << " size " << cloud_xyz.data.size() << " pt step " << cloud_xyz.point_step << std::endl;
  for (size_t i = 0; i < cloud_xyz.fields.size(); i++)
  {
    std::cout << "field " << i << ": " << cloud_xyz.fields[i].name << " offset " << cloud_xyz.fields[i].offset << std::endl;
  } */

  // merge cloud from scan and transformed cloud_in
  sensor_msgs::PointCloud2 merged;
  // Data size (4918968 bytes) does not match width (307514) times height (1) times point_step (12).  Dropping message.
  // pcl::concatenatePointCloud(cloud_from_scan, cloud_xyz, merged);
  pcl::concatenatePointCloud(merged, cloud_base, merged);
  cloud_merged_ = merged;

  // std::cout << "merged has " << merged.fields.size() << " fields " << merged.data.size() << " " << merged.point_step << std::endl;
  std::cout << "merged frame id " << merged.header.frame_id << " size " << merged.data.size() << " pt step " << merged.point_step << std::endl;
  for (size_t i = 0; i < merged.fields.size(); i++)
  {
    std::cout << "field " << i << ": " << merged.fields[i].name << " offset " << merged.fields[i].offset << std::endl;
  }

  merged_point_cloud_pub_.publish(merged);
}

void PointCloudMergerNode::stripUnimportantFields(const sensor_msgs::PointCloud2& cloud_in, sensor_msgs::PointCloud2& cloud_out)
{
  uint32_t point_step = cloud_in.point_step;
  uint32_t row_step = cloud_in.row_step;

  cloud_out.header = cloud_in.header;

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
  cloud_out.fields[1].offset = 0;
  cloud_out.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_out.fields[1].count = 1;

  cloud_out.fields[2].name = "z";
  cloud_out.fields[2].offset = 0;
  cloud_out.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_out.fields[2].count = 1;

  cloud_out.data.resize(cloud_out.height * cloud_out.point_step);
  int data_out_idx = 0;
  for (size_t i = 0; i < cloud_in.height * cloud_in.width * cloud_in.point_step; i += cloud_in.point_step)
  {
    for (size_t j = 0; j < cloud_out.point_step; j++)
    {
      cloud_out.data[data_out_idx] = cloud_in.data[i+j];
      data_out_idx += 1;
    }
  }
}

int main(int argc, char** argv)
{
  #ifdef MOVEL_LICENSE
    MovelLicense ml(8);
    if (!ml.login())
    return 1;
  #endif

  ros::init(argc, argv, "point_cloud_merger");
  PointCloudMergerNode point_cloud_merger;
  
  #ifdef MOVEL_LICENSE
    ml.logout();
  #endif

  return(0);
}