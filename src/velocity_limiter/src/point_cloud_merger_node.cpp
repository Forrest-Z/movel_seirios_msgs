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
  // projector_.transformLaserScanToPointCloud(p_target_frame_, *scan_in, cloud_from_scan, tf_listener_,
  //                                           laser_geometry::channel_option::Default || 
  //                                           laser_geometry::channel_option::Distance);
  // projector_.transformLaserScanToPointCloud(p_target_frame_, *scan_in, cloud_from_scan, tf_listener_,
  //                                           laser_geometry::channel_option::Distance || laser_geometry::channel_option::Intensity);
  projector_.transformLaserScanToPointCloud(p_target_frame_, *scan_in, cloud_from_scan, tf_listener_,
                                            -1.0, laser_geometry::channel_option::None);

  std::cout << "cloud from scan has " << cloud_from_scan.fields.size() << " fields, field size: " << cloud_from_scan.data.size() << std::endl;
  for (size_t i = 0; i < cloud_from_scan.fields.size(); i++)
  {
    std::cout << "field " << i << ": " << cloud_from_scan.fields[i].name << std::endl;
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

  std::cout << "cloud_base has " << cloud_base.fields.size() << " fields, data size: " << cloud_base.data.size() << std::endl;
  for (size_t i = 0; i < cloud_base.fields.size(); i++)
  {
    std::cout << "field " << i << ": " << cloud_base.fields[i].name << std::endl;
  }

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*cloud_in, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_in(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc2, *pcl_in);

  // strip RGB
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in_stripped(new pcl::PointCloud<pcl::PointXYZ>);
  pcl_in_stripped->points.resize(pcl_in->size());
  for (size_t i = 0; i < pcl_in->points.size(); i++)
  {
    pcl_in_stripped->points[i].x = pcl_in->points[i].x;
    pcl_in_stripped->points[i].y = pcl_in->points[i].y;
    pcl_in_stripped->points[i].z = pcl_in->points[i].z;
  }

  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(*pcl_in_stripped, ros_cloud);
  ros_cloud.header = cloud_in->header;

  std::cout << "ros_cloud has " << ros_cloud.fields.size() << " fields, data size: " << ros_cloud.data.size() << std::endl;
  for (size_t i = 0; i < ros_cloud.fields.size(); i++)
  {
    std::cout << "field " << i << ": " << ros_cloud.fields[i].name << std::endl;
  }

  // merge cloud from scan and transformed cloud_in
  sensor_msgs::PointCloud2 merged;
  // pcl::concatenatePointCloud(cloud_from_scan, cloud_base, merged);
  // pcl::concatenatePointCloud(merged, cloud_base, merged);
  // pcl::concatenatePointCloud(cloud_from_scan, ros_cloud, merged);
  pcl::concatenatePointCloud(ros_cloud, ros_cloud, merged);
  cloud_merged_ = merged;

  merged_point_cloud_pub_.publish(merged);
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