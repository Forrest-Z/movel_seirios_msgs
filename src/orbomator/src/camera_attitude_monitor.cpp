#include <orbomator/camera_attitude_monitor.hpp>

CameraAttitudeMonitor::CameraAttitudeMonitor()
: have_dep_info_(false)
, use_pointcloud_(false)
{
  setupParams();
  setupTopics();
}

bool CameraAttitudeMonitor::setupTopics()
{
  if (use_pointcloud_)
  {
    dep_sub_ = nh_.subscribe("/camera/depth/color/points", 1, 
                             &CameraAttitudeMonitor::cloudCb, this);
  }
  else
  {
    dep_sub_ = nh_.subscribe("/camera/aligned_depth_to_color/image_raw", 1,
                             &CameraAttitudeMonitor::depthCb, this);
    dep_info_sub_ = nh_.subscribe("/camera/aligned_depth_to_color/camera_info", 1,
                                  &CameraAttitudeMonitor::depthInfoCb, this);
  }

  dep_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("depth_cloud", 1);
  seg_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("plane_segment_cloud", 1);
  
  return true;
}

bool CameraAttitudeMonitor::setupParams()
{
  ros::NodeHandle nh_local("~");
  min_dep_ = 0.001;
  if (nh_local.hasParam("min_dep"))
    nh_local.getParam("min_dep", min_dep_);

  max_dep_ = 1000.0;
  if (nh_local.hasParam("max_dep"))
    nh_local.getParam("max_dep", max_dep_);

  seg_distance_ = 0.01;
  if (nh_local.hasParam("seg_distance"))
    nh_local.getParam("seg_distance", seg_distance_);

  ransac_iter_ = 10;
  if (nh_local.hasParam("ransac_iter"))
    nh_local.getParam("ransac_iter", ransac_iter_);

  max_plane_count_ = 1;
  if (nh_local.hasParam("max_plane_count"))
    nh_local.getParam("max_plane_count", max_plane_count_);

  cam_frame_ = "orb_camera_link";
  if (nh_local.hasParam("camera_frame"))
    nh_local.getParam("camera_frame", cam_frame_);

  level_cam_frame_ = "level_orb_camera_link";
  if (nh_local.hasParam("level_camera_frame"))
    nh_local.getParam("level_camera_frame", level_cam_frame_);

  Mat R_opt_to_cam_tmp = (cv::Mat_<double>(3, 3) << 0.0,  0.0, 1.0,
                                                   -1.0,  0.0, 0.0,
                                                    0.0, -1.0, 0.0);
  R_opt_to_cam_ = R_opt_to_cam_tmp.clone();
  
  return true;
}

void CameraAttitudeMonitor::depthCb(const sensor_msgs::ImageConstPtr msg)
{
  // ROS_INFO("new depth image");
  // convert to CV
  cv_bridge::CvImagePtr dep_ptr;
  dep_ptr = cv_bridge::toCvCopy(msg);
  dep_ptr->image.convertTo(dep_ptr->image, CV_64F);

  // calculate point cloud
  int min_col, max_col;
  // min_col = dep_ptr->image.cols/3;
  // max_col = dep_ptr->image.cols*2/3;
  min_col = 0;
  max_col = dep_ptr->image.cols;
  // ROS_INFO("%d column range %d, %d", dep_ptr->image.cols, min_col, max_col);
  
  int min_row, max_row;
  min_row = dep_ptr->image.rows/2;
  max_row = dep_ptr->image.rows;
  // min_row = 0;
  // max_row = dep_ptr->image.rows;
  // ROS_INFO("%d row range %d, %d", dep_ptr->image.rows, min_row, max_row);

  Cloud::Ptr depth_cloud (new Cloud);
  for (int i = min_row; i < max_row; i++)
  {
    for (int j = min_col; j < max_col; j++)
    {
      Pt pt_i;
      double z = dep_ptr->image.at<double>(i, j)/1000.0;
      if (std::isnan(z) || z < min_dep_ || z > max_dep_)
      {
        // ROS_INFO("z is %5.2f, sensible?", z);
        continue;
      }

      Mat uvw = (cv::Mat_<double>(3, 1) << j, i, 1.0);
      Mat xyz = dep_proj_.inv() * uvw;
      xyz = z * xyz;
      xyz = R_opt_to_cam_ * xyz;

      pt_i.x = xyz.at<double>(0, 0);
      pt_i.y = xyz.at<double>(1, 0);
      pt_i.z = xyz.at<double>(2, 0);
      depth_cloud->points.push_back(pt_i);
    }
  }
  // ROS_INFO("cloud calca OK, %lu points", depth_cloud->points.size());

  // publish point cloud
  sensor_msgs::PointCloud2 depth_cloud_msg;
  pcl::toROSMsg(*depth_cloud, depth_cloud_msg);
  depth_cloud_msg.header.stamp = msg->header.stamp;
  depth_cloud_msg.header.frame_id = "camera_link";
  dep_cloud_pub_.publish(depth_cloud_msg);
  // ROS_INFO("publish OK");

  // find floor
  tf2::Quaternion quat;
  if (findFloorNormal(depth_cloud, quat))
  {
    // construct transform
    tf2::Transform tf2_cam_to_level;
    tf2_cam_to_level.setRotation(quat);
    tf2::Vector3 origin;
    origin.setZero();
    tf2_cam_to_level.setOrigin(origin);

    // convert to msg
    geometry_msgs::TransformStamped tf_cam_to_level;
    tf_cam_to_level.header.frame_id = cam_frame_;
    tf_cam_to_level.header.stamp = msg->header.stamp;
    tf_cam_to_level.child_frame_id = level_cam_frame_;

    tf_cam_to_level.transform = tf2::toMsg(tf2_cam_to_level);

    // send transform
    tf_mouth_.sendTransform(tf_cam_to_level);
  }
}

void CameraAttitudeMonitor::depthInfoCb(const sensor_msgs::CameraInfoConstPtr msg)
{
  if (!have_dep_info_)
  {
    Mat dep_proj_tmp;
    dep_proj_tmp = cv::Mat(3, 3, CV_64F, (void *) msg->K.data());
    dep_proj_ = dep_proj_tmp.clone();
    have_dep_info_ = true;
  }
}

void CameraAttitudeMonitor::cloudCb(const sensor_msgs::PointCloud2ConstPtr msg)
{
  // pre-treat pointcloud

  // find floor
}

bool CameraAttitudeMonitor::findFloorNormal(Cloud::Ptr cloud_ptr, tf2::Quaternion &quat)
{
  // ROS_INFO("where is floor?");
  Cloud::Ptr in_cloud (new Cloud);
  pcl::copyPointCloud(*cloud_ptr, *in_cloud);
  // ROS_INFO("copy OK");

  pcl::SACSegmentation<Pt> segmentor;
  segmentor.setOptimizeCoefficients(true);
  segmentor.setModelType(pcl::SACMODEL_PLANE);
  segmentor.setMethodType(pcl::SAC_RANSAC);
  segmentor.setMaxIterations(32);
  segmentor.setDistanceThreshold(seg_distance_);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  vector<double> normal_candidate(3);
  bool have_valid_normal = false;

  pcl::ExtractIndices<Pt> extractor;

  CloudI::Ptr out_cloud (new CloudI);

  int plane_idx = 0;

  int original_size = in_cloud->points.size();
  // ROS_INFO("setup OK, %d points", original_size);
  while (in_cloud->points.size() > 0.25*original_size)
  {
    // find biggest plane still in cloud
    segmentor.setInputCloud(in_cloud);
    segmentor.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
      // ROS_INFO("no more planes");
      break;
    }

    // evaluate coefficients
    double x, y, z, r;
    x = coefficients->values[0];
    y = coefficients->values[1];
    z = coefficients->values[2];
    r = sqrt(x*x + y*y + z*z);

    // evaluate best normal by z-projection
    // 
    if (z/r > 0.5)
    {
      if (!have_valid_normal)
      {
        have_valid_normal = true;
        normal_candidate[0] = x/r;
        normal_candidate[1] = y/r;
        normal_candidate[2] = z/r;
      }
      else if (z/r > normal_candidate[2])
      {
        normal_candidate[0] = x/r;
        normal_candidate[1] = y/r;
        normal_candidate[2] = z/r;
      }
    }

    // include in annotated cloud
    for (int idx = 0; idx < inliers->indices.size(); idx++)
    {
      PtI pt_i;
      pt_i.x = in_cloud->points[inliers->indices[idx]].x;
      pt_i.y = in_cloud->points[inliers->indices[idx]].y;
      pt_i.z = in_cloud->points[inliers->indices[idx]].z;
      pt_i.intensity = plane_idx;
      out_cloud->points.push_back(pt_i);
    }
    
    // remove plane from input
    extractor.setInputCloud(in_cloud);
    extractor.setIndices(inliers);
    extractor.setNegative(true);
    Cloud cloud_outliers;
    extractor.filter(cloud_outliers);
    in_cloud->swap(cloud_outliers);

    // ROS_INFO("plane %d, %lu points", plane_idx, inliers->indices.size());

    plane_idx += 1;
    if (plane_idx == max_plane_count_)
      break;
  }

  sensor_msgs::PointCloud2 seg_cloud_msg;
  pcl::toROSMsg(*out_cloud, seg_cloud_msg);
  seg_cloud_msg.header.stamp = ros::Time::now();
  seg_cloud_msg.header.frame_id = "camera_link";
  seg_cloud_pub_.publish(seg_cloud_msg);

  // calculate roll and pitch from best normal
  if (have_valid_normal)
  {
    double th_roll = atan2(normal_candidate[1], normal_candidate[2]);
    double a, b;
    a = normal_candidate[0];
    b = sin(th_roll)*normal_candidate[1] + cos(th_roll)*normal_candidate[2];
    double th_pitch = atan(-1.0 * a / b);

    // move to correct direction
    th_roll *= -1.0;
    th_pitch *= -1.0;

    // construct quaternion
    quat.setRPY(th_roll, th_pitch, 0.0);
    ROS_INFO("camera roll %6.4f, pitch %6.4f", th_roll, th_pitch);
    return true;
  }

  return false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_attitude_monitor");
  
  CameraAttitudeMonitor cam;
  ros::spin();

  return 0;
}
