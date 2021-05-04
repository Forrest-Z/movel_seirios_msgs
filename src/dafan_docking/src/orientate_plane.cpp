#include <dafan_docking/orientate_plane.hpp>

double quaternionToYaw(geometry_msgs::Quaternion q)
{
  ROS_INFO("Q: x(%f), y(%f), z(%f), w(%f)", q.x, q.y, q.z, q.w);
  double aa = 2.0*(q.w*q.z + q.y*q.x);
  double bb = 1.0 - 2.0 * (q.y*q.y + q.z*q.z);
  return atan2(aa, bb);
}

OrientatePlane::OrientatePlane() : tf_ear_(tf_buffer_)
{
  //subscribers
  cloudSub_ = nh_.subscribe("camera/depth/color/points", 1, &OrientatePlane::pointcloudCB, this);
  goalReceivedSub_ = nh_.subscribe("/goal2_received", 1, &OrientatePlane::goalReceivedCB , this);

  //publishers
  filteredCloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 1);
  clusterCloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cluster_cloud", 1, true);
  // debugCloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/debug_cloud", 1, true);
  dockingPlanePub_ = nh_.advertise<sensor_msgs::PointCloud2>("/docking_plane_cloud", 1);
  XAxisPub_ = nh_.advertise<geometry_msgs::PoseStamped>("/x_axis", 1);
  dockingPosePub_ = nh_.advertise<geometry_msgs::PoseStamped>("/pid_goal2", 1, true);

  //initialization
  loadParams();
  loadPipelineParams();

  //dynamic reconfigure server
  callbackType = boost::bind(&OrientatePlane::reconfigureCB, this, _1, _2);
  configServer.setCallback(callbackType);

  ros::spin();
}

void OrientatePlane::loadParams(){
  pub_docking_goal_ = true;
  seg_itr = 0;
  
  ros::param::param<bool>("~print_debug", print_debug_, false);

  //Goal offset
  ros::param::param<float>("~goal_offset_x", goal_offset_x_, 0.0);
  ros::param::param<float>("~goal_offset_y", goal_offset_y_, 0.0);
  //0: Passthrough
  ros::param::param<float>("~min_z", min_z_, -10.0);
  ros::param::param<float>("~max_z", max_z_, 10.0);
  ros::param::param<float>("~min_x", min_x_, 0.0);
  ros::param::param<float>("~max_x", max_x_, 20.0);
  //1: Voxel Filter
  ros::param::param<float>("~leaf_size", leaf_size_, 0.025);
  ros::param::param<int>("~min_points_voxel", min_points_voxel_, 10);
  //2: SOR Filter params
  ros::param::param<int>("~sor_knn", sor_knn_, 5);
  ros::param::param<float>("~sor_stddev", sor_stddev_, 1.0);
  //3: Radius Outlier removal
  ros::param::param<float>("~outrem_rad_search", outrem_rad_search_, 0.8);
  ros::param::param<int>("~outrem_min_neigh", outrem_min_neigh_, 2);
  //4: Planar segmentation
  ros::param::param<int>("~seg_max_itr", seg_max_itr_, 30);
  ros::param::param<float>("~seg_dist_thresh", seg_dist_thresh_, 0.01);
  //5: Euclidean cluster extraction
  ros::param::param<float>("~ec_tol", ec_tol_, 0.02);
  ros::param::param<int>("~ec_min_size", ec_min_size_, 100);
  ros::param::param<int>("~ec_max_size", ec_max_size_, 25000);

  ROS_INFO("Loaded ROS params");
}

void OrientatePlane::loadPipelineParams(){
  /////////////////////////////////////////////////////////
  //0: PASSTHROUGH FILTER
  /////////////////////////////////////////////////////////
    passthrough_filter_z_.setFilterFieldName("z");
    passthrough_filter_z_.setFilterLimits(min_z_, max_z_);

    passthrough_filter_x_.setFilterFieldName("x");
    passthrough_filter_x_.setFilterLimits(min_x_, max_x_);
  /////////////////////////////////////////////////////////
  //1: STATISTICAL OUTLIER REMOVAL FILTER
  /////////////////////////////////////////////////////////
    //num of neighbors to analyze
    sor_filter_.setMeanK(sor_knn_); 
    //standard deviation multiplier all points who have a distance larger than 1 standard deviation 
    //of the mean distance to the query point will be marked as outliers and removed.
    sor_filter_.setStddevMulThresh(sor_stddev_); 
  /////////////////////////////////////////////////////////
  //2: VOXEL FILTER
  /////////////////////////////////////////////////////////
    voxel_filter_.setLeafSize (leaf_size_, leaf_size_, leaf_size_);
    voxel_filter_.setMinimumPointsNumberPerVoxel(min_points_voxel_);
  /////////////////////////////////////////////////////////
  //3: RADIUS OUTLIER REMOVAL FILTER
  /////////////////////////////////////////////////////////
    outrem.setRadiusSearch(outrem_rad_search_);
    outrem.setMinNeighborsInRadius (outrem_min_neigh_);
    outrem.setKeepOrganized(true);
  /////////////////////////////////////////////////////////
  //4: CLUSTER EXTRACTION
  /////////////////////////////////////////////////////////
    ec.setClusterTolerance (ec_tol_); 
    ec.setMinClusterSize (ec_min_size_); 
    ec.setMaxClusterSize (ec_max_size_);  

  /////////////////////////////////////////////////////////
  //5: PLANAR SEGMENTATION
  /////////////////////////////////////////////////////////
    // Optional
    plane_seg_filter_.setOptimizeCoefficients (true);
    // Mandatory
    plane_seg_filter_.setModelType (pcl::SACMODEL_PLANE);
    plane_seg_filter_.setMethodType (pcl::SAC_RANSAC);
    plane_seg_filter_.setMaxIterations(seg_max_itr_);
    plane_seg_filter_.setDistanceThreshold (seg_dist_thresh_); //how close a point must be to the model in order to be considered an inlier.

}

void OrientatePlane::pointcloudCB(const sensor_msgs::PointCloud2ConstPtr cloud)
{
  //Init ROS data structures
    sensor_msgs::PointCloud2 ros_filtered_cloud;
    sensor_msgs::PointCloud2 ros_debug_cloud;
    sensor_msgs::PointCloud2 ros_plane_cloud;
    sensor_msgs::PointCloud2 ros_cluster_cloud;
    ROS_INFO_STREAM("CLOUD SIZE: " << (cloud->row_step *cloud->height));
  //PCL data structures
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //cluster extraction
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //planar segmentation
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_non_plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_plane_cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    //Debugging visualization
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_debug_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  tf::StampedTransform transform;
  bool status;
  //Transform from base_link to the cloud frame for the obstacle_cloud message
  status = tfEar_.waitForTransform("base_link",  cloud->header.frame_id, ros::Time(0.0), ros::Duration(10.0));
  tfEar_.lookupTransform("base_link", cloud->header.frame_id, ros::Time(0.0), transform);
  pcl_ros::transformPointCloud("base_link", transform, *cloud, ros_filtered_cloud);

  pcl::fromROSMsg(ros_filtered_cloud, *pcl_filtered_cloud);

  int initial_pc_size = pcl_filtered_cloud->points.size();

  /////////////////////////////////////////////////////////
  //0: PASSTHROUGH FILTER
  /////////////////////////////////////////////////////////
  passthrough_filter_z_.setInputCloud(pcl_filtered_cloud);
  passthrough_filter_z_.filter(*pcl_filtered_cloud);

  passthrough_filter_x_.setInputCloud(pcl_filtered_cloud);
  passthrough_filter_x_.filter(*pcl_filtered_cloud);

  /////////////////////////////////////////////////////////
  //1: STATISTICAL OUTLIER REMOVAL FILTER
  /////////////////////////////////////////////////////////
  sor_filter_.setInputCloud(pcl_filtered_cloud);
  sor_filter_.filter(*pcl_filtered_cloud);

  /////////////////////////////////////////////////////////
  //2: VOXEL FILTER
  /////////////////////////////////////////////////////////
  voxel_filter_.setInputCloud(pcl_filtered_cloud);
  voxel_filter_.filter(*pcl_filtered_cloud);

  /////////////////////////////////////////////////////////
  //3: RADIUS OUTLIER REMOVAL FILTER
  /////////////////////////////////////////////////////////
  // outrem.setInputCloud(pcl_filtered_cloud);
  // outrem.filter (*pcl_filtered_cloud);
  // // *pcl_debug_cloud = *pcl_filtered_cloud;

  if (pcl_filtered_cloud->points.size() == 0){
    // ROS_WARN("pcl_filtered_cloud is empty, skipping cluster extraction!");
    return;
  }

  /////////////////////////////////////////////////////////
  //4: CLUSTER EXTRACTION
  /////////////////////////////////////////////////////////
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (pcl_filtered_cloud);
  //extract indices of the different clusters
  std::vector<pcl::PointIndices> cluster_indices;
  ec.setSearchMethod (tree); 
  ec.setInputCloud (pcl_filtered_cloud); 
  ec.extract (cluster_indices);

  int j = 0;
  for ( std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); 
        it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (
      new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); 
          pit != it->indices.end (); ++pit)
    {
      cloud_cluster->push_back ((*pcl_filtered_cloud)[*pit]); 
    }
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    j++;

    //get the largest cloud cluster
    if (pcl_cluster_cloud->points.size() < cloud_cluster->points.size()){
      *pcl_cluster_cloud = *cloud_cluster;
    }
  }

  if (pcl_cluster_cloud->points.size() == 0){
    // ROS_WARN("pcl_cluster_cloud is empty, skipping plane segmentation!");
    return;
  }

  /////////////////////////////////////////////////////////
  //5: PLANAR SEGMENTATION
  /////////////////////////////////////////////////////////
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  
  *pcl_non_plane_cloud = *pcl_cluster_cloud;

  // int plane_seg_itr = 0, nr_points = (int) pcl_non_plane_cloud->size ();
  // while (pcl_non_plane_cloud->size () > 0.3 * nr_points)
  {
    plane_seg_filter_.setInputCloud(pcl_non_plane_cloud);
    plane_seg_filter_.segment(*inliers, *coefficients);

    if (inliers->indices.size () == 0){
      ROS_ERROR ("Could not estimate a planar model for the given dataset.\n");
      // break;
    }
    //extract the inliers
    extract_filter_.setInputCloud(pcl_non_plane_cloud);
    extract_filter_.setIndices(inliers);
    extract_filter_.setNegative(false); //choose only inliers
    extract_filter_.filter(*pcl_plane_cloud);
    // Create the filtering object
    extract_filter_.setNegative (true);
    extract_filter_.filter (*pcl_plane_cloud_f);
    pcl_non_plane_cloud.swap (pcl_plane_cloud_f);
  //   plane_seg_itr++;
  }

  ////////////////////////
  //6: Conversion to pose
  ////////////////////////
  geometry_msgs::PoseStamped pose_, x_axis_;
  planeToPose(pose_, "base_link", 
              goal_offset_x_, goal_offset_y_, 0,
              coefficients->values[0], coefficients->values[1], 
              coefficients->values[2], coefficients->values[3]);
  planeToPose(x_axis_, "base_link", 
              0, 0, 0,
              1, 0, 
              0, 0);

  pcl::toROSMsg(*pcl_filtered_cloud, ros_filtered_cloud);
  pcl::toROSMsg(*pcl_debug_cloud, ros_debug_cloud);
  pcl::toROSMsg(*pcl_cluster_cloud, ros_cluster_cloud);
  pcl::toROSMsg(*pcl_plane_cloud, ros_plane_cloud);

  ros_filtered_cloud.header.frame_id = "base_link";
  ros_cluster_cloud.header.frame_id = "base_link";
  ros_plane_cloud.header.frame_id = "base_link";
  ros_debug_cloud.header.frame_id = "base_link";

  filteredCloudPub_.publish(ros_filtered_cloud);
  clusterCloudPub_.publish(ros_cluster_cloud);
  dockingPlanePub_.publish(ros_plane_cloud);
  XAxisPub_.publish(x_axis_);
  // debugCloudPub_.publish(ros_debug_cloud);

  if (pub_docking_goal_){
    //get transform from base_link(source frame) dto map (target frame)
    //this keeps the pose in absolute coordinates (and not relative to base_link)
    geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform( "map", pose_.header.frame_id, ros::Time(0));
    tf2::doTransform(pose_, pose_, transform);
    pose_.header.frame_id = "map";
    dockingPosePub_.publish(pose_);  

    // double pose_yaw = quaternionToYaw(pose_.pose.orientation);
    // ROS_INFO("pose_: x(%f), y(%f), yaw(%f), frame(%s)", 
    //           pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z, pose_yaw,
    //           pose_.header.frame_id.c_str());

    // ROS_INFO("Publishing pose. itr(%d)", seg_itr);
    //seg_itr++;
  }

  if (print_debug_){
    ROS_INFO("Num points downsampled: Before(%lu) After(%lu)", 
              initial_pc_size, pcl_filtered_cloud->points.size());
    ROS_INFO("Num points final: Plane(%lu)", pcl_plane_cloud->points.size());
  }

}

void OrientatePlane::goalReceivedCB(const std_msgs::BoolConstPtr msg){
  // ROS_INFO("Received msg from planner adjuster, stopping the publishing of docking goal");
  if (msg->data){
    pub_docking_goal_ = false;
  }
}

void OrientatePlane::reconfigureCB(dafan_docking::orientate_planeConfig &config, uint32_t level) {
  /* Don't load initial config, since it will overwrite the rosparam settings */
  if (level == 0xFFFFFFFF) {
      return;
  }

  //Goal offset
  goal_offset_x_ = config.goal_offset_x;
  goal_offset_y_ = config.goal_offset_y;

  //0: Passthrough
  min_z_ = config.min_z;
  max_z_ = config.max_z;
  min_x_ = config.min_x;
  max_x_ = config.max_x;
  //1: Voxel filter
  leaf_size_ = config.leaf_size;
  min_points_voxel_ = config.min_points_voxel;
  //2: Statistical Outlier Removal 
  sor_knn_ = config.sor_knn;
  sor_stddev_ = config.sor_stddev;
  //3: Radius Outlier removal
  outrem_rad_search_ = config.outrem_rad_search;
  outrem_min_neigh_ = config.outrem_min_neigh;
  //4: Planar segmentation
  seg_max_itr_ = config.seg_max_itr;
  seg_dist_thresh_ = config.seg_dist_thresh;
  //5: Euclidean cluster extraction
  ec_tol_ = config.ec_tol;
  ec_min_size_ = config.ec_min_size;
  ec_max_size_ = config.ec_max_size;

  loadPipelineParams();

  ROS_INFO("Reconfigured params");
}


void OrientatePlane::planeToPose(geometry_msgs::PoseStamped& pose_, const std::string& frame_id, 
                            const double& t_x, const double& t_y, const double& t_z,
                            const double& ax, const double& ay, const double& az, const double& d) 
{
  float angle = atan2(ay, ax); //get angle about the z axis
  pose_.header.frame_id = frame_id; 
  pose_.pose.position.x = t_x;
  pose_.pose.position.y = t_y;
  pose_.pose.position.z = t_z; //pose should be on the parallel z plane with base link
  pose_.pose.orientation.x = 0;
  pose_.pose.orientation.y = 0;
  pose_.pose.orientation.z = sin(angle/2);
  pose_.pose.orientation.w = cos(angle/2);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "orientate_plane");
  OrientatePlane orientate_plane_;

  return 0;
}
