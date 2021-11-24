#include <pallet_detection/pallet_detection.h>

PalletDetection::PalletDetection() : tfListener_(tfBuffer_),
                                     goal_published_(false),
                                     status_received_(false),
                                     stages_complete_(false),
                                     next_stage_(false),
				                             reset(true),
                                     history_index_(0)
{
  //callbackType = boost::bind(&HumanDetection::reconfigureCB, this, _1, _2);
  //configServer.setCallback(callbackType);
}

bool PalletDetection::loadParams()
{
  ros::param::param<std::string>("~target_frame", target_frame_ , "");
  ros::param::param<bool>("~print_fps", print_fps_, true);  
  ros::param::param<double>("~z_axis_min", z_axis_min_, -1.0);
  ros::param::param<double>("~z_axis_max", z_axis_max_, -0.25);
  ros::param::param<double>("~x_axis_min", x_axis_min_, 1.0);
  ros::param::param<double>("~x_axis_max", x_axis_max_, 5.0);
  ros::param::param<double>("~voxel_grid_leaf_size", voxel_grid_size_, 0.01);
  ros::param::param<int>("~cluster_size_min", cluster_size_min_, 30);
  ros::param::param<int>("~cluster_size_max", cluster_size_max_, 150);
  ros::param::param<double>("~smoothness_threshold", smoothness_threshold_, 5.0);
  ros::param::param<double>("~curvature_threshold", curvature_threshold_, 1.0);
  ros::param::param<double>("~verticality_threshold", verticality_threshold_, 20.0);
  ros::param::param<std::vector<double> >("~pallet_widths", pallet_widths_, {});
  ros::param::param<double>("~length_tolerance", length_tolerance_, 0.1);
  ros::param::param<double>("~angle_tolerance", angle_tolerance_, 10.0);
  ros::param::param<double>("~midpoint_tolerance", midpoint_tolerance_, 0.1);
  ros::param::param<double>("~max_z_diff", max_z_diff_, 0.05);
  ros::param::param<double>("~x_offset", x_offset_, 0.5);
  ros::param::param<double>("~y_offset", y_offset_, 0.0);
  ros::param::param<double>("~yaw_offset", yaw_offset_, 0.0);
  ros::param::param<double>("~goal_xy_tolerance", goal_xy_tolerance_, 0.05);
  ros::param::param<double>("~goal_yaw_tolerance", goal_yaw_tolerance_, 1);
  ros::param::param<int>("~frames_tracked", frames_tracked_, 5);
  ros::param::param<double>("~inlier_dist", inlier_dist_, 0.2);
  ros::param::param<double>("~inlier_yaw", inlier_yaw_, 5);
  ros::param::param<bool>("~use_move_base", use_move_base_, false);
  ROS_INFO("use move_base: %d", use_move_base_);
  ROS_INFO("widths: %lu", pallet_widths_.size());
  return true;
}

void PalletDetection::statusCallback(std_msgs::Bool success)
{
  if(!use_move_base_)
  {
    if(status_received_)
    {
      ROS_INFO("[pallet_detection] 2nd stage done");
      stages_complete_ = true;
      return;
    }
    ROS_INFO("[pallet_detection] 1st stage done");
    status_received_ = true;
    next_stage_ = true;
  }
}

void PalletDetection::mbCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr &msg)
{
  if(use_move_base_ && (msg->status.status == 3 || msg->status.status == 4))
  {
    if(status_received_)
    {
      ROS_INFO("[pallet_detection] 2nd stage done");
      stages_complete_ = true;
      return;
    }
    ROS_INFO("[pallet_detection] 1st stage done");
    status_received_ = true;
    next_stage_ = true;
  }
}

void PalletDetection::filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_interm(new pcl::PointCloud<pcl::PointXYZ>),
                                      pc_interm2(new pcl::PointCloud<pcl::PointXYZ>),
                                      pc_interm3(new pcl::PointCloud<pcl::PointXYZ>);

  //! Filter for region of interest
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud_in);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (z_axis_min_, z_axis_max_);
  pass.filter (*pc_interm);

  pass.setInputCloud (pc_interm);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (x_axis_min_, x_axis_max_);
  pass.filter (*pc_interm2);

  //! Filter outlier points
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (pc_interm2);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*pc_interm3);

  //! Downsample point cloud
  pcl::VoxelGrid<pcl::PointXYZ> downsample;
  downsample.setInputCloud(pc_interm3);
  downsample.setLeafSize(voxel_grid_size_, voxel_grid_size_, voxel_grid_size_);
  downsample.filter(*cloud_out);
}

void PalletDetection::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2_in)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc_in(new pcl::PointCloud<pcl::PointXYZ>),
                                      filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  //! Reset clock for tracking frames per second
  if(print_fps_ && reset)
  {
    frames = 0;
    start_time = ros::Time::now();
    reset = false;
  }

  //! Transform to target frame
  if(!target_frame_.empty())
  {
    sensor_msgs::PointCloud2 cloud_out;
    geometry_msgs::TransformStamped transform;
    try
    {
      transform = tfBuffer_.lookupTransform(target_frame_, ros_pc2_in->header.frame_id, ros::Time(0), ros::Duration(0.5));
      tf2::doTransform(*ros_pc2_in, cloud_out, transform);
      pcl::fromROSMsg(cloud_out, *pcl_pc_in);
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      return;
    }
  }
  else
    //! Convert ROS message to PCL
    pcl::fromROSMsg(*ros_pc2_in, *pcl_pc_in);

  filterCloud(pcl_pc_in, filtered_cloud);

  //! Normal estimation
  pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (filtered_cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  //! Region growing segmentation into clusters
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (cluster_size_min_);
  reg.setMaxClusterSize (cluster_size_max_);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (filtered_cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (smoothness_threshold_ / 180.0 * M_PI);
  reg.setCurvatureThreshold (curvature_threshold_);
  std::vector <pcl::PointIndices> cluster_indices;
  reg.extract (cluster_indices);

  if(cluster_indices.size() > 3)
  {
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr > > clusters;
    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++)
    {
   	  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
      for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      {
        cluster->points.push_back(filtered_cloud->points[*pit]);
  	  }
      cluster->width = cluster->size();
      cluster->height = 1;
      cluster->is_dense = true;
	    clusters.push_back(cluster);
    }

    //! Filter for vertical surfaces
    std::vector<Eigen::Vector4f> centroids;
    std::vector<Vector3D> normal_vectors;
    Eigen::Vector4f centroid;
    Vector3D normal_vector;
    for(size_t i = 0; i < clusters.size(); i++)
    {
      //pcl::search::Search<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
      //pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
      //normal_estimator.setSearchMethod (tree);
      //normal_estimator.setKSearch (50);
      pcl::PointCloud <pcl::Normal>::Ptr cluster_normals (new pcl::PointCloud <pcl::Normal>);
      normal_estimator.setInputCloud (clusters[i]);
      normal_estimator.compute (*cluster_normals);

      double n_x = cluster_normals->points[cluster_normals->size()/2].normal_x;
      double n_y = cluster_normals->points[cluster_normals->size()/2].normal_y;
      double n_z = cluster_normals->points[cluster_normals->size()/2].normal_z;
      double angle = atan(fabs(n_z) / fabs(sqrt(n_x*n_x + n_y*n_y))) * 180 / M_PI;

      if(angle < verticality_threshold_)
      {
        pcl::compute3DCentroid(*clusters[i], centroid);
        centroids.push_back(centroid);
        normal_vector.x = n_x;
        normal_vector.y = n_y;
        normal_vector.z = n_z;
        normal_vectors.push_back(normal_vector);
      }
    }

    //! Compare distances between centroids of segmented regions
    std::vector< std::pair<Eigen::Vector4f,Eigen::Vector4f> > centroid_pairs;
    std::pair<Eigen::Vector4f,Eigen::Vector4f> centroid_pair;
    double distance;
    for(size_t i = 0; i < centroids.size(); i++)
    {
      for(size_t j = 0; j < centroids.size(); j++)
      {
        if(i == j)
          continue;
        distance = calcDistance(centroids[i], centroids[j]);
        /*if(distance > (pallet_length_ - length_tolerance_) && distance < (pallet_length_ + length_tolerance_)
           && calcAngleDiff(normal_vectors[i], normal_vectors[j]) < angle_tolerance_)
        {
          centroid_pair = std::make_pair(centroids[i], centroids[j]);
          centroid_pairs.push_back(centroid_pair);
        }*/
        for(size_t k = 0; k < pallet_widths_.size(); k++)
        {
          if(distance > (pallet_widths_[k] - length_tolerance_) && distance < (pallet_widths_[k] + length_tolerance_)
             && calcAngleDiff(normal_vectors[i], normal_vectors[j]) < angle_tolerance_)
          {
            centroid_pair = std::make_pair(centroids[i], centroids[j]);
            centroid_pairs.push_back(centroid_pair);
          }
        }
      }
    }

    //! Match midpoint of centroid pairs to centroids of segmented regions
    std::vector< std::pair<Eigen::Vector4f,Eigen::Vector4f> > centroid_pairs2;
    std::vector<Eigen::Vector4f> midpoints;
    Eigen::Vector4f midpoint;
    double z_dist;
    for(size_t i = 0; i < centroid_pairs.size(); i++)
    {
      midpoint[0] = (centroid_pairs[i].first[0] + centroid_pairs[i].second[0])/2;
      midpoint[1] = (centroid_pairs[i].first[1] + centroid_pairs[i].second[1])/2;
      midpoint[2] = (centroid_pairs[i].first[2] + centroid_pairs[i].second[2])/2;

      distance = 0;
      for(size_t j = 0; j < centroids.size(); j++)
      {
        if(distance == 0)
          distance = calcDistance(midpoint, centroids[j]);
        else
          distance = std::min(distance, calcDistance(midpoint, centroids[j]));
      }

      z_dist = fabs(centroid_pairs[i].first[2] - centroid_pairs[i].second[2]);
      if(distance < midpoint_tolerance_ && z_dist < max_z_diff_)
      {
        centroid_pairs2.push_back(centroid_pairs[i]);
        midpoints.push_back(midpoint);
      }
    }

    //! Select closest pallet if multiple pallets detected
    size_t index = 0;
    if(centroid_pairs2.size() > 1)
    {
      Eigen::Vector4f origin(0,0,0,0);
      distance = 0;
      double distance_temp = 0;
      for(size_t i = 0; i < centroid_pairs2.size(); i++)
      {
        if(i == 0)
          distance = calcDistance(midpoints[i], origin);
        else
        {
	  distance_temp = calcDistance(midpoints[i], origin);
	  if(distance > distance_temp)
	  {
            distance = distance_temp;
            index = i;
	  }
        }
      }
    }
    
    //! Publish tf from sensor to pallet, and tf from pallet to docking position
    if(centroid_pairs2.size() > 0)
    {
      double yaw = getYaw(centroid_pairs2[index]);
      tf::Quaternion q;
      q.setRPY(0, 0, yaw);
      geometry_msgs::Quaternion quat;
      tf::quaternionTFToMsg(q, quat);

      geometry_msgs::TransformStamped pallet_tf;
      pallet_tf.header.stamp = ros::Time::now();
      if(target_frame_.empty())
        pallet_tf.header.frame_id = ros_pc2_in->header.frame_id;
      else
        pallet_tf.header.frame_id = target_frame_;
      pallet_tf.child_frame_id = "pallet";
      pallet_tf.transform.translation.x = midpoints[index][0];
      pallet_tf.transform.translation.y = midpoints[index][1];
      pallet_tf.transform.translation.z = midpoints[index][2];
      pallet_tf.transform.rotation = quat;
      br_.sendTransform(pallet_tf);

      geometry_msgs::TransformStamped dock_tf;
      dock_tf.header.stamp = ros::Time::now();
      dock_tf.header.frame_id = "pallet";
      dock_tf.child_frame_id = "pallet_dock";
      dock_tf.transform.translation.y = y_offset_;
      q.setRPY(0, 0, yaw_offset_ * M_PI / 180);
      tf::quaternionTFToMsg(q, quat);
      dock_tf.transform.rotation = quat;

      if(!status_received_)
      {
        geometry_msgs::TransformStamped base_link_to_pallet;
        try
        {
          base_link_to_pallet = tfBuffer_.lookupTransform("base_link", "pallet", ros::Time(0), ros::Duration(0.5));
        }
        catch (tf2::TransformException &ex)
        {
          ROS_WARN("%s", ex.what());
          return;
        }
        double dx = base_link_to_pallet.transform.translation.x;
        double dy = base_link_to_pallet.transform.translation.y;
        double distance = sqrt(dx*dx + dy*dy);
        double transform_yaw = atan(fabs(dy) / fabs(dx));
        double yaw_relative;
        if((dx > 0 && dy > 0) || (dx < 0 && dy < 0))
          yaw_relative = transform_yaw - yaw;
        else if ((dx > 0 && dy < 0) || (dx < 0 &&  dy > 0))
          yaw_relative = transform_yaw + yaw;
        dock_tf.transform.translation.x = 0.9 * -distance * cos(fabs(yaw_relative));
        br_.sendTransform(dock_tf);
      }
      else
      {
        if(next_stage_)
        {
          history_index_ = 0;
          x_history_.clear();
          y_history_.clear();
          geometry_msgs::PoseStamped clear;
          recorded_goal_ = clear;
          next_stage_ = false;
          goal_published_ = false;
        }
        dock_tf.transform.translation.x = -x_offset_;
        br_.sendTransform(dock_tf);
      }

      historyAveraging();
    }

    //! Publish pallet marker
    if(marker_pub_.getNumSubscribers() > 0)
    {
      visualization_msgs::Marker marker;
      marker.header.stamp = ros::Time::now();
      if(target_frame_.empty())
        marker.header.frame_id = ros_pc2_in->header.frame_id;
      else
        marker.header.frame_id = target_frame_;
      marker.ns = "pallet";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.action = visualization_msgs::Marker::ADD;
      
      geometry_msgs::Point p;
      for(size_t i = 0; i < centroid_pairs2.size(); i++)
      {
        p.x = centroid_pairs2[i].first[0];
        p.y = centroid_pairs2[i].first[1];;
        p.z = centroid_pairs2[i].first[2];;
        marker.points.push_back(p);
        p.x = centroid_pairs2[i].second[0];
        p.y = centroid_pairs2[i].second[1];
        p.z = centroid_pairs2[i].second[2];
        marker.points.push_back(p);
      }
      
      marker.scale.x = 0.02;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.5;
      marker.lifetime = ros::Duration(0.5);
      if(marker.points.size())
      {
        marker_pub_.publish(marker);
      }
    }

    //! Publish centroids of segmented clusters from region growing algorithm
    if(centroids_pub_.getNumSubscribers() > 0)
    {
      visualization_msgs::Marker marker2;
      marker2.header.stamp = ros::Time::now();
      if(target_frame_.empty())
        marker2.header.frame_id = ros_pc2_in->header.frame_id;
      else
        marker2.header.frame_id = target_frame_;
      marker2.ns = "centroids";
      marker2.id = 0;
      marker2.type = visualization_msgs::Marker::POINTS;
      marker2.action = visualization_msgs::Marker::ADD;
      
      geometry_msgs::Point p;
      for(size_t i = 0; i < centroids.size(); i++)
      {
        p.x = centroids[i][0];
        p.y = centroids[i][1];;
        p.z = centroids[i][2];;
        marker2.points.push_back(p);
      }
      
      marker2.scale.x = 0.02;
      marker2.scale.y = 0.02;
      marker2.color.a = 1.0;
      marker2.color.r = 1.0;
      marker2.color.g = 0.0;
      marker2.color.b = 0.5;
      marker2.lifetime = ros::Duration(0.5);
      if(marker2.points.size())
      {
        centroids_pub_.publish(marker2);
      }
    }

    //! Publish filtered point cloud input
    if(cloud_filtered_pub_.getNumSubscribers() > 0)
    {
      pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
      sensor_msgs::PointCloud2 ros_pc2_out;
      pcl::toROSMsg(*colored_cloud, ros_pc2_out);
      if(target_frame_.empty())
        ros_pc2_out.header.frame_id = ros_pc2_in->header.frame_id;
      else
        ros_pc2_out.header.frame_id = target_frame_;
      cloud_filtered_pub_.publish(ros_pc2_out);
    }
  }

  //! Log frames per second
  if(print_fps_ && ++frames>10)
  {
    ROS_INFO_STREAM("[pallet_detection] fps = "<<float(frames)/(float(ros::Time::now().toSec()-start_time.toSec())));
    reset = true;
  }
}

double PalletDetection::calcDistance(Eigen::Vector4f c1, Eigen::Vector4f c2)
{
  double dx = c1[0] - c2[0];
  double dy = c1[1] - c2[1];
  double dz = c1[2] - c2[2];

  return sqrt(dx*dx + dy*dy + dz*dz);
}

double PalletDetection::calcAngleDiff(Vector3D v1, Vector3D v2)
{
  float dot_product = v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
  float v1_magnitude = sqrt(v1.x*v1.x + v1.y*v1.y + v1.z*v1.z);
  float v2_magnitude = sqrt(v2.x*v2.x + v2.y*v2.y + v2.z*v2.z);
  return acos(dot_product/(v1_magnitude*v2_magnitude)) * 180 / M_PI;
}

double PalletDetection::calcAngleDiff(geometry_msgs::Pose init_pose, geometry_msgs::Pose target_pose)
{
  tf::Quaternion q1(init_pose.orientation.x, init_pose.orientation.y,
                    init_pose.orientation.z, init_pose.orientation.w),
                 q2(target_pose.orientation.x, target_pose.orientation.y,
                    target_pose.orientation.z, target_pose.orientation.w);
  tf::Matrix3x3 m1(q1), m2(q2);
  double r1, p1, y1, r2, p2, y2;
  m1.getRPY(r1, p1, y1);
  m2.getRPY(r2, p2, y2);
  
  double dtheta = fabs(y1 - y2);
  dtheta = std::min(dtheta, 2.0*M_PI - dtheta);
  dtheta = dtheta / M_PI * 180;
  return dtheta;
}

Vector3D PalletDetection::crossProduct(Vector3D vect_A, Vector3D vect_B) 
{
  Vector3D cross_P;
  cross_P.x = vect_A.y * vect_B.z - vect_A.z * vect_B.y;
  cross_P.y = vect_A.z * vect_B.x - vect_A.x * vect_B.z;
  cross_P.z = vect_A.x * vect_B.y - vect_A.y * vect_B.x;
  return cross_P;
}

double PalletDetection::getYaw(std::pair<Eigen::Vector4f,Eigen::Vector4f> centroid_pair)
{
  //! Normal vector of horizontal plane
  Vector3D n_horizontal;
  n_horizontal.x = 0;
  n_horizontal.y = 0;
  n_horizontal.z = 1;

  //! Vector between centroid pair
  Vector3D pair_vector;
  pair_vector.x = centroid_pair.first[0] - centroid_pair.second[0];
  pair_vector.y = centroid_pair.first[1] - centroid_pair.second[1];
  pair_vector.z = centroid_pair.first[2] - centroid_pair.second[2];

  Vector3D cross_product = crossProduct(n_horizontal, pair_vector);

  //! Get yaw of vector perpendicular to line between centroid pair
  return atan(cross_product.y/cross_product.x);
}

void PalletDetection::tfToPose(geometry_msgs::TransformStamped tf, geometry_msgs::PoseStamped& pose)
{
  double norm = sqrt(pow(tf.transform.rotation.z, 2) + pow(tf.transform.rotation.w, 2));
  pose.header.frame_id = "odom";
  pose.pose.position.x = tf.transform.translation.x;
  pose.pose.position.y = tf.transform.translation.y;
  pose.pose.position.z = 0;
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.z = tf.transform.rotation.z / norm;
  pose.pose.orientation.w = tf.transform.rotation.w / norm;
}

void PalletDetection::historyAveraging()
{
  geometry_msgs::TransformStamped dock_goal;
  geometry_msgs::PoseStamped goal_pose;
  
  //! Record a few transforms of docking goal to get the average transform
  if(x_history_.size() < frames_tracked_)
  {
    try
    {
      dock_goal = tfBuffer_.lookupTransform("odom", "pallet_dock", ros::Time(0), ros::Duration(0.5));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }
    tfToPose(dock_goal, goal_pose);
    x_history_.push_back(dock_goal.transform.translation.x);
    y_history_.push_back(dock_goal.transform.translation.y);
    return;
  }
  
  double sum_x = 0;
  double sum_y = 0;
  double distance = 0;

  try
  {
    dock_goal = tfBuffer_.lookupTransform("odom", "pallet_dock", ros::Time(0), ros::Duration(0.5));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }

  //! Filter out outlier docking position
  if(goal_published_)
  {
    geometry_msgs::PoseStamped current_goal;
    tfToPose(dock_goal, current_goal);
    Eigen::Vector4f p1(recorded_goal_.pose.position.x, recorded_goal_.pose.position.y, 0, 0),
                    p2(current_goal.pose.position.x, current_goal.pose.position.y, 0, 0);
    distance = calcDistance(p1, p2);
    double yaw_diff = calcAngleDiff(recorded_goal_.pose, current_goal.pose);
    if(distance > inlier_dist_ || yaw_diff > inlier_yaw_)
      return;
  }

  //! Overwrite old xy transform of goal with latest transform
  x_history_[history_index_] = dock_goal.transform.translation.x;
  y_history_[history_index_] = dock_goal.transform.translation.y;

  history_index_ = (history_index_ + 1) % frames_tracked_;

  //! Find average xy transform of goal
  for(size_t i = 0; i < frames_tracked_; i++)
  {
    sum_x += x_history_[i];
    sum_y += y_history_[i];
  }
  dock_goal.transform.translation.x = sum_x / frames_tracked_;
  dock_goal.transform.translation.y = sum_y / frames_tracked_;
  tfToPose(dock_goal, goal_pose);

  //! Publish docking goal
  if(!goal_published_)
  {
    goal_pub_.publish(goal_pose);
    recorded_goal_ = goal_pose;
    goal_published_ = true;
  }
  else if(status_received_)
  {
    Eigen::Vector4f g1(recorded_goal_.pose.position.x, recorded_goal_.pose.position.y, 0, 0),
                    g2(goal_pose.pose.position.x, goal_pose.pose.position.y, 0, 0);
    distance = calcDistance(g1, g2);
    double angle = calcAngleDiff(recorded_goal_.pose, goal_pose.pose);
    if(distance > goal_xy_tolerance_ || angle > goal_yaw_tolerance_)
    {
      if(!stages_complete_)
      {
        std_msgs::Bool stop;
        stop.data = true;
        stop_pub_.publish(stop);
        ros::Duration(0.1).sleep();
        goal_pub_.publish(goal_pose);
        recorded_goal_ = goal_pose;
      }
      else
      {
        current_goal_pub_.publish(goal_pose);
        recorded_goal_ = goal_pose;
      }
    }
  }
}