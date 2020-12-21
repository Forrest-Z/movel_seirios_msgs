#include <human_detection/human_detection.h>

HumanDetection::HumanDetection() : map_pc(new pcl::PointCloud<pcl::PointXYZ>),
				   map_cropped(new pcl::PointCloud<pcl::PointXYZ>),
				   scan_pc(new pcl::PointCloud<pcl::PointXYZ>),
				   cluster_array_seq_(0),
                                   pose_array_seq_(0),
				   reset(true),
				   history_index_(0)
{}

void HumanDetection::getMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
  //! Downsample map for faster processing
  pcl::VoxelGrid<pcl::PointXYZ> downsample;
  downsample.setInputCloud(cloud_in);
  downsample.setLeafSize(voxel_grid_size_, voxel_grid_size_, 0.0);
  downsample.filter(*map_pc);
}

void HumanDetection::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  //! Process map from occupancy grid to point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);
  for(size_t i = 0; i < map->info.width * map->info.height; i++)
  {
    if(map->data[i] == 100)
    {
      pcl::PointXYZ p;
      p.x = (i % map->info.width) * map->info.resolution + map->info.origin.position.x;
      p.y = (i / map->info.width) * map->info.resolution + map->info.origin.position.y;
      p.z = 1.0;
      cloud_raw->push_back(p);
    }
  }
  getMap(cloud_raw);
}

double HumanDetection::getCroppedMap(geometry_msgs::Pose msg)
{
  boost::unique_lock<boost::mutex> scoped_lock(mtx_);
  //! Crop map w.r.t. robot pose
  pcl::CropBox<pcl::PointXYZ> boxFilter;
  boxFilter.setMin(Eigen::Vector4f(-detection_range_, -detection_range_, 0.0, 1.0));
  boxFilter.setMax(Eigen::Vector4f(detection_range_, detection_range_, 2.0, 1.0));

  boxFilter.setTranslation(Eigen::Vector3f(msg.position.x, msg.position.y, msg.position.z));
  tf::Quaternion q(msg.orientation.x, 
		   msg.orientation.y, 
		   msg.orientation.z, 
		   msg.orientation.w);
  tf::Matrix3x3 m(q);
  double r, p, y;
  m.getRPY(r, p, y);
  boxFilter.setRotation(Eigen::Vector3f(r, p, y));

  boxFilter.setInputCloud(map_pc);
  boxFilter.filter(*map_cropped);
  robot_coordinate.x = msg.position.x;
  robot_coordinate.y = msg.position.y;
  return y;
}

void HumanDetection::poseCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
  double y = getCroppedMap(*msg);

  //! Publish point cloud of cropped map
  if(map_pub_.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2 map_cloud;
    pcl::toROSMsg(*map_cropped, map_cloud);
    map_cloud.header.frame_id = map_frame_;
    map_pub_.publish(map_cloud);
  }

  //! Publish visualization marker on edges of cropped map
  if(crop_marker_pub_.getNumSubscribers() > 0)
  {
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = map_frame_;
    marker.ns = "human_detection";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
      
    geometry_msgs::Point points[5];
    points[0].x = msg->position.x + sqrt(pow(detection_range_, 2) * 2) * cos(M_PI/4 + y);  points[0].y = msg->position.y + sqrt(pow(detection_range_, 2) * 2) * sin(M_PI/4 + y);  points[0].z = 0;
    points[1].x = msg->position.x - sqrt(pow(detection_range_, 2) * 2) * sin(M_PI/4 + y);  points[1].y = msg->position.y + sqrt(pow(detection_range_, 2) * 2) * cos(M_PI/4 + y);  points[1].z = 0;
    points[2].x = msg->position.x - sqrt(pow(detection_range_, 2) * 2) * cos(M_PI/4 + y);  points[2].y = msg->position.y - sqrt(pow(detection_range_, 2) * 2) * sin(M_PI/4 + y);  points[2].z = 0;
    points[3].x = msg->position.x + sqrt(pow(detection_range_, 2) * 2) * sin(M_PI/4 + y);  points[3].y = msg->position.y - sqrt(pow(detection_range_, 2) * 2) * cos(M_PI/4 + y);  points[3].z = 0;
    points[4].x = msg->position.x + sqrt(pow(detection_range_, 2) * 2) * cos(M_PI/4 + y);  points[4].y = msg->position.y + sqrt(pow(detection_range_, 2) * 2) * sin(M_PI/4 + y);  points[4].z = 0;

    for(int i = 0; i < 5; i++) 
    {
      marker.points.push_back(points[i]);
    }
      
    marker.scale.x = 0.02;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.5;
    marker.lifetime = ros::Duration(0.1);
    crop_marker_pub_.publish(marker);
  }
}

void HumanDetection::cropCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out)
{
  //! Crop point cloud input to match cropped map
  pcl::CropBox<pcl::PointXYZI> boxFilter;
  boxFilter.setMin(Eigen::Vector4f(-detection_range_, -detection_range_, -5.0, 1.0));
  boxFilter.setMax(Eigen::Vector4f(detection_range_, detection_range_, 5.0, 1.0));
  boxFilter.setInputCloud(cloud_in);
  boxFilter.filter(*cloud_out);
}

void HumanDetection::removePlanes(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::IndicesPtr pc_indices)
{
  //! Remove floor and ceiling from point cloud input
  pcl::PassThrough<pcl::PointXYZI> pt;
  pt.setInputCloud(cloud_in);
  pt.setFilterFieldName("z");
  pt.setFilterLimits(z_axis_min_, z_axis_max_);
  pt.filter(*pc_indices);
}

void HumanDetection::adaptiveClustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::IndicesPtr pc_indices,   std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr > > &clusters)
{
  //! Divide the point cloud into nested circular regions
  std::array<std::vector<int>, region_max_> indices_array;
  for(int i = 0; i < pc_indices->size(); i++) {
    float range = 0.0;
    for(int j = 0; j < region_max_; j++) {
      float d2 = cloud_in->points[(*pc_indices)[i]].x * cloud_in->points[(*pc_indices)[i]].x +
	cloud_in->points[(*pc_indices)[i]].y * cloud_in->points[(*pc_indices)[i]].y +
	cloud_in->points[(*pc_indices)[i]].z * cloud_in->points[(*pc_indices)[i]].z;
      if(d2 > range * range && d2 <= (range+regions_[j]) * (range+regions_[j])) {
      	indices_array[j].push_back((*pc_indices)[i]);
      	break;
      }
      range += regions_[j];
    }
  }
  
  //! Euclidean clustering
  float tolerance = 0.0;
  for(int i = 0; i < region_max_; i++) {
    tolerance += 0.1;
    if(indices_array[i].size() > cluster_size_min_) {
      boost::shared_ptr<std::vector<int> > indices_array_ptr(new std::vector<int>(indices_array[i]));
      pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
      tree->setInputCloud(cloud_in, indices_array_ptr);
      
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
      ec.setClusterTolerance(tolerance);
      ec.setMinClusterSize(cluster_size_min_);
      ec.setMaxClusterSize(cluster_size_max_);
      ec.setSearchMethod(tree);
      ec.setInputCloud(cloud_in);
      ec.setIndices(indices_array_ptr);
      ec.extract(cluster_indices);
      
      for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
      	pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
      	for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
      	  cluster->points.push_back(cloud_in->points[*pit]);
  	}
      	cluster->width = cluster->size();
      	cluster->height = 1;
      	cluster->is_dense = true;
	clusters.push_back(cluster);
      }
    }
  }
}

void HumanDetection::checkDimensions(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr > > &clusters_in, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr > > &clusters_out)
{
  for(size_t j = 0; j < clusters_in.size(); j++)
  {
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*clusters_in[j], pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*clusters_in[j], pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*clusters_in[j], *cloudPointsProjected, projectionTransform);

    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZI minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);

    //! Check width and height of bounding box of each point cloud cluster
    if((maxPoint.x - minPoint.x) < max_width_ && (maxPoint.y - minPoint.y) < max_width_ && (maxPoint.x - minPoint.x) > min_width_ && (maxPoint.y - minPoint.y) > min_width_ && fabs((maxPoint.x - minPoint.x) - (maxPoint.y - minPoint.y)) < max_width_length_diff_ && (maxPoint.x - minPoint.x) + (maxPoint.y - minPoint.y) < min_width_ + max_width_)
    {
      if((maxPoint.z - minPoint.z) < max_height_ && (maxPoint.z - minPoint.z) > min_height_)
      {
        clusters_out.push_back(clusters_in[j]);
      }
      else if((maxPoint.z - minPoint.z) < min_height_ && (maxPoint.z - minPoint.z) > close_dist_min_height_)
      {
        double distance = sqrt(pow(pcaCentroid[0] - robot_coordinate.x, 2) + pow(pcaCentroid[1] - robot_coordinate.y, 2));
        if(distance < cutoff_distance_)
          clusters_out.push_back(clusters_in[j]);
      }
    }
  }
}

void HumanDetection::filterStaticObjects(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr > > &clusters_in, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr > > &clusters_out)
{
  Eigen::Vector4f min_filter, max_filter;
  bool pass;
  {
    for(size_t i = 0; i < clusters_in.size(); i++)
    {
      pass = true;
      pcl::getMinMax3D(*clusters_in[i], min_filter, max_filter);
      for(pcl::PointCloud<pcl::PointXYZ>::iterator it = map_cropped->begin(); it != map_cropped->end(); it++)
      {
        //! Filter out point cloud clusters with map points representing static obstacles within the range of bounding boxes
        if((it->x > min_filter[0] && it->x < max_filter[0]) && (it->y > min_filter[1] && it->y < max_filter[1]))
        {
          pass = false;
          break;
        }
        else
        {
          double d1, d2, d3, d4;
          d1 = sqrt(pow((it->x - min_filter[0]), 2) + pow((it->y - min_filter[1]), 2));
          d2 = sqrt(pow((it->x - min_filter[0]), 2) + pow((it->y - max_filter[1]), 2));
          d3 = sqrt(pow((it->x - max_filter[0]), 2) + pow((it->y - min_filter[1]), 2));
          d4 = sqrt(pow((it->x - max_filter[0]), 2) + pow((it->y - max_filter[1]), 2));

          if(d1<map_inflation_dist_ || d2<map_inflation_dist_ || d3<map_inflation_dist_ || d4<map_inflation_dist_)
          {
            pass = false;
            break;
          }
        }
      }
      if(pass)
        clusters_out.push_back(clusters_in[i]);
    }
  }
}

bool HumanDetection::historyAveraging(size_t clusters_size, double &average)
{
  if(detection_history.size() < frames_tracked_)
  {
    //! Store detection result of previous frames
    detection_history.push_back(clusters_size > 0);

    return false;
  }
  else
  {
    int sum = 0;

    //! Overwrite old detection results with recent frames
    detection_history[history_index_] = clusters_size > 0;
    history_index_ = (history_index_ + 1) % frames_tracked_;

    //! Find average detection result in tracked frames
    for(size_t i = 0; i < frames_tracked_; i++)
    {
      sum += detection_history[i];
    }
    average = double(sum) / frames_tracked_;

    return true;
  }
}

void HumanDetection::filterLaser(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr > > &clusters_in, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr > > &clusters_out)
{
  //! Narrow down relevant point cloud with laser scan converted from 3d point cloud
  Eigen::Vector4f min3D, max3D;
  bool pass;
  {
    for(size_t i = 0; i < clusters_in.size(); i++)
    {
      pass = false;
      pcl::getMinMax3D(*clusters_in[i], min3D, max3D);
      for(pcl::PointCloud<pcl::PointXYZ>::iterator it = scan_pc->begin(); it != scan_pc->end(); it++)
      {
        if((it->x > min3D[0] && it->x < max3D[0]) && (it->y > min3D[1] && it->y < max3D[1]))
        {
          pass = true;
          break;
        }
      }
      if(pass)
        clusters_out.push_back(clusters_in[i]);
    }
  }
}

void HumanDetection::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2_in)
{
  boost::unique_lock<boost::mutex> scoped_lock(mtx_);
  //! Reset clock for tracking frames per second
  if(print_fps_ && reset)
  {
    frames = 0;
    start_time = clock();
    reset = false;
  }

  //! Convert ROS message to PCL
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_in(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*ros_pc2_in, *pcl_pc_in);
  
  //! Crop cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZI>);
  cropCloud(pcl_pc_in, cloud_cropped);

  //! Remove ground and ceiling
  pcl::IndicesPtr pc_indices(new std::vector<int>);
  removePlanes(cloud_cropped, pc_indices);

  //! Adaptive clustering
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr > > clusters;
  adaptiveClustering(cloud_cropped, pc_indices, clusters);

  //! Filter using laser scan converted from 3d point cloud
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr > > clusters2;
  filterLaser(clusters, clusters2);

  //! Transform to map frame
  tf::StampedTransform transform;
  try
  {
    listener_->waitForTransform(map_frame_, laser_frame_, ros_pc2_in->header.stamp, ros::Duration(0.5));
    listener_->lookupTransform(map_frame_, laser_frame_, ros_pc2_in->header.stamp, transform);

    for(size_t i = 0; i < clusters2.size(); i++)
    {
      pcl_ros::transformPointCloud(*clusters2[i], *clusters2[i], transform);
    }
  }
  catch(tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    return;
  }

  //! Filter for dynamic objects using map
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr > > clusters3;
  filterStaticObjects(clusters2, clusters3);

  //! Bounding box dimension check
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr > > clusters_filtered;
  checkDimensions(clusters3, clusters_filtered);

  //! Average detection result of tracked frames
  double average;
  if(historyAveraging(clusters_filtered.size(), average))
  {
    std_msgs::Float64 f;
    f.data = average;
    detection_pub_.publish(f);
  }

  //! Publish filtered point cloud input
  if(cloud_filtered_pub_.getNumSubscribers() > 0)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_out(new pcl::PointCloud<pcl::PointXYZI>);
    sensor_msgs::PointCloud2 ros_pc2_out;
    pcl::copyPointCloud(*cloud_cropped, *pc_indices, *pcl_pc_out);
    pcl::toROSMsg(*pcl_pc_out, ros_pc2_out);
    cloud_filtered_pub_.publish(ros_pc2_out);
  }

  /**
   * Some sections below are commented out as they are either unnecessary or they only help with debugging,
   * and having them in use will increase overall processing time.
   */

  //human_detection::ClusterArray cluster_array;
  //geometry_msgs::PoseArray pose_array;
  visualization_msgs::MarkerArray marker_array;
  //visualization_msgs::MarkerArray all_clusters_array;

  /*for(int i = 0; i < clusters.size(); i++)
  {
    Eigen::Vector4f min, max;
    pcl::getMinMax3D(*clusters[i], min, max);

    //! Publish visualization markers for all detected clusters in point cloud input
    //if(all_clusters_pub_.getNumSubscribers() > 0) 
    {      
      visualization_msgs::Marker marker;
      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = laser_frame_;
      marker.ns = "adaptive_clustering";
      marker.id = i;
      marker.type = visualization_msgs::Marker::LINE_LIST;
      
      geometry_msgs::Point p[24];
      p[0].x = max[0];  p[0].y = max[1];  p[0].z = max[2];
      p[1].x = min[0];  p[1].y = max[1];  p[1].z = max[2];
      p[2].x = max[0];  p[2].y = max[1];  p[2].z = max[2];
      p[3].x = max[0];  p[3].y = min[1];  p[3].z = max[2];
      p[4].x = max[0];  p[4].y = max[1];  p[4].z = max[2];
      p[5].x = max[0];  p[5].y = max[1];  p[5].z = min[2];
      p[6].x = min[0];  p[6].y = min[1];  p[6].z = min[2];
      p[7].x = max[0];  p[7].y = min[1];  p[7].z = min[2];
      p[8].x = min[0];  p[8].y = min[1];  p[8].z = min[2];
      p[9].x = min[0];  p[9].y = max[1];  p[9].z = min[2];
      p[10].x = min[0]; p[10].y = min[1]; p[10].z = min[2];
      p[11].x = min[0]; p[11].y = min[1]; p[11].z = max[2];
      p[12].x = min[0]; p[12].y = max[1]; p[12].z = max[2];
      p[13].x = min[0]; p[13].y = max[1]; p[13].z = min[2];
      p[14].x = min[0]; p[14].y = max[1]; p[14].z = max[2];
      p[15].x = min[0]; p[15].y = min[1]; p[15].z = max[2];
      p[16].x = max[0]; p[16].y = min[1]; p[16].z = max[2];
      p[17].x = max[0]; p[17].y = min[1]; p[17].z = min[2];
      p[18].x = max[0]; p[18].y = min[1]; p[18].z = max[2];
      p[19].x = min[0]; p[19].y = min[1]; p[19].z = max[2];
      p[20].x = max[0]; p[20].y = max[1]; p[20].z = min[2];
      p[21].x = min[0]; p[21].y = max[1]; p[21].z = min[2];
      p[22].x = max[0]; p[22].y = max[1]; p[22].z = min[2];
      p[23].x = max[0]; p[23].y = min[1]; p[23].z = min[2];
      for(int i = 0; i < 24; i++)
      {
  	marker.points.push_back(p[i]);
      }
      
      marker.scale.x = 0.02;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 0.5;
      marker.color.b = 1.0;
      marker.lifetime = ros::Duration(0.4);
      all_clusters_array.markers.push_back(marker);
    }
  }*/

  for(int i = 0; i < clusters_filtered.size(); i++)
  {
    Eigen::Vector4f min, max;
    pcl::getMinMax3D(*clusters_filtered[i], min, max);

    //! Publish point cloud clusters representing humans
    /*if(cluster_array_pub_.getNumSubscribers() > 0)
    {
      sensor_msgs::PointCloud2 ros_pc2_out;
      pcl::toROSMsg(*clusters_filtered[i], ros_pc2_out);
      cluster_array.clusters.push_back(ros_pc2_out);
    }*/

    //! Publish poses of detected humans
    /*if(pose_array_pub_.getNumSubscribers() > 0)
    {
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*clusters_filtered[i], centroid);
      
      geometry_msgs::Pose pose;
      pose.position.x = centroid[0];
      pose.position.y = centroid[1];
      pose.position.z = centroid[2];
      pose.orientation.w = 1;
      pose_array.poses.push_back(pose);
    }*/

    //! Publish visualization markers for point cloud clusters representing humans
    if(marker_array_pub_.getNumSubscribers() > 0)
    {
      visualization_msgs::Marker marker;
      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = map_frame_;
      marker.ns = "human_detection";
      marker.id = i;
      marker.type = visualization_msgs::Marker::LINE_LIST;
      
      geometry_msgs::Point p[24];
      p[0].x = max[0];  p[0].y = max[1];  p[0].z = max[2];
      p[1].x = min[0];  p[1].y = max[1];  p[1].z = max[2];
      p[2].x = max[0];  p[2].y = max[1];  p[2].z = max[2];
      p[3].x = max[0];  p[3].y = min[1];  p[3].z = max[2];
      p[4].x = max[0];  p[4].y = max[1];  p[4].z = max[2];
      p[5].x = max[0];  p[5].y = max[1];  p[5].z = min[2];
      p[6].x = min[0];  p[6].y = min[1];  p[6].z = min[2];
      p[7].x = max[0];  p[7].y = min[1];  p[7].z = min[2];
      p[8].x = min[0];  p[8].y = min[1];  p[8].z = min[2];
      p[9].x = min[0];  p[9].y = max[1];  p[9].z = min[2];
      p[10].x = min[0]; p[10].y = min[1]; p[10].z = min[2];
      p[11].x = min[0]; p[11].y = min[1]; p[11].z = max[2];
      p[12].x = min[0]; p[12].y = max[1]; p[12].z = max[2];
      p[13].x = min[0]; p[13].y = max[1]; p[13].z = min[2];
      p[14].x = min[0]; p[14].y = max[1]; p[14].z = max[2];
      p[15].x = min[0]; p[15].y = min[1]; p[15].z = max[2];
      p[16].x = max[0]; p[16].y = min[1]; p[16].z = max[2];
      p[17].x = max[0]; p[17].y = min[1]; p[17].z = min[2];
      p[18].x = max[0]; p[18].y = min[1]; p[18].z = max[2];
      p[19].x = min[0]; p[19].y = min[1]; p[19].z = max[2];
      p[20].x = max[0]; p[20].y = max[1]; p[20].z = min[2];
      p[21].x = min[0]; p[21].y = max[1]; p[21].z = min[2];
      p[22].x = max[0]; p[22].y = max[1]; p[22].z = min[2];
      p[23].x = max[0]; p[23].y = min[1]; p[23].z = min[2];
      for(int i = 0; i < 24; i++) {
  	marker.points.push_back(p[i]);
      }
      
      marker.scale.x = 0.02;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.5;
      marker.lifetime = ros::Duration(0.4);
      marker_array.markers.push_back(marker);
    }
  }

  /*if(cluster_array.clusters.size())
  {
    cluster_array.header.seq = ++cluster_array_seq_;
    cluster_array.header.stamp = ros::Time::now();
    cluster_array.header.frame_id = map_frame_;
    cluster_array_pub_.publish(cluster_array);
  }

  if(pose_array.poses.size())
  {
    pose_array.header.seq = ++pose_array_seq_;
    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = map_frame_;
    pose_array_pub_.publish(pose_array);
  }*/

  /*if(all_clusters_array.markers.size())
  {
    all_clusters_pub_.publish(all_clusters_array);
  }*/

  if(marker_array.markers.size())
  {
    marker_array_pub_.publish(marker_array);
  }

  //! Log frames per second
  if(print_fps_ && ++frames>10)
  {
    ROS_INFO_STREAM("[human_detection] fps = "<<float(frames)/(float(clock()-start_time)/CLOCKS_PER_SEC)<<", timestamp = "<<clock()/CLOCKS_PER_SEC);
    reset = true;
  }
}

void HumanDetection::sensormodelConfig()
{
  //! Divide the point cloud into nested circular regions centred at the sensor.
  //! For more details, see IROS-17 paper "Online learning for human classification in 3D LiDAR-based tracking"
  if(sensor_model_ == "VLP-16") {
    regions_[0] = 2; regions_[1] = 3; regions_[2] = 3; regions_[3] = 3; regions_[4] = 3;
    regions_[5] = 3; regions_[6] = 3; regions_[7] = 2; regions_[8] = 3; regions_[9] = 3;
    regions_[10]= 3; regions_[11]= 3; regions_[12]= 3; regions_[13]= 3;
  } else if (sensor_model_ == "HDL-32E") {
    regions_[0] = 4; regions_[1] = 5; regions_[2] = 4; regions_[3] = 5; regions_[4] = 4;
    regions_[5] = 5; regions_[6] = 5; regions_[7] = 4; regions_[8] = 5; regions_[9] = 4;
    regions_[10]= 5; regions_[11]= 5; regions_[12]= 4; regions_[13]= 5;
  } else if (sensor_model_ == "HDL-64E") {
    regions_[0] = 14; regions_[1] = 14; regions_[2] = 14; regions_[3] = 15; regions_[4] = 14;
  }
}

void HumanDetection::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  //! Get laser scan converted from 3d point cloud for processing
  boost::unique_lock<boost::mutex> scoped_lock(mtx_);
  sensor_msgs::PointCloud2 cloud;
  projector_.projectLaser(*scan, cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(cloud, *cloud_raw);

  pcl::CropBox<pcl::PointXYZ> boxFilter;
  boxFilter.setMin(Eigen::Vector4f(-detection_range_, -detection_range_, -5.0, 1.0));
  boxFilter.setMax(Eigen::Vector4f(detection_range_, detection_range_, 5.0, 1.0));
  boxFilter.setInputCloud(cloud_raw);
  boxFilter.filter(*cloud_raw);

  pcl::VoxelGrid<pcl::PointXYZ> downsample;
  downsample.setInputCloud(cloud_raw);
  downsample.setLeafSize(voxel_grid_size_, voxel_grid_size_, 0.0);
  downsample.filter(*scan_pc);

  if(scan_pub_.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2 scan_cloud;
    pcl::toROSMsg(*scan_pc, scan_cloud);
    scan_cloud.header.frame_id = laser_frame_;
    scan_pub_.publish(scan_cloud);
  }
}
