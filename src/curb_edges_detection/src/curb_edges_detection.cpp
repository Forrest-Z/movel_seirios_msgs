#include <curb_edges_detection/curb_edges_detection.h>

#define PI 3.14159265

CurbEdgesDetection::CurbEdgesDetection()
{
    setupParams();
    setupTopics();
}

void CurbEdgesDetection::setupTopics()
{
    pcl_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, p_lidar_topic_, 1);
    depth_points_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, p_depth_topic_, 1);

    cam_pcl_sync_ = new message_filters::Synchronizer<sync_pc2>(sync_pc2(10), *pcl_sub_, *depth_points_sub_); 
    cam_pcl_sync_->registerCallback(boost::bind(&CurbEdgesDetection::dataCB, this, _1, _2)); // Magic: _X up to num subs
    
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/movel_ramp_detection/cloud_merged", 1);
    segmented_pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/movel_ramp_detection/cloud",1);
    rga_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/movel_ramp_detection/rga_cloud",1);
    laser_sub_ = nh_.subscribe(p_lidar_topic_, 1, &CurbEdgesDetection::laserCB, this);
    filtered_cloud_sub_ = nh_.subscribe(filtered_topic_, 1, &CurbEdgesDetection::filteredCB, this);
    //convex_hull_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("convex_hull",1);
}

//5.0 / 180.0 * M_PI = 0.087222
void CurbEdgesDetection::setupParams()
{
    ros::param::param<int>("~inlier_th", inlier_th_, 5);
    ros::param::param<int>("~k", k_, 7);   
    ros::param::param<float>("~leaf_size", leaf_size_, 0.03);    
    ros::param::param<double>("~dist_th", p_dist_th_, 0.7);
    ros::param::param<double>("~point_th", point_th_, 0.02);
    ros::param::param<double>("~z_th", z_th_, 0.03);
    ros::param::param<bool>("~merge_cloud", merge_cloud_, true);  
    ros::param::param<bool>("~debug", debug_, false);    
    ros::param::param<std::string>("~lidar_topic", p_lidar_topic_, "/lslidar_c16/lslidar_point_cloud");    
    ros::param::param<std::string>("~depth_topic", p_depth_topic_, "/camera/depth/points");
    ros::param::param<std::string>("~filtered_topic", filtered_topic_, "/pcl_filters/psz/output");       
    ros::param::param<std::string>("~target_frame", p_target_frame_, "base_link");    
    ros::param::param<double>("~min_z", min_z_, 0.04);    
    ros::param::param<double>("~max_z", max_z_, 3.0);    
    ros::param::param<double>("~angle_th", angle_th_, 0.7);  
    ros::param::param<double>("~smooth_th", smooth_th_, 0.087222);  
    ros::param::param<double>("~curve_th", curve_th_, 0.7);
}


void CurbEdgesDetection::dataCB(
    const sensor_msgs::PointCloud2::ConstPtr& depthmsg,
    const sensor_msgs::PointCloud2::ConstPtr& pclmsg
)
{
    // merge cloud from both point cloud sources
    if (merge_cloud_)
    {
    cloud_merged_ = mergePCL(depthmsg, pclmsg);
    cloud_merged_.header.frame_id = p_target_frame_;
    cloud_merged_.header.stamp = ros::Time::now();
    pcl_pub_.publish(cloud_merged_);
    //planeSegmentation();
    }
}

void CurbEdgesDetection::laserCB(const sensor_msgs::PointCloud2::ConstPtr& lidarmsg)
{
    if (!merge_cloud_)
    {
    sensor_msgs::PointCloud2 cloud_base1;
    try
    {
        pcl_ros::transformPointCloud(p_target_frame_, *lidarmsg, cloud_base1, tf_listener_);
    }
    catch(tf::TransformException e)
    {
        ROS_ERROR("%s", e.what());
    }
    filtered_cloud_merged_ = cloud_base1;
    planeSegmentation();
    }
}

void CurbEdgesDetection::filteredCB(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (merge_cloud_)
    {
    filtered_cloud_merged_ = *msg;
    planeSegmentation();
    }
}

sensor_msgs::PointCloud2 CurbEdgesDetection::mergePCL(    
    const sensor_msgs::PointCloud2::ConstPtr& pcl_1,
    const sensor_msgs::PointCloud2::ConstPtr& pcl_2)
{
    /*
    pcl::PCLPointCloud2::Ptr temp_cloud (new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr filtered_temp_cloud (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr down_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 cloud_1, cloud_2;
    //pcl_conversions::toPCL(*pcl_1, *temp_cloud);
    // Create the filtering object: downsample the dataset using a leaf size of 2cm
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (temp_cloud);
    sor.setLeafSize (0.02f, 0.02f, 0.02f);
    sor.filter (*filtered_temp_cloud);
    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2(*filtered_temp_cloud, *down_cloud);
    toROSMsg(*down_cloud, cloud_2);
    */
    // Merge Depth Points and 3D Lidar PCL
    sensor_msgs::PointCloud2 cloud_base1, cloud_base2;
    try
    {
        pcl_ros::transformPointCloud(p_target_frame_, *pcl_1, cloud_base1, tf_listener_);
        pcl_ros::transformPointCloud(p_target_frame_, *pcl_2, cloud_base2, tf_listener_);
    }
    catch(tf::TransformException e)
    {
        ROS_ERROR("%s", e.what());
    }
    sensor_msgs::PointCloud2 cloud_xyz1, cloud_xyz2;
    stripUnimportantFields(cloud_base1, cloud_xyz1);
    stripUnimportantFields(cloud_base2, cloud_xyz2);

    sensor_msgs::PointCloud2 merged;
    pcl::concatenatePointCloud(cloud_xyz1, cloud_xyz2, merged);
    return merged;

}

void CurbEdgesDetection::planeSegmentation()
{
    // Printout the size of the pointcloud data
    // std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;
    pcl::PCLPointCloud2::Ptr temp_cloud (new pcl::PCLPointCloud2), filtered_temp_cloud (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_initial (new pcl::PointCloud<pcl::PointXYZ>), cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>), cloud_segmented (new pcl::PointCloud<pcl::PointXYZ>), cloud_done (new pcl::PointCloud<pcl::PointXYZ>);
    
  
    pcl_conversions::toPCL(filtered_cloud_merged_, *temp_cloud);
    pcl::fromPCLPointCloud2(*temp_cloud, *cloud_initial);
    
    // Filter out the too low and the too high
    pcl::PassThrough<pcl::PointXYZ> passthrough;
    passthrough.setInputCloud(cloud_initial);
    passthrough.setFilterFieldName("z");
    passthrough.setFilterLimits(min_z_, max_z_);
    passthrough.filter(*cloud_filtered);
    pcl::toPCLPointCloud2 (*cloud_filtered, *temp_cloud);
    
    // Create the filtering object: downsample the dataset using a leaf size 
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (temp_cloud);
    sor.setLeafSize (leaf_size_, leaf_size_, leaf_size_);
    sor.filter (*filtered_temp_cloud);

    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2 (*filtered_temp_cloud, *cloud_filtered);
    /*
    //Filter based on radius outliers
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(cloud_filtered);
    outrem.setRadiusSearch(0.8);
    outrem.setMinNeighborsInRadius (2);
    outrem.setKeepOrganized(true);
    // apply filter
    outrem.filter (*cloud_filtered);
    */
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud_filtered);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);

    // Compute the features
    ne.compute (*cloud_normals);
    
    // Implement Region Growing Algorithm
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (50);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (cloud_filtered);
    reg.setInputNormals (cloud_normals);
    reg.setSmoothnessThreshold (smooth_th_);
    reg.setCurvatureThreshold (curve_th_);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    //std::cout << "Number of clusters: " << clusters.size () << std::endl;
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    
    if (debug_){
        sensor_msgs::PointCloud2 rga_cloud;
        pcl::toROSMsg(*colored_cloud, rga_cloud);
        rga_cloud.header.frame_id = "base_link";    
        rga_pub_.publish(rga_cloud);
    }
    
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setNumberOfThreads(4);
    // Optional
    seg.setOptimizeCoefficients (true);
    
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (p_dist_th_);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());       // Storing model coefficients
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());                      // Storing all inlers indices
    int nr_points = (int) colored_cloud->size ();
    copyPointCloud(*colored_cloud, *cloud_f);

    for (int a = 0; a < clusters.size(); a++)
    {  
        // Segment the largest planar component from the remaining cloud
       
        seg.setInputCloud (cloud_f);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () < inlier_th_)
        {
        std::cerr << "Could not estimate a planar model with <= "<< inlier_th_ <<" points for the given dataset." << std::endl;
        break;
        }

        // Extract the inliers
        extract.setInputCloud (cloud_f);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);
        copyPointCloud(*cloud_p, *cloud_filtered);
         cv::Vec3f normalVector;
         normalVector[0] = coefficients->values[0];
         normalVector[1] = coefficients->values[1];
         normalVector[2] = coefficients->values[2];
        double slope = convertNormVectorToSlope(normalVector);
        if (fabs(slope) >= angle_th_)
        {
            pcl::PointCloud<pcl::PointXYZ> edges = planarEdgeExtractionV2(cloud_filtered, M_PI_2);
            *cloud_segmented += edges;
        }
        else if  (fabs(slope) < angle_th_)
        {
            pcl::PointCloud<pcl::PointXYZ> edges = planarEdgeExtractionV2(cloud_filtered, M_PI_2);
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            kdtree.setInputCloud (cloud_f);

            pcl::PointXYZ searchPoint = edges.points[0];
           // Neighbors within radius search

            std::vector<int> pointIdxKSearch(k_);
            std::vector<float> pointKSquaredDistance(k_);
            unsigned int count = 0;
 
            if ( kdtree.nearestKSearch (searchPoint, k_, pointIdxKSearch, pointKSquaredDistance) > 0 )
            {
            pcl::PointXYZ prev_pt = searchPoint;
            for (std::size_t i = 0; i < pointIdxKSearch.size (); ++i)
            {       
            pcl::PointXYZ actual_point =  (*cloud_p)[ pointIdxKSearch[i] ];
            float diff_z = fabs(actual_point.z - prev_pt.z);
            if (diff_z >= z_th_) //if diff_z is significant, increment count
                {
                    ++count;
                }
            prev_pt = actual_point;
            }

            }
            unsigned int num = 0.5*k_;
            if (count >= num) //assume planes with higher counts are curbs
                {*cloud_segmented += edges;}
        } 
    
    /*
    pcl::ConvexHull<pcl::PointXYZ> cHull;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cHull_points (new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<pcl::Vertices> polygons_alpha;
    cHull.setComputeAreaVolume(true);
    cHull.setInputCloud(cloud_p);
    cHull.reconstruct(*cHull_points, polygons_alpha);

    pcl::PointCloud<pcl::PointXYZ>::Ptr OnePolygon (new pcl::PointCloud<pcl::PointXYZ>);

    for (int ii=0; ii<4; ii++){
    OnePolygon->points.push_back(cHull_points->points[polygons_alpha[0].vertices[ii]]);
    } 
    sensor_msgs::PointCloud2 convex_hull;
    pcl::toROSMsg(*cHull_points, convex_hull);
    convex_hull.header.frame_id = "base_link";
    convex_hull_pub_.publish(convex_hull);
    std::vector<cv::Point2f> points;
    for (const auto& point : *cHull_points) {
      points.push_back(cv::Point2f(point.x, point.y));
    }
    cv::RotatedRect rectangle = cv::minAreaRect(points);
    ROS_INFO_STREAM("cv::rectangle's center: " << rectangle.center);
    ROS_INFO_STREAM("cv::rectangle's width: " << rectangle.size.width);
    ROS_INFO_STREAM("cv::rectangle's height: " << rectangle.size.height);
    //Eigen::Vector4f centroid;
    //pcl::compute3DCentroid (*cloud_segmented, centroid);
    pcl::PointXYZ min_pt, max_pt;
    pcl::PointCloud<pcl::PointXYZ> cloud = *cHull_points;
    pcl::getMinMax3D(cloud, min_pt, max_pt);
    ROS_INFO_STREAM("min_pt: " << min_pt << "max_pt: " << max_pt); */
   
        extract.setNegative (true);
        extract.filter (*cloud_p);
        cloud_f.swap (cloud_p);
    }
    sensor_msgs::PointCloud2 seg_cloud;
    pcl::toROSMsg(*cloud_segmented, seg_cloud);
    seg_cloud.header.frame_id = "base_link_flattened";
    seg_cloud.header.stamp = ros::Time::now();
    segmented_pcl_pub_.publish(seg_cloud);
}

double CurbEdgesDetection::convertNormVectorToSlope(cv::Vec3f normalVector)
{
  cv::Vec3f zAxisVector = {0, 0, 1};
    double num =  zAxisVector[0]*normalVector[0] + 
                  zAxisVector[1]*normalVector[1] + 
                  zAxisVector[2]*normalVector[2];
    double den =  sqrt(normalVector[0]*normalVector[0] +
                       normalVector[1]*normalVector[1] +
                       normalVector[2]*normalVector[2]);
    double angle = acos(num/den); //radians
    return angle;
}


pcl::PointCloud<pcl::PointXYZ> CurbEdgesDetection::planarEdgeExtractionV2(pcl::PointCloud<pcl::PointXYZ>::Ptr &targetCloud, double minInteriorAngleRad)
{
        pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
        kdTree.setInputCloud(targetCloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr edgePts(new pcl::PointCloud<pcl::PointXYZ>);

        for (int i = 0; i < targetCloud->points.size(); i++) //loops through each point in input cloud
        {
                std::vector<int> indicies;
                std::vector<float> distances;
                std::vector<Eigen::Vector3f> ptDirections;
                pcl::PointXYZ tp = targetCloud->points.at(i);
                Eigen::Vector3d referenceDirection = Eigen::Vector3d(0, 0, 0);
                Eigen::Vector3d z_vector = Eigen::Vector3d(0, 0, 1); //z axis
                Eigen::Vector3d angle_plane;
                Eigen::Matrix3d m;
                double averageAngle = 0;
                double count = 0;
                double unsigned_angle, signed_angle, axis_similarity;

                kdTree.nearestKSearch(i, 24, indicies, distances); //k = 24
                for (int ii = 0; ii < indicies.size(); ii++) //loop thru each neighbor found
                {
                        if (i != indicies.at(ii))  //ascertain that it is a neighbor pt, not the pt itself
                        {
                                pcl::PointXYZ cp = targetCloud->points.at(indicies.at(ii));
                                Eigen::Vector3d currentDirection = Eigen::Vector3d(tp.x - cp.x, tp.y - cp.y, tp.z - cp.z);

                                if (referenceDirection.norm() == 0 && currentDirection.norm() != 0)
                                {referenceDirection = currentDirection;}

                                if (currentDirection.norm() != 0)
                                {
                                        unsigned_angle = angleBetweenVectors(referenceDirection, currentDirection);
                                        angle_plane = referenceDirection.cross(currentDirection);
                                        axis_similarity = angleBetweenVectors(z_vector, angle_plane);
                                        if (axis_similarity < M_PI_2)
                                        {signed_angle = unsigned_angle;}
                                        else
                                        {signed_angle = -unsigned_angle;}
                                        averageAngle += signed_angle;     
                                        count += 1;
                                }
                        }
                }
                averageAngle = averageAngle / count;

                //apply correction rotation to reference vector to place it at the directional average
                
                m = Eigen::AngleAxisd(averageAngle, z_vector);
                
                Eigen::Vector3d correctRefDirection = m*referenceDirection;
                correctRefDirection.normalize();

                double maxAngle = -PI;
                double minAngle = PI;
                indicies.clear();
                distances.clear();

                kdTree.nearestKSearch(i, 24, indicies, distances);
                for (int ii = 0; ii < indicies.size(); ii++)
                {
                        if (i != indicies.at(ii))
                        {
                                pcl::PointXYZ cp = targetCloud->points.at(indicies.at(ii));
                                Eigen::Vector3d currentDirection = Eigen::Vector3d(tp.x - cp.x, tp.y - cp.y, tp.z - cp.z);

                                if (currentDirection.norm() != 0)
                                {       
                                        unsigned_angle = angleBetweenVectors(correctRefDirection, currentDirection);
                                        angle_plane = correctRefDirection.cross(currentDirection);
                                        axis_similarity = angleBetweenVectors(z_vector, angle_plane);
                    
                                        if (axis_similarity < M_PI_2)
                                        {signed_angle = unsigned_angle;}
                                        else
                                        {signed_angle = -unsigned_angle;}
                                        if (signed_angle > maxAngle) { maxAngle = signed_angle; }
                                        if (signed_angle < minAngle) { minAngle = signed_angle; }
                                }
                        }
                }
                if ((2.0*M_PI - (maxAngle - minAngle)) > minInteriorAngleRad) {edgePts->points.push_back(tp);}
        }
        return *edgePts;
}

void CurbEdgesDetection::stripUnimportantFields(const sensor_msgs::PointCloud2& cloud_in, sensor_msgs::PointCloud2& cloud_out)
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

double CurbEdgesDetection::angleBetweenVectors(Eigen::Vector3d a, Eigen::Vector3d b) 
{
    double angle = 0.0;
    angle = std::atan2(a.cross(b).norm(), a.dot(b));
    return angle;
}
