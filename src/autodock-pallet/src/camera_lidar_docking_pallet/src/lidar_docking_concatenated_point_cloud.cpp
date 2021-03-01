//Include Libraries
#include "lidar_docking_/LidarDocking.hpp"
#include <cameralidarDock/camera_lidar_docking.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


namespace lidar_docking {


/**
 * @brief LidarDocking::~LidarDocking contructor
 */
LidarDocking::~LidarDocking()
{

}
/*
    Function for Camera + Navigation based docking

    - readParams            : load parameters
    - goto_docking_position : take the robot to the start position i.e from where the docking starts

*/


/**
 * @brief lidar_docking::LidarDocking::readParams reading parameters
 * @param nodeHandle
 */
void lidar_docking::LidarDocking::readParams(ros::NodeHandle& nodeHandle)
{
    nodeHandle.getParam("/camera_lidar_docking_pallet/resolution", res);
    nodeHandle.getParam("/camera_lidar_docking_pallet/topic_name", topic_name_);
    nodeHandle.getParam("/camera_lidar_docking_pallet/pallet_intensity", pallet_intensity_);
    nodeHandle.getParam("/camera_lidar_docking_pallet/makrer_intensity_threshold", intensity_threshold_);
    nodeHandle.getParam("/camera_lidar_docking_pallet/max_delta_error_slope", max_delta_error_slope_);
    nodeHandle.getParam("/camera_lidar_docking_pallet/min_points_obstacle", min_points_obstacle_);
    nodeHandle.getParam("/camera_lidar_docking_pallet/kp", kp_);
    nodeHandle.getParam("/camera_lidar_docking_pallet/kd", kd_);
    nodeHandle.getParam("/camera_lidar_docking_pallet/cluster_depth_threshold", cluster_depth_threshold_);
    nodeHandle.getParam("/camera_lidar_docking_pallet/pallet_length", pallet_length_);
    nodeHandle.getParam("/camera_lidar_docking_pallet/kp_orientation", kp_orientation_);
    nodeHandle.getParam("/camera_lidar_docking_pallet/kd_orientation", kd_orientation_);
    nodeHandle.getParam("/camera_lidar_docking_pallet/kp_movement", kp_movement_);
    nodeHandle.getParam("/camera_lidar_docking_pallet/kd_movement", kd_movement_);
    nodeHandle.getParam("/camera_lidar_docking_pallet/pallet_orientation_angle", pallet_orientation_angle_);
    nodeHandle.getParam("/camera_lidar_docking_pallet/max_linear_speed", max_linear_speed_);
    nodeHandle.getParam("/camera_lidar_docking_pallet/max_angular_speed", max_angular_speed_);

}


/**
 * @brief lidar_docking::LidarDocking::rotate_and_check of marker not found, rotate and robot
 * and search for marker
 */
/* ------------------ Lidar-based-docking ------------------ */
void lidar_docking::LidarDocking::start_recovery()
{
    //wait for the action server to come up
    while(!nav_ac_ptr_->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    if(go_for_pickup_)
    {
        goal.target_pose.pose.position.x = pickup_x_;
        goal.target_pose.pose.position.y = pickup_y_;
        goal.target_pose.pose.orientation.z = pickup_theta_;
        goal.target_pose.pose.orientation.w = 1;//1.0;
    }
    if(go_for_dropping_)
    {
        goal.target_pose.pose.position.x = dropping_x_;
        goal.target_pose.pose.position.y = dropping_y_;
        goal.target_pose.pose.orientation.z = dropping_theta_;
        goal.target_pose.pose.orientation.w = 1.0;
    }
    nav_ac_ptr_->sendGoal(goal);

    nav_ac_ptr_->waitForResult();
    std::cout<<"Done. recovery done "<<std::endl;
}


/**
 * @brief LidarDocking::move - move function to move the robot
 * @param linear_speed
 * @param angular_speed
 */
void lidar_docking::LidarDocking::move(float linear_speed, float angular_speed)
{
    ptr.linear.x = linear_speed;
    ptr.linear.y = 0;
    ptr.linear.z = 0;
    ptr.angular.x = 0;
    ptr.angular.y = 0;
    ptr.angular.z = angular_speed;

    pub.publish(ptr);
}


/**
 * @brief LidarDocking::find_length - finding length of the cluster
 * @param cluster
 * @return
 */
/* ------------------ Lidar-based-docking ------------------ */
float LidarDocking::find_length(std::vector<pcl::PointXYZI> cluster)
{

    float length = (cluster[0].x - cluster[cluster.size()-1].x)*(cluster[0].x - cluster[cluster.size()-1].x) +
            (cluster[0].y - cluster[cluster.size()-1].y)*(cluster[0].y - cluster[cluster.size()-1].y);
    //    std::cout<<"cluster size :"<<cluster.size()<<" ";
    //    std::cout<<"length of cluster is "<<sqrt(length)<<" ";
    //    std::cout<<"mean x :" << (cluster[0].x + cluster[cluster.size()-1].x)/2 <<" ";
    //    std::cout<<"mean y :" << (cluster[0].y + cluster[cluster.size()-1].y)/2 <<std::endl;

    return sqrt(length);

}


/**
 * @brief LidarDocking::computeMean - computing mean value of the array
 * @param arr
 * @return
 */
/* ------------------ Lidar-based-docking ------------------ */
int LidarDocking::computeMean(std::vector<int> arr)
{
    return arr[arr.size()/2];
}


/**
 * @brief lidar_docking::LidarDocking::correct_pose
 * orient the robot wrt the legs of the pallet
 * @param centroid_x
 * @param centroid_y
 */
/* ------------------ Lidar-based-docking ------------------ */
void lidar_docking::LidarDocking::correct_pose(float centroid_x, float centroid_y)
{
    // centroid wrt base_link negative we need to go right else left
    static float prev_centroid_y = 0;                        //centroid_y should be 0
    static float prev_max_deviation = 0;
    std::cout<<"Angle: "<<angle_<<std::endl;
    float  max_deviation = pallet_orientation_angle_ - angle_ ;   //-170    angle comes out be negative

    if (max_deviation < -180)    // i.e starts from -181
    {
        max_deviation = abs(max_deviation + 180);
    }
    //    std::cout<<"centroid y is "<<centroid_y<<std::endl;            // for debugging
    //    std::cout<<"max deviation is "<<max_deviation<<std::endl;     // for debugging

    if(abs(centroid_x)  < 0.2 and(centroid_y > 2 || centroid_y < -2))
    {
        std::cout<<"deviation out of limits. Trying to reorient "<<std::endl;
        start_recovery();
    }

    //            move(0,0);     // for debugging
    if((centroid_y > 0.1))   //0.1
    {
//        std::cout<<"rotaitng left"<<std::endl;

        float angular_speed = (kp_movement_)*abs(centroid_y) + kd_movement_*(centroid_y - prev_centroid_y);

        if(angular_speed > max_angular_speed_)                             // max angular speed - 0.1
            angular_speed = max_angular_speed_;

        move(max_linear_speed_, angular_speed);             //0.05   // +ve rotation    0.1 used
    }

    else if(centroid_y <  -0.1)   // -0.1
    {
        //        std::cout<<"rotating right"<<std::endl;     // debugging
        float angular_speed = (kp_movement_)*(centroid_y) + kd_movement_*(centroid_y - prev_centroid_y);

        if(angular_speed < -max_angular_speed_)
            angular_speed = -max_angular_speed_;

        move(max_linear_speed_, angular_speed);   //max_linear_speed- 0.05
    }

    else
    {
        std::cout<<"max deviation computed "<< max_deviation <<std::endl;

        if(max_deviation <  -5 && max_deviation >  -10)   // go right
        {
            //            std::cout<<"rotating right for proper orientation "<<std::endl;
            float angular_speed = kp_orientation_*(max_deviation) + kd_orientation_*(max_deviation - prev_max_deviation);

            if(angular_speed < -max_angular_speed_ / 3)
                angular_speed = -max_angular_speed_ / 3;

            move(max_linear_speed_, -max_angular_speed_ / 3);
        }
        else if(max_deviation > 5 && max_deviation < 10)   // go left
        {
            //            std::cout<<"rotating left for proper orientation "<<std::endl;
            float angular_speed = kp_orientation_*(max_deviation) + kd_orientation_*(max_deviation - prev_max_deviation);

            if(angular_speed > max_angular_speed_ / 3)
                angular_speed = max_angular_speed_ / 3;

            move(max_linear_speed_, max_angular_speed_ / 3);
        }
        else if(max_deviation <  -10)   // go right
        {
            move(0.0, -max_angular_speed_ / 2);
        }
        else if(max_deviation > 10)   // go left
        {
            move(0.0, max_angular_speed_ / 2);
        }
        else
        {
//            std::cout<<"not rotating"<<std::endl;
            move(max_linear_speed_, 0);
        }
    }

    prev_centroid_y = centroid_y;
    prev_max_deviation = max_deviation;

}

/**
 * @brief lidar_docking::LidarDocking::goto_docking_position take the robot to the start position i.e from where the docking starts
 */
void lidar_docking::LidarDocking::goto_docking_position()
{
    search_for_markers_pub_.publish(search_for_markers_);
    tf::TransformListener listener;
    tf::StampedTransform transform;
    float transform_found = false;
    while(!transform_found)
    {
        try
        {
            // listen to the odom-object tf
            listener.waitForTransform("odom", "object", ros::Time(0), ros::Duration(60.0) );
            listener.lookupTransform("odom", "object", ros::Time(0), transform);
            transform_found = true;
        } catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
        }
    }

    transform_found = false;
    tf::TransformListener listener_base;
    tf::StampedTransform transform_base;

    tf::Quaternion q(
                transform.getRotation().x(),
                transform.getRotation().y(),
                transform.getRotation().z(),
                transform.getRotation().w());

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    std::cout<<"roll received "<<roll<<std::endl;

    //wait for the action server to come up
    while(!nav_ac_ptr_->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    float threshold = 2;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    std::cout<<"goal is when yaw is "<<yaw<<" : "<<  transform.getOrigin().x() - cos(yaw)*threshold<<" "<< transform.getOrigin().y() - sin(yaw)*threshold<<std::endl;
    pallet_point_x = (transform.getOrigin().x()) + cos(yaw)*pallet_length_/2;
    pallet_point_y = (transform.getOrigin().y()) + sin(yaw)* pallet_length_/2;

    goal.target_pose.pose.position.x = transform.getOrigin().x() - cos(yaw)*threshold;
    goal.target_pose.pose.position.y = transform.getOrigin().y() - sin(yaw)*threshold;

    if(roll < 0.1)
    {
        std::cout<<"-------front -----------------"<<std::endl;
    }
    else
    {
        std::cout<<"-------rear -----------------"<<std::endl;
    }

    goal.target_pose.pose.orientation.z = yaw;
    goal.target_pose.pose.orientation.w = 1.0;
    std::cout<<" mid points of pallet "<<pallet_point_x <<" "<<pallet_point_y<<std::endl;

    // Move the robot to the desired position
    std::cout<<"Aligning the robot"<<std::endl;
    search_for_markers_.data = false;

    navigationLoop(goal);
    if (!cancel_dock_){
        std::cout<<"Aligned"<<std::endl;
        ready_for_docking_ = true;      //is ready for docking
    }
}


/**
 * @brief lidar_docking::LidarDocking::lineFit
 * Line fitting for pose computation in terms orientation (theta)
 * @param cloud
 * @return
 */
/* ------------------ Lidar-based-docking ------------------ */
float lidar_docking::LidarDocking::lineFit(pcl::PointCloud<pcl::PointXYZI> cloud)
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_LINE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.1);
    seg.setInputCloud (cloud.makeShared());
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a LINE model for the given dataset.");
    }
    float angle=2*asin(coefficients->values[4])*180.0/M_PI;//formulae for quarternions.
    //        std::cout <<"deviation from the pallet: "<<angle<<std::endl;
    angle_  = angle;
    //    return angle;

}


/**
 * @brief lidar_docking::LidarDocking::compute_centroid
 * computing centroid along one axis
 * @param cloud
 * @param axis
 * @return
 */
/* ------------------ Lidar-based-docking ------------------ */
float  lidar_docking::LidarDocking::compute_centroid(pcl::PointCloud<pcl::PointXYZI> cloud, string axis)
{
    float sum = 0;
    for(int i = 0; i < cloud.size(); i++)
    {
        if(axis == "x")
        {
            sum+=cloud.points[i].x;
        }
        else if(axis =="y")
        {
            sum+=cloud.points[i].y;
        }
        else if(axis =="z")
        {
            sum+=cloud.points[i].z;
        }
    }

    return sum / cloud.size();
}


/**
 * @brief lidar_docking::LidarDocking::minimum_depth
 * finding minimum depth of a cluster so that an array of depth of clusters can be formed
 * @param cluster
 * @return
 */
/* ------------------ Lidar-based-docking ------------------ */
float lidar_docking::LidarDocking::minimum_depth(std::vector<pcl::PointXYZI> cluster)
{
    float min_depth = 3;
    for(int i = 0; i < cluster.size(); i++)
    {
        if (cluster[i].x < min_depth and cluster[i].x > 0.1)
        {
            min_depth = cluster[i].x;
        }
    }
    return min_depth;
}


/**
 * @brief lidar_docking::LidarDocking::minimum
 * finding the index of minimum depth in a vector or finding the indeex of smalles number in a vector
 * @param depth
 * @return
 */
/* ------------------ Lidar-based-docking ------------------ */
int lidar_docking::LidarDocking::minimum(vector<float> depth)
{
    int index = -1;
    float minimum_depth = 3;
    for(int i = 0;  i< depth.size(); i++)
    {
        if(depth[i] < minimum_depth)
        {
            index = i;
        }
    }

    if(index == -1)
    {
        std::cout<<"index woth minimum depth not found "<<std::endl;
        //        exit(0);
    }
    return index;

}


/**
 * @brief lidar_docking::LidarDocking::maximum
 * finding maximum point in the point cloud on the basis of axis
 * @param cloud
 * @param axis
 * @return
 */
/* ------------------ Lidar-based-docking ------------------ */
float lidar_docking::LidarDocking::maximum(pcl::PointCloud<pcl::PointXYZI> cloud, string axis)
{
    float max_range =  -10;
    for(int i = 0;  i< cloud.size(); i++)
    {
        if(axis == "x")
        {
            if (cloud.points[i].x > max_range)
            {
                max_range = cloud.points[i].x;
            }
        }

        if(axis == "y")
        {
            if (cloud.points[i].y > max_range)
            {
                max_range = cloud.points[i].y;
            }
        }

    }

    return max_range;

}


/**
 * @brief lidar_docking::LidarDocking::visualize_clusters
 * function to visualize all clusters wrt baselink
 * @param all_clusters
 */
/* ------------------ Lidar-based-docking ------------------ */
void lidar_docking::LidarDocking::visualize_clusters(vector<vector<pcl::PointXYZI> > all_clusters)
{
    pcl::PointCloud<pcl::PointXYZI> cloud;
    for(int j = 0; j < all_clusters.size(); j++)
    {
        std::vector<pcl::PointXYZI>closest_cluster  = all_clusters[j];
        for(int i = 0; i < closest_cluster.size();i++)
        {
            pcl::PointXYZI pt;
            pt.x = closest_cluster[i].x;
            pt.y = closest_cluster[i].y;
            pt.z = closest_cluster[i].z;
            pt.intensity = closest_cluster[i].intensity;
            cloud.push_back(pt);
        }
    }

    sensor_msgs::PointCloud2 msg2;
    pcl::toROSMsg(cloud, msg2);
    msg2.header.stamp = ros::Time::now();
    msg2.header.frame_id= "/base_link";

    cloud_pub_.publish(msg2);
}


/**
 * @brief lidar_docking::LidarDocking::compute_allcluster_centroids
 * compute centroid along x and y for all clusters and return  clusters along the x axis wrt robot
 * @param all_clusters
 * @return
 */
/* ------------------ Lidar-based-docking ------------------ */
vector<pair<float, int> > lidar_docking::LidarDocking::compute_allcluster_centroids(vector<vector<pcl::PointXYZI>> all_clusters)
{
    vector< pair <float, int> > vect_centroids;
    for(int j = 0; j < all_clusters.size(); j++)
    {

        float x_centroid = compute_centroid(cluster_to_point_cloud(all_clusters[j]), "x");
        float y_centroid = compute_centroid(cluster_to_point_cloud(all_clusters[j]), "y");
        //        std::cout<<"x_ centoird "<<x_centroid <<" y cent : "<<y_centroid<<std::endl;
        float dist_centroids = sqrt(x_centroid*x_centroid + y_centroid*y_centroid);
        //        if(x_centroid > 0)
        {
            vect_centroids.push_back(make_pair(x_centroid, j)); //vector of x_centroids and subsequent indices.
        }
    }

    return vect_centroids;
}


/**
 * @brief lidar_docking::LidarDocking::minimum
 * finding minimum point in the point cloud on the bais if axis
 * @param cloud
 * @param axis
 * @return
 */
/* ------------------ Lidar-based-docking ------------------ */
float lidar_docking::LidarDocking::minimum(pcl::PointCloud<pcl::PointXYZI> cloud, string axis)
{
    float min_range =  10;
    for(int i = 0;  i< cloud.size(); i++)
    {
        if(axis == "x")
        {
            if (cloud.points[i].x < min_range)
            {
                min_range = cloud.points[i].x;
            }
        }

        if(axis == "y")
        {
            if (cloud.points[i].y < min_range)
            {
                min_range = cloud.points[i].y;
            }
        }

    }
    return min_range;

}


/**
 * @brief lidar_docking::LidarDocking::compute_closest_points
 *compute closest points in the cluster
 * @param all_clusters
 * @param minimum_depth
 * @return
 */
/* ------------------ Lidar-based-docking ------------------ */
vector<pcl::PointXYZI>lidar_docking::LidarDocking::compute_closest_points(std::vector<std::vector<pcl::PointXYZI> > all_clusters, float minimum_depth)
{
    vector<pcl::PointXYZI> closest_points;
    for(int j = 0; j < all_clusters.size(); j++)
    {
        std::vector<pcl::PointXYZI>closest_cluster  = all_clusters[j];
        for(int i = 0; i < closest_cluster.size();i++)
        {
            if(closest_cluster[i].x < minimum_depth + cluster_depth_threshold_ and closest_cluster[i].x > minimum_depth - cluster_depth_threshold_)
            {
                closest_points.push_back(closest_cluster[i]);
            }
        }
    }
    //    std::cout<<"no of closest points "<<closest_points.size();
    return closest_points;
}


/**
 * @brief lidar_docking::LidarDocking::distance_based_clustering
 * clusterng based on eucledian distance between points
 * @param cloud
 * @return
 */
/* ------------------ Lidar-based-docking ------------------ */
std::vector<std::vector<pcl::PointXYZI> > lidar_docking::LidarDocking::distance_based_clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    cloud_cluster = cloud;

    std::vector<std::vector<pcl::PointXYZI> > all_clusters;
    std::vector<pcl::PointXYZI> cluster;
    for(int i = 0; i < cloud_cluster->size()-1; i++)
    {
        float distance = sqrt((cloud_cluster->points[i].x - cloud_cluster->points[i+1].x)*(cloud_cluster->points[i].x - cloud_cluster->points[i+1].x) +
                (cloud_cluster->points[i].y - (cloud_cluster->points[i+1].y))*(cloud_cluster->points[i].y - (cloud_cluster->points[i+1].y)));
        float distance_points =  sqrt((cloud_cluster->points[i].x)*(cloud_cluster->points[i].x) +
                                      (cloud_cluster->points[i].y)*(cloud_cluster->points[i].y));  // dist of points from centre of robot
        if(distance < 0.2 && cloud_cluster->points[i].intensity == pallet_intensity_ &&  distance_points < 3)
        {

            //            std::cout<<"point pushed back"<<i<<std::endl;
            cluster.push_back(cloud_cluster->points[i]);
            if(i == cloud_cluster->size()-2)
            {
                cluster.push_back(cloud_cluster->points[i+1]);
            }
        }
        else
        {
            if (cluster.size() > 0)
            {
                cluster.push_back(cloud_cluster->points[i]);
                //                std::cout<<"pushed cluster"<<std::endl;
                all_clusters.push_back(cluster);
            }
            cluster.clear();
        }

    }
    if(cluster.size() > 0){
        all_clusters.push_back(cluster);
    }

    //    std::cout<<"size of all clusters " <<all_clusters.size()<<std::endl;

    //    visualize_clusters(all_clusters);      // debugging purposes
    return all_clusters;
}


/**
 * @brief lidar_docking::LidarDocking::concatenate_clusters
 * concatenate 2 clusters and return the final concatenated cluster
 * @param cluster_a
 * @param cluster_b
 * @return
 */
/* ------------------ Lidar-based-docking ------------------ */
std::vector<pcl::PointXYZI> lidar_docking::LidarDocking::concatenate_clusters(std::vector<pcl::PointXYZI> cluster_a, std::vector<pcl::PointXYZI> cluster_b)
{
    std::vector<pcl::PointXYZI> final_cluster;
    for(int j = 0; j < cluster_a.size(); j++)
    {
        pcl::PointXYZI pt;
        pt.x = cluster_a[j].x;
        pt.y = cluster_a[j].y;
        pt.z = cluster_a[j].z;
        pt.intensity = cluster_a[j].intensity;
        final_cluster.push_back(pt);
    }

    for(int j = 0; j < cluster_b.size(); j++)
    {
        pcl::PointXYZI pt;
        pt.x = cluster_b[j].x;
        pt.y = cluster_b[j].y;
        pt.z = cluster_b[j].z;
        pt.intensity = cluster_b[j].intensity;
        final_cluster.push_back(pt);
    }

    return final_cluster;

}


/**
 * @brief lidar_docking::LidarDocking::compute_pairs\
 * finding pairs of computed clusters of the pallet legs i.e corresponding pallet leg clusters on the basis of distance
 * @param centroids
 * @return
 */
/* ------------------ Lidar-based-docking ------------------ */
std::vector<std::pair<int, int> > lidar_docking::LidarDocking::compute_pairs( vector<pair<float, int> > centroids)
{

    vector<pair<int, int> > pairs;
    for(int i = 0; i < centroids.size() - 1; i++)
    {
        if(centroids[i].first - centroids[i+1].first < 0.2 and centroids[i].first - centroids[i+1].first > -0.2)

        {
            //            std::cout<<"found pair";
            pairs.push_back(make_pair(centroids[i].second, centroids[i+1].second));
        }

    }
    return pairs;
}


/**
 * @brief lidar_docking::LidarDocking::intensityClusters_
 * @param cloud
 */
/* ------------------ Lidar-based-docking ------------------ */
void lidar_docking::LidarDocking::intensityClusters_(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{

    std::vector<std::vector<pcl::PointXYZI> > all_clusters;
    all_clusters = distance_based_clustering(cloud);
    if(all_clusters.size()>0)
    {
        vector<pair<float, int> > centroid_disctances = compute_allcluster_centroids(all_clusters);

        sort(centroid_disctances.begin(), centroid_disctances.end());
        vector<pair<int, int> > cluster_pairs = compute_pairs(centroid_disctances);
        if (cluster_pairs.size() == 0)
        {
            find_closest_cluster_centroid(centroid_disctances, all_clusters);
        }

        else
        {
            //std::cout<<"pairs are "<< cluster_pairs[i].first <<" and "<< cluster_pairs[i].second<<std::endl;
            vector<pcl::PointXYZI> concatenated_cluster = concatenate_clusters(all_clusters[ cluster_pairs[0].first], all_clusters[ cluster_pairs[0].second]);
            pcl::PointCloud<pcl::PointXYZI> nearest_cluster = cluster_to_point_cloud(concatenated_cluster);
            generate_velocity_commands(nearest_cluster);
        }
    }
}


/**
 * @brief lidar_docking::LidarDocking::cluster_to_point_cloud
 * conversion of a cluster to a point cloud
 * @param cluster
 * @return
 */
pcl::PointCloud<pcl::PointXYZI>lidar_docking::LidarDocking::cluster_to_point_cloud(vector<pcl::PointXYZI> cluster)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for(int i=0;i < cluster.size(); i++ )
    {
        pcl::PointXYZI pt;
        pt.x = cluster[i].x;
        pt.y = cluster[i].y;
        pt.z = cluster[i].z;
        pt.intensity =  cluster[i].intensity;
        point_cloud->push_back(pt);
    }
    return *point_cloud;
}

/**
 * @brief lidar_docking::LidarDocking::generate_velocity_commands
 * generate velocity and orientation commands on the basis if the closest cluster pair computed
 * @param nearest_cluster
 */
void lidar_docking::LidarDocking::generate_velocity_commands (pcl::PointCloud<pcl::PointXYZI> nearest_cluster )
{
    float centroid_y = maximum(nearest_cluster, "y") + minimum(nearest_cluster, "y");//compute_centroid(nearest_cluster, "y");
    float centroid_x = maximum(nearest_cluster, "x") + minimum(nearest_cluster, "x"); //compute_centroid(nearest_cluster, "x");
    lineFit(nearest_cluster);

    tf::TransformListener listener;
    tf::StampedTransform transform;

    try
    {
        listener.waitForTransform("base_link", "base_scan_front", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("base_link", "base_scan_front", ros::Time(0), transform);
    } catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }

    //    std::cout<<"centroiud wrt base link y: "<<centroid_y + transform.getOrigin().y()<<std::endl;
    //    std::cout<<"centroiud wrt base link X: "<<centroid_x + transform.getOrigin().x()<<std::endl;
    correct_pose(centroid_x, centroid_y);

    sensor_msgs::PointCloud2 msg2;
    pcl::toROSMsg(nearest_cluster, msg2);
    msg2.header.stamp = ros::Time::now();
    msg2.header.frame_id= "/base_link";

    cloud_pub_.publish(msg2);

}


/**
 * @brief lidar_docking::LidarDocking::find_closest_cluster_centroid
 * computing the closest cluster pair
 * @param centroids
 * @param all_clusters
 */
void lidar_docking::LidarDocking::find_closest_cluster_centroid(vector<pair<float, int> >centroids ,std::vector<std::vector<pcl::PointXYZI> > all_clusters)
{
    //    std::cout<<"number of clusters computed "<<centroids.size()<<std::endl;
    vector<pcl::PointXYZI>closest_points_in_cluster;

    //    if(centroids.size()>=2)
    {
        for(int i = 0; i <2; i++)
        {
            std::vector<pcl::PointXYZI>closest_cluster  = all_clusters[centroids[i].second];
            for(int j = 0; j < closest_cluster.size(); j++)
            {
                pcl::PointXYZI pt;
                pt.x = closest_cluster[j].x;
                pt.y = closest_cluster[j].y;
                pt.z = closest_cluster[j].z;
                pt.intensity = closest_cluster[j].intensity;
                closest_points_in_cluster.push_back(pt);
            }
        }
    }
    //    pcl::PointCloud<pcl::PointXYZI> nearest_cluster = cluster_to_point_cloud(all_clusters[cluster_with_minimum_depth]);
    pcl::PointCloud<pcl::PointXYZI> nearest_cluster = cluster_to_point_cloud(closest_points_in_cluster);

    generate_velocity_commands(nearest_cluster);
}

} /* namespace */
