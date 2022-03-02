#ifndef CURB_EDGES_DETECTION_H_
#define CURB_EDGES_DETECTION_H_
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "tf/message_filter.h"
#include "tf/transform_listener.h"
#include "filters/filter_chain.hpp"
#include <Eigen/Dense> 
#include <Eigen/Geometry> 
#include <ros_utils/ros_utils.h>
#include <iostream>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include "opencv2/core/types.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/mat.hpp"

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <movel_hasp_vendor/license.h>

class CurbEdgesDetection
{
public:
    CurbEdgesDetection();
    ~CurbEdgesDetection(){};

    void setupTopics();

    void dataCB(
        const sensor_msgs::PointCloud2::ConstPtr& depthmsg,
        const sensor_msgs::PointCloud2::ConstPtr& pclmsg);

    sensor_msgs::PointCloud2 mergePCL(   
        const sensor_msgs::PointCloud2::ConstPtr& pcl_1,
        const sensor_msgs::PointCloud2::ConstPtr& pcl_2);
    void laserCB(const sensor_msgs::PointCloud2::ConstPtr& lidarmsg);
    void filteredCB(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void stripUnimportantFields(const sensor_msgs::PointCloud2& cloud_in, sensor_msgs::PointCloud2& cloud_out);
    void setupParams();
    void planeSegmentation();
    double convertNormVectorToSlope(cv::Vec3f normalVector);
    pcl::PointCloud<pcl::PointXYZ> planarEdgeExtractionV2(pcl::PointCloud<pcl::PointXYZ>::Ptr &targetCloud, double minInteriorAngleRad);
    double angleBetweenVectors(Eigen::Vector3d a, Eigen::Vector3d b);

protected:

    ros::NodeHandle nh_;
    
    message_filters::Subscriber<sensor_msgs::PointCloud2> *depth_points_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *pcl_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> sync_pc2;
    message_filters::Synchronizer<sync_pc2> *cam_pcl_sync_;
    ros::Publisher segmented_pcl_pub_;
    ros::Publisher rga_pub_;
    //ros::Publisher convex_hull_pub_;
    ros::Publisher pcl_pub_;
    ros::Subscriber laser_sub_;
    ros::Subscriber filtered_cloud_sub_;
    float leaf_size_;
    double p_dist_th_;
    double angle_th_;
    double z_th_;
    double point_th_;
    std::string p_target_frame_;
    std::string p_lidar_topic_;
    std::string p_depth_topic_;
    std::string filtered_topic_;
    tf::TransformListener tf_listener_;
    sensor_msgs::PointCloud2 cloud_merged_;
    sensor_msgs::PointCloud2 filtered_cloud_merged_;
    int inlier_th_;
    double min_z_;
    double max_z_;
    double smooth_th_;
    double curve_th_;
    int k_;
    bool merge_cloud_;
    bool debug_;
    int max_curb_pts_;
};

#endif
