#ifndef LASERSCAN_MULTI_FILTER_H_
#define LASERSCAN_MULTI_FILTER_H_

#include <Eigen/Dense>
#include "yaml-cpp/yaml.h"
#include <ros_utils/ros_utils.h>
#include <yaml_utils/yaml_utils.h>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>

#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <dynamic_reconfigure/client.h>
#include <message_filters/subscriber.h> //movel_laser_filters
#include <tf/message_filter.h> //movel_laser_filters
#include <tf/transform_listener.h> 
#include <tf/transform_broadcaster.h>
#include <filters/filter_chain.h> //movel_laser_filters

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h> 
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/io/pcd_io.h>

#include <LaserScanProfileUpdate/LaserScanProfileUpdateServer.h> //to import the ProfileUpdateServer class

class LaserScanMultiFilter
{
public:
    /**
     * Class constructor with input fields to read from the config file
     * @param input_topics_field the relative rosparam namespace for input laser scan topics
     * @param default_profile_name the default laser filter profile to load at start
     */
    LaserScanMultiFilter(std::string input_topic_field, std::string default_profile);
    ~LaserScanMultiFilter();

    /**
     * Parses the scan topics in node_name/input_topic_field and checks if only processes those present in the rosparams server
     * @param input_topics_field the relative rosparam namespace for input laser scan topics
     */
    void initScanTopics(std::string input_topic_field);

    /**
     * Initializes a pre-defined set of movel_laser_filters for each scan topic.
     * @param default_profile the default movel_laser_filters profile to initialize with
     */
    void initFilterScanTopics(std::string default_profile);

    /**
     * Converts point clouds to sensor_msgs::LaserScan and publishes them on merged_scan_publisher
     * @param points profile id to be initialized with
     * @param merged_cloud profile id to be initialized with
     * @see filterCallback 
     */
    void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud);

    /**
     * Callback for tf_filter_. This callback merges all the filtered laserscan topics into a single laserscan topic.
     * This is done by converting each of the filtered laserscan topics into pointclouds, concatenating them and 
     * then converting it back into laserscan and finally publishing it on merged_scan_publisher.
     * @param points profile id to be initialized with
     * @param merged_cloud profile id to be initialized with
     * @see pointcloud_to_laser_scan
     */
    void filterCallback(const sensor_msgs::LaserScan::ConstPtr& msg_in, const int i);
    
    /**
     * Initialize Dynamic Reconfigure clients based on the selected profile_id
     * @param profile_id profile id to be initialized with
     * @return success of obtaining config from parameter server
     */
    void initProfileUpdateServer(std::string default_profile);


    void staticTFBroadcast(const ros::TimerEvent& event);

    std::vector<std::string> input_scan_topics;
    
    ros::NodeHandle nh_;
    LaserScanProfileUpdateServer laserscanprofileupdateserver;

protected:
    ros::NodeHandle private_nh_;

    /** laser filter component**/
    std::vector<filters::FilterChain<sensor_msgs::LaserScan>*> filter_chain_;
    std::vector<std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> > scan_subs;
    std::vector<tf::TransformListener*> tf_;
    std::vector<tf::MessageFilter<sensor_msgs::LaserScan>*> tf_filter_;
    std::vector<sensor_msgs::LaserScan> msg_;
    std::vector<ros::Publisher> scan_publishers;
    std::vector<std::string> scan_destination_topics;

    /** laser merger component **/
    tf::TransformListener tfListener_;
    laser_geometry::LaserProjection projector_;
    std::vector<pcl::PCLPointCloud2> clouds;
    std::vector<bool> clouds_modified;
    ros::Publisher merged_scan_publisher_;

    tf::TransformBroadcaster static_br;
    std::vector<double> tf_array;
    ros::Timer poller;
    
    
    double angle_min;
    double angle_max;
    double angle_increment;
    double time_increment;
    double scan_time;
    double range_min;
    double range_max;
    double tf_filter_tolerance_;
    std::string tf_message_filter_target_frame;
    std::string merged_destination_topic;
    std::string destination_frame;
    std::string parent_frame;
    std::string laserscan_topics;
};

LaserScanMultiFilter::LaserScanMultiFilter(std::string input_topic_field, std::string default_profile): 
    private_nh_("~")
{
    private_nh_.param<std::string>("merged_destination_topic", merged_destination_topic, "/scan_multi");
    private_nh_.param<std::string>("laserscan_topics", laserscan_topics, "");

    private_nh_.param<std::string>("destination_frame", destination_frame, "laser_merged");
    private_nh_.param<std::string>("parent_frame", parent_frame, "base_link");
    private_nh_.getParam("transform", tf_array); 

    private_nh_.param("angle_min", angle_min, -3.14159);
    private_nh_.param("angle_max", angle_max, 3.14159);
    private_nh_.param("angle_increment", angle_increment, 0.0058);
    private_nh_.param("time_increment", time_increment, 0.0);
    private_nh_.param("scan_time", scan_time, 0.0333333);
    private_nh_.param("range_min", range_min, 0.25);
    private_nh_.param("range_max", range_max, 25.0);

    this->initScanTopics(input_topic_field);
    this->initFilterScanTopics(default_profile);
    this->initProfileUpdateServer(default_profile);

    merged_scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan> (merged_destination_topic.c_str(), 1, false);

    //Advertise frame transform
    poller = private_nh_.createTimer(ros::Duration(0.1), boost::bind(&LaserScanMultiFilter::staticTFBroadcast, this, _1));
}

void LaserScanMultiFilter::staticTFBroadcast(const ros::TimerEvent& event){
    tf::Transform static_tf;
    static_tf.setOrigin( tf::Vector3(tf_array[0], tf_array[1], tf_array[2]));
    tf::Quaternion q;
    q.setRPY(tf_array[3], tf_array[4], tf_array[5]);
    static_tf.setRotation(q); 
    static_br.sendTransform(tf::StampedTransform(static_tf, ros::Time::now(), parent_frame, destination_frame ));
}

LaserScanMultiFilter::~LaserScanMultiFilter()
{
    for (int i = 0; i < tf_filter_.size(); ++i)
        delete tf_filter_[i];
    for (int i = 0; i < tf_.size(); ++i)
        delete tf_[i];
}

void LaserScanMultiFilter::initScanTopics(std::string input_topic_field)
{
    // Retrieve topics from rostopic server
    ros::master::V_TopicInfo topics;
	ros::master::getTopics(topics);
    
    XmlRpc::XmlRpcValue config_xml; 
    std::vector<std::string> tmp_scan_topics1; //holds desired scan topics in config file

    std::istringstream iss(laserscan_topics);
	copy(std::istream_iterator<std::string>(iss), 
         std::istream_iterator<std::string>(), 
         std::back_inserter<std::vector<std::string> >(tmp_scan_topics1));

    // If desired laserscan topics are in the rostopic server, add them to tmp_scan_topics2
	for(int i=0;i<tmp_scan_topics1.size();++i)
	{
        for(int j=0;j<topics.size();++j)
		{
			if( (tmp_scan_topics1[i].compare(topics[j].name) == 0) && (topics[j].datatype.compare("sensor_msgs/LaserScan") == 0) )
			{
				input_scan_topics.push_back(topics[j].name);
			}
		}
	}
}

void LaserScanMultiFilter::initFilterScanTopics(std::string default_profile)
{
    for(int i=0; i<input_scan_topics.size(); ++i){
        scan_destination_topics.push_back(input_scan_topics[i] + "_filtered");
    }
    msg_.resize(input_scan_topics.size());
    scan_publishers.resize(input_scan_topics.size());
    clouds.resize(input_scan_topics.size());
    clouds_modified.resize(input_scan_topics.size());
    std::fill(clouds_modified.begin(), clouds_modified.end(), false);

    for(int i=0; i<input_scan_topics.size(); ++i)
    {
        std::string scan_topic = input_scan_topics[i].substr(1, input_scan_topics[i].size()); //scan_topic should not contain any slashes "/"

        filters::FilterChain<sensor_msgs::LaserScan> *temp_filterchain = new filters::FilterChain<sensor_msgs::LaserScan>("sensor_msgs::LaserScan");
        temp_filterchain->configure(scan_topic + "/" + default_profile + "/scan_filter_chain", private_nh_);
        filter_chain_.push_back(temp_filterchain);

        private_nh_.getParam(scan_topic + "/" + default_profile + "/tf_message_filter_target_frame", tf_message_filter_target_frame);
        private_nh_.param(scan_topic+ "/" + default_profile + "/tf_message_filter_tolerance", tf_filter_tolerance_, 0.03);

        scan_publishers[i] = nh_.advertise<sensor_msgs::LaserScan> (scan_destination_topics[i].c_str(), 1, false);

        //subscribe to laserscan topic
        std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> temp_scan(new message_filters::Subscriber<sensor_msgs::LaserScan>());
        scan_subs.push_back(std::move(temp_scan));
        scan_subs[i]->subscribe(private_nh_, input_scan_topics[i] , 50);

        tf_.push_back(new tf::TransformListener());
        tf_filter_.push_back(new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_subs[i], *tf_[i], "", 50) );  

        tf_filter_[i]->setTargetFrame(tf_message_filter_target_frame);
        tf_filter_[i]->setTolerance(ros::Duration(tf_filter_tolerance_));

        tf_filter_[i]->registerCallback(boost::bind(&LaserScanMultiFilter::filterCallback, this,  _1, i)); // Setup tf::MessageFilter generates callback
    }
}

void LaserScanMultiFilter::pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud)
{
	sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
	output->header = pcl_conversions::fromPCL(merged_cloud->header);
	output->header.frame_id = destination_frame.c_str();
	output->header.stamp = ros::Time::now();  //fixes #265
	output->angle_min = this->angle_min;
	output->angle_max = this->angle_max;
	output->angle_increment = this->angle_increment;
	output->time_increment = this->time_increment;
	output->scan_time = this->scan_time;
	output->range_min = this->range_min;
	output->range_max = this->range_max;

	uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
	output->ranges.assign(ranges_size, output->range_max + 1.0);

	for(int i=0; i<points.cols(); i++)
	{
		const float &x = points(0,i);
		const float &y = points(1,i);
		const float &z = points(2,i);

		if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
		{
			ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
			continue;
		}

		double range_sq = y*y+x*x;
		double range_min_sq_ = output->range_min * output->range_min;
		if (range_sq < range_min_sq_) {
			ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
			continue;
		}

		double angle = atan2(y, x);
		if (angle < output->angle_min || angle > output->angle_max)
		{
			ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
			continue;
		}
		int index = (angle - output->angle_min) / output->angle_increment;
        
		if (output->ranges[index] * output->ranges[index] > range_sq)
			output->ranges[index] = sqrt(range_sq);
	}
	merged_scan_publisher_.publish(output);
}

void LaserScanMultiFilter::filterCallback(const sensor_msgs::LaserScan::ConstPtr& msg_in, const int i)
{   
    if (filter_chain_[i]->update(*msg_in, msg_[i])){
        scan_publishers[i].publish(msg_[i]);
    }
    else{
        ROS_ERROR_THROTTLE(1, "Filtering the scan from time %i.%i failed.", msg_in->header.stamp.sec, msg_in->header.stamp.nsec);
    }

	sensor_msgs::PointCloud tmpCloud1,tmpCloud2;
	sensor_msgs::PointCloud2 tmpCloud3;

    // Verify that TF knows how to transform from the received scan to the destination scan frame
	tfListener_.waitForTransform(msg_[i].header.frame_id.c_str(), destination_frame.c_str(), msg_[i].header.stamp, ros::Duration(1));

    //transform laserscan to pc1 and then transform pc1 to destination frame in pc2
    projector_.transformLaserScanToPointCloud(msg_[i].header.frame_id, msg_[i], tmpCloud1, tfListener_, laser_geometry::channel_option::Distance);
	try{
		tfListener_.transformPointCloud(destination_frame.c_str(), tmpCloud1, tmpCloud2); 
	}catch (tf::TransformException ex){ROS_ERROR("%s",ex.what());return;}

    //convert to PCL in clouds[i]
    sensor_msgs::convertPointCloudToPointCloud2(tmpCloud2,tmpCloud3);
    pcl_conversions::toPCL(tmpCloud3, clouds[i]);
    clouds_modified[i] = true;

    // Count number of scans
	int totalClouds = 0;
	for(int i=0; i<clouds_modified.size(); ++i)
    {
		if(clouds_modified[i])
			++totalClouds;
    }

    // Go ahead only if all subscribed scans have arrived
	if(totalClouds == clouds_modified.size())
	{
		pcl::PCLPointCloud2 merged_cloud = clouds[0];
		clouds_modified[0] = false;

		for(int i=1; i<clouds_modified.size(); ++i)
		{
			pcl::concatenatePointCloud(merged_cloud, clouds[i], merged_cloud);
			clouds_modified[i] = false;
		}

		Eigen::MatrixXf points;
		getPointCloudAsEigen(merged_cloud, points);

		pointcloud_to_laserscan(points, &merged_cloud);
	}
}

void LaserScanMultiFilter::initProfileUpdateServer(std::string default_profile)
{
    laserscanprofileupdateserver.initParams(ros::this_node::getName());
    laserscanprofileupdateserver.initDynClients(default_profile);
}

#endif