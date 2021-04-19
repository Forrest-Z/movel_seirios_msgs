#include "yaml-cpp/yaml.h"
#include <ros_utils/ros_utils.h>
#include <yaml_utils/yaml_utils.h>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>

#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <dynamic_reconfigure/server.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/point_cloud_conversion.h> 

#include <message_filters/subscriber.h> //movel_laser_filters
#include <tf/message_filter.h> //movel_laser_filters
#include <tf/transform_listener.h> 
#include <filters/filter_chain.h> //movel_laser_filters

using namespace std;

class LaserScanMultiFilter
{
public:
    LaserScanMultiFilter();
    ~LaserScanMultiFilter();

    void subScanTopics();
    void initFilterScanTopics();
    void filterCallback(const sensor_msgs::LaserScan::ConstPtr& msg_in);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan, std::string topic);

    std::string node_name_;
    vector<std::string> input_scan_topics;

protected:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    /** laser filter **/
    filters::FilterChain<sensor_msgs::LaserScan> filter_chain_;
    std::vector< std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> > scan_subs;
    std::vector<tf::TransformListener*> tf_;
    std::vector<tf::MessageFilter<sensor_msgs::LaserScan>*> tf_filter_;
    std::vector<sensor_msgs::LaserScan> msg_;
    std::vector<std::string> scan_frame_ids;

    // tf::TransformListener tfListener_;

    bool first_time_;
    int iteration_;
    int num_scan_topics;

    /** laser merger component **/
    laser_geometry::LaserProjection projector_;
    std::vector<ros::Publisher> scan_publishers;
    std::vector<ros::Subscriber> scan_subscribers;

    double angle_min;
    double angle_max;
    double angle_increment;
    double time_increment;
    double scan_time;
    double range_min;
    double range_max;
    double tf_filter_tolerance_;
    std::string tf_message_filter_target_frame;
    std::string destination_frame;
    // std::string scan_destination_topic;
    std::vector<std::string> scan_destination_topics;
};

LaserScanMultiFilter::LaserScanMultiFilter(): private_nh_("~"), filter_chain_("sensor_msgs::LaserScan")
{
	// ros::NodeHandle nh("~");

    private_nh_.param<std::string>("destination_frame", destination_frame, "base_link");
    // private_nh_.param<std::string>("scan_destination_topic", scan_destination_topic, "/scan_multi");
    // private_nh_.param("angle_min", angle_min, -2.36);
    // private_nh_.param("angle_max", angle_max, 2.36);
    // private_nh_.param("angle_increment", angle_increment, 0.0058);
    // private_nh_.param("scan_time", scan_time, 0.0333333);
    // private_nh_.param("range_min", range_min, 0.45);
    // private_nh_.param("range_max", range_max, 25.0);

    this->subScanTopics();



    this->initFilterScanTopics();

}

LaserScanMultiFilter::~LaserScanMultiFilter()
{
    for (int i = 0; i < tf_filter_.size(); ++i)
        delete tf_filter_[i];
    for (int i = 0; i < tf_.size(); ++i)
        delete tf_[i];
}

void LaserScanMultiFilter::subScanTopics()
{
    //retrieve topics from rostopic server
    ros::master::V_TopicInfo topics;
	ros::master::getTopics(topics);

    std::string parampath = "/laserscan_multi_filter/input_scan_topics"; //TOFIX: Replace this hardcoded line
    
    //retrieve the desired input scan topics from the YAML file
    XmlRpc::XmlRpcValue config_xml; 
    vector<std::string> tmp_scan_topics1; //holds the scan topics in the YAML config file
    vector<std::string> tmp_scan_topics2; //holds the scan topics that are actually present in the rosparam server
    
    //seperate the string by commas into their individual scan topics
    if(ros::param::get(parampath, config_xml))
    {
        std::string scantopics = static_cast<std::string>(config_xml["topics"]);
        std::stringstream ss(scantopics);
        std::string topic_;
        while (getline(ss, topic_, ',')) {
            tmp_scan_topics1.push_back("/" + topic_);
        }
    }

    //check if scan topics in yaml file is in the rostopic server, if so then add them to tmp_scan_topics2
	for(int i=0;i<tmp_scan_topics1.size();++i)
	{
        for(int j=0;j<topics.size();++j)
		{
			if( (tmp_scan_topics1[i].compare(topics[j].name) == 0) && (topics[j].datatype.compare("sensor_msgs/LaserScan") == 0) )
			{
				tmp_scan_topics2.push_back(topics[j].name);
			}
		}
	}


	// // Do not re-subscribe if the topics are the same
	if( (tmp_scan_topics2.size() != input_scan_topics.size()) || !std::equal(tmp_scan_topics2.begin(),tmp_scan_topics2.end(),input_scan_topics.begin()))
	{
		// Unsubscribe from previous topics
		// for(int i=0; i<scan_subscribers.size(); ++i)
		// 	scan_subscribers[i].shutdown();

		input_scan_topics = tmp_scan_topics2;

		// if(input_scan_topics.size() > 0)
		// {
        //     scan_subscribers.resize(input_scan_topics.size());
        //     ROS_INFO("Subscribing to topics\t%ld", scan_subscribers.size());
		// 	for(int i=0; i<input_scan_topics.size(); ++i)
		// 	{
        //         scan_subscribers[i] = nh_.subscribe<sensor_msgs::LaserScan> 
        //             (input_scan_topics[i].c_str(), 1, boost::bind(&LaserScanMultiFilter::scanCallback,this, _1, input_scan_topics[i]));
		// 		std::cout << input_scan_topics[i] << std::endl;
		// 	}
		// }
		// else
        //     ROS_INFO("Not subscribed to any topic.");
	}

    num_scan_topics = input_scan_topics.size();

    msg_.resize(num_scan_topics);
    scan_frame_ids.resize(num_scan_topics);

    //get message from topic once to retrieve the frame id of each lidar
    for (int i=0; i<num_scan_topics; ++i)
    {
        boost::shared_ptr<sensor_msgs::LaserScan const> msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>(tmp_scan_topics2[i], private_nh_);
        // boost::shared_ptr<M const> waitForMessage(const std::string& topic, ros::NodeHandle& nh)
        scan_frame_ids[i] = msg->header.frame_id;        
        ROS_INFO("scan_frame_ids[i]: %s",scan_frame_ids[i].c_str());
    }

    
}

void LaserScanMultiFilter::initFilterScanTopics()
{
    first_time_ = true;

    //retrieve parameters from YAML config file
    filter_chain_.configure("init/scan_filter_chain", private_nh_); //TOFIX: do not hardcore the "init" profile
    private_nh_.getParam("init/tf_message_filter_target_frame", tf_message_filter_target_frame);
    private_nh_.param("init/tf_message_filter_tolerance", tf_filter_tolerance_, 0.03);

    //resize
    scan_publishers.resize(num_scan_topics);
    
    // TOFIX: Make sure scan destination is not hard coded
    scan_destination_topics.push_back("scan1_filtered");
    scan_destination_topics.push_back("scan2_filtered");
    
    for(int i=0; i<num_scan_topics; ++i)
    {
        ROS_INFO("A");
        scan_publishers[i] = nh_.advertise<sensor_msgs::LaserScan> (scan_destination_topics[i].c_str(), 1, false);
        std::cout << "advertised: index " << i << " " << scan_destination_topics[i] << std::endl;

        ROS_INFO("B");
        this->iteration_ = i;

        //subscribe to laserscan topic
        std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> temp_scan(new message_filters::Subscriber<sensor_msgs::LaserScan>());
        scan_subs.push_back(std::move(temp_scan));
        scan_subs[i]->subscribe(private_nh_, input_scan_topics[i] , 50);

        ROS_INFO("C");
        // tf_ = new tf::TransformListener();
        // tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_subs[i], *tf_, "", 50);

        tf_.push_back(new tf::TransformListener());
        tf_filter_.push_back(new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_subs[i], *tf_[i], "", 50) );  

        ROS_INFO("D");
        tf_filter_[i]->setTargetFrame(tf_message_filter_target_frame);
        tf_filter_[i]->setTolerance(ros::Duration(tf_filter_tolerance_));

        ROS_INFO("E");
        // Setup tf::MessageFilter generates callback
        tf_filter_[i]->registerCallback(boost::bind(&LaserScanMultiFilter::filterCallback, this,  _1));
    }
    first_time_ = false;

}


void LaserScanMultiFilter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan, std::string topic)
{
	// sensor_msgs::PointCloud tmpCloud1,tmpCloud2;
	// sensor_msgs::PointCloud2 tmpCloud3;

    // Verify that TF knows how to transform from the received scan to the destination scan frame
	// tfListener_.waitForTransform
    //     (scan->header.frame_id.c_str(), destination_frame.c_str(), scan->header.stamp, ros::Duration(1));

    // //convert to PC and store in tmpCloud1
    // projector_.transformLaserScanToPointCloud
    //     (scan->header.frame_id, *scan, tmpCloud1, tfListener_, laser_geometry::channel_option::Distance); 
}

void LaserScanMultiFilter::filterCallback(const sensor_msgs::LaserScan::ConstPtr& msg_in)
{   

    if (first_time_)
    {
        int i = this->iteration_;
        if (filter_chain_.update(*msg_in, msg_[i])){
            ROS_INFO("message filtered");
            ROS_INFO(msg_in->header.frame_id.c_str());
            // ROS_INFO_THROTTLE(1, "message filtered");
            // ROS_INFO_THROTTLE(1, msg_in->header.frame_id.c_str());
            scan_publishers[i].publish(msg_[i]);
        }
        else
            ROS_ERROR_THROTTLE(1, "Filtering the scan from time %i.%i failed.", msg_in->header.stamp.sec, msg_in->header.stamp.nsec);
    }
    else
    {
        int j = 0;
        for (int i=0; i<num_scan_topics; ++i)
        {
            ROS_INFO("msg_in: %s",msg_in->header.frame_id.c_str());
            ROS_INFO("scan_frame_ids[i]: %s",scan_frame_ids[i].c_str());
            if (msg_in->header.frame_id == scan_frame_ids[i])
            {
                ROS_INFO(msg_in->header.frame_id.c_str());
                j = i;
                break;
            }
        }

        if (filter_chain_.update(*msg_in, msg_[j])){
            // ROS_INFO_THROTTLE(1, "message filtered");
            scan_publishers[j].publish(msg_[j]);
            //TODO: Merging of laser filters here
        }
        else
            ROS_ERROR_THROTTLE(1, "Filtering the scan from time %i.%i failed.", msg_in->header.stamp.sec, msg_in->header.stamp.nsec);
    }

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "laserscan_multi_filter");

    LaserScanMultiFilter lasermultifilter;

    ros::spin();

    // while (ros::ok())
    // {
    //     ros::spinOnce();
    // }
	
	return 0;
}
