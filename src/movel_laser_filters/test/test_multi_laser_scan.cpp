#include "yaml-cpp/yaml.h"
#include <ros_utils/ros_utils.h>
#include <yaml_utils/yaml_utils.h>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <message_filters/subscriber.h> //laser_filters
#include <tf/message_filter.h> //laser_filters
#include <tf/transform_listener.h> 
#include <filters/filter_chain.h> //laser_filters

#include <laser_geometry/laser_geometry.h>
#include <dynamic_reconfigure/server.h>

using namespace std;

class LaserScanMultiFilter
{
public:
    LaserScanMultiFilter();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan, std::string topic);

    std::string node_name_;
    vector<std::string> input_scan_topics;

private:
    void obtainScanTopics();
    
    ros::NodeHandle node_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener_;

    ros::Publisher laser_scan_publisher_;
    vector<ros::Subscriber> scan_subscribers;

    double angle_min;
    double angle_max;
    double angle_increment;
    double time_increment;
    double scan_time;
    double range_min;
    double range_max;

    std::string destination_frame;
    std::string scan_destination_topic;
};



LaserScanMultiFilter::LaserScanMultiFilter()
{
	ros::NodeHandle nh("~");

    nh.param<std::string>("destination_frame", destination_frame, "cart_frame");
    nh.param<std::string>("scan_destination_topic", scan_destination_topic, "/scan_multi");
    nh.param("angle_min", angle_min, -2.36);
    nh.param("angle_max", angle_max, 2.36);
    nh.param("angle_increment", angle_increment, 0.0058);
    nh.param("scan_time", scan_time, 0.0333333);
    nh.param("range_min", range_min, 0.45);
    nh.param("range_max", range_max, 25.0);

    this->obtainScanTopics();

	laser_scan_publisher_ = node_.advertise<sensor_msgs::LaserScan> (scan_destination_topic.c_str(), 1, false);

}

void LaserScanMultiFilter::obtainScanTopics()
{
    //retrieve topics from rostopic server
    ros::master::V_TopicInfo topics;
	ros::master::getTopics(topics);

    std::string parampath = "/lasermultifilter/input_scan_topics"; //TOFIX: stop uploading under the node namespace and upload to global namespace instead?
    //TOFIX: Replace this hardcoded line

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

    // //TEST: output matching scan topics
    // for (std::string topic_: input_scan_topics)
    //     std::cout<< topic_ <<std::endl;
    

	// Do not re-subscribe if the topics are the same
	if( (tmp_scan_topics2.size() != input_scan_topics.size()) || !equal(tmp_scan_topics2.begin(),tmp_scan_topics2.end(),input_scan_topics.begin()))
	{

		// Unsubscribe from previous topics
		for(int i=0; i<scan_subscribers.size(); ++i)
			scan_subscribers[i].shutdown();

		input_scan_topics = tmp_scan_topics2;
		if(input_scan_topics.size() > 0)
		{
            scan_subscribers.resize(input_scan_topics.size());
            ROS_INFO("Subscribing to topics\t%ld", scan_subscribers.size());
			for(int i=0; i<input_scan_topics.size(); ++i)
			{
                scan_subscribers[i] = node_.subscribe<sensor_msgs::LaserScan> 
                    (input_scan_topics[i].c_str(), 1, boost::bind(&LaserScanMultiFilter::scanCallback,this, _1, input_scan_topics[i]));
				std::cout << input_scan_topics[i] << " ";
			}
		}
		else
            ROS_INFO("Not subscribed to any topic.");
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "lasermultifilter");

    LaserScanMultiFilter lasermultifilter;
    lasermultifilter.node_name_ = "/lasermultifilter/";


	ros::spin();

	return 0;
}
