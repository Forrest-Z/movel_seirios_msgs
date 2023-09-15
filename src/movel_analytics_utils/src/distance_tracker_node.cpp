#include <ros/ros.h>

#include "movel_analytics_utils/distance_tracker.h"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "distance_tracker");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Rate rate(100);
    movel_analytics_utils::DistanceTracker distance_tracker(nh, private_nh);

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
    ros::shutdown();
    return 0;
}
