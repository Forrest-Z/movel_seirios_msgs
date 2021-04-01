#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

ros::Subscriber cloud_sub;
ros::Publisher cloud_pub;

void cloudCb(sensor_msgs::PointCloud2 cloud)
{
    if (cloud.width == 1)
    {
        int h = cloud.height;
        cloud.width = h;
        cloud.height = 1;
    }
    cloud_pub.publish(cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_transposer");
    ros::NodeHandle nh;

    cloud_sub = nh.subscribe("cloud_in", 1, cloudCb);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_out", 1);

    ros::spin();

    return 0;
}
