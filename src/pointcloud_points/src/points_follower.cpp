#include <ros/ros.h>
#include <movel_seirios_msgs/movelPointcloud2.h>
#include <movel_seirios_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>

using namespace std::chrono;
class PointCloudFollower
{
public:
    PointCloudFollower()
    {
        setupTopics();
    };
    
    ~PointCloudFollower(){};

protected:
    void setupTopics()
    {
        ros::NodeHandle nh("~");
        cloud_sub_ = nh.subscribe("/velodyne_points", 1, &PointCloudFollower::cloudCb, this);
        cloud_pub_ = nh.advertise<movel_seirios_msgs::movelPointcloud2>("/movel_pointcloud", 1);
    }

    void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& input)
    {
        // auto start = high_resolution_clock::now();

        pcl::PCLPointCloud2::Ptr pcl_cloud (new pcl::PCLPointCloud2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl_conversions::toPCL(*input, *pcl_cloud);
        pcl::fromPCLPointCloud2(*pcl_cloud, *temp_cloud);

        movel_seirios_msgs::movelPointcloud2 out_msg;
        movel_seirios_msgs::Point pt;

        for (int i = 0 ; i < temp_cloud->points.size(); i++)
        {
            pt.x = temp_cloud->points[i].x;
            pt.y = temp_cloud->points[i].y;
            pt.z = temp_cloud->points[i].z;
            out_msg.points.push_back(pt);
            out_msg.point_count++;
        }
        out_msg.header = input->header;
        out_msg.header.stamp = ros::Time::now();

        cloud_pub_.publish(out_msg);

        // auto stop = high_resolution_clock::now();
        // auto duration = duration_cast<microseconds>(stop - start);
        // std::cout<<duration.count()<<std::endl;
    }

    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_follower");
    PointCloudFollower pcl_follower;
    ros::spin();
    return 0;
}