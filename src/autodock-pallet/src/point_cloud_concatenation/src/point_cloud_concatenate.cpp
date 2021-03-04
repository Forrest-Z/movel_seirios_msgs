#include <ros/ros.h>
#include <point_cloud_concatenation/point_cloud_concatenation.h>
#include<map>
#include<tf/transform_broadcaster.h>


FusedPcl::FusedPcl()
{
    ros::NodeHandle nodeHandle("~");
    std::cout<<"constructor"<<std::endl;
    tf::TransformListener listener_front;
    tf::StampedTransform transform_front;
    try
    {
        listener_front.waitForTransform("base_link", "base_scan_front", ros::Time(0), ros::Duration(10.0) );
        listener_front.lookupTransform("base_link", "base_scan_front", ros::Time(0), transform_front);
    } catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }
    front_lidar_base_link_offset_x = transform_front.getOrigin().x();
    front_lidar_base_link_offset_y = transform_front.getOrigin().y();
    tf::TransformListener listener_rear;
    tf::StampedTransform transform_rear;
    try
    {
        listener_rear.waitForTransform("base_link", "base_scan_rear", ros::Time(0), ros::Duration(10.0) );
        listener_rear.lookupTransform("base_link", "base_scan_rear", ros::Time(0), transform_rear);
    } catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }
    rear_lidar_base_link_offset_x = transform_rear.getOrigin().x();
    rear_lidar_base_link_offset_y = transform_rear.getOrigin().y();
/*
    message_filters::Subscriber<sensor_msgs::PointCloud2> frontCloudCallback_(nodeHandle, "/camera_docking/point_cloud_front_base_link", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> rearCloudCallback_(nodeHandle, "/camera_docking/point_cloud_front_base_link_rear", 10);

    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), frontCloudCallback_, rearCloudCallback_);
    sync.registerCallback(boost::bind(&FusedPcl::callback, this, _1, _2));
*/
    point_cloud_subscriber_front = nodeHandle.subscribe("/scan_front", 10, &FusedPcl::frontCloudCallback, this);
    //fused_point_cloud_front = nodeHandle.advertise<sensor_msgs::PointCloud2>("point_cloud_front_base_link", 10);
    point_cloud_subscriber_rear = nodeHandle.subscribe("/scan_rear", 10, &FusedPcl::rearCloudCallback, this);
   //fused_point_cloud_rear = nodeHandle.advertise<sensor_msgs::PointCloud2>("point_cloud_front_base_link_rear", 10);
    fused_point_cloud = nodeHandle.advertise<sensor_msgs::PointCloud2>("fused_cloud", 10);

}


void FusedPcl::callback(const sensor_msgs::PointCloud2ConstPtr &msg_front, const sensor_msgs::PointCloud2ConstPtr &msg_rear)
{
    fused_cloud.clear();
    front_cloud.clear();
    std::cout<<"working"<<std::endl;
    /*
    for(int i = 0; i < msg_front->ranges.size(); i++)    // greater than 270
    {
        {
            pcl::PointXYZI pt;
            pt.y = msg_front->ranges[i]*sin(msg_front->angle_min + i*msg_front->angle_increment);
            pt.x = msg_front->ranges[i]*cos(msg_front->angle_min + i*msg_front->angle_increment);
            pt.z = 0;
            pt.intensity = msg_front->intensities[i];

            fused_cloud.points.push_back(pt);

        }
    }

    rear_cloud.clear();
    for(int i = 0; i < msg_rear->ranges.size(); i++)    // greater than 270
    {
        {
            pcl::PointXYZI pt;
            pt.y = -msg_rear->ranges[i]*sin(msg_rear->angle_min + i*msg_rear->angle_increment) - 0.4 ;
            pt.x = -msg_rear->ranges[i]*cos(msg_rear->angle_min + i*msg_rear->angle_increment) - 0.2 ;
            pt.z = 0;
            pt.intensity = msg_rear->intensities[i];

            fused_cloud.points.push_back(pt);
        }
    }

        sensor_msgs::PointCloud2 msg2;
        pcl::toROSMsg(fused_cloud, msg2);
        msg2.header.stamp = ros::Time::now();
        msg2.header.frame_id= "/base_scan_front";
        fused_point_cloud_front.publish(msg2);
        */
}


void FusedPcl::frontCloudCallback(const sensor_msgs::LaserScan& msg)
{
    front_msg = msg;

    front_cloud.clear();
    for(int i = 0; i < msg.ranges.size(); i++)    // greater than 270
    {
        //        if( msg.ranges[i] < 2)
        {
            pcl::PointXYZI pt;
            pt.y = msg.ranges[i]*sin(msg.angle_min + i*msg.angle_increment) + front_lidar_base_link_offset_y; //0.4
            pt.x = msg.ranges[i]*cos(msg.angle_min + i*msg.angle_increment) + front_lidar_base_link_offset_x; // 0.92
            pt.z = 0;
            pt.intensity = msg.intensities[i];
            front_cloud.points.push_back(pt);
            fused_cloud.points.push_back(pt);
        }
    }

    sensor_msgs::PointCloud2 msg2;
    pcl::toROSMsg(front_cloud, msg2);
    msg2.header.stamp = ros::Time::now();
    msg2.header.frame_id= "/base_scan_front";

}


void FusedPcl::rearCloudCallback(const sensor_msgs::LaserScan& msg)
{

    rear_msg = msg;
    rear_cloud.clear();
    for(int i = 0; i < msg.ranges.size(); i++)    // greater than 270
    {
        {
            pcl::PointXYZI pt;
            pt.y = -msg.ranges[i]*sin(msg.angle_min + i*msg.angle_increment) + rear_lidar_base_link_offset_y ; // -0.4
            pt.x = -msg.ranges[i]*cos(msg.angle_min + i*msg.angle_increment) + rear_lidar_base_link_offset_x ; // -0.92
            pt.z = 0;
            pt.intensity = msg.intensities[i];
            rear_cloud.points.push_back(pt);
            fused_cloud.points.push_back(pt);
        }
    }

    sensor_msgs::PointCloud2 msg2;
    pcl::toROSMsg(fused_cloud, msg2);
    msg2.header.stamp = ros::Time::now();
    msg2.header.frame_id= "base_link";

    fused_point_cloud.publish(msg2);
    fused_cloud.clear();
}


