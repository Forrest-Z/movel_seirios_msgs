#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <movel_seirios_msgs/StringTrigger.h>

class PointCloudSaver
{
public:
    PointCloudSaver():
        resolution_(0.05),
        binary_(true),
        compressed_(true),
        map_topic_("/map_3d"),
        cloud_ (new pcl::PCLPointCloud2())
    {
        loadParams();
        setupTopics();
    }

    ~PointCloudSaver(){};

    void loadParams()
    {
        ros::NodeHandle private_nh("~");
        private_nh.getParam("resolution", resolution_);
        private_nh.getParam("is_binary", binary_);
        private_nh.getParam("is_compressed", compressed_);
        private_nh.getParam("map_topic", map_topic_);
    }

    void setupTopics()
    {
        ros::NodeHandle private_nh("~");
        cloud_sub_ = private_nh.subscribe(map_topic_, 1, &PointCloudSaver::onCloudCallback, this);
        save_server_ = private_nh.advertiseService("export_pcd", &PointCloudSaver::onSaveService, this);
    }

    void onCloudCallback(const sensor_msgs::PointCloud2::Ptr &msg)
    {
        pcl::PCLPointCloud2::Ptr temp_cloud (new pcl::PCLPointCloud2);
        pcl_conversions::toPCL(*msg, *temp_cloud);
        cloud_ = temp_cloud;
    }

    bool onSaveService(movel_seirios_msgs::StringTrigger::Request &req, 
                        movel_seirios_msgs::StringTrigger::Response &res)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromPCLPointCloud2(*cloud_, *cloud_out);
        
        // Downsampling the pointcloud
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud (cloud_out);
        vg.setLeafSize (resolution_, resolution_, resolution_);
        vg.filter (*cloud_out);

        // Save PCD
        std::string file_name = req.input + ".pcd";
        pcl::PCDWriter writer;
        writer.write<pcl::PointXYZI> (file_name, *cloud_out, false); 
        ROS_INFO("[pointcloud_saver] 3D Map saved in %s", req.input.c_str());
        res.success = true;
        return true;
    }

protected:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::ServiceServer save_server_;

    pcl::PCLPointCloud2::Ptr cloud_;
    
    double resolution_;
    double binary_;
    double compressed_;
    std::string map_topic_;
    
};


int main (int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_saver");
    PointCloudSaver pcl_saver;
    ros::spin();

    return 0;
}