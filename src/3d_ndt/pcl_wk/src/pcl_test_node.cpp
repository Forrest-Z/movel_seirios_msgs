#include "pcl_test_core.h"
#include "config.h"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include "Eigen/Dense"
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>






bool isStarted = false;


PclTestCore::PclTestCore(ros::NodeHandle &nh){
    sub_point_cloud_ = nh.subscribe("/rslidar_points",10, &PclTestCore::point_cb, this);
    sub_RPY_ = nh.subscribe("/roll_pitch_yaw",10, &PclTestCore::rot_array_cb, this);
    pub_filtered_points_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points22", 10);
    pub_occupancy_grid_ = nh.advertise<nav_msgs::OccupancyGrid>("/local_map", 10);
 
    ros::spin();
}
 
PclTestCore::~PclTestCore(){}
 
void PclTestCore::Spin(){
}

float PclTestCore::DegreeToRadian(float degrees)
{
    return (degrees*(M_PI/180));
}
    
void PclTestCore::rot_array_cb(const std_msgs::Float32MultiArrayConstPtr& in_array)
{
    // for(std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	// {
	// 	Arr[i] = *it;
	// 	i++;
	// }

    std::vector<float> rpy = in_array->data;
    tf2::Quaternion quat;
    quat.setRPY(DegreeToRadian(rpy[0]),DegreeToRadian(rpy[1]),DegreeToRadian(rpy[2]));
    quaternion_={quat[0],quat[1],quat[2],quat[3]};
    isStarted = true;
    ROS_INFO(" CHECK /local_map TOPIC. ");
}

void PclTestCore::point_cb(const sensor_msgs::PointCloud2ConstPtr & in_cloud_ptr){

    if(isStarted || ROTATION_REQUIRED == false)
    {

        
        sensor_msgs::PointCloud2 cloud_in, cloud_out;
        geometry_msgs::TransformStamped transformR;

        transformR.transform.translation.x = 0;
        transformR.transform.translation.y = 0;
        transformR.transform.translation.z = 0;//, 

        transformR.transform.rotation.x = quaternion_[0];
        transformR.transform.rotation.y = quaternion_[1];
        transformR.transform.rotation.z = quaternion_[2];
        transformR.transform.rotation.w = quaternion_[3];

        tf2::doTransform (*in_cloud_ptr, cloud_out, transformR);

        sensor_msgs::PointCloud2* transformed_cloud_ptr = &cloud_out;
        pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr dwn_sampled_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
        int sizeX=(X_END - X_START)/RESOLUTION;
        int sizeY=(Y_END - Y_START)/RESOLUTION;

        pcl::fromROSMsg(*transformed_cloud_ptr, *current_pc_ptr);
    
        // //DOWNSAMPLING
        // pcl::VoxelGrid<pcl::PointXYZI> vg;
        // vg.setInputCloud(current_pc_ptr);
        // vg.setLeafSize(0.2f, 0.2f, 0.2f);
        // vg.filter(*dwn_sampled_ptr);
    
        //CHOPPING BASED ON AXIS
        pcl::PassThrough<pcl::PointXYZI> passX;
        passX.setInputCloud (current_pc_ptr);
        passX.setFilterFieldName ("x");
        passX.setFilterLimits (X_START, X_END);
        // pass.setFilterLimitsNegative (true);
        passX.filter (*cloud_filtered);

        pcl::PassThrough<pcl::PointXYZI> passY;
        passY.setInputCloud (cloud_filtered);
        passY.setFilterFieldName ("y");
        passY.setFilterLimits (Y_START, Y_END);
        // pass.setFilterLimitsNegative (true);
        passY.filter (*cloud_filtered);


        //OCCUPANCY GRID GENERATION
        nav_msgs::OccupancyGrid occupancyGrid;
        occupancyGrid.header.stamp = ros::Time::now();
        occupancyGrid.header.frame_id = "local_map";
        occupancyGrid.info.resolution = RESOLUTION;
        occupancyGrid.info.width = sizeX;
        occupancyGrid.info.height = sizeY;
        geometry_msgs::Pose pose;
        pose.position.x = 0;
        pose.position.x = 0;
        pose.position.x = 0;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;
        occupancyGrid.info.origin = pose;

        Eigen::MatrixXd makeGrid(sizeX,sizeY);
        makeGrid.fill(0.0);
        Eigen::MatrixXd countGrid(sizeX,sizeY);
        countGrid.fill(0.0);

        #pragma omp for
        for (size_t i =0; i < cloud_filtered->points.size(); i++)
        {

            if(cloud_filtered->points[i].z > Z_START)
            {
                int x_index=int((cloud_filtered->points[i].x - X_START)/RESOLUTION);
                int y_index=int((cloud_filtered->points[i].y - Y_START)/RESOLUTION);
                countGrid(x_index,y_index)+=1;
                if(countGrid(x_index,y_index) >= POINT_THRESHOLD)
                {
                     makeGrid(x_index,y_index)=100;
                }
            }
        }

        std::vector<double> vec(makeGrid.size());
        Eigen::Map<Eigen::MatrixXd>(vec.data(), makeGrid.cols(),makeGrid.rows())  = makeGrid;
        std::vector<int8_t> conv_local_map(vec.begin(),vec.end());
        occupancyGrid.data = conv_local_map;
        pub_occupancy_grid_.publish(occupancyGrid);
        

        sensor_msgs::PointCloud2 pub_pc;
        pcl::toROSMsg(*cloud_filtered, pub_pc);
        pub_pc.header = in_cloud_ptr->header;
        pub_filtered_points_.publish(pub_pc);
        
        
    }
    else
    {
        ROS_WARN(" 'ROTATION_REQUIRED : TRUE' IN CONFIG.H. PUBLISH THE TOPIC /roll_pitch_yaw  IN DEGREES FOR ROTATION");
    }
    

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_test");
 
    ros::NodeHandle nh;
 
    PclTestCore core(nh);
    return 0;
}
