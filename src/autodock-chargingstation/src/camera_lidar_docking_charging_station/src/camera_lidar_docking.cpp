#include <ros/ros.h>
#include <camera_docking_/camera_docking.h>
#include "camera_lidar_docking_charging_station/StartAutoDock.h"
#include <cameralidarDock/camera_lidar_docking.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;
using namespace cv;

/**
 * @brief CameraLidar::CameraLidar - constructor
 * @param nodeHandle
 */

CameraLidar::CameraLidar(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle)
{
//    FiducialsNode* node = new FiducialsNode();
//    lidar_docking::LidarDocking LidarDocking(nodeHandle_);
}
/**
 * @brief CameraLidar::autoDock - service to start autodocking to charger
 * @param req
 * @param res
 * @return
 */
bool CameraLidar::autoDock(camera_lidar_docking_charging_station::StartAutoDock::Request  &req,
                           camera_lidar_docking_charging_station::StartAutoDock::Response &res)
{
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = req.x;
    goal.target_pose.pose.position.y = req.y;
    goal.target_pose.pose.orientation.z = req.theta;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();
    std::cout<<"done"<<std::endl;
    auto_dock_start_position_reached = true;
        FiducialsNode* node = new FiducialsNode();
        lidar_docking::LidarDocking LidarDocking();
    res.success = true;
    return true;
}



/**
 * @brief FiducialsNode::FiducialsNode
 */
FiducialsNode::FiducialsNode() : nh(), pnh("~"), it(nh)
{
    obj = new lidar_docking::LidarDocking(/*nh*/);
    frameNum = 0;

    // Camera intrinsics
    cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);

    // distortion coefficients
    distortionCoeffs = cv::Mat::zeros(1, 5, CV_64F);

    haveCamInfo = false;
    enable_detections = true;

    int dicno;

    detectorParams = new aruco::DetectorParameters();

    pnh.param<bool>("publish_images", publish_images, false);
    pnh.param<double>("fiducial_len", fiducial_len, 0.14);
    pnh.param<int>("dictionary", dicno, 7);
    pnh.param<bool>("do_pose_estimation", doPoseEstimation, true);
    pnh.param<bool>("publish_fiducial_tf", publishFiducialTf, true);
    pnh.param<double>("kd", kd, 5);
    pnh.param<double>("kp", kp, 5);
    pnh.param<double>("max_speed_forward", max_speed_forward_, 0.05);
    pnh.param<double>("max_speed_angular", max_speed_angular_ , 0.1);
    pnh.param<double>("goal_position_orientation", goal_position_orientation_ , 0);
    pnh.param<double>("marker_for_docking", marker_for_docking_ , 23);
    pnh.param<double>("max_y_error", max_y_error_ , 0.01);
    pnh.param<double>("time_serach_marker", time_serach_marker_ , 5);


    std::string str;
    std::vector<std::string> strs;

    pnh.param<string>("ignore_fiducials", str, "");
    handleIgnoreString(str);

    /*
    fiducial size can take comma separated list of size: id or size: range,
    e.g. "200.0: 12, 300.0: 200-300"
    */
    pnh.param<string>("fiducial_len_override", str, "");
    boost::split(strs, str, boost::is_any_of(","));
    for (const string& element : strs) {
        if (element == "") {
            continue;
        }
        std::vector<std::string> parts;
        boost::split(parts, element, boost::is_any_of(":"));
        if (parts.size() == 2) {
            double len = std::stod(parts[1]);
            std::vector<std::string> range;
            boost::split(range, element, boost::is_any_of("-"));
            if (range.size() == 2) {
                int start = std::stoi(range[0]);
                int end = std::stoi(range[1]);
                ROS_INFO("Setting fiducial id range %d - %d length to %f",
                         start, end, len);
                for (int j=start; j<=end; j++) {
                    fiducialLens[j] = len;
                }
            }
            else if (range.size() == 1){
                int fid = std::stoi(range[0]);
                ROS_INFO("Setting fiducial id %d length to %f", fid, len);
                fiducialLens[fid] = len;
            }
            else {
                ROS_ERROR("Malformed fiducial_len_override: %s", element.c_str());
            }
        }
        else {
            ROS_ERROR("Malformed fiducial_len_override: %s", element.c_str());
        }
    }

    image_pub = it.advertise("/fiducial_images", 1);
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    dictionary = aruco::getPredefinedDictionary(dicno);
    img_sub = it.subscribe("/camera/rgb/image_raw", 1,
                           &FiducialsNode::imageCallback, this);

    caminfo_sub = nh.subscribe("/camera/rgb/camera_info", 1,
                               &FiducialsNode::camInfoCallback, this);


    ROS_INFO("Aruco detection ready");
}


/**
 * @brief FiducialsNode::imageCallback - subscribers to image so as to give pose of the marker wrt camera
 * and generate command velocities accordingly for autodock to the charging station
 * @param msg
 */
void FiducialsNode::imageCallback(const sensor_msgs::ImageConstPtr & msg) {
    if (enable_detections == false) {
        return; //return without doing anything
    }

    //    ROS_INFO("Got image %d", msg->header.seq);
    frameNum++;

    cv_bridge::CvImagePtr cv_ptr;

    tf::TransformListener listener;
    tf::StampedTransform transform;


    try {
        listener.waitForTransform("camera_rgb_frame", "base_link", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("camera_rgb_frame", "base_link", ros::Time(0), transform);
        //        std::cout<<"trabsformations are as follows "<<transform.getOrigin().y()<<" "<<transform.getOrigin().x()<<std::endl;
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        vector <int>  ids;
        vector <vector <Point2f> > corners, rejected;
        vector <Vec3d>  rvecs;
        vector<Vec3d> tvecs = {0,0,0};

        aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids, detectorParams);

        if(ids.size() > 0) {
            aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
            obj->camera_detection = true;
            std::cout<<"id is --------- "<<ids[0]<<std::endl;
        }
        if(ids.size()==0){
            found_marker_dock = false;
        }
        for(int i = 0 ; i < ids.size(); i++)
        {
            std::cout<<"detected: "<<ids[i]<<std::endl;
            if(ids[i] == marker_for_docking_)
            {
                found_marker_dock = true;

            }
            else
            {
                found_marker_dock = false;
                std::cout<<"marker "<<marker_for_docking_ <<" not found in camera"<<std::endl;
            }
        }

        if (doPoseEstimation) {
            if (!haveCamInfo) {
                if (frameNum > 5) {
                    ROS_ERROR("No camera intrinsics");
                }
                return;
            }

            vector <double>reprojectionError;
            //////////////////////////////////////////
            float direction = computeImageSide(cv_ptr->image, corners,ids);
            //            std::cout<<"direction received is "<<direction<<std::endl;
            ///////////////////////////////////////////
            estimatePoseSingleMarkers(ids, corners, (float)fiducial_len,
                                      cameraMatrix, distortionCoeffs,
                                      rvecs, tvecs,
                                      reprojectionError);


            std::cout<<"marker in camera "<<found_marker_dock <<"   marker in lidar "<<obj->marker_visible_lidar_<<std::endl;

            std::cout<<"is robot ready for docking  -------"<<obj->robot_ready_for_docking<<std::endl;
            if(obj->marker_visible_lidar_ == false && found_marker_dock == false && obj->obstacle_found_ == false && obj->robot_ready_for_docking==true)
            {
                time_counter_search_marker+=1;

                //                std::cout<<"marker neither found in camera nor in lidar"<<std::endl;
                std::cout<<"searching for marker "<<std::endl;
                ros::Duration(1.0).sleep();
                //                obj->rotate_and_check();
                //                if(obj->rotation_completed)
                if(time_counter_search_marker > time_serach_marker_)
                {
                    std::cout<<"aborting the operation"<<std::endl;
                    exit(0);
                }

            }
            else
                time_counter_search_marker = 0;


            std::cout<<"dist is "<<obj->distance<<" camera detect "<<obj->camera_detection<<std::endl;
            if( obj->distance >= obj->dist_camera_to_lidar_switch_ and  obj->camera_detection == true and found_marker_dock)    // obj->distance >= 0.3
            {
                if(ids.size() == 0)
                {
                    obj->camera_detection = false;
                    move(0,0);
                }

                for (size_t i=0; i<ids.size(); i++) {
                    aruco::drawAxis(cv_ptr->image, cameraMatrix, distortionCoeffs,
                                    rvecs[i], tvecs[i], (float)fiducial_len);

                    ROS_INFO("Detected id %d T %.2f %.2f %.2f R %.2f %.2f %.2f", ids[i],
                             tvecs[i][0], tvecs[i][1], tvecs[i][2],
                            rvecs[i][0], rvecs[i][1], rvecs[i][2]);

                    if (std::count(ignoreIds.begin(), ignoreIds.end(), ids[i]) != 0) {
                        continue;
                    }

                    /////////////////////navigation algo///////////////////
                    float speed = max_speed_forward_;
                    float angular_speed = max_speed_angular_;
                    if(tvecs[i][2] < 0.2)
                        speed = 0;

                    //                    angular_speed = direction;
                    y_error = -transform.getOrigin().y() - tvecs[i][0];   // error sideways  // 2cm tolerance

                    std::cout<<"--------------------------error in y -----------------------     "<<y_error<<std::endl;

                    current_angle = abs(rvecs[i][2])*180/M_PI;
                    if(previous_angle!=0)
                    {
                        current_angle  = (current_angle + previous_angle) / 2;    //smoothening the error jumps
                    }

                    float err_angle = goal_position_orientation_ - current_angle;   //goal_position_orientation_angle = 0
                    float delta_error = err_angle - previous_error;


                    if(abs(y_error) < max_y_error_)  // 0.01 is the value by default
                    {
                        if(err_angle < 5 and err_angle > -5)
                            angular_speed = 0;
                        else
                        {
                            if(direction < 0)
                                angular_speed = -abs(err_angle)*0.1 + delta_error*0.1;
                            else
                                angular_speed = abs(err_angle)*0.1 - delta_error*0.1;
                        }
                        angular_speed = 0;
                    }
                    else if(abs(y_error) > obj->max_error_y_axis_)
                    {
                        obj->start_recovery();
                    }
                    else
                    {
                        if(y_error > 0)
                        {
                            angular_speed =  kp*y_error + kd*(y_error - prev_y_error) ;    // 0.1 speed angular
                            //                            std::cout<<" positive"<<std::endl;
                        }
                        else if(y_error <= 0)
                        {
                            angular_speed = kp*y_error - kd*(y_error - prev_y_error);  // -0.1 speed angular
                            //                            std::cout<<" negative"<<std::endl;
                        }
                    }

                    prev_y_error = y_error;

                    //                    if(y_error > obj->max_error_y_axis_ or y_error < -1*obj->max_error_y_axis_)     //out of range measurement
                    //                    {
                    //                        speed = 0;
                    //                    }


                    if (angular_speed > max_speed_angular_)
                        angular_speed = max_speed_angular_;
                    if(angular_speed < -1*max_speed_angular_)
                        angular_speed = -1*max_speed_angular_;
                    cout<<"err "<<err_angle <<"  delta_error "<<delta_error<<std::endl;

                    std::cout<<"angular speed "<<angular_speed<<std::endl;
                    std::cout<<"forward speed "<<speed<<std::endl;

                    move(speed, angular_speed);

                    std::cout<<"------------------------------ camera----------------------------------------";

                    previous_angle = current_angle;
                    previous_error = err_angle;

                    //////////////////////////////////////////////////////////

                    double angle = norm(rvecs[i]);
                    Vec3d axis = rvecs[i] / angle;

                    //                    tf2::Quaternion q;
                    //                    q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);
                    //                    std::cout<<q.x() <<" "<< q.y()<<" "<<q.z() <<std::endl;

                }
            }
            else
                obj->camera_detection = false;
        }

        if (publish_images) {
            image_pub.publish(cv_ptr->toImageMsg());
        }
    }
    catch(cv_bridge::Exception & e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    catch(cv::Exception & e) {
        ROS_ERROR("cv exception: %s", e.what());
    }
}


/**
 * @brief lidar_docking::LidarDocking::pose_callback - checks for the pose of the robot
 * and starts recovery process if the distance along the y axis increases
 * @param msg
 */
void lidar_docking::LidarDocking::pose_callback(const nav_msgs::Odometry::ConstPtr& msg)
{

    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    robot_theta = msg->pose.pose.orientation.z ;

    //    std::cout<<"data from  pose is "<<robot_x <<"   "<<robot_y<<" "<<robot_theta<<std::endl;
    float err_y, err_th;

    err_y = charger_y_ - robot_y_;
    //    err_th = charger_theta_ - robot_theta;
    //    std::cout<<"error is  y and theta "<<err_y <<"  "<< err_th<<std::endl;
    if(abs(err_y) > max_error_y_axis_)
    {
        robot_ready_for_docking = false;
        start_recovery();
    }
    else
        robot_ready_for_docking = true;


}


/**
 * @brief lidar_docking::LidarDocking::readParams reading parameters
 * @param nodeHandle
 */
void lidar_docking::LidarDocking::readParams(ros::NodeHandle& nodeHandle)
{

    nodeHandle.getParam("/camera_lidar_docking_charging_station/resolution", res);
    nodeHandle.getParam("/camera_lidar_docking_charging_station/dist_to_dock_station", dist_to_dock_station);
    nodeHandle.getParam("/camera_lidar_docking_charging_station/marker_slope_lidar_threshold", marker_slope_lidar_threshold);
    nodeHandle.getParam("/camera_lidar_docking_charging_station/marker_intensity", marker_intensity_);
    nodeHandle.getParam("/camera_lidar_docking_charging_station/makrer_intensity_threshold", intensity_threshold_);
    nodeHandle.getParam("/camera_lidar_docking_charging_station/cluster_length", cluster_length_);
    nodeHandle.getParam("/camera_lidar_docking_charging_station/cluster_length_threshold", cluster_length_threshold_);
    nodeHandle.getParam("/camera_lidar_docking_charging_station/dist_camera_to_lidar_switch", dist_camera_to_lidar_switch_);
    nodeHandle.getParam("/camera_lidar_docking_charging_station/speed_translation_max", speed_translation_max_);
    nodeHandle.getParam("/camera_lidar_docking_charging_station/speed_angular_max", speed_angular_max_);
    nodeHandle.getParam("/camera_lidar_docking_charging_station/max_delta_error_slope", max_delta_error_slope_);
    nodeHandle.getParam("/camera_lidar_docking_charging_station/x", charger_x_);
    nodeHandle.getParam("/camera_lidar_docking_charging_station/y", charger_y_);
    nodeHandle.getParam("/camera_lidar_docking_charging_station/theta", charger_theta_);
    nodeHandle.getParam("/camera_lidar_docking_charging_station/min_points_obstacle", min_points_obstacle_);
    nodeHandle.getParam("/camera_lidar_docking_charging_station/max_err_y", max_error_y_axis_);
    nodeHandle.getParam("/camera_lidar_docking_charging_station/kp", kp_);
    nodeHandle.getParam("/camera_lidar_docking_charging_station/kd", kd_);
    nodeHandle.getParam("/camera_lidar_docking_charging_station/time_for_obstacle_clearance", time_for_obstacle_clearance_);
}


/**
 * @brief lidar_docking::LidarDocking::LidarDocking constructor
 */
lidar_docking::LidarDocking::LidarDocking()
{
    ros::NodeHandle nodeHandle("~");
    nodeHandle_ = nodeHandle;
    point_cloud_subscriber_ = nodeHandle_.subscribe("/scan", 10, &LidarDocking::read_point_cloud, this);
    pub = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    cloud_pub_ = nodeHandle.advertise<sensor_msgs::PointCloud2>("point_cloud", 10);
    //    pose_subscriber = nodeHandle.subscribe("/odom", 10, &lidar_docking::LidarDocking::pose_callback, this);
    client = nodeHandle.serviceClient<camera_lidar_docking_charging_station::FinishedDock>("completed_docking");
    readParams(nodeHandle);
}


/**
 * @brief lidar_docking::LidarDocking::LidarDocking constructor with arguments
 * @param nodeHandle
 */
lidar_docking::LidarDocking::LidarDocking(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle)
{
    point_cloud_subscriber_ = nodeHandle_.subscribe("/scan", 10, &LidarDocking::read_point_cloud, this);
    pub = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    cloud_pub_ = nodeHandle.advertise<sensor_msgs::PointCloud2>("point_cloud", 10);
    //    pose_subscriber = nodeHandle.subscribe("/odom", 10, &lidar_docking::LidarDocking::pose_callback, this);
    client = nodeHandle.serviceClient<camera_lidar_docking_charging_station::FinishedDock>("completed_docking");

    readParams(nodeHandle);
}


/**
 * @brief lidar_docking::LidarDocking::rotate_and_check of marker not found, rotate and robot
 * and search for marker
 */
void lidar_docking::LidarDocking::rotate_and_check()
{
    move(0,0.1);
    std::cout<<"rotating"<<std::endl;
    if(abs(robot_theta - charger_theta_) < 0.00010)
    {
        rotation_completed = true;
        std::cout<<"completed one rotation "<<std::endl;
        move(0,0);
    }
}


/**
 * @brief lidar_docking::LidarDocking::intensityClusters forming clusters from point cloud on the basis if intensity
 *and generating command velocities on the basis of cluster of the marker
 * @param cloud
 */
void lidar_docking::LidarDocking::intensityClusters(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{

    std::vector<std::vector<pcl::PointXYZI> > all_clusters;
    std::vector<pcl::PointXYZI> cluster;
    std::vector<int> cluster_indices;
    std::vector<std::vector<int> >all_cluster_indices;
    for(int i = 0 ; i < cloud->size() ; i++)
    {
        if(cloud->points[i].intensity > marker_intensity_ - intensity_threshold_ && cloud->points[i].intensity < marker_intensity_ + intensity_threshold_)
        {
            cluster.push_back(cloud->points[i]);
            cluster_indices.push_back(i);
        }
        else
        {
            cluster.push_back(cloud->points[i]);
            cluster_indices.push_back(i);
            if (cluster.size() > 1)
            {
                all_clusters.push_back(cluster);
                all_cluster_indices.push_back(cluster_indices);
            }
            cluster.clear();
            cluster_indices.clear();
        }
    }

    //        std::cout<<"size of all clusters " <<all_clusters.size()<<std::endl;
    float slope ;
    pcl::PointCloud<pcl::PointXYZI> cluster_cloud;

    for(int i = 0 ; i < all_clusters.size(); i++)
    {
        float len_cluster = find_length(all_clusters[i]);
        std::cout<<"length of cluster "<<len_cluster<<std::endl;
        if(len_cluster > cluster_length_ - cluster_length_threshold_ and len_cluster < cluster_length_ + cluster_length_threshold_)
        {
            marker_visible_lidar_ = true;
            int theta = int(all_clusters[i].size()/(2));
            //final_theta = computeMean(all_cluster_indices[i]);//float(all_cluster_indices[i][0]+ all_cluster_indices[i][all_cluster_indices[i].size() - 1])/float(2);
            final_position_x = all_clusters[i][0].x ;  //
            //final_position_x = all_clusters[i][theta].x;
            std::cout<<(all_clusters[i][theta].x)<< " "<< (all_clusters[i][theta].y)<<std::endl;
            final_position_y = all_clusters[i][0].y; //
            // final_position_y = all_clusters[i][theta].y ;
            distance = final_position_x;
            slope =  atan2((all_clusters[i][0].y - all_clusters[i][all_clusters[i].size()-1].y) ,(all_clusters[i][0].x - all_clusters[i][all_clusters[i].size()-1].x))*180/M_PI;
            std::cout<<"Slope: "<<slope<<std::endl;
            /////
            cluster_cloud = cluster_to_point_cloud(all_clusters[i]);
            /////
            break;
        }
        else
        {
            marker_visible_lidar_ = false;
        }
    }


    if( ((final_position_x < dist_camera_to_lidar_switch_ /*&& (final_position_y < 1 && final_position_y > -1) */) or (camera_detection == false)) && marker_visible_lidar_ ==true){

        std::cout<<"----------------------------------------------lidar --------------------------------"<<std::endl;
        float angle_with_marker = lineFit(cluster_cloud);
        float err_slope = marker_slope_lidar_threshold - slope;    //-112   marker slope threshold
        //        float err_slope = -180 - angle_with_marker  ;
        std::cout<<"slope and error  are :"<<" " << slope<<" "<<(err_slope)<<std::endl;
        std::cout<<final_position_x<<" m away from the charging doc"<<std::endl;

        float speed_x = speed_translation_max_;
        float angular_speed = speed_angular_max_;
        if(docking_complete_)
        {
            camera_lidar_docking_charging_station::FinishedDock srv;
            srv.request.docking_complete = true;
            if (client.call(srv))
            {
//                ROS_INFO("Sum: %ld", (long int)srv.response.sum);'
                std::cout<<srv.response.success<<std::endl;
            }
            else
            {
                ROS_ERROR("Failed to call service complete docking");
            }

        }

        if(distance > 0.5)
        {
            {
                if(err_slope > max_delta_error_slope_)     // err_slope > 2
                    angular_speed = 0.05*abs(err_slope) - 0.05*(err_slope - prev_err_slope);
                else if (err_slope < -max_delta_error_slope_)
                    angular_speed = -0.05*abs(err_slope) + 0.05*(err_slope - prev_err_slope);
                // speed_x = 0;
            }
        }
        else
        {
            if(err_slope > max_delta_error_slope_)
                angular_speed = 0.05*abs(err_slope) - 0.05*(err_slope - prev_err_slope);
            else if(err_slope < -max_delta_error_slope_)
                angular_speed = -0.05*abs(err_slope) + 0.05*(err_slope - prev_err_slope);
            else
                angular_speed = 0;

        }

        if(angular_speed >  speed_angular_max_)
        {
            angular_speed = speed_angular_max_;
        }

        if(angular_speed < -1*speed_angular_max_)
        {
            angular_speed = -1*speed_angular_max_;
        }

        if (final_position_x  <= dist_to_dock_station) //0.16
        {
            speed_x = 0;
            angular_speed = 0;
            std::cout<<"reached charging dock"<<std::endl;
            docking_complete_ = true;

            //            exit(0);
        }
        prev_err_slope = err_slope;
        std::cout<<"angular speed lidar "<<-angular_speed<< " forward speed "<<speed_x<<std::endl;

        move(speed_x, -angular_speed);

        prev_x = final_position_x;
        prev_y = final_position_y;
        prev_theta = final_theta;

    }

}
