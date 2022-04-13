#include <movel_aruco_tools/aruco_amcl.h>


double quaternionToYaw(geometry_msgs::Quaternion q)
{
    double aa = 2.0 * (q.w * q.z + q.y * q.x);
    double bb = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double theta = atan2(aa, bb);
    return theta;
}

geometry_msgs::Quaternion yawToQuaternion(double yaw)
{
    double pitch = 0;
    double roll = 0;
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    geometry_msgs::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

ARUCOAMCL::ARUCOAMCL(): 
    tf_ear_(tf_buffer_),
    initialized_(false),
    max_error_x_(0.5),
    max_error_y_(0.5),
    max_error_theta_(0.5),
    max_dev_x_(0.1),
    max_dev_y_(0.1),
    max_dev_theta_(0.1),
    num_holding_(5),
    cooldown_time_(5.0)
{
    ROS_WARN("[aruco_amcl] Please load the file contaning aruco pose on the map!");
    setupParams();
    setupTopics();   
    // Find transform between d435_color_optical_frame and base_link
    try {
        camera_to_bl_ = tf_buffer_
            .lookupTransform("base_link", optical_frame_ , ros::Time(0.0), ros::Duration(1.0)); 
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("[aruco_amcl] Transform lookup from msg's frame to base_link failed %s", ex.what());
        return;
    }
}

void ARUCOAMCL::setupParams()
{
    last_init_pose_ = ros::Time::now();
    ros::NodeHandle private_nh("~");

    private_nh.getParam("max_error_x", max_error_x_);
    private_nh.getParam("max_error_y", max_error_y_);
    private_nh.getParam("max_error_theta", max_error_theta_);

    private_nh.getParam("max_dev_x", max_dev_x_);
    private_nh.getParam("max_dev_y", max_dev_y_);
    private_nh.getParam("max_dev_theta", max_dev_theta_);

    private_nh.getParam("num_holding", num_holding_);

    private_nh.getParam("cooldown_time", cooldown_time_);
    private_nh.getParam("correction_range", correction_range_);
    private_nh.getParam("optical_frame", optical_frame_);
}

void ARUCOAMCL::setupTopics()
{
    ros::NodeHandle private_nh("~");

    aruco_sub_ = private_nh.subscribe("/fiducial_transforms", 10, &ARUCOAMCL::arucoCallback, this);
    initpose_pub_ = private_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",10);
    load_aruco_serv_ = private_nh.advertiseService("load_aruco", &ARUCOAMCL::loadArucoPoseFile, this);
}

void ARUCOAMCL::arucoCallback(const fiducial_msgs::FiducialTransformArrayPtr &msg)
{
    if(!initialized_ || ros::Time::now() - last_init_pose_ < ros::Duration(cooldown_time_))
        return;

    geometry_msgs::TransformStamped trans_aruco, bl_to_map;

    // Find transform between aruco frame to map
    try {
        trans_aruco = tf_buffer_
            .lookupTransform("map", msg->header.frame_id , ros::Time(0.0), ros::Duration(1.0)); 
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("[aruco_amcl] Transform lookup failed %s", ex.what());
        return;
    }

    // Find current pose
    try {
        bl_to_map = tf_buffer_
            .lookupTransform("map", "base_link" , ros::Time(0.0), ros::Duration(1.0)); 
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("[aruco_amcl] Transform lookup failed %s", ex.what());
        return;
    }

    // Looping for detected aruco
    for (int i = 0 ; i < msg->transforms.size(); i++)
    {
        int id = msg->transforms[i].fiducial_id;

        // Find aruco in the database
        auto found = aruco_map_.find(id);
        if (found != aruco_map_.end())
        {
            geometry_msgs::PoseStamped distance_pose;
            // TRANSFORM THE ARUCO POSE TO MAP FRAME
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = msg->transforms[i].transform.translation.x;
            pose.pose.position.y = msg->transforms[i].transform.translation.y;
            pose.pose.position.z = msg->transforms[i].transform.translation.z;
            pose.pose.orientation = msg->transforms[i].transform.rotation;
            

            // Transform to find distance between base_link and aruco marker
            tf2::doTransform(pose, distance_pose, camera_to_bl_);
            double dist = pow(distance_pose.pose.position.x, 2) + pow(distance_pose.pose.position.y, 2);
            dist = sqrt(dist);
            if (dist <= correction_range_)
            {
                std::cout << "correcting using aruco amcl" <<std::endl;
            // Transform the pose to map
            tf2::doTransform(pose, pose, trans_aruco);

            // // VERBOSE
            // std::cout<<"1. Detected QRCode pose"<<std::endl;
            // std::cout<<"x: "<<pose.pose.position.x<<" ; y: "<<pose.pose.position.y<<" ; th: "<<quaternionToYaw(pose.pose.orientation)<<std::endl;
            
            // std::cout<<"2. Reference QRCode pose"<<std::endl;
            // std::cout<<"x: "<<found->second.x<<" ; y: "<<found->second.y<<" ; th: "<<found->second.theta<<std::endl;

            // Check the error threshold
            double dtheta = fabs(quaternionToYaw(pose.pose.orientation) - found->second.theta);
            if (dtheta > M_PI)
                dtheta -= 2 * M_PI;
            else if (dtheta < -M_PI)
                dtheta += 2* M_PI;

            // If the detected aruco pose is to far from the one in the data base
            // try to save the last n-detection to ensure the pose is correct.
            if ( fabs(pose.pose.position.x - found->second.x) >= max_error_x_ || 
                fabs(pose.pose.position.y - found->second.y) >= max_error_y_ ||
                dtheta >= max_error_theta_)
            {
                // Check on the temp variable
                auto found_flush = aruco_flush_.find(id);
                if(found_flush == aruco_flush_.end())     // If the robot doesn't have any temp yet
                {
                    aruco new_aruco;
                    new_aruco.x = pose.pose.position.x;
                    new_aruco.y = pose.pose.position.y;
                    new_aruco.theta = quaternionToYaw(pose.pose.orientation);
                    new_aruco.n = 1;
                    aruco_flush_[id] = new_aruco;
                    ROS_INFO("[aruco_amcl] Robot fails to localize. Trying to use ARuCO for initialize the pose!");
                }
                else if (found_flush->second.n < num_holding_)     
                {
                    // std::cout<<found_flush->second.n<<std::endl;
                    // check the error threshold
                    double dtheta = fabs(quaternionToYaw(pose.pose.orientation) - found_flush->second.theta);
                    if (dtheta > M_PI)
                        dtheta -= 2 * M_PI;
                    else if (dtheta < -M_PI)
                        dtheta += 2* M_PI;
                    // std::cout<<fabs(pose.pose.position.x - found_flush->second.x)<<" ; "<<fabs(pose.pose.position.y - found_flush->second.y)<<" ; "<<dtheta<<std::endl;
                    if ( fabs(pose.pose.position.x - found_flush->second.x) <= max_dev_x_ && 
                        fabs(pose.pose.position.y - found_flush->second.y) <=max_dev_y_ &&
                        dtheta <= max_dev_theta_)
                    {   
                        found_flush->second.n = found_flush->second.n + 1;
                    }
                    else
                    {
                        found_flush->second.x = pose.pose.position.x;
                        found_flush->second.y = pose.pose.position.y;
                        found_flush->second.theta = quaternionToYaw(pose.pose.orientation);
                        found_flush->second.n = 0;
                        ROS_INFO("[aruco_amcl] Breaks the count. Start from 0 !");
                    }

                }
                else if (found_flush->second.n == num_holding_)
                {
                    ROS_INFO("[aruco_amcl] Initialize pose based on the qr code!");
                    // Find transformation between two identical qr code
                    double theta_transformation = found->second.theta - quaternionToYaw(pose.pose.orientation);

                    if (theta_transformation < -M_PI)
                        theta_transformation += 2 * M_PI;
                    else if (theta_transformation > M_PI)
                        theta_transformation -= 2 * M_PI;

                    // Rotate the base_link from qr code position
                    double bl_ori = quaternionToYaw(bl_to_map.transform.rotation) + theta_transformation;
                    if (bl_ori < -M_PI)
                        bl_ori += 2 * M_PI;
                    else if (bl_ori > M_PI)
                        bl_ori -= 2 * M_PI;
                    // std::cout<<"base_link: "<<bl_to_map.transform.translation.x<< " ; "<<bl_to_map.transform.translation.y<<std::endl;
                    double new_x_aruco = cos(1 * theta_transformation) * (pose.pose.position.x - bl_to_map.transform.translation.x) - sin(1 * theta_transformation) * (pose.pose.position.y - bl_to_map.transform.translation.y) + bl_to_map.transform.translation.x;
                    double new_y_aruco = sin(1 * theta_transformation) * (pose.pose.position.x - bl_to_map.transform.translation.x) + cos(1 * theta_transformation) * (pose.pose.position.y - bl_to_map.transform.translation.y) + bl_to_map.transform.translation.y;
                    // std::cout<<"new aruco : "<<new_x_aruco<<" ; "<<new_y_aruco<<std::endl;
                    // Transform the robot on x and y axis
                    double x_transformation = found->second.x - new_x_aruco;
                    double y_transformation = found->second.y - new_y_aruco;
                    double bl_x = bl_to_map.transform.translation.x + x_transformation;
                    double bl_y = bl_to_map.transform.translation.y + y_transformation;
                    // std::cout<<x_transformation<<" ; "<<y_transformation<<" ; "<<theta_transformation<<std::endl;
                    


                    //Publish initialpose
                    geometry_msgs::PoseWithCovarianceStamped message;
                    message.header.stamp = ros::Time::now();
                    message.header.frame_id = "map";
                    message.pose.pose.position.x = bl_x;
                    message.pose.pose.position.y = bl_y;
                    message.pose.pose.orientation = yawToQuaternion(bl_ori);
                    message.pose.covariance[0] = max_dev_x_;
                    message.pose.covariance[7] = max_dev_y_;
                    message.pose.covariance[35] = max_dev_theta_;
                    initpose_pub_.publish(message);

                    last_init_pose_ = ros::Time::now();
                    //Erase particular map
                    aruco_flush_.erase(found_flush);
                    ROS_INFO("[aruco_amcl] THE ROBOT HAS BEEN INTIALIZED");
                }
            }
        }
        }
        else
        {
            ROS_WARN("[aruco_amcl] Aruco id %d is not found in the database file. Please do remapping!", msg->transforms[i].fiducial_id);
            continue;
        }
    }
}


bool ARUCOAMCL::loadArucoPoseFile(movel_seirios_msgs::StringTrigger::Request& req,
                                  movel_seirios_msgs::StringTrigger::Response& res)
{
    aruco_map_.clear();
    aruco_flush_.clear();
    ROS_INFO("[aruco_amcl] Loading %s file to the map!",req.input.c_str());
    std::ifstream myfile(req.input);
    int id;
    double x,y,theta;
    if (myfile.is_open())
    {
        while (myfile >> id >> x >> y >> theta)
        {
            aruco new_marker;
            new_marker.x = x;
            new_marker.y = y;
            new_marker.theta = theta;
            aruco_map_[id] = new_marker;
        }
    myfile.close();
    res.success = true;
    ROS_INFO("[aruco_amcl] LOADED!");
    initialized_ = true;
    }
    else 
    {
        res.message = "[aruco_amcl] Invalid full path!";
        res.success = false;
    }
    return true;
}

int main(int argc, char* argv[])
{
#ifdef MOVEL_LICENSE
  MovelLicense ml;
  if (!ml.login())
    return 1;
#endif
    ros::init(argc, argv,"movel_aruco_amcl");
    ARUCOAMCL aruco_amcl;
    while(ros::ok())
    {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
#ifdef MOVEL_LICENSE
  ml.logout();
#endif
    return 0;
}