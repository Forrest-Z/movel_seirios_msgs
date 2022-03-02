#include <movel_aruco_tools/aruco_saver.h>


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
ARUCOSaver::ARUCOSaver(): tf_ear_(tf_buffer_)
{
    setupParams();
    setupTopics();
    // Find transform between d435_color_optical_frame and base_link
    try {
        camera_to_bl_ = tf_buffer_
            .lookupTransform("base_link", "d435_color_optical_frame" , ros::Time(0.0), ros::Duration(1.0)); 
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("[aruco_amcl] Transform lookup from msg's frame to base_link failed %s", ex.what());
        return;
    }
}

void ARUCOSaver::setupParams()
{
    ros::NodeHandle private_nh("~");
    private_nh.getParam("correction_range", correction_range_);
}

void ARUCOSaver::setupTopics()
{
    ros::NodeHandle private_nh("~");

    aruco_sub_ = private_nh.subscribe("/fiducial_transforms", 1, &ARUCOSaver::arucoCallback, this);
    aruco_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("aruco_markers",1);
    save_aruco_ = private_nh.advertiseService("save_aruco", &ARUCOSaver::savePoseToFile, this);
}


void ARUCOSaver::arucoCallback(const fiducial_msgs::FiducialTransformArrayPtr &msg)
{
    geometry_msgs::TransformStamped trans;
    try {
        trans = tf_buffer_
            .lookupTransform("map", msg->header.frame_id , ros::Time(0.0), ros::Duration(1.0)); 
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("[aruco_saver] Transform lookup failed %s", ex.what());
        return;
    }
    visualization_msgs::MarkerArray markers;
    for (int i = 0 ; i < msg->transforms.size(); i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = msg->transforms[i].transform.translation.x;
        pose.pose.position.y = msg->transforms[i].transform.translation.y;
        pose.pose.position.z = msg->transforms[i].transform.translation.z;
        pose.pose.orientation = msg->transforms[i].transform.rotation;
        geometry_msgs::PoseStamped distance_pose;
        tf2::doTransform(pose, distance_pose, camera_to_bl_);
        double dist = pow(distance_pose.pose.position.x, 2) + pow(distance_pose.pose.position.y, 2);
        dist = sqrt(dist);
        auto found = aruco_map_.find(msg->transforms[i].fiducial_id);
        if (dist <= correction_range_){
        // Transform the pose to map
        tf2::doTransform(pose, pose, trans);
        
        // std::cout<<"Original pose on the map frame"<<std::endl;
        // std::cout<<"x: "<<pose.pose.position.x<<" ; y: "<<pose.pose.position.y<<" ; th: "<<quaternionToYaw(pose.pose.orientation)<<std::endl;
        if(found == aruco_map_.end()) 
        {
            ROS_INFO("[aruco_saver] Aruco markers with %d is not found yet", msg->transforms[i].fiducial_id);
            aruco new_aruco;
            new_aruco.x = pose.pose.position.x;
            new_aruco.y = pose.pose.position.y;
            new_aruco.theta = quaternionToYaw(pose.pose.orientation);
            new_aruco.n = 1;
            new_aruco.xvar = 0;
            new_aruco.yvar = 0;
            aruco_map_[msg->transforms[i].fiducial_id] = new_aruco;
        }
        else
        {
            // std::cout<<(msg->transforms[i].fiducial_id)<<std::endl;
            // std::cout<<"Before"<<std::endl;
            // std::cout<<"x: "<<(found->second.x)<<" ; y: "<<(found->second.y)<<" ; th: "<<(found->second.x)<<" ; n: "<<(int)(found->second.n)<<std::endl;
            // std::cout<<"xvar: "<<(found->second.xvar)<<" ; yvar: "<<(found->second.yvar)<<std::endl;
            double dtheta = fabs(quaternionToYaw(pose.pose.orientation) - found->second.theta);
            if (dtheta > M_PI)
                dtheta -= 2 * M_PI;
            else if (dtheta < -M_PI)
                dtheta += 2* M_PI;

            if ( ( -3 <=(pose.pose.position.x - found->second.x) / sqrt(found->second.xvar) <= 3 ) &&
                 ( -3 <=(pose.pose.position.y - found->second.y) / sqrt(found->second.yvar) <= 3 ) &&
                 ( dtheta <= M_PI_4))
            {
                found->second.xvar = ((found->second.xvar + pow(found->second.x, 2)) * found->second.n + pow(pose.pose.position.x, 2)) / (found->second.n + 1);
                found->second.yvar = ((found->second.yvar + pow(found->second.y, 2)) * found->second.n + pow(pose.pose.position.y, 2)) / (found->second.n + 1);

                found->second.x = ( (found->second.x * found->second.n) + pose.pose.position.x) / (found->second.n + 1);
                found->second.y = ( (found->second.y * found->second.n) + pose.pose.position.y) / (found->second.n + 1);
                found->second.theta = quaternionToYaw(pose.pose.orientation);
                found->second.xvar = found->second.xvar - pow(found->second.x, 2);
                found->second.yvar = found->second.yvar - pow(found->second.y, 2);
                if (found->second.n <5)
                    found->second.n += 1;
                // std::cout<<"After"<<std::endl;
                // std::cout<<"x: "<<(found->second.x)<<" ; y: "<<(found->second.y)<<" ; th: "<<quaternionToYaw(pose.pose.orientation)<<" ; n: "<<(int)(found->second.n)<<std::endl;
                // std::cout<<"xvar: "<<(found->second.xvar)<<" ; yvar: "<<(found->second.yvar)<<std::endl<<std::endl;
                //     std::cout<<"Number of aruco in map: "<<aruco_map_.size()<<std::endl;
            }

        }
        }
        visualization_msgs::Marker marker_start;
        marker_start.header.frame_id = "map";
        marker_start.header.stamp = ros::Time::now();
        marker_start.id = msg->transforms[i].fiducial_id;
        uint32_t shape = visualization_msgs::Marker::CUBE;
        marker_start.type = shape;
        marker_start.action =  visualization_msgs::Marker::MODIFY;

        geometry_msgs::Quaternion q = yawToQuaternion(found->second.theta);
        marker_start.pose.position.x = found->second.x;
        marker_start.pose.position.y = found->second.y;
        marker_start.pose.position.z = 0.1;
        marker_start.pose.orientation.x = q.x;
        marker_start.pose.orientation.y = q.y;
        marker_start.pose.orientation.z = q.z;
        marker_start.pose.orientation.w = q.w;

        // Set the scale of the marker_start -- 1x1x1 here means 1m on a side
        marker_start.scale.x = 0.18;
        marker_start.scale.y = 0.05;
        marker_start.scale.z = 0.18;

        // Set the color -- be sure to set alpha to something non-zero!
        marker_start.color.r = 1.0f;
        marker_start.color.g = 0.0f;
        marker_start.color.b = 0.0f;
        marker_start.color.a = 1.0;

        marker_start.ns = "fiducial";
        // marker_start.lifetime = ros::Duration(1.0);
        markers.markers.push_back(marker_start);

    }
    aruco_pub_.publish(markers);
}

bool ARUCOSaver::savePoseToFile(movel_seirios_msgs::StringTrigger::Request& req,
                               movel_seirios_msgs::StringTrigger::Response& res)
{
    std::ofstream stream(req.input);
    for(auto& kv : aruco_map_)
    {   
        stream << kv.first << ' ';
        stream << kv.second.x << ' ';
        stream << kv.second.y << ' ';
        stream << kv.second.theta << '\n';
    }
    stream.close();
    res.success = true;
    ROS_INFO("[aruco_saver] Saved at %s", req.input.c_str());
    return true;
}

int main(int argc, char* argv[])
{
#ifdef MOVEL_LICENSE
  MovelLicense ml;
  if (!ml.login())
    return 1;
#endif
    ros::init(argc, argv,"movel_aruco_saver");
    ARUCOSaver aruco_slam;
    ros::spin();
#ifdef MOVEL_LICENSE
  ml.logout();
#endif
    return 0;
}