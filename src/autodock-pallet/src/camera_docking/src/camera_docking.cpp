#include <camera_docking/camera_docking.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;
using namespace cv;


/* Functionality and Data Path 

    CONSTRUCTOR                             : FiducialsNode = Init all the things
        - handleIgnoreString                : ignogre fiducials can take comma separated list of individual fiducial ids or ranges, eg "1,4,8,9-12,30-40"
    - ImageCallback                         : retrieving the image from front camera
        - rotate                            : rotate the robot to search for the markers
        - estimatePoseSingleMarker          : calculate pose single marker
            - getSingleMarkerObjectPoints   : set coordinate system in the middle of the marker, with Z pointing out
            - getReprojectionError          : projection error
    - ImageCallback_rear                    : retrieving the image from rear camera
        -rotate                             : rotate the robot to search for the markers
        - estimatePoseSingleMarker          : calculate pose single marker
            - getSingleMarkerObjectPoints   : set coordinate system in the middle of the marker, with Z pointing out
            - getReprojectionError          : projection error
    - camInfoCallback                       : retrieving the information about the camera from a topic

*/

/**
  * @brief Return object points for the system centered in a single marker, given the marker length
  */
void FiducialsNode::getSingleMarkerObjectPoints(float markerLength, vector<Point3f>& objPoints) {

    CV_Assert(markerLength > 0);

    // set coordinate system in the middle of the marker, with Z pointing out
    objPoints.clear();
    objPoints.push_back(Vec3f(-markerLength / 2.f, markerLength / 2.f, 0));
    objPoints.push_back(Vec3f( markerLength / 2.f, markerLength / 2.f, 0));
    objPoints.push_back(Vec3f( markerLength / 2.f,-markerLength / 2.f, 0));
    objPoints.push_back(Vec3f(-markerLength / 2.f,-markerLength / 2.f, 0));
}


/**
  * @brief Return eucledian distance between 2 points
  */
double FiducialsNode::dist(const cv::Point2f &p1, const cv::Point2f &p2)
{
    double x1 = p1.x;
    double y1 = p1.y;
    double x2 = p2.x;
    double y2 = p2.y;

    double dx = x1 - x2;
    double dy = y1 - y2;

    return sqrt(dx*dx + dy*dy);
}


/**
 * Compute area in image of a fiducial, using Heron's formula
 * to find the area of two triangles
 **/

/* ------------------ DEPRECIATED --------------------*/
double FiducialsNode::calcFiducialArea(const std::vector<cv::Point2f> &pts)
{
    const Point2f &p0 = pts.at(0);
    const Point2f &p1 = pts.at(1);
    const Point2f &p2 = pts.at(2);
    const Point2f &p3 = pts.at(3);

    double a1 = dist(p0, p1);
    double b1 = dist(p0, p3);
    double c1 = dist(p1, p3);

    double a2 = dist(p1, p2);
    double b2 = dist(p2, p3);
    double c2 = c1;

    double s1 = (a1 + b1 + c1) / 2.0;
    double s2 = (a2 + b2 + c2) / 2.0;

    a1 = sqrt(s1*(s1-a1)*(s1-b1)*(s1-c1));
    a2 = sqrt(s2*(s2-a2)*(s2-b2)*(s2-c2));
    return a1+a2;
}


/**
 * @brief FiducialsNode::getReprojectionError  estimate reprojection error
 * @param objectPoints
 * @param imagePoints
 * @param cameraMatrix
 * @param distCoeffs
 * @param rvec
 * @param tvec
 * @return
 */
double FiducialsNode::getReprojectionError(const vector<Point3f> &objectPoints,
                                           const vector<Point2f> &imagePoints,
                                           const Mat &cameraMatrix, const Mat  &distCoeffs,
                                           const Vec3d &rvec, const Vec3d &tvec) {

    vector<Point2f> projectedPoints;

    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix,
                      distCoeffs, projectedPoints);

    // calculate RMS image error
    double totalError = 0.0;

    for (unsigned int i=0; i<objectPoints.size(); i++) {
        double error = dist(imagePoints[i], projectedPoints[i]);
        totalError += error*error;
    }
    double rerror = totalError/(double)objectPoints.size();
    return rerror;
}


void FiducialsNode::estimatePoseSingleMarkers(const vector<int> &ids,
                                              const vector<vector<Point2f > >&corners,
                                              float markerLength,
                                              const cv::Mat &cameraMatrix,
                                              const cv::Mat &distCoeffs,
                                              vector<Vec3d>& rvecs, vector<Vec3d>& tvecs,
                                              vector<double>& reprojectionError) {

    CV_Assert(markerLength > 0);

    vector<Point3f> markerObjPoints;
    int nMarkers = (int)corners.size();
    rvecs.reserve(nMarkers);
    tvecs.reserve(nMarkers);
    reprojectionError.reserve(nMarkers);

    // for each marker, calculate its pose
    for (int i = 0; i < nMarkers; i++) {
        double fiducialSize = markerLength;

        std::map<int, double>::iterator it = fiducialLens.find(ids[i]);
        if (it != fiducialLens.end()) {
            fiducialSize = it->second;
        }

        getSingleMarkerObjectPoints(fiducialSize, markerObjPoints);
        cv::solvePnP(markerObjPoints, corners[i], cameraMatrix, distCoeffs,
                     rvecs[i], tvecs[i]);

        reprojectionError[i] =
                getReprojectionError(markerObjPoints, corners[i],
                                     cameraMatrix, distCoeffs,
                                     rvecs[i], tvecs[i]);
        markers_tvec_x.push_back(tvecs[i][2]);
        markers_tvec_y.push_back(tvecs[i][0]);
        markers_rvec_theta.push_back((rvecs[i][2]));

    }
}


void FiducialsNode::camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    if (haveCamInfo) {
        return;
    }

    if (msg->K != boost::array<double, 9>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})) {
        for (int i=0; i<3; i++) {
            for (int j=0; j<3; j++) {
                cameraMatrix.at<double>(i, j) = msg->K[i*3+j];
            }
        }

        for (int i=0; i<5; i++) {
            distortionCoeffs.at<double>(0,i) = msg->D[i];
        }

        haveCamInfo = true;
        frameId = msg->header.frame_id;
    }
    else {
        ROS_WARN("%s", "CameraInfo message has invalid intrinsics, K matrix all zeros");
    }
}

/* ------------------ DEPRECIATED --------------------*/
map<int, int> count_marker_ids( vector <int>  ids)
{
    map<int , int> count_ids;
    sort(ids.begin(), ids.end());

    for(int i = 0; i < ids.size(); i+2)
    {
        int count_id = count(ids.begin(), ids.end() , ids[i]);
        count_ids.insert(pair<int, int>(ids[i], count_id));
    }

    return count_ids;
}


void FiducialsNode::move(float translation_speed, float angular_speed)
{
    geometry_msgs::Twist ptr;
    ptr.linear.x = translation_speed;
    ptr.linear.y = 0;
    ptr.linear.z = 0;
    ptr.angular.x = 0;
    ptr.angular.y = 0;
    ptr.angular.z = angular_speed;

        pub.publish(ptr);
}


void FiducialsNode::serachMarkerCallback(const std_msgs::Bool::ConstPtr &msg)
{
    search_for_markers_ = msg->data;
    std::cout<<"search for markers is "<<search_for_markers_<<std::endl;
}


void FiducialsNode::rotate(float speed_translation, float speed_angular)
{
       move(speed_translation, speed_angular);
}


void FiducialsNode::imageCallback(const sensor_msgs::ImageConstPtr & msg) {
    if (enable_detections == false) {
        return; //return without doing anything
    }


    //    ROS_INFO("Got image %d", msg->header.seq);
    frameNum++;

    cv_bridge::CvImagePtr cv_ptr;
    tf::TransformListener listener;
    tf::StampedTransform transform;

    try
    {
        listener.waitForTransform("odom", "camera_rgb_frame", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("odom", "camera_rgb_frame", ros::Time(0), transform);
        std::cout<<"transformations between camera_rgb_frame and odom are as follows "<<transform.getOrigin().y()<<" "<<transform.getOrigin().x()<<std::endl;
    } catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        vector <int>  ids;
        vector <vector <Point2f> > corners, rejected;
        vector <Vec3d>  rvecs;
        vector<Vec3d> tvecs = {0,0,0};

        aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids, detectorParams);
        ROS_INFO("Detected %d markers", (int)ids.size());

        // If the marker < 2
        if(ids.size() < 2)
        {
            std::cout<<"marker found "<<ids.size()<<std::endl;
            if(search_for_markers_)
            {
                std::cout<<"rotating "<<std::endl;
                rotate(0, 0.15);
            }
        }
        else if(ids.size() ==2) {       //Found 2 markers
            aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
            //            map<int, int> marker_counts = count_marker_ids(ids);
            //        }

            if (doPoseEstimation) {
                if (!haveCamInfo) {
                    if (frameNum > 5) {
                        ROS_ERROR("No camera intrinsics");
                    }
                    return;
                }

                vector <double>reprojectionError;
                markers_tvec_x.clear();
                markers_tvec_y.clear();
                markers_rvec_theta.clear();
                estimatePoseSingleMarkers(ids, corners, (float)fiducial_len,
                                          cameraMatrix, distortionCoeffs,
                                          rvecs, tvecs,
                                          reprojectionError);


                float dy = (markers_tvec_y[0] + markers_tvec_y[1])/2;   // error wrt to robot in y direction of robot
                float dx = (markers_tvec_x[0] + markers_tvec_x[1])/2;
                float dtheta = (markers_rvec_theta[0] + markers_rvec_theta[1]);
                std::cout<<"rotation is "<< transform.getRotation().z()<<std::endl;
                tf::Quaternion q(
                            transform.getRotation().x(),
                            transform.getRotation().y(),
                            transform.getRotation().z(),
                            transform.getRotation().w());

                tf::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                dy = -dy;  // for proper coordinate system

                std::cout<<"roll pitch yaw "<< roll<<" "<<pitch <<" "<< yaw<<std::endl;
                std::cout<<"dy  is "<<dy <<std::endl;
                std::cout<<"dx  is "<<dx <<std::endl;
                std::cout<<" dtheta is "<<dtheta<<std::endl;
                if(ids.size() <=1)
                {

                }

                else
                {
                    static tf::TransformBroadcaster br;     // TF broadcaster
                    tf::Transform transform;
                    transform.setOrigin( tf::Vector3(dx, dy, 0.0) );
                    tf::Quaternion q_ob;
                    q_ob.setRPY(0, 0, 0);
                    transform.setRotation(q_ob);
                    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_rgb_frame", "object"));
                    std_msgs::String msg;
                    msg.data ="front" ;
                    detection_side.publish(msg);
                }

                if (publish_images) {
                    image_pub.publish(cv_ptr->toImageMsg());
                }
            }
        }
    }
    catch(cv::Exception & e) {
        ROS_ERROR("cv exception: %s", e.what());
    }
}


void FiducialsNode::imageCallback_rear(const sensor_msgs::ImageConstPtr & msg) {
    if (enable_detections == false) {
        return; //return without doing anything
    }


    //    ROS_INFO("Got image %d", msg->header.seq);
    frameNum++;

    cv_bridge::CvImagePtr cv_ptr;
    tf::TransformListener listener;
    tf::StampedTransform transform;

    try
    {
        listener.waitForTransform("odom", "camera_rgb_frame_rear", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("odom", "camera_rgb_frame_rear", ros::Time(0), transform);
        std::cout<<"transformations between camera_rgb_frame_rear and odom are as follows in rear "<<transform.getOrigin().y()<<" "<<transform.getOrigin().x()<<std::endl;
    } catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        vector <int>  ids;
        vector <vector <Point2f> > corners, rejected;
        vector <Vec3d>  rvecs;
        vector<Vec3d> tvecs = {0,0,0};

        aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids, detectorParams);
        ROS_INFO("Detected %d markers", (int)ids.size());

        if(ids.size() < 2)
        {
            std::cout<<"marker found "<<ids.size()<<std::endl;
            if(search_for_markers_)
            {
                std::cout<<"rotating "<<std::endl;
                rotate(0, 0.15);
            }
        }
        else if(ids.size() ==2) {
            aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
            //            map<int, int> marker_counts = count_marker_ids(ids);
            //        }

            if (doPoseEstimation) {
                if (!haveCamInfo) {
                    if (frameNum > 5) {
                        ROS_ERROR("No camera intrinsics");
                    }
                    return;
                }

                vector <double>reprojectionError;
                markers_tvec_x.clear();
                markers_tvec_y.clear();
                markers_rvec_theta.clear();
                estimatePoseSingleMarkers(ids, corners, (float)fiducial_len,
                                          cameraMatrix, distortionCoeffs,
                                          rvecs, tvecs,
                                          reprojectionError);


                float dy = (markers_tvec_y[0] + markers_tvec_y[1])/2;   // error wrt to robot in y direction of robot
                float dx = (markers_tvec_x[0] + markers_tvec_x[1])/2;
                float dtheta = (markers_rvec_theta[0] + markers_rvec_theta[1]);
                std::cout<<"rotation is "<< transform.getRotation().z()<<std::endl;
                tf::Quaternion q(
                            transform.getRotation().x(),
                            transform.getRotation().y(),
                            transform.getRotation().z(),
                            transform.getRotation().w());

                tf::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                dy = -dy;  // for proper coordinate system

                std::cout<<"roll pitch yaw rear "<< roll<<" "<<pitch <<" "<< yaw<<std::endl;
                std::cout<<"dy  is rear "<<dy <<std::endl;
                std::cout<<"dx  is rear "<<dx <<std::endl;
                std::cout<<" dtheta is "<<dtheta<<std::endl;
                if(ids.size() <=1)
                {

                }

                else
                {

                    static tf::TransformBroadcaster br;
                    tf::Transform transform;
                    transform.setOrigin( tf::Vector3(dx, dy, 0.0) );
                    tf::Quaternion q_ob;
                    q_ob.setRPY(3.14, 0, 0);
                    transform.setRotation(q_ob);

                    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_rgb_frame_rear", "object"));
                    std_msgs::String msg;
                    msg.data ="rear" ;
                    detection_side.publish(msg);
                }

               // if (publish_images) {
                //    image_pub.publish(cv_ptr->toImageMsg());
               // }
            }
        }
    }
    catch(cv::Exception & e) {
        ROS_ERROR("cv exception: %s", e.what());
    }
}


void FiducialsNode::handleIgnoreString(const std::string& str)
{
    /*
    ignogre fiducials can take comma separated list of individual
    fiducial ids or ranges, eg "1,4,8,9-12,30-40"
    */
    std::vector<std::string> strs;
    boost::split(strs, str, boost::is_any_of(","));
    for (const string& element : strs) {
        if (element == "") {
            continue;
        }
        std::vector<std::string> range;
        boost::split(range, element, boost::is_any_of("-"));
        if (range.size() == 2) {
            int start = std::stoi(range[0]);
            int end = std::stoi(range[1]);
            //            ROS_INFO("Ignoring fiducial id range %d to %d", start, end);
            for (int j=start; j<=end; j++) {
                ignoreIds.push_back(j);
            }
        }
        else if (range.size() == 1) {
            int fid = std::stoi(range[0]);
            //            ROS_INFO("Ignoring fiducial id %d", fid);
            ignoreIds.push_back(fid);
        }
        else {
            ROS_ERROR("Malformed ignore_fiducials: %s", element.c_str());
        }
    }
}


bool FiducialsNode::enableDetectionsCallback(std_srvs::SetBool::Request &req,
                                             std_srvs::SetBool::Response &res)
{
    enable_detections = req.data;
    if (enable_detections){
        res.message = "Enabled aruco detections.";
        ROS_INFO("Enabled aruco detections.");
    }
    else {
        res.message = "Disabled aruco detections.";
        ROS_INFO("Disabled aruco detections.");
    }

    res.success = true;
    return true;
}


FiducialsNode::FiducialsNode() : nh(), pnh("~"), it(nh)
{
    frameNum = 0;       // for camera intrinsic purposes

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

    // Publish
    image_pub = it.advertise("/fiducial_images", 1);
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    detection_side = pnh.advertise<std_msgs::String>("detection_side",10);
    service_enable_detections = nh.advertiseService("enable_detections",
                                                    &FiducialsNode::enableDetectionsCallback, this);

    dictionary = aruco::getPredefinedDictionary(dicno);

    // Subscribe
    img_sub = it.subscribe("/camera/rgb/image_raw", 1,
                           &FiducialsNode::imageCallback, this);
    img_sub_rear = it.subscribe("/camera_rear/rgb/image_raw_rear", 1,
                           &FiducialsNode::imageCallback_rear, this);
    search_marker_sub = pnh.subscribe("/camera_lidar_docking/search_markers", 100,
                                     &FiducialsNode::serachMarkerCallback, this);
    caminfo_sub = nh.subscribe("/camera/rgb/camera_info", 1,
                               &FiducialsNode::camInfoCallback, this);
    // caminfo_sub_rear = nh.subscribe("/camera/rgb/camera_info_rear", 1,
    //                          &FiducialsNode::camInfoCallback_rear, this);


    ROS_INFO("Aruco detection ready");
}


