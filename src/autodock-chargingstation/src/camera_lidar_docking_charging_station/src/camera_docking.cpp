#include <ros/ros.h>
#include <camera_docking_/camera_docking.h>

using namespace std;
using namespace cv;



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
 * @brief FiducialsNode::calcFiducialArea Compute area in image of a fiducial
 * to find the area of two triangles
 * @param pts
 * @return
 */
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
 * @brief FiducialsNode::getReprojectionError-- estimate reprojection error
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


/**
 * @brief FiducialsNode::estimatePoseSingleMarkers -- estimate the pose of marker
 * @param ids
 * @param corners
 * @param markerLength
 * @param cameraMatrix
 * @param distCoeffs
 * @param rvecs
 * @param tvecs
 * @param reprojectionError
 */
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
        //        cv::solvePnPRansac(markerObjPoints, corners[i], cameraMatrix, distCoeffs,
        //                     rvecs[i], tvecs[i],  1000);

        reprojectionError[i] =
                getReprojectionError(markerObjPoints, corners[i],
                                     cameraMatrix, distCoeffs,
                                     rvecs[i], tvecs[i]);
    }
}

/**
 * @brief FiducialsNode::move  - move robot with a specific speed
 * @param linear_speed
 * @param angular_speed
 */
void FiducialsNode::move(float linear_speed, float angular_speed)
{
    ptr.linear.x = linear_speed;
    ptr.linear.y = 0;
    ptr.linear.z = 0;
    ptr.angular.x = 0;
    ptr.angular.y = 0;
    ptr.angular.z = angular_speed;

    pub.publish(ptr);
}

/**
 * @brief FiducialsNode::camInfoCallback
 * @param msg
 */
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


/**
 * @brief FiducialsNode::computeImageSide - check which side of the marker is the robot
 * @param image
 * @param corners
 * @param ids
 * @return
 */
float FiducialsNode::computeImageSide(Mat image, vector <vector <Point2f> > corners, vector<int>ids)
{
    int rows = image.rows;
    int cols = image.cols;
    cv::Point mid_pt = cv::Point(rows/2,cols/2);
    float sum_x = 0, sum_y = 0;

    for(int i = 0 ; i < ids.size(); i++)
    {

        if(ids[i] == marker_for_docking_)
        {
            for(int j =0; j < 4; j++)
            {

                //                std::cout<<"[ " <<int(corners[i][j].x)<<" , "<<int(corners[i][j].y)<<" ";
                sum_x+= int(corners[i][j].x);
                sum_y+= int(corners[i][j].y);
            }
        }
    }

    int mean_x_corner = sum_x/4;
    int mean_y_corner = sum_y/4;
    cv::Point mean_corner = Point(mean_x_corner, mean_y_corner);

    //    std::cout<<"mean corner "<< mean_corner <<" mid point image "<<mid_pt<<std::endl;

    if (mid_pt.y > mean_corner.x)
    {
        //        std::cout<<"go left"<<std::endl;
        return float((mid_pt.y - mean_corner.x)/float(mid_pt.y));
    }

    else
    {
//        std::cout<<"go right"<<std::endl;
        return  float((mid_pt.y - mean_corner.x)/float(mid_pt.y));
    }

}



/**
 * @brief FiducialsNode::handleIgnoreString
 * @param str
 */
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


/**
 * @brief FiducialsNode::enableDetectionsCallback
 * @param req
 * @param res
 * @return
 */
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


