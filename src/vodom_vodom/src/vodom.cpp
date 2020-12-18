#include <vodom_vodom/vodom.hpp>

RGBDVodom::RGBDVodom(ros::NodeHandle nh) : 
nh_(nh), rgb_dep_sync_(rgb_sub_, dep_sub_, 1), N_keypoints_(512), nn_distance_ratio_(0.8)
, rotation_integral_(3, 3, CV_64F), translation_integral_(3, 1, CV_64F)
, matcher_(cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING)) // for ORB only
//, matcher_(cv::DescriptorMatcher::create(cv::NORM_HAMMING)) // for LIOF or ORB
{
  setupTopics();
  setupCameras();
  setupParams();

  R_opt_to_cam_ = (cv::Mat_<double>(3, 3, CV_64F) << 0.0,  0.0,  1.0,
                                                    -1.0,  0.0,  0.0,
                                                     0.0, -1.0,  0.0);
  T_opt_to_cam_ = Mat::zeros(3, 1, CV_64F);

  R_cam_to_opt_ = R_opt_to_cam_.t();
  T_cam_to_opt_ = -R_cam_to_opt_ * T_opt_to_cam_;

  detector_ = cv::ORB::create(N_keypoints_);
  // detector_ = std::shared_ptr<feature_detector::LIOF>(new feature_detector::LIOF());
  // detector_ = std::shared_ptr<feature_detector::LIOF>(new feature_detector::LIOF(32, 8, 0.03, 3.00, 25));
  rotation_integral_ = Mat::eye(3, 3, CV_64F);
  translation_integral_ = Mat::zeros(3, 1, CV_64F);

  ros::spin();
}

void RGBDVodom::setupTopics()
{
  rgb_sub_.subscribe(nh_, "/camera/color/image_raw", 1);
  dep_sub_.subscribe(nh_, "/camera/depth/image_rect_raw", 1);
  rgb_dep_sync_.registerCallback(boost::bind(&RGBDVodom::rgbdPairCb, this, _1, _2));

  feat_pub_ = nh_.advertise<Image>("/vodom/features", 1);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/vodom/odom", 1);

  reset_sub_ = nh_.subscribe("/vodom/reset", 1, &RGBDVodom::resetOdometryCb, this);
  reset_rot_sub_ = nh_.subscribe("/vodom/reset_rotation", 1, &RGBDVodom::resetRotationCb, this);
}

void RGBDVodom::filterMatches(vector< vector<cv::DMatch> > matches, 
                              vector<cv::DMatch> &good_matches,
                              double distance_ratio)
{
  for (int i = 0; i < matches.size(); i++)
  {
    double d1, d2;
    d1 = matches[i][0].distance;
    d2 = matches[i][1].distance;
    if (d1 < d2*distance_ratio)
      good_matches.push_back(matches[i][0]);
  }
}

void RGBDVodom::setupCameras()
{
  ROS_INFO("Waiting for camera infos");
  sensor_msgs::CameraInfoConstPtr rgb_cam_info = 
    ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/color/camera_info");
  sensor_msgs::CameraInfoConstPtr dep_cam_info = 
    ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/depth/camera_info");

  Mat rgb_proj_tmp, dep_proj_tmp;
  rgb_proj_tmp = cv::Mat(3, 3, CV_64FC1, (void *) rgb_cam_info->K.data());
  rgb_proj_ = rgb_proj_tmp.clone();
  ROS_INFO_STREAM("rgb projection " << std::endl << rgb_proj_);
  
  dep_proj_tmp = cv::Mat(3, 3, CV_64FC1, (void *) dep_cam_info->K.data());
  dep_proj_ = dep_proj_tmp.clone();
  ROS_INFO_STREAM("depth projection " << std::endl << dep_proj_);
}

void RGBDVodom::setupParams()
{
  ros::NodeHandle nh_local("~");

  if (nh_local.hasParam("N_keypoints"))
    nh_local.getParam("N_keypoints", N_keypoints_);
  if (nh_local.hasParam("nn_distance_ratio"))
    nh_local.getParam("nn_distance_ratio", nn_distance_ratio_);
  if (nh_local.hasParam("min_valid_depth"))
    nh_local.getParam("min_valid_depth", min_valid_depth_);
  if (nh_local.hasParam("max_valid_depth"))
    nh_local.getParam("max_valid_depth", max_valid_depth_);
  if (nh_local.hasParam("max_translation"))
    nh_local.getParam("max_translation", max_translation_);
  if (nh_local.hasParam("max_roll"))
    nh_local.getParam("max_roll", max_roll_);
  if (nh_local.hasParam("max_pitch"))
    nh_local.getParam("max_pitch", max_pitch_);
  if (nh_local.hasParam("max_yaw"))
    nh_local.getParam("max_yaw", max_yaw_);
}

void RGBDVodom::getObjectPts(Mat &dep, vector<cv::KeyPoint> &kps, vector<cv::Point3d> &obj_pts,
                             vector<int> &invalid_kp_indices)
{
  Mat dep_proj_inv = dep_proj_.inv();
  for (int i = 0; i < kps.size(); i++)
  {
    // get depth from a window around the keypoint
    int cx = kps[i].pt.x;
    int cy = kps[i].pt.y;
    double z = dep.at<double>(cy, cx)/1000.0;
    if (std::isnan(z) || z <= min_valid_depth_ || z >= max_valid_depth_)
      z = max_valid_depth_;

    for (int ix = cx-2; ix <= cx+2; ix++)
    {
      for (int iy = cy-2; iy <= cy+2; iy++)
      {
        double z_i = dep.at<double>(iy, ix)/1000.0;
        if (z_i < z && !std::isnan(z_i) && z_i > min_valid_depth_ && z < max_valid_depth_)
          z = z_i;
      }
    }
    // ROS_INFO("final z: %5.2f", z);
    if (z >= max_valid_depth_)
    {
      invalid_kp_indices.push_back(i);
      continue;
    }

    // calculate xyz coördinate
    Mat uvw = (cv::Mat_<double>(3, 1) << kps[i].pt.x, kps[i].pt.y, 1.0);
    // ROS_INFO_STREAM("uvw " << uvw.t());
    Mat xyz = dep_proj_inv * uvw;
    xyz = z * xyz;
    // ROS_INFO_STREAM("xyz" << xyz.t());

    // append to result    
    cv::Point3d pt_xyz(xyz.at<double>(0, 0), xyz.at<double>(1, 0), xyz.at<double>(2, 0));
    // ROS_INFO_STREAM("object point " << pt_xyz);
    obj_pts.push_back(pt_xyz);
  }
}

bool RGBDVodom::validateTransform(Mat &rotation, Mat &translation)
{
  double distance = sqrt(translation.dot(translation));
  if (distance > max_translation_)
  {
    ROS_INFO("%5.2f [m] is larger than %5.2f [m]", distance, max_translation_);
    return false;
  }

  double roll, pitch, yaw;
  rotToRPY(rotation, roll, pitch, yaw);
  if (fabs(roll) > max_roll_ || fabs(pitch) > max_pitch_ || fabs(yaw) > max_yaw_)
  {
    ROS_INFO("at least one of these three (%5.2f, %5.2f, %5.2f) is largen than one of these three (%5.2f, %5.2f, %5.2f)",
             roll, pitch, yaw, max_roll_, max_pitch_, max_yaw_);
    return false;
  }
  return true;
}

void RGBDVodom::visualiseMatches(cv_bridge::CvImagePtr frame_ptr, vector<cv::KeyPoint> &kp_new, 
                                 vector<cv::KeyPoint> &kp_old)
{
  int N_kp = std::min(kp_new.size(), kp_old.size()); // they are the same

  // visualise current frame's keypoints
  Mat rgb_annot (frame_ptr->image);
  for (int i = 0; i < N_kp; i++)
  {
    cv::circle(rgb_annot, kp_new[i].pt, 5, cv::Scalar(0, 0, 255));
    cv::line(rgb_annot, kp_old[i].pt, kp_new[i].pt, cv::Scalar(255, 0, 0));
  }

  // publish visualisation
  cv_bridge::CvImage rgb_annot_cv (frame_ptr->header, frame_ptr->encoding, rgb_annot);
  Image rgb_annot_msg;
  rgb_annot_cv.toImageMsg(rgb_annot_msg);
  feat_pub_.publish(rgb_annot_msg);
}

void RGBDVodom::resetOdometry(geometry_msgs::Pose init_pose)
{
  Mat T_new = (cv::Mat_<double>(3, 1) << init_pose.position.x,
                                         init_pose.position.y,
                                         init_pose.position.z);

  Mat R_new = Mat::eye(3, 3, CV_64F);
  quaternionToRot(init_pose.orientation, R_new);

  translation_integral_ = T_new;
  rotation_integral_ = R_new;
}

void RGBDVodom::resetOdometry()
{
  geometry_msgs::Pose pose_zero;
  pose_zero.position.x = 0.0;
  pose_zero.position.y = 0.0;
  pose_zero.position.z = 0.0;

  pose_zero.orientation.x = 0.0;
  pose_zero.orientation.y = 0.0;
  pose_zero.orientation.z = 0.0;
  pose_zero.orientation.w = 1.0;

  resetOdometry(pose_zero);
}

void RGBDVodom::resetRotation()
{
  rotation_integral_ = Mat::eye(3, 3, CV_64F);
}

void RGBDVodom::rgbdPairCb(const sensor_msgs::ImageConstPtr rgb,
                           const sensor_msgs::ImageConstPtr dep)
{
  ROS_INFO("new pair");
  ros::Time t_start = ros::Time::now();
  ros::Time t_inter;

  // conversion
  cv_bridge::CvImagePtr rgb_ptr, dep_ptr;
  rgb_ptr = cv_bridge::toCvCopy(rgb);
  dep_ptr = cv_bridge::toCvCopy(dep);
  dep_ptr->image.convertTo(dep_ptr->image, CV_64F);

  // double depmin, depmax;
  // cv::minMaxLoc(dep_ptr->image, &depmin, &depmax);
  // ROS_INFO("min depth %5.2f, max depth %5.2f", depmin, depmax);

  // image pre-treatment
  Mat img;
  cv::cvtColor(rgb_ptr->image, img, cv::COLOR_RGB2GRAY);
  cv::GaussianBlur(img, img, cv::Size(3, 3), 0, 0);

  // keypoints and descriptors
  vector<cv::KeyPoint> kps;
  detector_->detect(img, kps);
  if (kps.size() == 0)
  {
    ROS_WARN("This frame has zero keypoints");
    return;
  }
  t_inter = ros::Time::now();
  ROS_INFO("there are %lu keypoints", kps.size());
  ROS_INFO("detection took %5.2f [s]", (t_inter-t_start).toSec());

  Mat descs;
  detector_->compute(img, kps, descs);
  ros::Time t_inter_tmp = ros::Time::now();
  ROS_INFO("computation took %5.2f [s]", (t_inter_tmp - t_inter).toSec());
  t_inter = t_inter_tmp;

  // update buffers
  kp_buffer_.push_back(kps);
  desc_buffer_.push_back(descs);
  rgb_buffer_.push_back(rgb_ptr);
  dep_buffer_.push_back(dep_ptr);

  // check for first frame
  if (kp_buffer_.size() <= 1)
    return;

  // trim buffers
  if (kp_buffer_.size() >= N_frame_buffer_)
  {
    kp_buffer_.erase(kp_buffer_.begin());
    desc_buffer_.erase(desc_buffer_.begin());
    rgb_buffer_.erase(rgb_buffer_.begin());
    dep_buffer_.erase(dep_buffer_.begin());
  }

  // match to previous frame
  int prev_idx = desc_buffer_.size() - 2;
  Mat prev_descs = desc_buffer_[prev_idx];
  vector<cv::KeyPoint> prev_kps = kp_buffer_[prev_idx];

  vector< vector<cv::DMatch> > matches;
  matcher_->knnMatch(descs, prev_descs, matches, 2);
  t_inter_tmp = ros::Time::now();
  ROS_INFO("matching took %5.2f [s]", (t_inter_tmp - t_inter).toSec());
  t_inter = t_inter_tmp;

  // filter matches for strong ones
  vector<cv::DMatch> valid_matches;
  filterMatches(matches, valid_matches, nn_distance_ratio_);
  ROS_INFO("there are %lu valid matches", valid_matches.size());

  // get object points
  vector<cv::KeyPoint> valid_kps, valid_prev_kps;
  for (int i = 0; i < valid_matches.size(); i++)
  {
    valid_kps.push_back(kps[valid_matches[i].queryIdx]);
    valid_prev_kps.push_back(prev_kps[valid_matches[i].trainIdx]);
  }

  vector<cv::Point3d> obj_pts;
  vector<int> invalid_kp_indices;
  getObjectPts(dep_ptr->image, valid_kps, obj_pts, invalid_kp_indices);

  // filter out invalid keypoint indices
  for (int i = invalid_kp_indices.size()-1; i >= 0; i--)
  {
    valid_kps.erase(valid_kps.begin()+invalid_kp_indices[i]);
    valid_prev_kps.erase(valid_prev_kps.begin()+invalid_kp_indices[i]);
  }
  ROS_INFO("valid keypoint count, now %lu, prev %lu", valid_kps.size(), valid_prev_kps.size());

  // prep image points
  vector<cv::Point2d> img_pts;
  kp2pt(valid_prev_kps, img_pts);
  
  visualiseMatches(rgb_ptr, valid_kps, valid_prev_kps);

  // calculate motion
  if (valid_kps.size() <= 6)
  {
    ROS_INFO("%lu is not enough for PnP", valid_kps.size());
    return;
  }

  Mat rvec, tvec, R;
  cv::solvePnPRansac(obj_pts, img_pts, dep_proj_, cv::Mat(1, 5, CV_32F, 0.0), rvec, tvec);
  cv::Rodrigues(rvec, R);

  // ROS_INFO_STREAM("rotation :" << std::endl << R);
  // ROS_INFO_STREAM("translation :" << std::endl << tvec);

  // wrap rotation and translation in optical to camera
  Mat R_cam = R_opt_to_cam_ * R * R_cam_to_opt_;
  Mat T_cam = R_opt_to_cam_ * (R * T_cam_to_opt_ + tvec) + T_opt_to_cam_;

  // ROS_INFO_STREAM("rotation (cam):" << std::endl << R_cam);
  // ROS_INFO_STREAM("translation (cam):" << std::endl << T_cam);
  
  // validate, integrate, publish
  if (!validateTransform(R_cam, T_cam))
    return;

  integrateOdometry(R_cam, T_cam);
  // ROS_INFO_STREAM("rotation (integral):" << std::endl << rotation_integral_);
  // ROS_INFO_STREAM("translation (integral):" << std::endl << translation_integral_);

  // calculate covariance
  Mat cov;
  cv_bridge::CvImagePtr prev_dep = dep_buffer_[dep_buffer_.size()-2];
  calcMatchCovariance(prev_dep->image, valid_prev_kps, obj_pts, R, tvec, cov);
  Mat cov_transform = rotation_integral_ * R_opt_to_cam_;
  cov = cov_transform * cov * cov_transform.t();

  publishOdometry(cov);

  ros::Duration dt_process = ros::Time::now() - t_start;
  ROS_INFO("processing took %5.2f [s]\n", dt_process.toSec());
}

void RGBDVodom::resetOdometryCb(geometry_msgs::Pose msg)
{
  ROS_INFO("Resetting Odometry");
  resetOdometry(msg);
}

void RGBDVodom::resetRotationCb(std_msgs::Empty msg)
{
  ROS_INFO("Resetting Rotation");
  resetRotation();
}

void RGBDVodom::kp2pt(vector<cv::KeyPoint> &kps, vector<cv::Point2d> &pts)
{
  for (int i = 0; i < kps.size(); i++)
  {
    cv::Point2d pt_i(kps[i].pt.x, kps[i].pt.y);
    pts.push_back(pt_i);
  }
}

void RGBDVodom::integrateOdometry(Mat &rotation, Mat &translation)
{
  translation_integral_ = rotation_integral_ * translation + translation_integral_;
  rotation_integral_ = rotation_integral_ * rotation;
}

void RGBDVodom::publishOdometry(Mat &cov)
{
  nav_msgs::Odometry odom_msg;

  // header
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.header.frame_id = "vodom";
  odom_msg.child_frame_id = "camera_link";
  
  // position
  odom_msg.pose.pose.position.x = translation_integral_.at<double>(0, 0);
  odom_msg.pose.pose.position.y = translation_integral_.at<double>(1, 0);
  odom_msg.pose.pose.position.z = translation_integral_.at<double>(2, 0);

  // orientation
  double roll, pitch, yaw;
  rotToRPY(rotation_integral_, roll, pitch, yaw);
  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
  odom_msg.pose.pose.orientation = quat;

  // position covariance
  for (int i = 0; i < 9; i++)
  {
    int r, c;
    r = i / 3;
    c = i % 3;

    int cidx = r*6 + c;
    odom_msg.pose.covariance[cidx] = cov.at<double>(r, c);
  }

  odom_pub_.publish(odom_msg);
}

void RGBDVodom::calcMatchCovariance(Mat &prev_dep, vector<cv::KeyPoint> &prev_kps, 
                                    vector<cv::Point3d> &curr_obj_pts, Mat &R, Mat &T, Mat &cov)
{
  // get object points from previous frame
  vector<cv::Point3d> prev_obj_pts;
  vector<int> invalid_kp_indices;
  getObjectPts(prev_dep, prev_kps, prev_obj_pts, invalid_kp_indices);

  // remove invalid object points
  for (int i = invalid_kp_indices.size()-1; i >= 0; i--)
  {
    prev_obj_pts.erase(prev_obj_pts.begin() + invalid_kp_indices[i]);
    curr_obj_pts.erase(curr_obj_pts.begin() + invalid_kp_indices[i]);
  }

  // calculate error accumulators
  Mat XXT_accu = Mat::eye(3, 3, CV_64F);
  Mat mu_accu = Mat::zeros(3, 1, CV_64F);
  int N_sample = 0;

  for (int i = 0; i < prev_obj_pts.size(); i++)
  {
    // prep matrices
    Mat xyz_prev = (cv::Mat_<double>(3, 1) << prev_obj_pts[i].x, prev_obj_pts[i].y, prev_obj_pts[i].z);
    Mat xyz_curr = (cv::Mat_<double>(3, 1) << curr_obj_pts[i].x, curr_obj_pts[i].y, curr_obj_pts[i].z);

    // get current object point to prev reference frame
    xyz_curr = R*xyz_curr + T;
    
    // calculate error
    Mat err = xyz_prev - xyz_curr;

    // update accumulators
    XXT_accu += err * err.t();
    mu_accu += err;
    N_sample += 1;
  }

  // calculate covariance matrix
  if (N_sample > 0)
  {
    Mat mu = mu_accu / N_sample;
    cov = XXT_accu / N_sample - mu * mu.t();
  }
}

void rotToRPY(Mat &rot, double &roll, double &pitch, double &yaw)
{
  double r00, r10, r20, r21, r22, ryy;
  r00 = rot.at<double>(0, 0);
  r10 = rot.at<double>(1, 0);
  r20 = rot.at<double>(2, 0);
  r21 = rot.at<double>(2, 1);
  r22 = rot.at<double>(2, 2);
  ryy = sqrt(r00*r00 + r10*r10);

  roll = atan2(r21, r22);
  pitch = atan2(-r20, ryy);
  if (ryy < 1.0e-6)
    yaw = 0.0;
  else
    yaw = atan2(r10, r00);
}

void quaternionToRot(geometry_msgs::Quaternion &q, Mat &R)
{
  double qx, qy, qz, qw;
  qx = q.x;
  qy = q.y;
  qz = q.z;
  qw = q.w;
  double qmag = qx * qx + qy * qy + qz * qz + qw * qw;
  if (fabs(qmag - 1.0) > 1e-3)
  {
    qmag = sqrt(qmag);
    qx /= qmag;
    qy /= qmag;
    qz /= qmag;
    qw /= qmag;
  }

  R.at<double>(0, 0) = 1 - 2 * (qy * qy + qz * qz);
  R.at<double>(0, 1) = 2 * (qx * qy - qz * qw);
  R.at<double>(0, 2) = 2 * (qx * qz + qy * qw);

  R.at<double>(1, 0) = 2 * (qx * qy + qz * qw);
  R.at<double>(1, 1) = 1 - 2 * (qx * qx + qz * qz);
  R.at<double>(1, 2) = 2 * (qy * qz - qx * qw);

  R.at<double>(2, 0) = 2 * (qx * qz - qy * qw);
  R.at<double>(2, 1) = 2 * (qy * qz + qx * qw);
  R.at<double>(2, 2) = 1 - 2 * (qx * qx + qy * qy);
}

geometry_msgs::Quaternion rotToQuaternion(Mat &R)
{
  double r00, r11, r22, r01;
  r00 = R.at<double>(0, 0);
  r11 = R.at<double>(1, 1);
  r22 = R.at<double>(2, 2);
  r01 = R.at<double>(0, 1);

  float a, b, c, qx, qy, qz, qw;
  b = (1.0 + r11 - r22 - r00) / 4.0;
  c = (1 - 2 * b - r00) / 2.0;
  a = (1 - 2 * c - r11) / 2.0;

  qx = sqrt(fabs(a));
  qy = sqrt(fabs(b));
  qz = sqrt(fabs(c));
  qw = (2 * qx * qy - r01) / (2 * qz);

  geometry_msgs::Quaternion out_q;
  out_q.x = qx;
  out_q.y = qy;
  out_q.z = qz;
  out_q.w = qw;
  return out_q;
}
