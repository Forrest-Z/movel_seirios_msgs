#ifndef vodom_hpp_
#define vodom_hpp_

#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <liof/liof.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ximgproc.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Empty.h>
#include <tf/tf.h>

using cv::Mat;
using cv_bridge::CvImage;
using cv_bridge::CvImagePtr;
using sensor_msgs::Image;
using std::vector;

class RGBDVodom 
{
public:
  RGBDVodom(ros::NodeHandle nh);
  ~RGBDVodom(){}

  void setupTopics();
  void setupCameras();
  void setupParams();

  void filterMatches(vector< vector<cv::DMatch> > matches, vector<cv::DMatch> &good_matches,
                     double distance_ratio);

  void getObjectPts(Mat &dep, vector<cv::KeyPoint> &kps, vector<cv::Point3d> &obj_pts,
                    vector<int> &invalid_kp_indices);

  bool validateTransform(Mat &rotation, Mat &translation);

  void visualiseMatches(cv_bridge::CvImagePtr frame_ptr, vector<cv::KeyPoint> &kp_new, vector<cv::KeyPoint> &kp_old);

  void resetOdometry(geometry_msgs::Pose init_pose);
  void resetOdometry();
  void resetRotation();

private:
  // utilities
  ros::NodeHandle nh_;

  cv::Ptr<cv::ORB> detector_;
  // std::shared_ptr<feature_detector::LIOF> detector_;
  cv::Ptr<cv::DescriptorMatcher> matcher_;
  
  // parameters
  int N_keypoints_;
  int N_frame_buffer_;
  double nn_distance_ratio_;
  double min_valid_depth_, max_valid_depth_;
  Mat rgb_proj_, dep_proj_; // projection matrices
  Mat R_opt_to_cam_, T_opt_to_cam_;
  Mat R_cam_to_opt_, T_cam_to_opt_;

  double max_translation_;
  double max_roll_, max_pitch_, max_yaw_;

  // subscribers
  message_filters::Subscriber<Image> rgb_sub_;
  message_filters::Subscriber<Image> dep_sub_;
  message_filters::TimeSynchronizer<Image, Image> rgb_dep_sync_;
  ros::Subscriber reset_sub_;
  ros::Subscriber reset_rot_sub_;

  // publishers
  ros::Publisher feat_pub_;
  ros::Publisher odom_pub_;

  // bookkeeping
  vector<CvImagePtr> rgb_buffer_;
  vector<CvImagePtr> dep_buffer_;
  vector< vector<cv::KeyPoint> > kp_buffer_;
  vector<Mat> desc_buffer_;
  Mat rotation_integral_;
  Mat translation_integral_;

  // callbacks
  void rgbdPairCb(const sensor_msgs::ImageConstPtr rgb, const sensor_msgs::ImageConstPtr dep);
  void resetOdometryCb(geometry_msgs::Pose msg);
  void resetRotationCb(std_msgs::Empty msg);
  
  // utilities
  void kp2pt(vector<cv::KeyPoint> &kps, vector<cv::Point2d> &pts);
  void integrateOdometry(Mat &rotation, Mat &translation);
  void publishOdometry(Mat &cov);
  void calcMatchCovariance(Mat &prev_dep, vector<cv::KeyPoint> &prev_kps, 
                           vector<cv::Point3d> &curr_obj_pts, Mat &R, Mat &T, Mat &cov);
};

void rotToRPY(Mat &rot, double &roll, double &pitch, double &yaw);
void quaternionToRot(geometry_msgs::Quaternion &q, Mat &R);
geometry_msgs::Quaternion rotToQuaternion(Mat &R);

#endif