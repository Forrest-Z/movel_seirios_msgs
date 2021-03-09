#ifndef dock_detector_hpp
#define dock_detector_hpp

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

using namespace Eigen;
using std::vector;
using std::cout;
using std::endl;

typedef pcl::PointXYZI Pt;
typedef pcl::PointCloud<Pt> Cloud;

class DockDetector
{
public:
  DockDetector();
  ~DockDetector(){}

  void setupParams(double width, double offset);
  void setupTopics();

  bool findDock(sensor_msgs::LaserScan &scan, geometry_msgs::Pose &outpose);
  Vector2d calcLineParams(vector<Vector2d> points);
  Vector2d calcLineParams(int i_start, int i_end, vector<Vector2d> ranges);
  double calcOrthoDistance(Vector2d pt, Vector2d line_params);

private:
  double dock_width_;
  double dock_width_tolerance_;
  double dock_distance_;
  double dock_offset_; // from wall
  double max_dock_distance_;
  double line_consistency_threshold_;
  double min_angle_to_dock_;
  double max_angle_to_dock_;
  int laser_skip_;

  ros::NodeHandle nh_;
  ros::Publisher dock_candidates_pub_;
  ros::Publisher segments_pub_;

  void visDockCandidates(vector< vector<int> > &candidate_idx, 
                         vector<Vector2d> &ranges, sensor_msgs::LaserScan scan);
};

#endif