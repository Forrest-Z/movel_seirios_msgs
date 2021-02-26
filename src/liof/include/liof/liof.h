#ifndef LIOF_H
#define LIOF_H
#include <iostream>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/imgproc.hpp>
#include "utils.h"
//#include "visualize.h"

namespace feature_detector
{
using namespace cv;
using namespace cv::ximgproc;
using namespace std;

class LIOF
{
public:
  /**
   * @brief Construct a new LIOF object
   *
   * @param radius_threshold
   * @param final_radius
   * @param low_angle
   * @param high_angle
   * @param length_threshold
   * @param distance_threshold
   * @param canny_th1
   * @param canny_th2
   * @param canny_aperture_size
   * @param do_merge
   */
  LIOF(int radius_threshold = 32, int final_radius = 8, float low_angle = 1.0, float high_angle = 2.0,
       int length_threshold = 25, float distance_threshold = 1.41421356f, double canny_th1 = 5.0,
       double canny_th2 = 20.0, int canny_aperture_size = 3, bool do_merge = false);

  void detect_with_lines(const Mat& image, const Mat& mask, vector<KeyPoint>& keypoints, vector<Vec4i>& lines);

  /** Compute the LIOF features on an image
   * @param img the image to compute the features and descriptors on
   * @param mask the mask to apply (TODO)
   * @param keypoints the resulting keypoints
   */
  void detect(const Mat& image, const Mat& mask, vector<KeyPoint>& keypoints);

  /** Compute the LIOF features on an image
   * @param img the image to compute the features and descriptors on
   * @param keypoints the resulting keypoints
   */
  void detect(const Mat& image, vector<KeyPoint>& keypoints);

  /** Compute the LIOF descriptors on an image given a set of keypoints
   * @param img the image to compute the features and descriptors on
   * @param keypoints the resulting keypoints
   * @param descriptors the resulting descriptors
   */
  void compute(const Mat& image, const vector<KeyPoint>& keypoints, Mat& descriptors);

  /** Faster Compute of the LIOF descriptors on an image given a set of keypoints
   * For a higher accuracy use compute()
   * @param img the image to compute the features and descriptors on
   * @param keypoints the resulting keypoints
   * @param descriptors the resulting descriptors
   */
  void fast_compute(const Mat& image, const vector<KeyPoint>& keypoints, Mat& descriptors);

protected:
  Ptr<FastLineDetector> fld;

  /**
   * @brief Half of the descriptor's width
   * Bigger size will result in heigher matching accuracy
   * with loss in speed and features density around the image boundaries
   */
  int radius_threshold_;
  /**
   * @brief Radius of the furthest away pixel of a descriptor zone before applying an affine transform
   *  Used to Preserve descriptor information after rotation
   */
  int radius_threshold_extended_ = sqrt(2) * radius_threshold_;
  /**
   * @brief Half of the descriptor's width after resizing
   */
  int final_radius_;
  /**
   * @brief Boundaries for angle difference between two intersecting lines
   * Angle differences smaller than low_radius or bigger than high_radius are filtered
   * Unit: radians
   */
  float low_angle_, high_angle_;
  /**
   * @brief Cos, sin hash maps
   */
  std::vector<float> cos_, sin_;

private:
  /** Compute the LIOF features on an image given a line set
   * @param mask the mask to apply (TODO)
   * @param lines the lines to compute the features
   * @param keypoints the resulting keypoints
   */
  void intersect(const Mat& mask, const vector<Vec4i>& lines, vector<KeyPoint>& keypoints);

  /**
   * @brief
   *
   * @param image
   * @param output
   */
  void toScharr(const Mat& image, Mat& output);
};
}  // namespace feature_detector
#endif