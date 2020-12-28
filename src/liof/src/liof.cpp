#include <liof/liof.h>

namespace feature_detector
{
LIOF::LIOF(int radius_threshold, int final_radius, float low_angle, float high_angle, int length_threshold,
           float distance_threshold, double canny_th1, double canny_th2, int canny_aperture_size, bool do_merge)
{
  fld =
      createFastLineDetector(length_threshold, distance_threshold, canny_th1, canny_th2, canny_aperture_size, do_merge);

  for (int i = 0; i < 360; i++)
  {
    cos_.push_back(cos((i / 360.) * (2 * M_PI)));
    sin_.push_back(sin((i / 360.) * (2 * M_PI)));
  }

  radius_threshold_ = radius_threshold;
  radius_threshold_extended_ = sqrt(2) * radius_threshold_;
  final_radius_ = final_radius;
  low_angle_ = low_angle;
  high_angle_ = high_angle;
}

void LIOF::intersect(const Mat& mask, const vector<Vec4i>& lines, vector<KeyPoint>& keypoints)
{
  for (int i = 0; i < lines.size(); i++)
  {
    // unpack segment coordinates
    float x11 = lines[i][0];
    float y11 = lines[i][1];
    float x12 = lines[i][2];
    float y12 = lines[i][3];
    for (int j = 0; j < lines.size() && i != j; j++)
    {
      float x21 = lines[j][0];
      float y21 = lines[j][1];
      float x22 = lines[j][2];
      float y22 = lines[j][3];

      // compute intersection point
      Point2f p;
      p.x = ((((x11 * y12) - (y11 * x12)) * (x21 - x22)) - ((x11 - x12) * ((x21 * y22) - (y21 * x22)))) /
            ((x11 - x12) * (y21 - y22) - (y11 - y12) * (x21 - x22));
      p.y = ((((x11 * y12) - (y11 * x12)) * (y21 - y22)) - ((y11 - y12) * ((x21 * y22) - (y21 * x22)))) /
            ((x11 - x12) * (y21 - y22) - (y11 - y12) * (x21 - x22));

      // check for nan (parallel lines)
      if (isnan(p.x) || isnan(p.y))
        continue;

      // check mask and image boudaries
      if (p.x > (mask.cols - radius_threshold_extended_) || p.x < radius_threshold_extended_ ||
          p.y > (mask.rows - radius_threshold_extended_) || p.y < radius_threshold_extended_)
        continue;

      std::vector<float> dists;
      dists.push_back(sqrt(pow(y21 - y11, 2) + pow(x21 - x11, 2)));
      dists.push_back(sqrt(pow(y22 - y11, 2) + pow(x22 - x11, 2)));
      dists.push_back(sqrt(pow(y12 - y22, 2) + pow(x12 - x22, 2)));
      dists.push_back(sqrt(pow(y11 - y22, 2) + pow(x11 - x22, 2)));

      // TODO set max_line_distance parameter
      // check min distance to the intersection
      if (*(std::min_element(dists.begin(), dists.end())) > 60)
        continue;

      // TODO skip if mask[p.y,p.x] = 0
      // TODO improve middle angle calculation
      std::vector<float> dists1, dists2;
      dists1.push_back(sqrt(pow(y11 - p.y, 2) + pow(x11 - p.x, 2)));
      dists1.push_back(sqrt(pow(y12 - p.y, 2) + pow(x12 - p.x, 2)));

      dists2.push_back(sqrt(pow(y21 - p.y, 2) + pow(x21 - p.x, 2)));
      dists2.push_back(sqrt(pow(y22 - p.y, 2) + pow(x22 - p.x, 2)));

      // use the farthest point of each line to calculate then angle
      // this help unifing the angle between two lines
      Point2i p1 = dists1[0] > dists1[1] ? Point2i(x11, y11) : Point2i(x12, y12);
      Point2i p2 = dists2[0] > dists2[1] ? Point2i(x21, y21) : Point2i(x22, y22);

      float angle1 = normalize(atan2((p1.y - p.y), (p1.x - p.x)));
      float angle2 = normalize(atan2((p2.y - p.y), (p2.x - p.x)));
      float angle, angle_d;

      if (normalize(angle1 - angle2) < normalize(angle2 - angle1))
      {
        angle_d = normalize(angle1 - angle2);
        angle = normalize(angle2 + angle_d / 2.0);
      }
      else
      {
        angle_d = normalize(angle2 - angle1);
        angle = normalize(angle1 + angle_d / 2.0);
      }
      // TODO define parameters for the angle range
      // check accpeted keypoint angles by range
      if (angle_d < low_angle_ or angle_d > high_angle_)
        continue;

      // The keypoints angle corresponds to the dominant angle of the intersection
      // We define the dominant angle as the mean angle of the smaller angle of the intersection
      KeyPoint k;
      k.pt = p;
      k.angle = angle;
      keypoints.push_back(k);
    }
  }
}

void LIOF::detect_with_lines(const Mat& image, const Mat& mask, vector<KeyPoint>& keypoints, vector<Vec4i>& lines)
{
  if (image.empty())
  {
    // exit
    abort();
    return;
  }

  // line detection
  fld->detect(image, lines);
  keypoints.clear();
  intersect(image, lines, keypoints);
}

void LIOF::detect(const Mat& image, const Mat& mask, vector<KeyPoint>& keypoints)
{
  if (image.empty())
  {
    // exit
    abort();
    return;
  }

  // line detection
  vector<Vec4i> lines;
  fld->detect(image, lines);
  // displayKeylines(image, lines);
  keypoints.clear();
  intersect(image, lines, keypoints);
  // displayKeypoints(image, keypoints);
}

void LIOF::detect(const Mat& image, vector<KeyPoint>& keypoints)
{
  detect(image, Mat(), keypoints);
}

void LIOF::toScharr(const Mat& image, Mat& output)
{
  Mat dx, dy;
  Scharr(image, dx, CV_32F, 1, 0);
  Scharr(image, dy, CV_32F, 0, 1);

  // Compute gradient magnitude
  Mat grad(image.rows, image.cols, CV_8U, Scalar(0));
  float alpha = 3;  // alpha used to implify gradient magnetude
  for (int i = 0; i < image.rows; i++)
  {
    for (int j = 0; j < image.cols; j++)
    {
      float magnitude =
          (sqrt(pow(dx.at<float>(i, j), 2) + pow(dy.at<float>(i, j), 2)) / 4800);  // normalising to [0..1] range
      grad.at<uchar>(i, j) = (uchar)255 * alpha * magnitude;
    }
  }
  output = grad;
}

void LIOF::compute(const Mat& image, const vector<KeyPoint>& keypoints, Mat& descriptors)
{
  Mat _descriptors(keypoints.size(), 4 * final_radius_ * final_radius_, CV_8U, Scalar(0));

  for (int i = 0; i < keypoints.size(); i++)
  {
    // Crop minature around feature
    Rect roi;
    roi.x = keypoints[i].pt.x - radius_threshold_extended_;
    roi.y = keypoints[i].pt.y - radius_threshold_extended_;
    roi.width = 2 * radius_threshold_extended_;
    roi.height = 2 * radius_threshold_extended_;

    Mat cropped_extended(2 * radius_threshold_extended_, 2 * radius_threshold_extended_, CV_8U, cv::Scalar(0));
    cropped_extended = image(roi);

    // Rotate image around feature with dominant angle (counter clock-wise)
    Mat rot = getRotationMatrix2D(Point2f(radius_threshold_extended_, radius_threshold_extended_),
                                  360 * keypoints[i].angle / (2 * M_PI), 1);
    Mat rotated;
    warpAffine(cropped_extended, rotated, rot, cropped_extended.size());

    // Crop minature to the exact descriptor size
    roi.x = radius_threshold_extended_ - radius_threshold_;
    roi.y = radius_threshold_extended_ - radius_threshold_;
    roi.width = 2 * radius_threshold_;
    roi.height = 2 * radius_threshold_;

    Mat cropped(2 * radius_threshold_, 2 * radius_threshold_, CV_8U, cv::Scalar(0));
    cropped = rotated(roi);

    // downsampling
    Mat resized;
    resize(cropped, resized, Size(2 * final_radius_, 2 * final_radius_), 0, 0, cv::InterpolationFlags::INTER_AREA);
    Mat grad;
    toScharr(resized, grad);

    // flatten results and set keypoint descriptor
    for (int k = 0; k < grad.rows; k++)
    {
      for (int j = 0; j < grad.cols; j++)
      {
        _descriptors.at<uchar>(i, k * grad.cols + j) = grad.at<uchar>(k, j);
      }
    }
  }
  descriptors = _descriptors;
}

void LIOF::fast_compute(const Mat& image, const vector<KeyPoint>& keypoints, Mat& descriptors)
{
  Mat _descriptors(keypoints.size(), 4 * final_radius_ * final_radius_, CV_8U, Scalar(0));

  // downsampling
  Mat resized;
  int ratio = radius_threshold_ / final_radius_;
  resize(image, resized, Size(image.cols / ratio, image.rows / ratio), 0, 0, cv::InterpolationFlags::INTER_LINEAR);
  float real_ratio_x = image.cols / resized.cols;
  float real_ratio_y = image.rows / resized.rows;

  for (int i = 0; i < keypoints.size(); i++)
  {
    Mat rotated(2 * final_radius_, 2 * final_radius_, CV_8U, Scalar(0));
    for (int j = -final_radius_; j < final_radius_; j++)
    {
      for (int k = -final_radius_; k < final_radius_; k++)
      {
        int angle = int(360 * keypoints[i].angle / (2 * M_PI));
        int j_r = round(j * cos_[angle] - k * sin_[angle]) + keypoints[i].pt.x / real_ratio_x;
        int k_r = round(k * cos_[angle] + j * sin_[angle]) + keypoints[i].pt.y / real_ratio_y;
        rotated.at<uchar>(j + final_radius_, k + final_radius_) = resized.at<uchar>(k_r, j_r);
      }
    }
    Mat grad(rotated.rows, rotated.cols, CV_8U, Scalar(0));
    toScharr(rotated, grad);
    GaussianBlur(grad, grad, Size(3, 3), 1, 1);

    // flatten results and set keypoint descriptor
    for (int k = 0; k < grad.rows; k++)
    {
      for (int j = 0; j < grad.cols; j++)
      {
        _descriptors.at<uchar>(i, k * grad.cols + j) = grad.at<uchar>(k, j);
      }
    }
  }
  descriptors = _descriptors;
}

}  // namespace feature_detector