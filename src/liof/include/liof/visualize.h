#ifndef LIOF_VISUALIZE_H
#define LIOF_VISUALIZE_H

#include <opencv2/imgproc.hpp>
#include "opencv2/highgui.hpp"

void displayKeylines(cv::Mat image, std::vector<cv::Vec4i> keylines)
{
  cv::Mat image_out;
  cv::cvtColor(image, image_out, cv::COLOR_GRAY2BGR);
  for (size_t i = 0; i < keylines.size(); i++)
  {
    cv::Vec4i l = keylines[i];
    cv::line(image_out, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
  }
  cv::imshow("Keylines", image_out);
  cv::waitKey();
}

void displayKeypoints(cv::Mat image, std::vector<cv::KeyPoint> keypoints)
{
  cv::Mat _image = image.clone();
  cv::Mat out;
  int length = 5;
  for (size_t i = 0; i < keypoints.size(); i++)
  {
    cv::Point2f orient;
    orient.x = cos(keypoints[i].angle) * length + keypoints[i].pt.x;
    orient.y = sin(keypoints[i].angle) * length + keypoints[i].pt.y;

    line(_image, keypoints[i].pt, orient, cv::Scalar(0, 255, 0));
    circle(_image, keypoints[i].pt, length, cv::Scalar(255, 255, 0));
  }

  cv::imshow("Keypoints", _image);
  cv::waitKey(0);
}

#endif