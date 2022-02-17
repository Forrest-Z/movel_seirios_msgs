/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <combine_grids/grid_compositor.h>

#include <opencv2/stitching/detail/util.hpp>
#include <opencv2/opencv.hpp>
#include <ros/assert.h>

namespace combine_grids
{
namespace internal
{
nav_msgs::OccupancyGrid::Ptr GridCompositor::compose(
    const std::vector<cv::Mat>& grids, const std::vector<cv::Rect>& rois)
{
  ROS_ASSERT(grids.size() == rois.size());

  nav_msgs::OccupancyGrid::Ptr result_grid(new nav_msgs::OccupancyGrid());

  std::vector<cv::Point> corners;
  corners.reserve(grids.size());
  std::vector<cv::Size> sizes;
  sizes.reserve(grids.size());
  for (auto& roi : rois) {
    corners.push_back(roi.tl());
    sizes.push_back(roi.size());
  }
  cv::Rect dst_roi = cv::detail::resultRoi(corners, sizes);

  result_grid->info.width = static_cast<uint>(dst_roi.width);
  result_grid->info.height = static_cast<uint>(dst_roi.height);
  result_grid->data.resize(static_cast<size_t>(dst_roi.area()), -1);
  // create view for opencv pointing to newly allocated grid
  cv::Mat result(dst_roi.size(), CV_8S, result_grid->data.data());

  //for (size_t i = 0; i < grids.size(); ++i) {
  for (size_t i = 0; i < grids.size(); ++i) {
    // we need to compensate global offset
    cv::Rect roi = cv::Rect(corners[i] - dst_roi.tl(), sizes[i]);
    cv::Mat result_roi(result, roi);
    // reinterpret warped matrix as signed
    // we will not change this matrix, but opencv does not support const matrices
    cv::Mat warped_signed (grids[i].size(), CV_8S, const_cast<uchar*>(grids[i].ptr()));

    // BAD DEBUG PRINTS
    // std::cout << "this is annoying" << std::endl;
    //std::cout << warped_signed.at<int>(0,0) << std::endl;
    // std::cout << "this is end" << std::endl;
    
    // compose img into result matrix
    /*
    std::cout << "[DEBUG MATRIX STUFF]" << std::endl;
    cv::Mat mask(warped_signed.size(), warped_signed.type()); 
    cv::Mat bg(result_roi.size(), result_roi.type());
    cv::bitwise_and(result_roi, result_roi, bg, mask);
    cv::add(bg, warped_signed, result_roi);
    //cv::max(result_roi, warped_signed, result_roi);
    */

    /* 
    Modification to original grid_compositor
    [credits robert]
    */
    std::cout << "1. extract grey area from second image" << std::endl;
    /* create all zero mat and do a min with the second img */
    cv::Mat res1(warped_signed.size(), warped_signed.type()); 
    cv::Mat z_mask = cv::Mat::zeros(warped_signed.size(), warped_signed.type());
    cv::max(z_mask, warped_signed, res1);
    
    // std::cout << warped_signed << std::endl;
    // std::cout << "end" << std::endl;
    cv::Mat grey_mask = res1.clone();
    grey_mask += 1;
    std::cout << grey_mask.at<int>(0,0) << std::endl;
    //grey_mask *= -1;

    std::cout << "2. max extracted image and first image" << std::endl;
    cv::Mat res2(warped_signed.size(), warped_signed.type());
    cv::bitwise_and(warped_signed, grey_mask, result_roi);
    //cv::Mat res2(result_roi.size(), result_roi.type());
    //cv::max(grey_mask, result_roi, res2); // this is the mask..?
    
    // // std::cout << "2.5. make #1 black" << std::endl;
    // // cv::Mat res1_black(res1.size(), warped_signed.type(), cv::Scalar(0, 0, 0));
    
    // std::cout << "3. bitwise_and #1 with second image" << std::endl;
    // cv::Mat res3(warped_signed.size(), warped_signed.type());
    // cv::bitwise_and(grey_mask, warped_signed, res3);
    
    // std::cout << "4. bitwise_and #3 with #2" << std::endl;
    // cv::Mat res4;
    // cv::bitwise_and(res3, res2, res4);
    
    // std::cout << "5. add #3 and #4" << std::endl;
    // cv::add(res3, res4, result_roi);
  }
  //cv::destroyAllWindows();
  return result_grid;
}

}  // namespace internal

}  // namespace combine_grids
