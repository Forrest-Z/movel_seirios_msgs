#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <fstream>
#include <opencv2/imgproc.hpp>

class CropMap
{
protected:
  cv::Mat image_;
  std::vector<cv::Point> points_;
  cv::Mat cropped_image_;
  std::vector<cv::Point> crop_points_;
  cv::Point robot_coordinates_;
  double resolution_;
  double map_origin_x_;
  double map_origin_y_;
  std::string final_path_coordinates_;
  std::string path_to_cropped_image_;

  /**
   * @brief displayImage  - display image
   * @param image         - image to be displayed
   * @param window_name   - window name on which the image shpuld be displayed
   */
  void displayImage(cv::Mat image, std::string window_name);

  /**
   * @brief points_in_map_frame  - convert points from image frame to map frame
   * @param pts                  - points to be converted
   * @param resolution           - resolution of the map
   * @param image                - image
   * @return                     - point in map frame
   */
  cv::Point_<double> getPointsInMapFrame(cv::Point pts);

  /**
   * @brief maskedImage  - creating a mask out of the points used for cropping
   * @param points       - crop points
   * @param image        - image on which the cropping needs to be done
   * @return             - masked image
   */
  cv::Mat maskImage();

  /**
   * @brief cropImage       - compute crop image
   * @param points          - points on which the cropping needs to be done.
   * @param masked_image    - masked imaged computed from maskedImage function
   * @return
   */
  void cropImage(cv::Mat masked_image);

  /**
   * @brief computeOrigin    - finding the origin of the cropped map
   * @param points           - points used for cropping
   * @param img              - image that needs to be cropped
   * @return                 - origin in the image frame
   */
  cv::Point computeOrigin();

  /**
   * @brief saveCoordinates      - save map origin and robot's position in a file
   * @param origin               - map origin
   * @param robot_coordinates    - robot's position
   * @param path                 - path of the file where you wish to save
   * @param img                  - image that needs to be cropped
   */
  void saveCoordinates(cv::Point origin);

public:
  CropMap();
  /**
   * @brief Main method to crop a map using this library
   * @param map_path Path to .pgm file of original map that should be cropped
   * @param polygon_path Path to .txt file which provies vertices for cropping polygon
   * @return True if cropping is successful
   */
  bool cropMap(std::string map_path, std::string polygon_path);

  /**
   * @brief Set the path to save coordinates text file from cropping
   * @param path Full path to file that should be saved, example: /home/coords.txt
   */
  void setCoordinatesSavePath(std::string path)
  {
    final_path_coordinates_ = path;
  }

  /**
   * @brief Set the path to save cropped map from cropping
   * @param path Full path to file that should be saved, example: /home/cropped.png
   */
  void setCroppedSavePath(std::string path)
  {
    path_to_cropped_image_ = path;
  }

  /**
   * @brief Set resolution of map that is being cropped
   * @param res Resolution, unit is meters/pixel
   */
  void setResolution(double res)
  {
    resolution_ = res;
  }

  /**
   * @brief Set the origin of map that is being cropped
   * @param origin_x X origin of map being cropped, unit meters
   * @param origin_y Y origin of map being cropped, unit meters
   */
  void setMapOrigin(double origin_x, double origin_y)
  {
    map_origin_x_ = origin_x;
    map_origin_y_ = origin_y;
  }

  /**
   * @brief Gets the currently set path for saving coordinates
   * @return String to full path, including file name and extension
   */
  std::string getCoordinatesSavePath()
  {
    return final_path_coordinates_;
  }

  /**
   * @brief get_contour_area	- calculates the area for the contour
   * @return                     	- area for contour
   */
  double getContourArea();


  /**
   * @brief Gets the currently set path for saving cropped map
   * @return String to full path, including file name and extension
   */
  std::string getCroppedSavePath()
  {
    return path_to_cropped_image_;
  }

  /**
   * @brief Gets the currently set resolution
   * @return Resolution in meters/pixel
   */
  double getResolution()
  {
    return resolution_;
  }

  /**
   * @brief Gets the currently set map origin
   * @return cv::Point templated to double, unit is in meters
   */
  cv::Point_<double> getMapOrigin()
  {
    cv::Point_<double> point;
    point.x = map_origin_x_;
    point.y = map_origin_y_;
    return point;
  }
};
