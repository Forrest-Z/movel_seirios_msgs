#include <crop_map/crop_map.h>
#include <exception>
#include <movel_hasp_vendor/license.h>

CropMap::CropMap()
{
}

bool CropMap::cropMap(std::string map_path, std::string polygon_path)
{
#ifdef MOVEL_LICENSE
  MovelLicense ml(22);
  if (!ml.login())
    return 1;
#endif

  try
  {
    image_ = cv::imread(map_path);
    std::string line;
    std::ifstream polygon_file(polygon_path);

    while (std::getline(polygon_file, line))
    {
      std::stringstream lineStream(line);

      int value;
      cv::Point pt;
      int i = 0;
      while (lineStream >> value)
      {
        if (i == 0)
          pt.x = value;
        else
          pt.y = value;

        i += 1;
      }
      crop_points_.push_back(pt);
    }

    robot_coordinates_ = (crop_points_[crop_points_.size() - 1]);
    //    cout<<"robot coordinates "<<robot_coordinates_<<endl;
    crop_points_.pop_back();

    cv::Mat result = maskImage();
    // displayImage(result, "res");
    cropImage(result);
    cv::imwrite(path_to_cropped_image_, cropped_image_);

    // displayImage( cropped_image_, "cropped");

    cv::Point origin = computeOrigin();
    saveCoordinates(origin);
    //    std::cout<<origin<<endl;
  }

  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return false;
  }

#ifdef MOVEL_LICENSE
  ml.logout();
#endif
  return true;
}

void CropMap::displayImage(cv::Mat image, std::string window_name)
{
  cv::imshow(window_name, image);
  cv::waitKey(0);
}

cv::Point_<double> CropMap::getPointsInMapFrame(cv::Point pts)
{
  cv::Point_<double> pt;
  pt.x = float(map_origin_x_ + float(pts.x * resolution_));                  // -10 is map's origin
  pt.y = float(map_origin_y_ + float((image_.rows - pts.y) * resolution_));  // -10 is map's origin

  return pt;
}

cv::Mat CropMap::maskImage()
{
  std::vector<std::vector<cv::Point>> contours;
  contours.push_back(crop_points_);
  cv::Mat mask(image_.rows, image_.cols, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::drawContours(mask, contours, -1, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
  cv::Mat result;
  cv::bitwise_and(image_, mask, result);
  return result;
}

double CropMap::getContourArea()
{
  return contourArea(crop_points_);
}

void CropMap::cropImage(cv::Mat masked_image)
{
  cv::Rect rect = boundingRect(crop_points_);
  cropped_image_ = masked_image(cv::Rect(rect.x, rect.y, rect.width, rect.height));
}

cv::Point CropMap::computeOrigin()
{
  int min_x_coor = image_.cols;
  int max_y_coor = 0;
  for (long unsigned int i = 0; i < crop_points_.size(); i++)
  {
    if (min_x_coor > crop_points_[i].x)
      min_x_coor = crop_points_[i].x;
    if (max_y_coor < crop_points_[i].y)
      max_y_coor = crop_points_[i].y;
  }
  return cv::Point(min_x_coor, max_y_coor);
}

void CropMap::saveCoordinates(cv::Point origin)
{
  std::ofstream coordinates_file;
  coordinates_file.open(final_path_coordinates_);

  cv::Point_<double> origin_in_map_frame = getPointsInMapFrame(origin);
  cv::Point_<double> robots_coordinates_in_map_frame = getPointsInMapFrame(robot_coordinates_);
  coordinates_file << origin_in_map_frame.x << " " << origin_in_map_frame.y << std::endl;
  coordinates_file << robots_coordinates_in_map_frame.x << " " << robots_coordinates_in_map_frame.y << std::endl;
}
