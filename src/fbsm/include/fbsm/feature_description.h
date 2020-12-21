#ifndef FEATURE_DESCRIPTION_H
#define FEATURE_DESCRIPTION_H

#include <vector>
#include <boost/array.hpp>
#include "fbsm/line.h"
#include <nav_msgs/OccupancyGrid.h>

namespace line_extraction
{

class FeatureDescription
{

public:
  FeatureDescription();
  ~FeatureDescription();

  std::vector<boost::array<double,2> > splitLineToPoints(Line line); 
  std::vector<double> calculateHistogram(const std::vector<double>& data_x,const std::vector<double>& data_y,const boost::array<double,2>& point);
  std::vector<double> calculateLineHistogram(const std::vector<double>& data_x,const std::vector<double>& data_y,Line& line);
  void setPointsHistogram(const std::vector<double>& data_x,const std::vector<double>& data_y,std::vector<Keypoint>& points);
  void setLinesHistogram(const std::vector<double>& data_x,const std::vector<double>& data_y,std::vector<Line>& lines);
  void match(std::vector<Line>& scan_lines, std::vector<Line>& map_lines);
  void matchPoints(std::vector<Keypoint>& scan_points, std::vector<Keypoint>& map_points); 
  void getTransformAngle(Line line,Line map_line, std::vector<double>& transform); 
  void cleanOutliers(std::vector<Line>& scan_lines, std::vector<Line>& map_lines);
  double getAngle(Line line,Line map_line);
  void cleanPointOutliers(const std::vector<Keypoint>& scan_points, std::vector<Keypoint>& map_points); 
  double matchScore(nav_msgs::OccupancyGrid::ConstPtr map,std::vector<double> scan_x, std::vector<double> scan_y);
  void histogram_filter(const std::vector<double>& v, std::vector<double>& out, double n_bins);

private:
  double search_threshold;
  double outlier_threshold;
  double filter_bin_size;
};
}

#endif