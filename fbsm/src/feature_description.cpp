#include "fbsm/feature_description.h"
#include <algorithm>
#include <iostream>
#include <limits>
#include <numeric>

namespace line_extraction
{

///////////////////////////////////////////////////////////////////////////////
// Constructor / destructor
///////////////////////////////////////////////////////////////////////////////
FeatureDescription::FeatureDescription()
{
	search_threshold = 1.5;
	outlier_threshold = 1;
	filter_bin_size = 8;
}

FeatureDescription::~FeatureDescription()
{
}

std::vector<boost::array<double, 2> > FeatureDescription::splitLineToPoints(Line line) 
{
	boost::array<double, 2>  end = line.getEnd();
	boost::array<double, 2>  start = line.getStart();
	
	double x_dis = end[0] - start[0];
	double y_dis = end[1] - start[1];
	boost::array<double, 2> p0 = {{start[0] + (x_dis / 4), start[1] + (y_dis / 4)}};
	boost::array<double, 2> p1 = {{ start[0] + (x_dis / 2), start[1] + (y_dis / 2)}};
	boost::array<double, 2> p2 = {{ start[0] + (3 * x_dis / 4), start[1] + (3 * y_dis / 4)}};
		
	std::vector<boost::array<double, 2> > points;
	points.push_back(p0);
	points.push_back(p1);
	points.push_back(p2);
	
	return points;
}


std::vector<double> FeatureDescription::calculateHistogram(const std::vector<double>& data_x,const std::vector<double>& data_y,const boost::array<double,2>& point)
{
	std::vector<double> histogram(8);
	//histogram intialization
	for (int i = 0; i < histogram.size(); ++i)
	{
		histogram[i] = 0;
	}

	int index;
	for (int i = 0; i < data_x.size(); ++i)
	{
		double distance = sqrt(pow(point[0] - data_x[i],2) + pow(point[1] - data_y[i],2));
		if (distance < search_threshold)
		{
			index = int(distance / (search_threshold / 8));
			histogram[index] += 1;
		}   
	}

	return histogram;
}

std::vector<double> FeatureDescription::calculateLineHistogram(const std::vector<double>& data_x,const std::vector<double>& data_y,Line& line)
{
	//TODO Normalize histogram
	std::vector<double> histogram;
	//histogram intialization
	std::vector<boost::array<double, 2> > points = splitLineToPoints(line);
	std::vector<double> point_hist;
	for (int i = 0; i < 3; ++i)
	{
		point_hist = calculateHistogram(data_x,data_y,points[i]);
		histogram.insert(histogram.end(),point_hist.begin(),point_hist.end());
	}

	return histogram;
}

void FeatureDescription::setPointsHistogram(const std::vector<double>& data_x,const std::vector<double>& data_y,std::vector<Keypoint>& points)
{
	std::vector<double> histogram;
	for (int i = 0; i < points.size(); ++i)
	{
		histogram = calculateHistogram(data_x,data_y,points[i].point);
		points[i].histogram = histogram;
	}
}

void FeatureDescription::setLinesHistogram(const std::vector<double>& data_x,const std::vector<double>& data_y,std::vector<Line>& lines)
{
	std::vector<double> histogram;
	for (int i = 0; i < lines.size(); ++i)
	{
		histogram = calculateLineHistogram(data_x,data_y,lines[i]);
		lines[i].setHistogram(histogram);
	}
}

void FeatureDescription::match(std::vector<Line>& scan_lines, std::vector<Line>& map_lines) 
{
	double sum, score;
	double inf = std::numeric_limits<double>::infinity();
	int match;
	std::vector<int> matches(map_lines.size());
	std::vector<double> scores(map_lines.size());
	for (int k = 0; k < map_lines.size(); ++k)
	{
		score = inf;
		match = -1;
		for (int i = 0; i < scan_lines.size(); ++i)
		{
			sum = 0;
			for (int j = 0; j < scan_lines[i].getHistogram().size(); ++j)
			{
				sum += pow(scan_lines[i].getHistogram()[j] - map_lines[k].getHistogram()[j],2);
			} 
			sum = sqrt(sum);
			if (sum < score) {
				score = sum;
				match = i;
			}
		}
		double dist_match = sqrt(pow(scan_lines[match].getEnd()[1] - scan_lines[match].getStart()[1],2) + pow(scan_lines[match].getEnd()[0] - scan_lines[match].getStart()[0],2));
		double dist_main = sqrt(pow(map_lines[k].getEnd()[1] - map_lines[k].getStart()[1],2) + pow(map_lines[k].getEnd()[0] - map_lines[k].getStart()[0],2));
		
		if (fabs(dist_match - dist_main) > outlier_threshold) {
			map_lines[k].setMatch(-1);
			map_lines[k].setScore(-1);
			matches[k] = -1;
			scores[k] = -1;
		} else {
			//Check if we already have the same match
			for (int i = 0; i < k; ++i)
			{
				if (matches[i] == match) {
					if (scores[i] > score) {
						map_lines[i].setMatch(-1);
						map_lines[i].setScore(-1);
					} else {
						match = -1;
						score = -1;
					}
				}
			}
			matches[k] = match;
			scores[k] = score;
			map_lines[k].setMatch(match);
			map_lines[k].setScore(score);
		}
	}
}

void FeatureDescription::matchPoints(std::vector<Keypoint>& scan_points, std::vector<Keypoint>& map_points) 
{
	double sum, score;
	double inf = std::numeric_limits<double>::infinity();
	int match;
	std::vector<int> matches(map_points.size());
	std::vector<double> scores(map_points.size());
	for (int k = 0; k < map_points.size(); ++k)
	{
		score = inf;
		match = -1;
		for (int i = 0; i < scan_points.size(); ++i)
		{
			sum = 0;
			for (int j = 0; j < scan_points[i].histogram.size(); ++j)
			{
				sum += pow(scan_points[i].histogram[j] - map_points[k].histogram[j],2);
				//sum+= 1;
			} 
			sum = sqrt(sum);
			if (sum < score) {
				score = sum;
				match = i;
			}
		}
		for (int i = 0; i < k; ++i)
		{
			if (matches[i] == match) {
				if (scores[i] > score) {
					map_points[i].match = -1;
					map_points[i].score = -1;
				} else {
					match = -1;
					score = -1;
				}
			}
		}
		matches[k] = match;
		scores[k] = score;
		map_points[k].match = match;
		map_points[k].score = score;
	}
}

void FeatureDescription::cleanOutliers(std::vector<Line>& scan_lines, std::vector<Line>& map_points) 
{
	std::vector<double> rots(map_points.size());
	double max = -1 * std::numeric_limits<double>::infinity();
	double min = std::numeric_limits<double>::infinity();
	for (int i = 0; i < map_points.size(); ++i)
	{
		if (map_points[i].getMatch() >= 0) {
			rots[i] = getAngle(scan_lines[map_points[i].getMatch()],map_points[i]);
			if (rots[i] > max) {
				max = rots[i];
			}
			if (rots[i] < min) {
				min = rots[i];
			}
		} else {
			rots[i] = NAN;
		}
	}

	if ( max - min < 0.01) {
		return;
	}
	std::vector<double> histogram(filter_bin_size);
	//histogram intialization
	for (int i = 0; i < histogram.size(); ++i)
	{
		histogram[i] = 0;
	}
	int index;
	double max_index = -1;
	for (int i = 0; i < rots.size(); ++i)
	{
		if (not std::isnan(rots[i])) {
			index = int((rots[i] - min) / ((max - min) / filter_bin_size));
			if (index == filter_bin_size) { //max element
				index = filter_bin_size - 1;
			}
			histogram[index] += 1;
			if (histogram[index] > histogram[max_index]) {
				max_index = index;
			}   
		}
	}
	//get the matches inside the highest histogram bin
	for (int i = 0; i < rots.size(); ++i)
	{
		if (not std::isnan(rots[i])) {
			index = int((rots[i] - min) / ((max - min) / filter_bin_size));
			if (index == filter_bin_size) { //max element
				index = filter_bin_size - 1;
			}
			if (histogram[index] >= histogram[max_index]) {
				//inlier
			} else {
				//outlier
				map_points[i].setMatch(-1);
			}
		}      
	}
}

void FeatureDescription::cleanPointOutliers(const std::vector<Keypoint>& scan_points,std::vector<Keypoint>& map_points) 
{
	std::vector<double> trans_x(map_points.size());
	std::vector<double> trans_y(map_points.size());  
	double inf = std::numeric_limits<double>::infinity();
	double max_x = -1 * std::numeric_limits<double>::infinity();
	double max_y = -1 * std::numeric_limits<double>::infinity();
	double min_x = std::numeric_limits<double>::infinity();
	double min_y = std::numeric_limits<double>::infinity();
	for (int i = 0; i < map_points.size(); ++i)
	{

		if (map_points[i].match >= 0) {
			trans_x[i] = map_points[i].point[0] - scan_points[map_points[i].match].point[0];
			trans_y[i] = map_points[i].point[1] - scan_points[map_points[i].match].point[1];
			if (trans_x[i] > max_x) {
				max_x = trans_x[i];
			}
			if (trans_x[i] < min_x and trans_x[i] > -inf) {
				min_x = trans_x[i];
			}
			if (trans_y[i] > max_y) {
				max_y = trans_x[i];
			}
			if (trans_y[i] < min_y and trans_y[i] > -inf) {
				min_y = trans_y[i];
			}
		} else {
			trans_x[i] = NAN;
			trans_y[i] = NAN;
		}
	}
	if (( fabs(max_x - min_x) < 0.01) or ( fabs(max_y - min_y) < 0.01)) {
		return;
	}
	std::vector<double> histogram(filter_bin_size);
	//histogram intialization
	for (int i = 0; i < histogram.size(); ++i)
	{
		histogram[i] = 0;
	}
	int index;
	double max_index = -1;
	for (int i = 0; i < trans_x.size(); ++i)
	{
		if (not std::isnan(trans_x[i])) {
			index = int((trans_x[i] - min_x) / ((max_x - min_x) / filter_bin_size));
			if (index == filter_bin_size) { //max element
				index = filter_bin_size - 1;
			}
			histogram[index] += 1;
			if (histogram[index] > histogram[max_index]) {
				max_index = index;
			}   
		}
	}
	for (int i = 0; i < trans_x.size(); ++i)
	{
		if (not std::isnan(trans_x[i])) {
			index = int((trans_x[i] - min_x) / ((max_x - min_x) / filter_bin_size));
			if (index == filter_bin_size) { //max element
				index = filter_bin_size - 1;
			}
			if (histogram[index] >= histogram[max_index]) {
				//inlier
			} else {
				//outlier
				map_points[i].match = -1;
			}
		}      
	}
	for (int i = 0; i < map_points.size(); ++i)
	{

		if (map_points[i].match >= 0) {
			trans_y[i] = map_points[i].point[1] - scan_points[map_points[i].match].point[1];
			if (trans_y[i] > max_y) {
				max_y = trans_x[i];
			}
			if (trans_y[i] < min_y and trans_y[i] > -inf) {
				min_y = trans_y[i];
			}
		} else {
			trans_x[i] = NAN;
			trans_y[i] = NAN;
		}
	}
	//histogram intialization
	for (int i = 0; i < histogram.size(); ++i)
	{
		histogram[i] = 0;
	}
	max_index = -1;
	for (int i = 0; i < trans_y.size(); ++i)
	{
		if (not std::isnan(trans_y[i])) {
			index = int((trans_y[i] - min_y) / ((max_y - min_y) / filter_bin_size));
			if (index == filter_bin_size) { //max element
				index = filter_bin_size - 1;
			}
			histogram[index] += 1;
			if (histogram[index] > histogram[max_index]) {
				max_index = index;
			}   
		}
	}
	for (int i = 0; i < trans_y.size(); ++i)
	{
		if (not std::isnan(trans_y[i])) {
			index = int((trans_y[i] - min_y) / ((max_y - min_y) / filter_bin_size));
			if (index == filter_bin_size) { //max element
				index = filter_bin_size - 1;
			}
			if (histogram[index] >= histogram[max_index]) {
				//inlier
			} else {
				//outlier
				map_points[i].match = -1;
			}
		}      
	}
}

void FeatureDescription::histogram_filter(const std::vector<double>& v, std::vector<double>& out, double n_bins) 
{
	out.clear();
	std::vector<double> histogram(n_bins);
	double inf = std::numeric_limits<double>::infinity();
	double max = -1 * inf;
	double min = inf;
	for (int i = 0; i < v.size(); ++i)
	{
		if (v[i] > max and v[i] < inf) {
			max = v[i];
		}
		if (v[i] < min and v[i] > -inf) {
			min = v[i];
		}
	}
	if (fabs(max - min) < 0.001) {
		out = v;
		return;
	}
	//histogram intialization
	for (int i = 0; i < histogram.size(); ++i)
	{
		histogram[i] = 0;
	}
	int index;
	double max_index = -1;
	for (int i = 0; i < v.size(); ++i)
	{
		if (not std::isnan(v[i])) {
			index = int((v[i] - min) / ((max - min) / n_bins));
			if (index == n_bins) { //max element
				index = n_bins - 1;
			}
			histogram[index] += 1;
			if (histogram[index] > histogram[max_index]) {
				max_index = index;
			}   
		}
	}
	for (int i = 0; i < v.size(); ++i)
	{
		if (not std::isnan(v[i])) {
			index = int((v[i] - min) / ((max - min) / n_bins));
			if (index == n_bins) { //max element
				index = n_bins - 1;
			}
			if (histogram[index] >= histogram[max_index]) {
				out.push_back(v[i]);
			} 
		}      
	}
}


double FeatureDescription::getAngle(Line line,Line map_line) 
{
		int p1 = std::inner_product(line.getStart().begin(), line.getStart().end(), map_line.getStart().begin(), 0);
		int p2 = std::inner_product(line.getEnd().begin(), line.getEnd().end(), map_line.getEnd().begin(), 0);
		int p3 = std::inner_product(line.getStart().begin(), line.getStart().end(), line.getStart().begin(), 0);
		int p4 = std::inner_product(line.getEnd().begin(), line.getEnd().end(), line.getEnd().begin(), 0);
		int p5 = std::inner_product(map_line.getStart().begin(), map_line.getStart().end(), map_line.getStart().begin(), 0);
		int p6 = std::inner_product(map_line.getEnd().begin(), map_line.getEnd().end(), map_line.getEnd().begin(), 0);
		double d = (p1 + p2) / (sqrt(p3 + p4) * sqrt(p5 + p6));

		if (d > 1) {
			d = 1;
		} else if (d < -1) {
			d = -1;
		}
		
		double angle = acos(d); 
		return angle;
}

void FeatureDescription::getTransformAngle(Line line,Line map_line, std::vector<double>& transform) 
{
		transform.clear();
		double l1_a, l1_b, l2_a, l2_b, l1_ang, l2_ang;
		l1_a = (line.getEnd()[1] - line.getStart()[1]) / (line.getEnd()[0] - line.getStart()[0]);
		l1_b = line.getEnd()[1] - line.getEnd()[0] * l1_a;
		l2_a = (map_line.getEnd()[1] - map_line.getStart()[1]) / (map_line.getEnd()[0] - map_line.getStart()[0]);
		l2_b = map_line.getEnd()[1] - map_line.getEnd()[0] * l2_a;
		
		l1_ang = atan2((line.getEnd()[1] - line.getStart()[1]),(line.getEnd()[0] - line.getStart()[0]));
		l2_ang = atan2((map_line.getEnd()[1] - map_line.getStart()[1]), (map_line.getEnd()[0] - map_line.getStart()[0]));

		double angle =  -1 * (M_PI - fabs(l1_ang - l2_ang));
		
		transform.push_back(map_line.getStart()[0] - line.getStart()[0]);
		transform.push_back(map_line.getStart()[1] - line.getStart()[1]);
		transform.push_back(angle);
}

double FeatureDescription::matchScore(nav_msgs::OccupancyGrid::ConstPtr map,std::vector<double> scan_x, std::vector<double> scan_y) 
{
	float width = map->info.width;
	float height = map->info.height;
	geometry_msgs::Pose origin = map->info.origin;
	float res = map->info.resolution;
	double score = 0;
	int index;
	for (int i = 0; i < scan_x.size(); ++i)
	{ 
		if (not (std::isnan(scan_x[i]) or std::isnan(scan_y[i])))
		{
			index = int(((scan_x[i] - origin.position.x) / res)) + (int(((scan_y[i] - origin.position.y) / res)) * width);
			if ((index >= 0) and (index < width * height) and (map->data[index] >= 90))
			{
				score += 1;
			} 
		}
	}
	score = score / scan_x.size();
	return score;
}

}