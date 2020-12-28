#include "fbsm/line_extraction_ros.h"
#include <ros/console.h>

namespace line_extraction
{
///////////////////////////////////////////////////////////////////////////////
// Constructor / destructor
///////////////////////////////////////////////////////////////////////////////
LineExtractionROS::LineExtractionROS()
  : nh_local_("~"), data_cached_(false), map_ready(false), map_available(false), sleeping_(false)
{
  initialize();
}

LineExtractionROS::~LineExtractionROS()
{
}

void LineExtractionROS::initialize()
{
  ros::Time::waitForValid();

  if (!loadParams())
  {
    ROS_FATAL("Error during parameter loading. Shutting down.");
    return;
  }
  ROS_INFO("All parameters loaded. Launching.");
  configureLineExtractors();
  setupTopics();

  main_timer_ = nh_.createTimer(ros::Duration(1.0 / p_loop_rate_), &LineExtractionROS::run, this);
  ros::spin();
}

void LineExtractionROS::setupTopics()
{
  line_publisher_ = nh_.advertise<fbsm::LineSegmentList>("line_segments", 1);
  map_line_publisher_ = nh_.advertise<fbsm::LineSegmentList>("map_line_segments", 1);
  map_scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>("map_scan", 1);
  laser_scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>("laser_scan", 1);
  ini_pos_publisher_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
  pc_pub = nh_.advertise<sensor_msgs::PointCloud>("pc", 1);
  fbsm_pub = nh_.advertise<std_msgs::Empty>("fbsm", 1);

  service = nh_.advertiseService("/correct_pose", &LineExtractionROS::correct_pose_srv, this);

  scan_subscriber_ = nh_.subscribe(p_scan_topic_, 1, &LineExtractionROS::laserScanCallback, this);
  map_subscriber_ = nh_.subscribe(p_map_topic_, 1, &LineExtractionROS::mapCallback, this);
  goal_subscriber_ = nh_.subscribe("/move_base_goal_latched", 1, &LineExtractionROS::goalCallback, this);
  reached_subscriber_ = nh_.subscribe("/reached_goal", 1, &LineExtractionROS::reachedCallback, this);

  if (p_pub_markers_)
  {
    marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("line_markers", 1);
    map_marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("map_line_markers", 1);
  }
}

bool LineExtractionROS::correct_pose_srv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  run();
  std_msgs::Empty msg;
  fbsm_pub.publish(msg);
  return true;
}
///////////////////////////////////////////////////////////////////////////////
// Run
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::run(const ros::TimerEvent&)
{
  run();
}

void LineExtractionROS::run()
{
  if (map_ready and data_cached_ and !sleeping_ and stop_count < 10)
  {
    ros::Time time_received = ros::Time::now();

    // Extract the lines
    std::vector<Line> lines;
    std::vector<Line> map_lines;
    line_extraction_.extractLines(lines);
    map_line_extraction_.extractLines(map_lines);
    if (lines.size() == 0 or map_lines.size() == 0)
    {
      ROS_WARN("Couldn't detect any lines.");
      return;
    }
    // Get Orientation estimate by matching lines from scan and map
    std::vector<double> transform_line;
    double best_yaw_score = 10000;
    double arg_best_yaw;
    double best_yaw;
    std::vector<double> yaws;
    std::vector<double> pos_yaws;

    if (not p_orientation_bf_)
    {
      int n_matches = 0;
      feature_desc_.setLinesHistogram(line_extraction_.getRangeData().xs, line_extraction_.getRangeData().ys, lines);
      feature_desc_.setLinesHistogram(map_line_extraction_.getRangeData().xs, map_line_extraction_.getRangeData().ys,
                                      map_lines);
      feature_desc_.match(lines, map_lines);
      for (int i = 0; i < map_lines.size(); ++i)
      {
        if (map_lines[i].getMatch() >= 0)
        {
          n_matches++;
          feature_desc_.getTransformAngle(map_lines[i], lines[map_lines[i].getMatch()], transform_line);

          if (map_lines[i].getScore() < best_yaw_score)
          {
            best_yaw_score = map_lines[i].getScore();
            arg_best_yaw = i;
            best_yaw = transform_line[2];
          }
          if (not std::isnan(transform_line[2]))
          {
            double c_angle = constrainAngle(transform_line[2]);
            double c_angle_180 = constrainAngle(transform_line[2] + M_PI);
            if (fabs(c_angle) < p_angle_upper_threshold_)
            {
              yaws.push_back(c_angle);
              yaws.push_back(-1 * c_angle);
            }
            if (fabs(c_angle_180) < p_angle_upper_threshold_)
            {
              yaws.push_back(c_angle_180);
              yaws.push_back(-1 * c_angle_180);
            }
          }
        }
      }
      std::vector<double> yaws_fill;
      feature_desc_.histogram_filter(yaws, yaws_fill, p_filter_bins_);
      yaws = yaws_fill;
    }
    else
    {
      for (int i = 0; i < map_lines.size(); ++i)
      {
        for (int j = 0; j < lines.size(); ++j)
        {
          feature_desc_.getTransformAngle(map_lines[i], lines[j], transform_line);
          if (not std::isnan(transform_line[2]))
          {
            double c_angle = constrainAngle(transform_line[2]);
            double c_angle_180 = constrainAngle(transform_line[2] + M_PI);
            if (fabs(c_angle) < p_angle_upper_threshold_)
            {
              yaws.push_back(c_angle);
              yaws.push_back(-1 * c_angle);
            }
            if (fabs(c_angle_180) < p_angle_upper_threshold_)
            {
              yaws.push_back(c_angle_180);
              yaws.push_back(-1 * c_angle_180);
            }
          }
        }
      }
    }
    // remove duplicates
    sort(yaws.begin(), yaws.end());
    yaws.erase(unique(yaws.begin(), yaws.end(), is_near_yaw), yaws.end());
    if (yaws.size() == 0)
    {
      ROS_WARN("0 orientations found. Try increasing the search thresholds.");
      // Even if there's 0 orientations, keep running to try finding a better translation
      yaws.push_back(0.0);
      // return;
    }

    std::vector<double> origin_scan, origin_map;
    std::vector<Keypoint> scan_keypoints, map_keypoints;
    std::vector<double> map_xs, map_ys, scan_xs, scan_ys;

    map_xs = map_line_extraction_.getRangeData().xs;
    map_ys = map_line_extraction_.getRangeData().ys;
    scan_xs = line_extraction_.getRangeData().xs;
    scan_ys = line_extraction_.getRangeData().ys;

    if (p_scan_line_mask_)
    {
      scanMaskCart(scan_xs, scan_ys, lines);
    }

    std::vector<double> tx_v, ty_v;
    if (p_mode_ == "fm")
    {
      // Use feature point matchinng to estimate translation
      scanToPoints(map_xs, map_ys, map_keypoints);
      scanToPoints(scan_xs, scan_ys, scan_keypoints);
      feature_desc_.setPointsHistogram(map_xs, map_ys, map_keypoints);
      feature_desc_.setPointsHistogram(scan_xs, scan_ys, scan_keypoints);
      feature_desc_.matchPoints(scan_keypoints, map_keypoints);

      double tx, ty;
      for (int i = 0; i < map_keypoints.size(); ++i)
      {
        if (map_keypoints[i].match > -1)
        {
          tx = map_keypoints[i].point[0] - scan_keypoints[map_keypoints[i].match].point[0];
          ty = map_keypoints[i].point[1] - scan_keypoints[map_keypoints[i].match].point[1];

          if ((fabs(tx) < p_dist_upper_threshold_) and (fabs(ty) < p_dist_upper_threshold_))
          {
            tx_v.push_back(tx);
            ty_v.push_back(ty);
          }
        }
      }
      // filter outliers
      std::vector<double> tx_v_fil, ty_v_fil;
      if (not p_translation_bf_)
      {
        feature_desc_.histogram_filter(tx_v, tx_v_fil, p_filter_bins_);
        feature_desc_.histogram_filter(tx_v_fil, tx_v, p_filter_bins_);
        feature_desc_.histogram_filter(ty_v, ty_v_fil, p_filter_bins_);
        feature_desc_.histogram_filter(ty_v_fil, ty_v, p_filter_bins_);

        tx_v = tx_v_fil;
        ty_v = ty_v_fil;
      }
    }
    double current_score = 0.0, valid_pts = 0.0;
    // double current_rotational_error = 0.0;
    double current_proj_score = 0, current_inliers_ratio, avg_inliers_dist, current_certitude;
    if (p_better_check_)
    {
      // calculate current matching score
      current_score = 0.0;
      tf::StampedTransform null_transform;
      // raycastWithRotation(null_transform,map_xs,map_ys,false);
      map_xs = map_line_extraction_.getRangeData().xs;
      map_ys = map_line_extraction_.getRangeData().ys;
      current_inliers_ratio = 0.0;
      avg_inliers_dist = 0.0;

      for (int i = 0; i < scan_xs.size(); ++i)
      {
        if ((!std::isnan(scan_xs[i])) and (!std::isnan(scan_ys[i])) and (!std::isnan(map_ys[i])) and
            (!std::isnan(map_xs[i])))
        {
          if ((fabs(scan_xs[i]) != inf) and (fabs(map_xs[i]) != inf) and (fabs(scan_ys[i]) != inf) and
              (fabs(map_ys[i]) != inf))
          {
            double c_r = sqrt(pow(scan_xs[i] - map_xs[i], 2) + pow(scan_ys[i] - map_ys[i], 2));
            valid_pts++;
            if (c_r < p_inlier_dist_)
            {
              current_inliers_ratio++;
              avg_inliers_dist += (p_inlier_dist_ - c_r);
            }
          }
        }
      }
      current_score = current_score / valid_pts;
      if (current_inliers_ratio != 0)
      {
        avg_inliers_dist = avg_inliers_dist / current_inliers_ratio;
      }
      current_inliers_ratio = current_inliers_ratio / valid_pts;
      current_certitude = pow(current_inliers_ratio, p_inliers_exp_) * avg_inliers_dist;

      if (std::isnan(current_score))
      {
        ROS_WARN("Error: current score is nan.");

        return;
      }
      if (std::isnan(current_certitude))
      {
        ROS_WARN("Error: certitude is nan.");
        return;
      }

      if (p_proj_score_)
      {
        current_proj_score = getProjectionScore(scan_xs, scan_ys, 0.0, 0.0, 0.0);
      }
    }

    double score, best_tx, best_ty, best_match_score, best_ir, best_id;
    // ROS_DEBUG("processing before matching time : %f", (ros::Time::now() - time_received).toSec());

    std::vector<double> result = getMatchScoreRaycast(scan_xs, scan_ys, tx_v, ty_v, yaws);
    best_tx = result[0];
    best_ty = result[1];
    best_yaw = result[2];
    best_match_score = result[3];
    best_ir = result[4];
    best_id = result[5];

    if (best_match_score > 1000)
    {
      ROS_WARN("Matching Failed.");
      return;
    }

    bool improved = true;

    if (p_better_check_ and (best_match_score - current_certitude < p_precision_))
    {
      improved = false;
    }
    if (best_ir < p_min_inliers_)
    {
      ROS_WARN("Match with less inliers than min_inliers : %f", best_ir);
      return;
    }

    if (p_proj_score_)
    {
      ROS_DEBUG("Current Score = %f, CPS = %f Best matching transform x = %f y = %f yaw = %f score = %f",
                current_certitude, current_proj_score, best_tx, best_ty, best_yaw, best_match_score);
    }
    else
    {
      ROS_DEBUG("Current Score = %f, IR = %f, ID= %f, Best matching transform x = %f y = %f yaw = %f score = %f",
                current_certitude, best_ir, best_id, best_tx, best_ty, best_yaw, best_match_score);
    }
    double lin_dist = sqrt(pow(goal_.pose.position.x - transform.getOrigin().x(), 2) +
                           pow(goal_.pose.position.y - transform.getOrigin().y(), 2));

    if (best_match_score < p_raycast_score_threshold_)
    {
      if (improved)
      {
        // On proj score check activated,
        if (p_proj_score_ and (fabs(current_certitude - best_match_score) < p_switch_to_proj_threshold_))
        {
          double score = getProjectionScore(scan_xs, scan_ys, best_tx, best_ty, best_yaw);
          if (score > p_proj_score_threshold_ and score > current_proj_score)
          {
            ROS_DEBUG("Projection Score = %f", score);
            publishInitialPose(best_tx, best_ty, best_yaw);
          }
          else
          {
            ROS_WARN("Projection failed, current projection score = %f  projection score = %f", current_proj_score,
                     score);
          }
        }
        else
        {
          if (p_docking_)
          {
            if (((lin_dist > 0.02) or (best_match_score - current_certitude) > 0.0001))
            {
              publishInitialPose(best_tx, best_ty, best_yaw);
              stop_count = 0;
            }
          }
          else
          {
            publishInitialPose(best_tx, best_ty, best_yaw);
          }
        }
      }
    }

    if ((p_docking_) and (lin_dist < 0.02))
    {
      // number of iterations without update
      stop_count++;
    }
    ROS_DEBUG("processing time : %f", (ros::Time::now() - time_received).toSec());

    // publish markers if parameter publish_markers is set to true
    if (p_pub_markers_)
    {
      // Populate message
      fbsm::LineSegmentList msg;
      populateLineSegListMsg(lines, msg);
      fbsm::LineSegmentList map_msg;
      populateLineSegListMsg(map_lines, map_msg);
      // Publish the lines
      line_publisher_.publish(msg);
      map_line_publisher_.publish(map_msg);

      visualization_msgs::Marker marker_msg;
      populateMarkerMsg(lines, marker_msg);
      marker_publisher_.publish(marker_msg);

      visualization_msgs::Marker map_marker_msg;
      populateMarkerMsg(map_lines, map_marker_msg);
      map_marker_publisher_.publish(map_marker_msg);

      sensor_msgs::LaserScan map_scan;
      RangeData map_range_data = map_line_extraction_.getRangeData();
      scanMask(map_range_data, map_lines);

      sensor_msgs::LaserScan laser_scan;
      laser_scan = map_scan;
      RangeData laser_range_data = line_extraction_.getRangeData();
      scanMask(laser_range_data, lines);

      std::vector<float> s_r(laser_range_data.ranges.begin(), laser_range_data.ranges.end());
      laser_scan.ranges = s_r;
      laser_scan_publisher_.publish(laser_scan);
    }
  }
  else if (stop_count >= 10)
  {
    ROS_WARN("Stopped");
  }
}

///////////////////////////////////////////////////////////////////////////////
// Load ROS parameters
///////////////////////////////////////////////////////////////////////////////
bool LineExtractionROS::loadParams()
{
  ros_utils::ParamLoader loader(nh_local_);

  // Parameters used by this node
  loader.get_required("loop_rate", p_loop_rate_);
  loader.get_required("laser_frame", p_laser_frame_);
  loader.get_required("map_frame", p_map_frame_);
  loader.get_required("base_frame", p_base_frame_);
  loader.get_required("odom_frame", p_odom_frame_);
  loader.get_required("scan_topic", p_scan_topic_);
  loader.get_required("map_topic", p_map_topic_);
  loader.get_required("publish_markers", p_pub_markers_);

  // Parameters used by the line extraction algorithm
  loader.get_required("bearing_std_dev", p_bearing_std_dev_);
  loader.get_required("range_std_dev", p_range_std_dev_);
  loader.get_required("least_sq_angle_thresh", p_least_sq_angle_thresh_);
  loader.get_required("least_sq_radius_thresh", p_least_sq_radius_thresh_);
  loader.get_required("max_line_gap", p_max_line_gap_);
  loader.get_required("min_line_length", p_min_line_length_);
  loader.get_required("min_range", p_min_range_);
  loader.get_required("min_split_dist", p_min_split_dist_);
  loader.get_required("outlier_dist", p_outlier_dist_);
  loader.get_required("min_line_points", p_min_line_points_);
  loader.get_required("max_range_cap", p_max_range_cap_def_);

  // Params used by the scan matchinng algorithm
  loader.get_required("dist_upper_threshold", p_dist_upper_threshold_);
  loader.get_required("angle_upper_threshold", p_angle_upper_threshold_);
  loader.get_required("raycast_score_threshold", p_raycast_score_threshold_);
  loader.get_required("proj_score", p_proj_score_);
  loader.get_required("proj_score_threshold", p_proj_score_threshold_);
  loader.get_required("filter_bins", p_filter_bins_);
  loader.get_required("scan_line_mask", p_scan_line_mask_);
  loader.get_required("orientation_bf", p_orientation_bf_);
  loader.get_required("translation_bf", p_translation_bf_);
  loader.get_required("better_check", p_better_check_);
  loader.get_required("docking", p_docking_);
  loader.get_required("mode", p_mode_);
  loader.get_required("switch_to_proj_threshold", p_switch_to_proj_threshold_);
  loader.get_required("docking_lin_dist", p_docking_lin_dist_);
  loader.get_required("docking_ang_dist", p_docking_ang_dist_);
  loader.get_required("inlier_dist", p_inlier_dist_);
  loader.get_required("inliers_exp", p_inliers_exp_);
  loader.get_required("min_inliers", p_min_inliers_);
  loader.get_required("precision", p_precision_);
  loader.get_required("multithreading", p_multithreading_);

  return loader.params_valid();
}

void LineExtractionROS::configureLineExtractors()
{
  line_extraction_.setBearingVariance(p_bearing_std_dev_ * p_bearing_std_dev_);
  map_line_extraction_.setBearingVariance(p_bearing_std_dev_ * p_bearing_std_dev_);

  line_extraction_.setRangeVariance(p_range_std_dev_ * p_range_std_dev_);
  map_line_extraction_.setRangeVariance(p_range_std_dev_ * p_range_std_dev_);

  line_extraction_.setLeastSqAngleThresh(p_least_sq_angle_thresh_);
  map_line_extraction_.setLeastSqAngleThresh(p_least_sq_angle_thresh_);

  line_extraction_.setLeastSqRadiusThresh(p_least_sq_radius_thresh_);
  map_line_extraction_.setLeastSqRadiusThresh(p_least_sq_radius_thresh_);

  line_extraction_.setMaxLineGap(p_max_line_gap_);
  map_line_extraction_.setMaxLineGap(p_max_line_gap_);

  line_extraction_.setMinLineLength(p_min_line_length_);
  map_line_extraction_.setMinLineLength(p_min_line_length_);

  line_extraction_.setMinRange(p_min_range_);
  map_line_extraction_.setMinRange(p_min_range_);

  line_extraction_.setMinSplitDist(p_min_split_dist_);
  map_line_extraction_.setMinSplitDist(p_min_split_dist_);

  line_extraction_.setOutlierDist(p_outlier_dist_);
  map_line_extraction_.setOutlierDist(p_outlier_dist_);

  line_extraction_.setMinLinePoints(static_cast<unsigned int>(p_min_line_points_));
  map_line_extraction_.setMinLinePoints(static_cast<unsigned int>(p_min_line_points_));
}

///////////////////////////////////////////////////////////////////////////////
// Populate messages
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::populateLineSegListMsg(const std::vector<Line>& lines, fbsm::LineSegmentList& line_list_msg)
{
  for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
  {
    fbsm::LineSegment line_msg;
    line_msg.angle = cit->getAngle();
    line_msg.radius = cit->getRadius();
    line_msg.covariance = cit->getCovariance();
    line_msg.start = cit->getStart();
    line_msg.end = cit->getEnd();
    line_list_msg.line_segments.push_back(line_msg);
  }
  line_list_msg.header.frame_id = p_laser_frame_;
  line_list_msg.header.stamp = ros::Time::now();
}

void LineExtractionROS::populateMarkerMsg(const std::vector<Line>& lines, visualization_msgs::Marker& marker_msg)
{
  marker_msg.ns = "line_extraction";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::Marker::LINE_LIST;
  marker_msg.scale.x = 0.1;
  marker_msg.color.r = 1.0;
  marker_msg.color.g = 0.0;
  marker_msg.color.b = 0.0;
  marker_msg.color.a = 1.0;
  for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
  {
    geometry_msgs::Point p_start;
    p_start.x = cit->getStart()[0];
    p_start.y = cit->getStart()[1];
    p_start.z = 0;
    marker_msg.points.push_back(p_start);
    geometry_msgs::Point p_end;
    p_end.x = cit->getEnd()[0];
    p_end.y = cit->getEnd()[1];
    p_end.z = 0;
    marker_msg.points.push_back(p_end);
  }
  marker_msg.header.frame_id = p_laser_frame_;
  marker_msg.header.stamp = ros::Time::now();
}

///////////////////////////////////////////////////////////////////////////////
// Cache data on first LaserScan message received
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::cacheData(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  std::vector<double> bearings, cos_bearings, sin_bearings;
  std::vector<unsigned int> indices;
  angle_min = scan_msg->angle_min;
  angle_max = scan_msg->angle_max;
  range_max = scan_msg->range_max;
  range_min = scan_msg->range_min;
  angle_increment = scan_msg->angle_increment;
  const std::size_t num_measurements =
      std::ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
  for (std::size_t i = 0; i < num_measurements; ++i)
  {
    const double b = scan_msg->angle_min + i * scan_msg->angle_increment;
    bearings.push_back(b);
    cos_bearings.push_back(cos(b));
    sin_bearings.push_back(sin(b));
    indices.push_back(i);
  }

  line_extraction_.setCachedData(bearings, cos_bearings, sin_bearings, indices);
  map_line_extraction_.setCachedData(bearings, cos_bearings, sin_bearings, indices);
  ROS_DEBUG("Data has been cached.");
}

///////////////////////////////////////////////////////////////////////////////
// Main LaserScan callback
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  if (!data_cached_)
  {
    cacheData(scan_msg);
    data_cached_ = true;
  }
  // scan_stamp = scan_msg->header.stamp + ros::Duration().fromSec(scan_msg->ranges.size()*scan_msg->time_increment);
  scan_stamp = ros::Time::now();
  try
  {
    bool state = listener_.waitForTransform(p_map_frame_, p_base_frame_, scan_stamp, ros::Duration(0.5));
    listener_.lookupTransform(p_map_frame_, p_base_frame_, scan_stamp, transform);

    state = listener_.waitForTransform(p_map_frame_, p_odom_frame_, scan_stamp, ros::Duration(0.5));
    listener_.lookupTransform(p_map_frame_, p_odom_frame_, scan_stamp, transform_amcl);

    state = listener_.waitForTransform(p_laser_frame_, p_map_frame_, scan_stamp, ros::Duration(0.5));
    listener_.lookupTransform(p_laser_frame_, p_map_frame_, scan_stamp, transform_laser);
  }
  catch (tf::TransformException& e)
  {
    ROS_WARN("Failed to get transform at scan time.");
    ROS_WARN("%s", e.what());
    return;
  }

  if (p_docking_)
  {
    double lin_dist = sqrt(pow(goal_.pose.position.x - transform.getOrigin().x(), 2) +
                           pow(goal_.pose.position.y - transform.getOrigin().y(), 2));
    double ang_dist = fabs(tf::getYaw(transform.getRotation()) - tf::getYaw(goal_.pose.orientation));
    if (lin_dist > p_docking_lin_dist_ or ang_dist > p_docking_ang_dist_ or lin_dist < 0.10)
    {
      if (!sleeping_)
      {
        ROS_WARN("Far from goal");
      }
      sleeping_ = true;
      return;
    }
    else
    {
      sleeping_ = false;
    }
  }

  std::vector<double> scan_ranges_doubles(scan_msg->ranges.begin(), scan_msg->ranges.end());
  max_range_cap = 0;
  double valid_pts = 0, valid_pts_cap = 0;
  for (int i = 0; i < scan_ranges_doubles.size(); ++i)
  {
    if (scan_ranges_doubles[i] > scan_msg->range_min and scan_ranges_doubles[i] < scan_msg->range_max)
    {
      valid_pts++;
      if (scan_ranges_doubles[i] > max_range_cap)
      {
        max_range_cap = scan_ranges_doubles[i];
      }
      if (scan_ranges_doubles[i] < p_max_range_cap_def_)
      {
        valid_pts_cap++;
      }
    }
  }
  // we set the range cap given the max range of the scan rays and the number of valid rays
  if ((max_range_cap > p_max_range_cap_def_) and (valid_pts_cap > (0.5 * valid_pts)))
  {
    max_range_cap = p_max_range_cap_def_;
  }
  line_extraction_.setMaxRange(max_range_cap);
  map_line_extraction_.setMaxRange(max_range_cap);
  line_extraction_.setRangeData(scan_ranges_doubles);
  if (map_available)
  {
    processMap(scan_msg->header.stamp);
  }
}

void LineExtractionROS::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  map = msg;
  resolution = map->info.resolution;
  float width = map->info.width;
  float height = map->info.height;

  float origin_x = map->info.origin.position.x;
  float origin_y = map->info.origin.position.y;

  map_pc = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>());

  std_msgs::Header header;
  header.stamp = msg->header.stamp;
  header.frame_id = p_map_frame_;
  map_pc->header = pcl_conversions::toPCL(header);

  pcl::PointXYZ point_xyz;
  for (int y = 0; y < height; y++)
    for (int x = 0; x < width; x++)
    {
      if (((x + y * width) < (width * height)) and (map->data[x + y * width] == 100))
      {
        point_xyz.x = (x)*resolution + origin_x;
        point_xyz.y = (y)*resolution + origin_y;
        point_xyz.z = 0;

        map_pc->points.push_back(point_xyz);
      }
    }
  map_tree.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
  map_tree->setInputCloud(map_pc);
  ROS_DEBUG("Initialising distance transform map...");
  intialize_distance_map();
  ROS_DEBUG("Done");
  map_available = true;
  if (data_cached_)
  {
    // processMap();
  }
}

void LineExtractionROS::intialize_distance_map()
{
  unsigned long width, height;
  width = map->info.width;
  height = map->info.height;

  std::vector<std::size_t> grid_size({ width, height });
  MMArray<float, 2> f(grid_size.data());
  MMArray<std::size_t, 2> indices(grid_size.data());

  for (std::size_t i = 0; i < width; ++i)
    for (std::size_t j = 0; j < height; ++j)
      if (map->data[i + width * j] == 100)
        f[i][j] = 0.0f;
      else
        f[i][j] = std::numeric_limits<float>::max();

  DistanceTransform::distanceTransformL2(f, f, indices, false);

  // allocate space in the vectors
  for (int i = 0; i < width; ++i)
  {
    std::vector<float> y_axis;
    for (int q = 0; q < height; ++q)
      y_axis.push_back(1.0);
    grid.push_back(y_axis);
  }

  // store to array
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      grid[x][y] = f[x][y] * map->info.resolution;
    }
  }
}

void LineExtractionROS::processMap(const ros::Time stamp)
{
  tf::Transform transform_to_laser;
  transform_to_laser = transform_laser.inverse();
  double angle, range;
  double roll, pitch, yaw;
  tf::Matrix3x3(transform_to_laser.getRotation()).getRPY(roll, pitch, yaw);

  a_clockwise_ = 1;
  if (not fabs(roll) < 0.2)  // map and laser frame are inverted
  {
    a_clockwise_ = -1;
  }

  int num_measurements = (int)((angle_max - angle_min) / angle_increment);
  std::vector<double> ranges(num_measurements);
  rayMarching(transform_to_laser, ranges);
  map_line_extraction_.setRangeData(ranges);
  map_ready = true;

  std::vector<float> m_r(ranges.begin(), ranges.end());

  sensor_msgs::LaserScan map_scan;
  map_scan.header.stamp = ros::Time::now();
  map_scan.header.frame_id = p_laser_frame_;
  map_scan.ranges = m_r;
  map_scan.angle_min = angle_min;
  map_scan.angle_max = angle_max;
  map_scan.angle_increment = angle_increment;
  map_scan.range_min = 0.01;
  map_scan.range_max = max_range_cap;
  map_scan.time_increment = 0.001;
  std::vector<float> intensities;

  for (int i = 0; i < num_measurements; ++i)
  {
    intensities.push_back(47.0);
  }
  map_scan.intensities = intensities;

  map_scan_publisher_.publish(map_scan);
}

void LineExtractionROS::raycastWithRotation(const tf::StampedTransform& rel_transform, std::vector<double>& xs,
                                            std::vector<double>& ys, bool pub_map_scan)
{
  double angle, range;
  tf::Transform transform_to_laser;

  if (rel_transform.frame_id_ == "")
  {
    transform_to_laser = transform_laser.inverse();
  }
  else
  {
    transform_to_laser = transform_laser.inverse() * rel_transform.inverse();
  }

  int num_measurements = (int)((angle_max - angle_min) / angle_increment);
  std::vector<double> ranges(num_measurements);
  rayMarching(transform_to_laser, ranges);

  if (pub_map_scan)
  {
    int num_measurements = (int)((angle_max - angle_min) / angle_increment);
    std::vector<float> m_r(ranges.begin(), ranges.end());

    sensor_msgs::LaserScan map_scan;
    map_scan.header.stamp = ros::Time::now();
    map_scan.header.frame_id = p_laser_frame_;
    map_scan.ranges = m_r;
    map_scan.angle_min = angle_min;
    map_scan.angle_max = angle_max;
    map_scan.angle_increment = angle_increment;
    map_scan.range_min = 0.01;
    map_scan.range_max = max_range_cap;
    map_scan.time_increment = 0.001;
    std::vector<float> intensities;

    for (int i = 0; i < num_measurements; ++i)
    {
      intensities.push_back(47.0);
    }
    map_scan.intensities = intensities;

    map_scan_publisher_.publish(map_scan);
  }

  map_line_extraction_.getXY(ranges, xs, ys);
}

void LineExtractionROS::rayMarching(const tf::Transform& tf_l_m, std::vector<double>& ranges)
{
  int num_measurements = (int)((angle_max - angle_min) / angle_increment);
  double angle, r;
  double distThreshold = 0.0;
  double step_coeff = 1;

  angle = tf::getYaw(tf_l_m.getRotation()) + a_clockwise_ * angle_min;
  double x = tf_l_m.getOrigin().x();
  double y = tf_l_m.getOrigin().y();

  for (int i = 0; i < num_measurements; ++i)
  {
    double ray_direction_x = cosf(angle);
    double ray_direction_y = sinf(angle);

    double px = 0;
    double py = 0;

    double t = 0.0;
    r = inf;
    while (t < range_max)
    {
      px = x + ray_direction_x * t;
      py = y + ray_direction_y * t;

      int index_x = int(((px - map->info.origin.position.x) / map->info.resolution));
      int index_y = int(((py - map->info.origin.position.y) / map->info.resolution));

      if (index_x >= map->info.width || index_x < 0 || index_y < 0 || index_y >= map->info.height)
      {
        break;
      }
      double d = grid[index_x][index_y];

      if (d <= distThreshold)
      {
        double xd = px - x;
        double yd = py - y;
        r = sqrtf(xd * xd + yd * yd);
        break;
      }
      t += std::max<double>(d * step_coeff, 0.001);
    }

    ranges[i] = r;
    angle += a_clockwise_ * angle_increment;
  }
}

void LineExtractionROS::scanMask(RangeData& scan, const std::vector<Line>& lines)
{
  std::vector<int> indices;
  for (int i = 0; i < lines.size(); ++i)
  {
    for (int j = 0; j < scan.xs.size(); ++j)
    {
      boost::array<double, 2> scan_pt = { { scan.xs[j], scan.ys[j] } };
      if (isBetween(lines[i].getStart(), lines[i].getEnd(), scan_pt))
      {
        indices.push_back(j);
      }
    }
  }

  std::vector<double> output;
  for (int i = 0; i < scan.ranges.size(); ++i)
  {
    output.push_back(inf);
  }

  for (int i = 0; i < indices.size(); ++i)
  {
    output[indices[i]] = scan.ranges[indices[i]];
  }

  scan.ranges = output;
}

void LineExtractionROS::scanMaskCart(std::vector<double>& xs, std::vector<double>& ys, const std::vector<Line>& lines)
{
  std::vector<bool> indices(xs.size());
  for (int i = 0; i < indices.size(); ++i)
  {
    indices[i] = false;
  }

  double inf = std::numeric_limits<double>::infinity();
  for (int i = 0; i < lines.size(); ++i)
  {
    for (int j = 0; j < xs.size(); ++j)
    {
      boost::array<double, 2> scan_pt = { { xs[j], ys[j] } };
      if (isBetween(lines[i].getStart(), lines[i].getEnd(), scan_pt))
      {
        indices[j] = true;
      }
    }
  }

  std::vector<double> output_x(xs.size()), output_y(ys.size());
  for (int i = 0; i < xs.size(); ++i)
  {
    if (indices[i])
    {
      output_x[i] = xs[i];
      output_y[i] = ys[i];
    }
    else
    {
      output_x[i] = inf;
      output_y[i] = inf;
    }
  }

  xs = output_x;
  ys = output_y;
}

double LineExtractionROS::getProjectionScore(const std::vector<double>& scan_x, const std::vector<double>& scan_y,
                                             double tx, double ty, double yaw)
{
  sensor_msgs::PointCloud pc, scan, output1, best_output1;
  std::vector<geometry_msgs::Point32> p(scan_x.size());
  pc.points = p;
  pc.header.frame_id = p_map_frame_;
  for (int i = 0; i < scan_x.size(); ++i)
  {
    pc.points[i].x = scan_x[i];
    pc.points[i].y = scan_y[i];
  }
  sensor_msgs::PointCloud2 pc2, output;
  sensor_msgs::convertPointCloudToPointCloud2(pc, pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_pcl(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(pc2, *pc_pcl);
  pcl::PointCloud<pcl::PointXYZ> transformed_scan;

  bool state = listener_.waitForTransform(p_map_frame_, p_laser_frame_, scan_stamp, ros::Duration(0.5));
  tf::StampedTransform transform_laser_m;
  listener_.lookupTransform(p_map_frame_, p_laser_frame_, scan_stamp, transform_laser_m);
  std::vector<double> trans_x, trans_y;

  output1.header.stamp = ros::Time::now();
  output1.header.frame_id = p_map_frame_;
  sensor_msgs::PointCloud out;
  double x, y;
  double score, bestmatch, best_tx, best_ty, best_yaw;
  bestmatch = 0;

  tf::StampedTransform transform_laser = transform_laser_m;
  trans_x.clear();
  trans_y.clear();
  tf::StampedTransform rel_transform;
  tf::Vector3 origin = rel_transform.getOrigin();
  origin.setX(tx);
  origin.setY(ty);
  rel_transform.setOrigin(origin);
  tf::Quaternion quat = rel_transform.getRotation();
  quat.setRPY(0.0, 0.0, yaw);
  quat.normalize();
  rel_transform.setRotation(quat);

  tf::Transform fbsm_tf;
  fbsm_tf = transform_laser * rel_transform;

  pcl_ros::transformPointCloud(*pc_pcl, transformed_scan, fbsm_tf);
  pcl::toROSMsg(transformed_scan, output);
  sensor_msgs::convertPointCloud2ToPointCloud(output, out);

  for (int i = 0; i < out.points.size(); ++i)
  {
    if (not(std::isnan(out.points[i].x) or std::isnan(out.points[i].y)))
    {
      if ((out.points[i].x < inf and out.points[i].x > -inf) and (out.points[i].y < inf and out.points[i].y > -inf))
      {
        trans_x.push_back(out.points[i].x);
        trans_y.push_back(out.points[i].y);
      }
    }
  }
  score = feature_desc_.matchScore(map, trans_x, trans_y);
  return score;
}

std::vector<double> LineExtractionROS::getMatchScoreRaycast(const std::vector<double>& scan_x,
                                                            const std::vector<double>& scan_y, std::vector<double> tx_v,
                                                            std::vector<double> ty_v, std::vector<double> yaw)
{
  tf::StampedTransform transform_raycast, transform_laser_r;
  double scan_valid_pts = 0.0, ray_range;
  for (int i = 0; i < scan_x.size(); ++i)
  {
    if ((!std::isnan(scan_x[i])) and (!std::isnan(scan_y[i])))
    {
      ray_range = sqrt((scan_x[i] * scan_x[i]) + (scan_y[i] * scan_y[i]));
      if (ray_range > range_min and ray_range < max_range_cap)
      {
        scan_valid_pts++;
      }
    }
  }

  bestmatch = -1 * inf;

  if (p_multithreading_)
  {
    std::vector<std::thread> threads_p;

    for (int k = 0; k < yaw.size(); ++k)
    {
      threads_p.push_back(std::thread(getMatchScoreRaycastPerRotation, this, scan_x, scan_y, yaw[k]));
    }

    for (int i = 0; i < threads_p.size(); ++i)
    {
      if (threads_p[i].joinable())
      {
        threads_p[i].join();
      }
    }
  }
  else
  {
    for (int k = 0; k < yaw.size(); ++k)
    {
      getMatchScoreRaycastPerRotation(this, scan_x, scan_y, yaw[k]);
    }
  }

  std::vector<double> results;
  results.push_back(best_tx);
  results.push_back(best_ty);
  results.push_back(best_yaw);
  results.push_back(bestmatch);
  results.push_back(best_ir);
  results.push_back(best_id);

  return results;
}

void LineExtractionROS::getMatchScoreRaycastPerRotation(void* pthis, const std::vector<double>& scan_x,
                                                        const std::vector<double>& scan_y, double yaw)
{
  LineExtractionROS* pt = (LineExtractionROS*)pthis;

  tf::StampedTransform transform_laser_r;
  transform_laser_r.frame_id_ = pt->p_laser_frame_;

  tf::Quaternion quat = transform_laser_r.getRotation();
  quat.setRPY(0.0, 0.0, yaw);
  // quat.normalize();
  tf::Vector3 origin = transform_laser_r.getOrigin();
  origin.setX(0);
  origin.setY(0);
  transform_laser_r.setOrigin(origin);
  transform_laser_r.setRotation(quat);
  std::vector<double> map_x, map_y;
  pt->raycastWithRotation(transform_laser_r, map_x, map_y, false);

  // use raycasting mode to estimate translation
  std::list<Transform_vector> t_l;
  double tx, ty;
  if (pt->p_mode_ == "raycast")
  {
    for (int j = 0; j < scan_x.size(); j++)
    {
      if ((fabs(scan_x[j]) != pt->inf) and (fabs(map_x[j]) != pt->inf) and (fabs(scan_y[j]) != pt->inf) and
          (fabs(map_y[j]) != pt->inf))
      {
        tx = scan_x[j] - map_x[j];
        ty = scan_y[j] - map_y[j];
        if (fabs(tx) < pt->p_dist_upper_threshold_ and fabs(ty) < pt->p_dist_upper_threshold_)
        {
          Transform_vector l;
          l.tx = tx;
          l.ty = ty;
          t_l.push_back(l);
        }
      }
    }

    t_l.unique(is_near());
  }
  if (pt->p_multithreading_)
  {
    std::vector<std::thread> threads_p;
    for (std::list<Transform_vector>::iterator it = t_l.begin(); it != t_l.end(); ++it)
    {
      tx = it->tx;
      ty = it->ty;
      threads_p.push_back(std::thread(calculateTransformScore, pt, scan_x, scan_y, tx, ty, yaw));
    }

    for (int i = 0; i < threads_p.size(); ++i)
    {
      if (threads_p[i].joinable())
      {
        threads_p[i].join();
      }
    }
  }
  else
  {
    for (std::list<Transform_vector>::iterator it = t_l.begin(); it != t_l.end(); ++it)
    {
      tx = it->tx;
      ty = it->ty;
      calculateTransformScore(pt, scan_x, scan_y, tx, ty, yaw);
    }
  }
}

void LineExtractionROS::calculateTransformScore(void* pthis, const std::vector<double>& scan_x,
                                                const std::vector<double>& scan_y, double tx, double ty, double yaw)
{
  LineExtractionROS* pt = (LineExtractionROS*)pthis;
  tf::StampedTransform transform_raycast;

  transform_raycast.frame_id_ = pt->p_laser_frame_;
  tf::Vector3 origin = transform_raycast.getOrigin();
  origin.setX(tx);
  origin.setY(ty);
  transform_raycast.setOrigin(origin);
  tf::Quaternion quat = transform_raycast.getRotation();
  quat.setRPY(0.0, 0.0, yaw);
  transform_raycast.setRotation(quat);

  std::vector<double> map_x, map_y;
  pt->raycastWithRotation(transform_raycast, map_x, map_y, false);

  double valid_pts = 0.0;
  double score = 0.0;
  double inliers_ratio = 0.0, avg_inliers_dist = 0.0;

  for (int i = 0; i < scan_x.size(); ++i)
  {
    if ((!std::isnan(scan_x[i])) and (!std::isnan(scan_y[i])) and (!std::isnan(map_y[i])) and (!std::isnan(map_x[i])))
    {
      if ((fabs(scan_x[i]) != pt->inf) and (fabs(map_x[i]) != pt->inf) and (fabs(scan_y[i]) != pt->inf) and
          (fabs(map_y[i]) != pt->inf))
      {
        valid_pts++;
        double r = sqrt(pow(scan_x[i] - map_x[i], 2) + pow(scan_y[i] - map_y[i], 2));
        if (r < pt->p_inlier_dist_)
        {
          score += r;
        }
        if (r < pt->p_inlier_dist_)
        {
          inliers_ratio++;
          avg_inliers_dist += (pt->p_inlier_dist_ - r);
        }
      }
    }
  }
  // rotational_error = rotational_error / valid_pts;
  if (inliers_ratio != 0)
  {
    avg_inliers_dist = avg_inliers_dist / inliers_ratio;
  }
  inliers_ratio = inliers_ratio / valid_pts;
  score = pow(inliers_ratio, pt->p_inliers_exp_) * avg_inliers_dist;

  std::lock_guard<std::mutex> lockGuard(pt->mutex);

  if ((score > pt->bestmatch))
  {
    pt->bestmatch = score;
    pt->best_ir = inliers_ratio;
    pt->best_id = avg_inliers_dist;
    pt->best_tx = tx;
    pt->best_ty = ty;
    pt->best_yaw = yaw;
  }
}

void LineExtractionROS::goalCallback(const geometry_msgs::PoseStamped& goal)
{
  goal_ = goal;
  stop_count = 0;
}

void LineExtractionROS::reachedCallback(const geometry_msgs::PoseStamped& goal)
{
  goal_.pose.position.x = inf;
  goal_.pose.position.y = inf;
}

void LineExtractionROS::publishInitialPose(double tx, double ty, double yaw)
{
  tf::StampedTransform transform_amcl_up, transform_up, transform_odom_up, transform_laser_base_l;
  tf::Transform rel_amcl, rel_motion, rel;
  tf::StampedTransform tx_odom;
  ros::Time now = ros::Time::now();

  try
  {
    bool state = listener_.waitForTransform(p_map_frame_, p_base_frame_, now, ros::Duration(0.5));
    listener_.lookupTransform(p_map_frame_, p_base_frame_, now, transform_up);
    rel = transform_up.inverseTimes(transform);

    if (transform_laser_base.stamp_.toSec() == 0)
    {
      ROS_DEBUG("Initialising base laser transform");
      state = listener_.waitForTransform(p_base_frame_, p_laser_frame_, now, ros::Duration(0.5));
      listener_.lookupTransform(p_base_frame_, p_laser_frame_, now, transform_laser_base_l);
      transform_laser_base = transform_laser_base_l;
    }
    state = listener_.waitForTransform(p_map_frame_, p_odom_frame_, now, ros::Duration(0.5));
    listener_.lookupTransform(p_map_frame_, p_odom_frame_, now, transform_amcl_up);
    rel_amcl = transform_amcl.inverseTimes(transform_amcl_up);
  }
  catch (tf::TransformException& e)
  {
    ROS_WARN("Failed to get transform now.");
    ROS_WARN("%s", e.what());
    return;
  }

  geometry_msgs::PoseWithCovarianceStamped pose;

  tf::Transform fbsm_tf;
  tf::Vector3 origin = fbsm_tf.getOrigin();
  origin.setX(tx);
  origin.setY(ty);
  tf::Quaternion fbsm_quat;
  fbsm_quat.setRPY(0.0, 0.0, yaw);
  fbsm_tf.setRotation(fbsm_quat);
  fbsm_tf.setOrigin(origin);

  // Ignore updates while the robot's heading is changing a lot (final spot turning)
  if ((fabs(tf::getYaw(rel.getRotation())) > 0.01))
  {
    ROS_WARN("Spot turning.");
    return;
  }

  // if ((fabs(rel.getOrigin().x()) < 0.01) and (fabs(rel.getOrigin().y()) < 0.01)) {
  //  ROS_WARN("Robot not moving.");
  //  return;
  //}
  tf::Transform final_tf =
      transform_up * transform_laser_base * fbsm_tf.inverse() * transform_laser_base.inverse() * rel_amcl.inverse();

  pose.header.frame_id = p_map_frame_;
  pose.header.stamp = now;
  pose.pose.pose.position.x = final_tf.getOrigin().x();
  pose.pose.pose.position.y = final_tf.getOrigin().y();
  tf::quaternionTFToMsg(final_tf.getRotation(), pose.pose.pose.orientation);

  if (std::isnan(pose.pose.pose.position.x) or std::isnan(pose.pose.pose.position.y) or
      std::isnan(tf::getYaw(final_tf.getRotation())))
  {
    // error calculating transform, abort
    ROS_WARN("NAN final transform");
    return;
  }
  ini_pos_publisher_.publish(pose);
  ROS_INFO("Reintialisation: x= %f y= %f yaw= %f", pose.pose.pose.position.x, pose.pose.pose.position.y,
           tf::getYaw(final_tf.getRotation()));

  // giving time for the pose reintialisation to take place
  ros::Duration(0.2).sleep();
}

}  // namespace line_extraction
