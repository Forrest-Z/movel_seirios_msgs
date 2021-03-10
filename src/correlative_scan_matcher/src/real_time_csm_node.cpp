#include <correlative_scan_matcher/real_time_csm_node.h>
#include <correlative_scan_matcher/range_routines.h>
#include <movel_hasp_vendor/license.h>

namespace correlative_scan_matcher
{
RealTimeCSM::RealTimeCSM() : nh_private_("~"),
                             got_map_(false),
                             got_laser_pose_(false),
                             map_(NULL),
                             low_res_table_(NULL),
                             high_res_table_(NULL),
                             laser_data_(NULL),
                             last_tf_valid_(false),
                             enabled_(false),
                             paused_(false),
                             have_first_raycast_(false)
{
  loadParams();
  setupTopics();
};

void RealTimeCSM::loadParams()
{
  nh_private_.param("sigma", sigma_, 0.05);
  ROS_INFO("sigma: %f", sigma_);

  nh_private_.param("search_x", search_x_, 0.1);
  ROS_INFO("search_x: %f", search_x_);

  nh_private_.param("search_y", search_y_, 0.1);
  ROS_INFO("search_y: %f", search_y_);

  nh_private_.param("search_a", search_a_, 0.35);
  ROS_INFO("search_a: %f", search_a_);

  nh_private_.param("min_score", min_score_, 0.5);
  ROS_INFO("min_score: %f", min_score_);

  nh_private_.param("high_grid_resolution", high_grid_resolution_, 0.03);
  ROS_INFO("high_grid_resolution: %f", high_grid_resolution_);

  nh_private_.param("enable_raycast", enable_raycast_, false);
  ROS_INFO("enable_raycast: %s", enable_raycast_ ? "true" : "false");

  nh_private_.param("raycast_lin_thres", raycast_lin_thres_, 0.5);
  ROS_INFO("raycast_lin_thres: %f", raycast_lin_thres_);

  nh_private_.param("raycast_ang_thres", raycast_ang_thres_, 1.57);
  ROS_INFO("raycast_ang_thres: %f", raycast_ang_thres_);

  nh_private_.param<std::string>("base_frame", base_frame_, "base_link");
  ROS_INFO("base_frame: %s", base_frame_.c_str());

  nh_private_.param<std::string>("reference_frame", reference_frame_, "map");
  ROS_INFO("reference_frame: %s", reference_frame_.c_str());

  nh_private_.param<std::string>("matched_frame", matched_frame_, "csm/base_link");
  ROS_INFO("matched_frame: %s", matched_frame_.c_str());

  /* nh_private_.param("show_corrected_scan", show_corrected_scan_, false);
   * ROS_INFO("show_corrected_scan: %s", show_corrected_scan_ ? "true" : "false"); */

  nh_private_.param("publish_initial_pose", publish_initial_pose_, false);
  ROS_INFO("publish_initial_pose: %s", publish_initial_pose_ ? "true" : "false");

  nh_private_.param("do_global_search", do_global_search_, false);
  ROS_INFO("do_global_search: %s", do_global_search_ ? "true" : "false");

  nh_private_.param("use_map_topic", use_map_topic_, true);
  ROS_INFO("use_map_topic: %s", use_map_topic_ ? "true" : "false");

  nh_private_.param("first_map_only", first_map_only_, true);
  ROS_INFO("first_map_only: %s", first_map_only_ ? "true" : "false");

  nh_private_.param("publish_last_valid_tf", publish_last_valid_tf_, false);
  ROS_INFO("publish_last_valid_tf: %s", publish_last_valid_tf_ ? "true" : "false");

  nh_private_.param("publish_tables", publish_tables_, false);
  ROS_INFO("publish_tables: %s", publish_tables_ ? "true" : "false");

  nh_private_.param("wait_for_transform", wait_for_transform_, 0.05);
  ROS_INFO("wait_for_transform: %f", wait_for_transform_);

  nh_private_.param("invert_transform", invert_transform_, false);
  ROS_INFO("invert_transform: %s", invert_transform_ ? "true" : "false");

  nh_private_.param("filter_window_size", filter_window_size_, 1);
  ROS_INFO("filter_window_size: %i", filter_window_size_);

  nh_private_.param("use_last_match", use_last_match_, false);
  ROS_INFO("use_last_match: %s", use_last_match_ ? "true" : "false");

  nh_private_.param("min_valid_points", min_valid_points_, 100);
  ROS_INFO("min_valid_points: %i", min_valid_points_);
}

void RealTimeCSM::setupTopics()
{
  if (use_map_topic_)
    map_sub_ = nh_private_.subscribe("map", 1, &RealTimeCSM::mapCallback, this); // ~map
  else
    set_map_srv_ = nh_.advertiseService("set_csm_map", &RealTimeCSM::onSetMapSrv, this); // /set_csm_map

  /* if (show_corrected_scan_)
   *   corrected_cloud_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("corrected_cloud", 1); */

  if (publish_initial_pose_)
    initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1); // /initialpose

  if (enable_raycast_)
    raycast_pub_ = nh_private_.advertise<sensor_msgs::LaserScan>("raycast_output", 1); // ~raycast_output

  if (publish_tables_)
  {
    low_res_grid_pub_ = nh_private_.advertise<nav_msgs::OccupancyGrid>("low_res_grid", 1, true); // ~low_res_grid
    high_res_grid_pub_ = nh_private_.advertise<nav_msgs::OccupancyGrid>("high_res_grid", 1, true); // ~high_res_grid
  }

  enable_srv_ = nh_.advertiseService("enable_csm", &RealTimeCSM::onEnable, this); // /enable_csm
  pause_srv_ = nh_.advertiseService("pause_csm", &RealTimeCSM::onPause, this); // /pause_csm
  scan_sub_ = nh_private_.subscribe("scan", 1, &RealTimeCSM::scanCallback, this); // ~scan
}

void RealTimeCSM::scanCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
  // only continues if the grid is not still being processed in RealTimeCSM::processMap
  if (!grid_mutex_.try_lock_for(std::chrono::milliseconds(500)))
    return;
  std::lock_guard<std::timed_mutex> grid_lock(grid_mutex_, std::adopt_lock_t());

  if (!got_map_ || !enabled_)
    return;

  // use the incoming time of the last ray i.e. the entire scan
  ros::Time scan_in_time = msg->header.stamp + ros::Duration().fromSec(msg->ranges.size() * msg->time_increment);

  if (paused_)
  {
    if (publish_last_valid_tf_ && last_tf_valid_)
    {
      tf_broadcaster_.sendTransform(tf::StampedTransform(last_reference_matched_tf_, scan_in_time, reference_frame_, matched_frame_));
    }
    return;
  }

  if (laser_data_ != NULL)
  {
    delete laser_data_;
    laser_data_ = NULL;
  }

  // assume laser position is constant wrt base
  if (!got_laser_pose_)
  {
    tf::StampedTransform base_laser_stamped;
    try
    {
      if (!tf_listener_.waitForTransform(base_frame_, msg->header.frame_id, ros::Time(0), ros::Duration(wait_for_transform_)))
      {
        ROS_WARN("No recent transform from %s to %s", base_frame_.c_str(), msg->header.frame_id.c_str());
        return;
      }
      tf_listener_.lookupTransform(base_frame_, msg->header.frame_id, ros::Time(0), base_laser_stamped);
    }
    catch (tf::TransformException &e)
    {
      ROS_WARN("Unable to get transform from base to laser frame: %s", e.what());
      return;
    }
    // TODO: 3D?
    // TODO: inverted lidar?
    laser_pose_ = Pose(base_laser_stamped.getOrigin().getX(), base_laser_stamped.getOrigin().getY(), tf::getYaw(base_laser_stamped.getRotation()));
    ROS_INFO("Laser position: (%f, %f, %f)", laser_pose_.x, laser_pose_.y, laser_pose_.theta);
    got_laser_pose_ = true;
  }

  // get min, max and increment of the angle in the base frame
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, msg->angle_min);
  tf::Stamped<tf::Quaternion> min_q(q, msg->header.stamp, msg->header.frame_id);
  q.setRPY(0.0, 0.0, msg->angle_min + msg->angle_increment);
  tf::Stamped<tf::Quaternion> inc_q(q, msg->header.stamp, msg->header.frame_id);

  try
  {
    if (!tf_listener_.waitForTransform(msg->header.frame_id, base_frame_, scan_in_time, ros::Duration(wait_for_transform_)))
    {
      ROS_WARN("No transform from %s to %s at scan in time", msg->header.frame_id.c_str(), base_frame_.c_str());
      return;
    }
    tf_listener_.transformQuaternion(base_frame_, min_q, min_q);
    tf_listener_.transformQuaternion(base_frame_, inc_q, inc_q);
  }
  catch (tf::TransformException &e)
  {
    ROS_WARN("Unable to transform min/max laser angles into base frame: %s", e.what());
    return;
  }

  laser_data_ = new LaserData;
  laser_data_->range_count = msg->ranges.size();
  laser_data_->valid_range_count= 0;
  laser_data_->range_max = msg->range_max;
  laser_data_->range_min = msg->range_min;

  double angle_min = tf::getYaw(min_q);
  double angle_increment = tf::getYaw(inc_q) - angle_min;

  angle_increment = fmod(angle_increment + 5 * M_PI, 2 * M_PI) - M_PI;
  search_a_step_ = angle_increment;

  laser_data_->ranges = new double[laser_data_->range_count][2];
  for (int i = 0; i < laser_data_->range_count; i++)
  {
    double ray_length = msg->ranges[i];
    laser_data_->ranges[i][0] = ray_length;
    laser_data_->ranges[i][1] = angle_min + (i * angle_increment);

    if (ray_length < laser_data_->range_max && ray_length >= laser_data_->range_min
        && std::isfinite(ray_length))
      laser_data_->valid_range_count++;;
  }

  if (laser_data_->valid_range_count < min_valid_points_)
  {
    ROS_WARN("Not enough valid points (%i)", laser_data_->valid_range_count);
    if (publish_last_valid_tf_ && last_tf_valid_)
    {
      tf_broadcaster_.sendTransform(tf::StampedTransform(last_reference_matched_tf_, scan_in_time, reference_frame_, matched_frame_));
    }
    return;
  }

  // this is our initial pose estimate (map <- base)
  tf::Transform map_base_tf;
  // if doing a global search, the initial pose estimate does not matter
  if (do_global_search_)
    map_base_tf.setIdentity();
  // reusing the previous corrected estimate
  else if (use_last_match_ && last_tf_valid_)
  {
    // matched = corrected map
    if (invert_transform_)
    {
      // reference <- base
      tf::StampedTransform reference_base_stamped;
      try
      {
        if (!tf_listener_.waitForTransform(reference_frame_, base_frame_, scan_in_time, ros::Duration(wait_for_transform_)))
        {
          ROS_WARN("No transform from %s to %s at scan in time", reference_frame_.c_str(), base_frame_.c_str());
          return;
        }
        tf_listener_.lookupTransform(reference_frame_, base_frame_, scan_in_time, reference_base_stamped);
      }
      catch (tf::TransformException &e)
      {
        ROS_WARN("Unable to get transform from reference to base frame: %s", e.what());
        return;
      }
      // corrected map <- base = corrected map <- reference * reference <- base
      map_base_tf = last_reference_matched_tf_.inverse() * tf::Transform(reference_base_stamped);
    }
    // matched = corrected base
    else
    {
      // reference <- map
      tf::StampedTransform reference_map_stamped;
      try
      {
        if (!tf_listener_.waitForTransform(reference_frame_, map_frame_, scan_in_time, ros::Duration(wait_for_transform_)))
        {
          ROS_WARN("No transform from %s to %s at scan in time", reference_frame_.c_str(), map_frame_.c_str());
          return;
        }
        tf_listener_.lookupTransform(reference_frame_, map_frame_, scan_in_time, reference_map_stamped);
      }
      catch (tf::TransformException &e)
      {
        ROS_WARN("Unable to get transform from reference to map frame: %s", e.what());
        return;
      }
      // map <- corrected base = map <- reference * reference <- corrected base
      map_base_tf = tf::Transform(reference_map_stamped).inverse() * last_reference_matched_tf_;
    }
  }
  // get map <- base at scan in time
  else
  {
    tf::StampedTransform map_base_stamped;
    try
    {
      if (!tf_listener_.waitForTransform(map_frame_, base_frame_, scan_in_time, ros::Duration(wait_for_transform_)))
      {
        ROS_WARN("No transform from %s to %s at scan in time", map_frame_.c_str(), base_frame_.c_str());
        return;
      }
      tf_listener_.lookupTransform(map_frame_, base_frame_, scan_in_time, map_base_stamped);
    }
    catch (tf::TransformException &e)
    {
      ROS_WARN("Unable to get transform from map to base at scan in time: %s", e.what());
      return;
    }

    map_base_tf = tf::Transform(map_base_stamped);
  }

  // input current position into queue of poses to search
  std::priority_queue<Pose> pose_q;
  Pose current(map_base_tf.getOrigin().getX(), map_base_tf.getOrigin().getY(), tf::getYaw(map_base_tf.getRotation()));
  ROS_INFO("Current pose: (%f, %f, %f)", current.x, current.y, current.theta);
  pose_q.push(current);

  ros::Time t;
  ros::Time tc;
  t = ros::Time::now();

  // if raycasting is enabled
  if (!do_global_search_ && enable_raycast_)
  {
    tc = ros::Time::now();
    // if (low_res_table_ != NULL)
    // {
    //   delete low_res_table_;
    //   low_res_table_ = NULL;
    // }

    // if (high_res_table_ != NULL)
    // {
    //   delete high_res_table_;
    //   high_res_table_ = NULL;
    // }

    // if the robot has moved enough since the last raycasted pose
    if (!have_first_raycast_
        || getLinearDist(last_raycast_pose_, current) >= raycast_lin_thres_
        || getAngularDist(last_raycast_pose_, current) >= raycast_ang_thres_)
    {
      resetCells(low_res_table_);
      resetCells(high_res_table_);
      ROS_INFO("Grid reset took %f seconds", (ros::Time::now() - tc).toSec());

      // raycast
      tc = ros::Time::now();
      sensor_msgs::LaserScan raycast_output;
      raycast_output = *msg;
      raycast_output.ranges.clear();
      raycast_output.intensities.clear();

      Pose laser = current;
      ROS_DEBUG("Base in map frame: (%f, %f, %f)", current.x, current.y, current.theta);
      /* laser.x += laser_pose_.x * cos(current.theta) + laser_pose_.y * cos(M_PI - current.theta);
       * laser.y += laser_pose_.x * sin(current.theta) + laser_pose_.y * sin(M_PI - current.theta); */
      laser.x += laser_pose_.x * cos(current.theta) - laser_pose_.y * sin(current.theta);
      laser.y += laser_pose_.x * sin(current.theta) + laser_pose_.y * cos(current.theta);
      laser.theta += laser_pose_.theta;
      ROS_DEBUG("Laser in map frame: (%f, %f, %f)", laser.x, laser.y, laser.theta);
      // first add the raycast ranges into the high resolution grid
      for (int i = 0; i < laser_data_->range_count; i++)
      {
        double theta = laser.theta + laser_data_->ranges[i][1];
        double range = range_routines::bresenham(map_, laser.x, laser.y, theta, msg->range_max);
        // low_res_table_->addRay(current.x, current.y, theta, range);
        high_res_table_->addRay(laser.x, laser.y, theta, range);

        raycast_output.ranges.push_back(range);
      }
      raycast_pub_.publish(raycast_output);
      ROS_INFO("Raycast took %f seconds", (ros::Time::now() - tc).toSec());

      tc = ros::Time::now();
      int oi = MAP_GXWX(high_res_table_, laser.x);
      int oj = MAP_GYWY(high_res_table_, laser.y);
      int update_size = msg->range_max / high_res_table_->resolution;

      // overlap the high res grid with the low res grid
      for (int i = oi - (update_size / 2); i < oi + (update_size / 2); i++)
      {
        for (int j = oj - (update_size / 2); j < oj + (update_size / 2); j++)
        {
          overlapGrid(high_res_table_, i, j, low_res_table_);
        }
      }

      ROS_DEBUG("High res update region: %i, %i, %i", oi, oj, update_size);
      high_res_table_->updateGrid(sigma_, oi, oj, update_size, update_size);

      oi = MAP_GXWX(low_res_table_, laser.x);
      oj = MAP_GYWY(low_res_table_, laser.y);
      update_size = msg->range_max / low_res_table_->resolution;
      ROS_DEBUG("Low res update region: %i, %i, %i", oi, oj, update_size);
      low_res_table_->updateGrid(sigma_, oi, oj, update_size, update_size);
      ROS_INFO("Probability grid generation took %f seconds", (ros::Time::now() - tc).toSec());

      if (publish_tables_)
      {
        low_res_grid_pub_.publish(getOccupancyGrid(low_res_table_));
        high_res_grid_pub_.publish(getOccupancyGrid(high_res_table_));
      }

      have_first_raycast_ = true;
      last_raycast_pose_ = current;
    }
  }

  // whether to ignore unknown and occupied cells as candidates
  bool ignore_unknown;
  bool ignore_obstacles;

  // low res search
  tc = ros::Time::now();
  /**
   * For global search we ignore unknown cells to reduce the search space.
   * However if raycast is enabled, all cells except the raycasted ranges will be unknown, hence
   * the correct pose may be in an unknown space.
   * @todo may need to refine the criteria for ignoring unknown space
   */
  ignore_unknown = (do_global_search_ || !enable_raycast_) ? true : false;
  /**
   * The low resolution grid takes the maximum of all the high resolution cells it overlaps with,
   * therefore even if a low res cell is occupied there may still be free spaces within the cell.
   * We cannot ignore occupied cells as candidates in the low res search stage.
   */
  ignore_obstacles = false;
  ROS_INFO("ignore_unknown: %s, ignore_obstacles: %s", ignore_unknown ? "true" : "false", ignore_obstacles ? "true" : "false");
  pose_q = match(pose_q, low_res_table_, search_x_, search_y_, search_a_, ignore_unknown, ignore_obstacles);
  ROS_INFO("Found %lu low res results in %f seconds", pose_q.size(), (ros::Time::now() - tc).toSec());

  // high res search
  // @todo rather than putting all the low res results into the high res search, just take the best one
  tc = ros::Time::now();
  ignore_unknown = (do_global_search_ || !enable_raycast_) ? true : false;
  // final candidates cannot be on obstacle cells
  ignore_obstacles = true;
  ROS_INFO("ignore_unknown: %s, ignore_obstacles: %s", ignore_unknown ? "true" : "false", ignore_obstacles ? "true" : "false");
  pose_q = match(pose_q, high_res_table_, low_res_table_->resolution, low_res_table_->resolution, search_a_step_, ignore_unknown, ignore_obstacles);
  ROS_INFO("Found %lu high res results in %f seconds", pose_q.size(), (ros::Time::now() - tc).toSec());

  ROS_INFO("Took %f seconds overall for a %f m x %f m x %f rad search in %f/%f m and %f rad resolution", ros::Time::now().toSec() - t.toSec(), search_x_, search_y_, search_a_, high_res_table_->resolution, low_res_table_->resolution, search_a_step_);

  Pose best;
  if (!pose_q.empty())
  {
    best = pose_q.top();
    /* if (filter_window_size_ > 1)
     * {
     *   addToWindow(pose_q.top());
     *   if (!getFilteredResult(best))
     *   {
     *     ROS_INFO("Waiting for initial filter population.");
     *     return;
     *   }
     * }
     * else
     *   best = pose_q.top(); */
  }
  else
  {
    // best = current;
    ROS_ERROR("No sufficient match found");
    if (publish_last_valid_tf_ && last_tf_valid_)
    {
      tf_broadcaster_.sendTransform(tf::StampedTransform(last_reference_matched_tf_, scan_in_time, reference_frame_, matched_frame_));
    }
    return;
  }

  ROS_INFO("Best pose: (%f, %f, %f) with score: %f", best.x, best.y, best.theta, best.score);

  tf::Transform best_tf;
  best_tf.setOrigin(tf::Vector3(best.x, best.y, 0.0));
  q.setRPY(0, 0, best.theta);
  best_tf.setRotation(q);

  // reference <- matched
  tf::Transform reference_matched_tf;
  // matched = corrected map
  if (invert_transform_)
  {
    // reference <- base
    tf::StampedTransform reference_base_stamped;
    try
    {
      // TODO: (sharmin) this tf was already retrieved earlier, so this is repetitive
      if (!tf_listener_.waitForTransform(reference_frame_, base_frame_, scan_in_time, ros::Duration(wait_for_transform_)))
      {
        ROS_WARN("No transform from %s to %s at scan in time", reference_frame_.c_str(), base_frame_.c_str());
        return;
      }
      tf_listener_.lookupTransform(reference_frame_, base_frame_, scan_in_time, reference_base_stamped);
    }
    catch (tf::TransformException &e)
    {
      ROS_WARN("Unable to get transform from reference to base frame: %s", e.what());
      return;
    }
    // reference <- corrected map = reference <- base * base <- corrected map
    reference_matched_tf = tf::Transform(reference_base_stamped) * best_tf.inverse();
  }
  // matched = corrected base
  else
  {
    // reference <- map
    tf::StampedTransform reference_map_stamped;
    try
    {
      // TODO: (sharmin) this tf was already retrieved earlier, so this is repetitive
      if (!tf_listener_.waitForTransform(reference_frame_, map_frame_, scan_in_time, ros::Duration(wait_for_transform_)))
      {
        ROS_WARN("No transform from %s to %s at scan in time", reference_frame_.c_str(), map_frame_.c_str());
        return;
      }
      tf_listener_.lookupTransform(reference_frame_, map_frame_, scan_in_time, reference_map_stamped);
    }
    catch (tf::TransformException &e)
    {
      ROS_WARN("Unable to get transform from reference to map frame: %s", e.what());
      return;
    }
    // reference <- corrected base = reference <- map * map <- corrected base
    reference_matched_tf = tf::Transform(reference_map_stamped) * best_tf;
  }

  if (filter_window_size_ > 1)
  {
    addToWindow(tf2Pose(reference_matched_tf));
    Pose filtered;
    if (!getFilteredResult(filtered))
    {
      // wait for the filter to be fully populated
      ROS_INFO("Waiting for initial filter population.");
      return;
    }
    reference_matched_tf = pose2TF(filtered);
  }

  tf_broadcaster_.sendTransform(tf::StampedTransform(reference_matched_tf, scan_in_time, reference_frame_, matched_frame_));
  last_reference_matched_tf_ = reference_matched_tf;
  last_tf_valid_ = true;

/*   if (show_corrected_scan_)
 *   {
 *     // is there a less complicated way to do this lol
 *     sensor_msgs::PointCloud2 scan_cloud;
 *     projector_.projectLaser(*msg, scan_cloud);
 *     pcl::PointCloud<pcl::PointXYZ> scan_pcl;
 *     pcl::fromROSMsg(scan_cloud, scan_pcl);
 *     pcl::PointCloud<pcl::PointXYZ> scan_in_map_pcl;
 *     pcl_ros::transformPointCloud(map_frame_, scan_in_time, scan_pcl, map_frame_, scan_in_map_pcl, tf_listener_);
 *     scan_in_map_pcl.header.frame_id = map_frame_; // shouldn't need this
 *     pcl::PointCloud<pcl::PointXYZ> corrected_scan_pcl;
 *     //pcl_ros::transformPointCloud(scan_in_map_pcl, corrected_scan_pcl, map_base_stamped.inverseTimes(best_tf));
 *     pcl_ros::transformPointCloud(scan_in_map_pcl, corrected_scan_pcl, ((map_base_stamped).inverse()).inverseTimes(best_tf.inverse()).inverse());
 *     sensor_msgs::PointCloud2 corrected_cloud;
 *     pcl::toROSMsg(corrected_scan_pcl, corrected_cloud);
 *
 *     corrected_cloud.header.stamp = scan_in_time;
 *     corrected_cloud.header.frame_id = map_frame_;
 *     corrected_cloud_pub_.publish(corrected_cloud);
 *   } */

  if (publish_initial_pose_)
  {
    geometry_msgs::PoseWithCovarianceStamped initialpose;
    initialpose.header.frame_id = map_frame_;
    initialpose.header.stamp = scan_in_time;
    initialpose.pose.pose.position.x = best_tf.getOrigin().getX();
    initialpose.pose.pose.position.y = best_tf.getOrigin().getY();
    initialpose.pose.pose.position.z = best_tf.getOrigin().getZ();
    geometry_msgs::Quaternion q;
    tf::quaternionTFToMsg(best_tf.getRotation(), q);
    initialpose.pose.pose.orientation = q;
    // TODO: covariance calculation
    initialpose.pose.covariance[0] = 0.05;
    initialpose.pose.covariance[7] = 0.05;
    initialpose.pose.covariance[35] = 0.1;
    initial_pose_pub_.publish(initialpose);
  }

  // TODO: add relative motion during search to final pose
}

bool RealTimeCSM::onSetMapSrv(correlative_scan_matcher::SetMap::Request &req, correlative_scan_matcher::SetMap::Response &resp)
{
  if (use_map_topic_)
  {
    ROS_ERROR("use_map_topic is true but the set map service was called. Ignoring incoming map.");
    resp.success = false;
  }
  else
  {
    processMap(req.map);
    got_map_ = true;
    reset();
    resp.success = true;
  }

  return true;
}

bool RealTimeCSM::onEnable(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
  enabled_ = req.data;
  reset();

  resp.success = true;

  return true;
}

bool RealTimeCSM::onPause(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
  paused_ = req.data;

  ROS_INFO("Paused: %s", paused_ ? "true" : "false");

  resp.success = true;

  return true;
}

void RealTimeCSM::mapCallback(const nav_msgs::OccupancyGridConstPtr &msg)
{
  if (!use_map_topic_)
    return;

  if (got_map_ && first_map_only_)
    return;

  processMap(*msg);

  got_map_ = true;
  reset();
}

void RealTimeCSM::processMap(const nav_msgs::OccupancyGrid &msg)
{
  // ensure that no one else is modifying or accessing the grids
  std::lock_guard<std::timed_mutex> grid_lock(grid_mutex_);

  ROS_INFO("Received map with width %d, height %d and resolution %f",
           msg.info.width, msg.info.height, msg.info.resolution);

  // get the map_frame_ from the map message
  map_frame_ = msg.header.frame_id;

  if (map_ != NULL)
    delete map_;
  map_ = new Map();
  map_->resolution = msg.info.resolution;
  map_->size_x = msg.info.width;
  map_->size_y = msg.info.height;
  map_->origin_x = msg.info.origin.position.x + (map_->size_x / 2) * map_->resolution;
  map_->origin_y = msg.info.origin.position.y + (map_->size_y / 2) * map_->resolution;
  map_->cells.resize(map_->size_x * map_->size_y);

  // if doing a global search, expand the search region to fill the map
  if (do_global_search_)
  {
    search_x_ = map_->size_x;
    search_y_ = map_->size_y;
    search_a_ = 2 * M_PI;
  }

  for (int i = 0; i < map_->size_x * map_->size_y; i++)
  {
    if (msg.data[i] == 0) // free
      map_->cells[i].state = 0;
    if (msg.data[i] == 100) // occupied
      map_->cells[i].state = 1;
    if (msg.data[i] == -1) // unknown
      map_->cells[i].state = -1;
  }

  if (low_res_table_ != NULL)
  {
    delete low_res_table_;
    low_res_table_ = NULL;
  }

  if (high_res_table_ != NULL)
  {
    delete high_res_table_;
    high_res_table_ = NULL;
  }

  // no raycast
  if (do_global_search_ || !enable_raycast_)
  {
    /*
    // Olson recommends 30cm and 3cm respectively
    // Choosing 25/2.5 because it directly scales with our 5cm resolution map
    low_res_table_ = createGridFromMap(map, 0.25); // 25cm
    high_res_table_ = createGridFromMap(map, 0.025); // 2.5cm
    */
    // Having a resolution lower than our map resolution doesn't actually help
    // Only helps if the reference is a scan rather than a map
    // Hence we'll use the map resolution for the high res and 10x for the low res
    low_res_table_ = createGridFromMap(map_, map_->resolution * 10);
    high_res_table_ = createGridFromMap(map_, map_->resolution);

    // overlap the low and high res grids
    for (int i = 0; i < map_->size_x; i++)
    {
      for (int j = 0; j < map_->size_y; j++)
      {
        overlapGrid(map_, i, j, low_res_table_);
        overlapGrid(map_, i, j, high_res_table_);
        /*
        if (msg->data[MAP_INDEX(map, i, j)] == 100)
        {
          overlapObstacles(map, i, j, low_res_table_);
          overlapObstacles(map, i, j, high_res_table_);
        }
        */
      }
    }

    // update the probabilities in the probability grids
    low_res_table_->updateGrid(sigma_, low_res_table_->size_x / 2, low_res_table_->size_y / 2, low_res_table_->size_x, low_res_table_->size_y);
    high_res_table_->updateGrid(sigma_, high_res_table_->size_x / 2, high_res_table_->size_y / 2, high_res_table_->size_x, high_res_table_->size_y);

    if (publish_tables_)
    {
      low_res_grid_pub_.publish(getOccupancyGrid(low_res_table_));
      high_res_grid_pub_.publish(getOccupancyGrid(high_res_table_));
    }
  }
  // performing raycast later, so initialize empty grids for now with the given resolutions
  else
  {
    low_res_table_ = createGridFromMap(map_, high_grid_resolution_ * 10);
    high_res_table_ = createGridFromMap(map_, high_grid_resolution_);
  }
}

/**
 * Gets the nav_msgs::OccupancyGrid representation of a ProbabilityGrid
 *
 * @param grid input probability grid
 * @result The nav_msgs::OccupancyGrid representation of the input
 */
nav_msgs::OccupancyGrid RealTimeCSM::getOccupancyGrid(ProbabilityGrid *grid)
{
  nav_msgs::OccupancyGrid occ_grid;
  occ_grid.header.frame_id = map_frame_;
  occ_grid.header.stamp = ros::Time::now();
  occ_grid.info.width = grid->size_x;
  occ_grid.info.height = grid->size_y;
  occ_grid.info.resolution = grid->resolution;
  occ_grid.info.origin.position.x = grid->origin_x - (grid->size_x / 2) * grid->resolution;
  occ_grid.info.origin.position.y = grid->origin_y - (grid->size_y / 2) * grid->resolution;

  // uncomment this block and comment the next block
  // to publish the probabilities of each cell
  for (int i = 0; i < grid->size_x * grid->size_y; i++)
  {
    // TODO: scale probability to 0-255 for visualization
    // assumes probability is in the range 0-255
    // occ_grid.data.push_back(grid->cells[i].probability);
  }

  // uncomment this block and comment the previous block
  // to publish the states of each cell
  // /*
  for (int i = 0; i < grid->size_x * grid->size_y; i++)
  {
    if (grid->cells[i].state == 0)
      occ_grid.data.push_back(0);
    if (grid->cells[i].state == -1)
      occ_grid.data.push_back(-1);
    if (grid->cells[i].state == 1)
      occ_grid.data.push_back(100);
  }
  // */
  return occ_grid;
}

/**
 * Overlaps one map above another.
 * Each cell in the top grid takes the maximum of the cells
 * it overlaps with in the bottom grid.
 *
 * @param a the bottom grid
 * @param i column index of the cell in grid a to overlap
 * @param j row index of the cell in grid a to overlap
 * @param b the top grid
 *
 * @todo the current implementation iterates through the entire grid,
 * which is really slow. please make it faster.
 */
void RealTimeCSM::overlapGrid(Map *a, int i, int j, Map *b)
{
  // get the region covered by the overlapped cell in map a
  double ox = MAP_WXGX(a, i);
  double oy = MAP_WYGY(a, j);
  double ex = MAP_WXGX(a, i) + a->resolution;
  double ey = MAP_WYGY(a, j) + a->resolution;

  // get the corresponding region in map b
  int b_oi = floor((ox - b->origin_x) / b->resolution) + b->size_x / 2;
  int b_oj = floor((oy - b->origin_y) / b->resolution) + b->size_y / 2;
  int b_ei = floor((ex - b->origin_x) / b->resolution) + b->size_x / 2;
  int b_ej = floor((ey - b->origin_y) / b->resolution) + b->size_y / 2;

  // check for out of bounds
  if (!(MAP_VALID(b, b_oi, b_oj) && MAP_VALID(b, b_ei, b_ej)))
    return;

  /*
  int b_oi = MAP_GXWX(b, ox);
  int b_oj = MAP_GYWY(b, oy);
  int b_ei = MAP_GXWX(b, ex);
  int b_ej = MAP_GYWY(b, ey);
  */

  // get the state of the overlapped cell in map a
  int state = a->cells[MAP_INDEX(a, i, j)].state;
  // set the origin cell of the region in map b
  // to the maximum of its own state and the overlapped cell's state
  // occupied = 1, free = 0, unknown = -1
  b->cells[MAP_INDEX(b, b_oi, b_oj)].state = std::max(b->cells[MAP_INDEX(b, b_oi, b_oj)].state, state);

  // do the same for the other cells in the overlapped region of map b
  for (int i = b_oi; i < b_ei; i++)
  {
    for (int j = b_oj; j < b_ej; j++)
    {
      b->cells[MAP_INDEX(b, b_oi, b_oj)].state = std::max(b->cells[MAP_INDEX(b, b_oi, b_oj)].state, state);
    }
  }
}

/**
 * Create a ProbabilityGrid object from a Map object
 *
 * @param map the input Map
 * @param resolution the desired resolution of the probability grid
 * @result The ProbabilityGrid object
 */
ProbabilityGrid *RealTimeCSM::createGridFromMap(Map *map, double resolution)
{
  ProbabilityGrid *grid = new ProbabilityGrid();
  grid->resolution = resolution;
  grid->size_x = (double)map->size_x * map->resolution / grid->resolution;
  grid->size_y = (double)map->size_y * map->resolution / grid->resolution;
  grid->origin_x = (map->origin_x - (map->size_x / 2) * map->resolution) + (grid->size_x / 2) * grid->resolution;
  grid->origin_y = (map->origin_y - (map->size_x / 2) * map->resolution) + (grid->size_y / 2) * grid->resolution;
  grid->cells = std::vector<Cell>(grid->size_x * grid->size_y); // forcing the default constructor of Cell to be called

  ROS_INFO("Created grid with width %d, height %d and resolution %f",
           grid->size_x, grid->size_y, grid->resolution);

  return grid;
}

void RealTimeCSM::resetCells(ProbabilityGrid *grid)
{
  std::fill(grid->cells.begin(), grid->cells.end(), Cell());
  // grid->cells = std::vector<Cell>(grid->size_x * grid->size_y); // forcing the default constructor of Cell to be called
}

/**
 * Get pose candidates.
 *
 * @param q initial poses
 * @param grid the probability grid to match against
 * @param search_x the linear x search region
 * @param search_y the linear y search region
 * @param search_a the angular search region
 * @param ignore_unknown whether to ignore unknown cells as potential candidates
 * @param ignore_obstacles whether to ignore obstacle cells as potential candidates
 * @return Pose candidates.
 *
 * @todo GPU implementation
 */
std::priority_queue<Pose> RealTimeCSM::match(std::priority_queue<Pose> &q, ProbabilityGrid *grid, double search_x, double search_y, double search_a, bool ignore_unknown, bool ignore_obstacles)
{
  std::priority_queue<Pose> refined_q;
  while (!q.empty())
  {
    Pose current = q.top();
    int current_mi = MAP_GXWX(grid, current.x); // cell index of the candidate
    int current_mj = MAP_GYWY(grid, current.y); // cell index of the candidate

    // get the laser position of the candidate
    // since we are matching laser scans, we have to take the laser offset into account
    Pose laser = current;
    /* laser.x += laser_pose_.x * cos(current.theta) + laser_pose_.y * cos(M_PI - current.theta); // position of the laser
     * laser.y += laser_pose_.x * sin(current.theta) + laser_pose_.y * sin(M_PI - current.theta); // position of the laser */
    laser.x += laser_pose_.x * cos(current.theta) - laser_pose_.y * sin(current.theta);
    laser.y += laser_pose_.x * sin(current.theta) + laser_pose_.y * cos(current.theta);
    // we do not have to take angular offset into account,
    // as the angles were already converted to the base frame when laser_data_ was populated
    /* laser.theta += laser_pose_.theta;                                                          // position of the laser */

    // TODO: check if the angles need to be wrapped
    double min_angle = current.theta - search_a / 2;
    double max_angle = current.theta + search_a / 2;
    ROS_DEBUG("Current theta: %f, min theta: %f, max theta: %f", current.theta, min_angle, max_angle);

    // TODO: wrap min and max to within map dimensions
    double min_x = current.x - search_x / 2;
    double max_x = current.x + search_x / 2;
    ROS_DEBUG("Current x: %f, min x: %f, max x: %f", current.x, min_x, max_x);

    double min_y = current.y - search_y / 2;
    double max_y = current.y + search_y / 2;
    ROS_DEBUG("Current y: %f, min y: %f, max y: %f", current.y, min_y, max_y);

    int num_x = std::ceil(((max_x - min_x) / grid->resolution) / 2);
    int num_y = std::ceil(((max_y - min_y) / grid->resolution) / 2);
    ROS_DEBUG("Number of cells to search in x: %d", num_x);
    ROS_DEBUG("Number of cells to search in y: %d", num_y);

    double obs_range, obs_bearing; // temporary variables for ray range and angle
    double ex, ey; // temporary variables for the cell index of a ray endpoint
    /**
     * Vector of column indexes of ray endpoints
     */
    std::vector<int> mx;
    /**
     * Vector of row indexes of ray endpoints
     */
    std::vector<int> my;
    // loop over the angular search space by getting the ray endpoints for each orientation
    // we loop over the angular search space first, because subsequent looping over
    // the linear search space is simply stepping the cell indices
    for (double theta = min_angle; theta < max_angle; theta += search_a_step_)
    {
      mx.clear();
      my.clear();
      // for each ray, we calculate the ray endpoint (the cell where the ray ends)
      for (int i = 0; i < laser_data_->range_count; i++)
      {
        obs_range = laser_data_->ranges[i][0];
        obs_bearing = laser_data_->ranges[i][1];

        if (obs_range >= laser_data_->range_max)
          continue;

        if (!(std::isfinite(obs_range)))
          continue;

        ex = laser.x + obs_range * cos(theta + obs_bearing);
        ey = laser.y + obs_range * sin(theta + obs_bearing);
        mx.push_back(MAP_GXWX(grid, ex));
        my.push_back(MAP_GYWY(grid, ey));
      }
      // loop over the linear search space by stepping the cell indices
      for (int x = -num_x; x < num_x; x++)
      {
        for (int y = -num_y; y < num_y; y++)
        {
          // check that the candidate is not out of bounds, unknown or an obstacle
          if (!MAP_VALID(grid, current_mi + x, current_mj + y))
            continue;
          // ignore unknown cells as candidates if ignore_unknown is true
          if (ignore_unknown && grid->cells[MAP_INDEX(grid, current_mi + x, current_mj + y)].state == -1)
            continue;
          // ignore obstacle cells as candidates if ignore_obstacles is true
          if (ignore_obstacles && grid->cells[MAP_INDEX(grid, current_mi + x, current_mj + y)].state == 1)
            continue;

          /**
           * The probability of the current pose candidate.
           */
          int p = 0;
          // for each candidate, loop over the ray endpoints to calculate the
          // total probability
          for (int i = 0; i < mx.size(); i++)
          {
            int mi, mj;
            mi = mx[i] + x;
            mj = my[i] + y;

            if (!MAP_VALID(grid, mi, mj))
              continue;

            p += grid->cells[MAP_INDEX(grid, mi, mj)].probability;
          }
          // normalize the probability over the number of valid rays
          double score = (double)p / (double)(MAX_PROBABILITY * laser_data_->valid_range_count);
          if (score >= min_score_)
            refined_q.push(Pose(current.x + x * grid->resolution, current.y + y * grid->resolution, theta, score));
          /* ROS_DEBUG("x: %d, y: %d, theta: %f, score: %f", x, y, theta, score); */
        }
      }
    }
    q.pop();
  }
  return refined_q;
}

/**
 * Add to an average filter window.
 * We use an average filter because the pose estimates tend to jump
 * due to noise and possibly multiple estimates with equally high probability.
 *
 * @param input the Pose object to add to the average filter window.
 *
 * @todo: move filter implementation out of this node so that
 * other packages can use it too
 */
void RealTimeCSM::addToWindow(const Pose& input)
{

  filter_window_.push_back(input);
  if (filter_window_.size() > filter_window_size_)
    filter_window_.pop_front();
}

/**
 * Get the filtered result from the average filter window.
 *
 * @param result the result passed in by reference
 * @result Returns false if the filter window has not yet been fully populated.
 */
bool RealTimeCSM::getFilteredResult(Pose& result)
{
  if (filter_window_.size() < filter_window_size_)
    return false;

  Pose sum;
  for (const auto& pose : filter_window_)
  {
    sum += pose;
  }
  result = sum / filter_window_size_;

  return true;
}

void RealTimeCSM::clearWindow()
{
  filter_window_.clear();
}

void RealTimeCSM::reset()
{
  last_tf_valid_ = false;
  have_first_raycast_ = false;
  paused_ = false;
  clearWindow();
}
} // namespace correlative_scan_matcher

int main(int argc, char **argv)
{
  #ifdef MOVEL_LICENSE                                                                                                    
  MovelLicense ml(23);                                                                                                   
  if (!ml.login())                                                                                                      
    return 1;                                                                                                           
  #endif

  ros::init(argc, argv, "fast_csm");
  correlative_scan_matcher::RealTimeCSM node;

  ros::Rate r(20.0);
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
#ifdef MOVEL_LICENSE
    ml.logout();
#endif
}
