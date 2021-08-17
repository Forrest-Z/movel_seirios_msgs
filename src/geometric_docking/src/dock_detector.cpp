#include <geometric_docking/dock_detector.hpp>

DockDetector::DockDetector()
{
  setupTopics();  
}

void DockDetector::setupParams(double width, double offset, bool dock_bk)
{
  dock_width_ = width;
  dock_offset_ = offset;
  dock_backwards_ = dock_bk;
  cout << "dock width: " << dock_width_ << endl;
  cout << "dock offset: " << dock_offset_ << endl;

  ros::NodeHandle nh_local("~");

  dock_distance_ = 0.5;
  if (nh_local.hasParam("dock_distance"))
    nh_local.getParam("dock_distance", dock_distance_);
  ROS_INFO("dock distance %5.2f m", dock_distance_);

  dock_width_tolerance_ = 0.5;
  if (nh_local.hasParam("dock_width_tolerance"))
    nh_local.getParam("dock_width_tolerance", dock_width_tolerance_);
  ROS_INFO("dock width tolerance %5.2f", dock_width_tolerance_);

  max_dock_distance_ = 1.0;
  if (nh_local.hasParam("max_dock_distance"))
    nh_local.getParam("max_dock_distance", max_dock_distance_);
  ROS_INFO("max dock distance: %5.2f m", max_dock_distance_);

  line_consistency_threshold_ = 0.1;
  if (nh_local.hasParam("line_consistency_threshold"))
    nh_local.getParam("line_consistency_threshold", line_consistency_threshold_);
  ROS_INFO("line consistency threshold: %5.2f m", line_consistency_threshold_);

  laser_skip_ = 0;
  if (nh_local.hasParam("laser_skip"))
    nh_local.getParam("laser_skip", laser_skip_);
  ROS_INFO("laser skip: %d m", laser_skip_);

  min_angle_to_dock_ = -0.5*M_PI;
  if (nh_local.hasParam("min_angle_to_dock"))
    nh_local.getParam("min_angle_to_dock", min_angle_to_dock_);
  
  max_angle_to_dock_ = 0.5*M_PI;
  if (nh_local.hasParam("max_angle_to_dock"))
    nh_local.getParam("max_angle_to_dock", max_angle_to_dock_);
}

void DockDetector::setupTopics()
{
  dock_candidates_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("dock_candidates", 1);
  segments_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("line_segments", 1);
}

bool DockDetector::findDock(sensor_msgs::LaserScan &scan, geometry_msgs::Pose &outpose)
{
  Cloud segment_cloud;
  int segment_count = 0;

  // double e_max = dock_offset_/4.0;
  double e_max = line_consistency_threshold_;
  cout << "finding dock" << endl;

  // filter scan to valid lines, convert to vectors
  vector<Vector2d> ranges;
  double r, theta, x, y;
  for (int i = 0; i < scan.ranges.size(); i+=(1+laser_skip_))
  {
    r = scan.ranges[i];
    if (std::isnan(r) || r < scan.range_min || r > scan.range_max)
      continue;

    theta = scan.angle_min + i*scan.angle_increment;
    x = r*cos(theta);
    y = r*sin(theta);
    Vector2d pt_i;
    pt_i << x, y;
    ranges.push_back(pt_i);
  }
  
  // find dock candidates
  int i_start = 0;
  int i_end;
  vector< vector<Vector2d> > dock_candidates; // dock candidates are described as a vector of pairs of start/end points
  vector< vector<int> > dock_candidate_indices;
  while (i_start < ranges.size())
  {
    // find end index
    i_end = i_start + 1;
    // cout << "i_start " << i_start << endl;
    while (true)
    {
      double d = (ranges[i_end] - ranges[i_start]).norm();
      if (d > dock_width_tolerance_*dock_width_)
      {
        i_end -= 1;
        break;
      }
      i_end += 1;
      i_end = i_end % ranges.size();
      if (i_end == i_start)
        return false;
    }
    // cout << "i_start/i_end " << i_start << "/" << i_end << endl;

    // check that there are > 1 points
    if (i_start == i_end)
    {
      i_start += 1;
      continue;
    }

    // get line parameter
    vector<Vector2d> segment_points;
    int i_end_ = i_end;
    if (i_end_ < i_start)
      i_end_ = i_end + ranges.size();
    for (int i = i_start; i <= i_end_; i++)
    {
      int i_ = i % ranges.size();
      segment_points.push_back(ranges[i_]);
    }
    Vector2d line_params = calcLineParams(segment_points);
    // cout << "line params: " << line_params.transpose() << endl;

    // get orthogonal distance in the segment
    // cout << "orthogonal distances" << endl;
    bool valid_segment = true;
    for (int i = 0; i < segment_points.size(); i++)
    {
      double err_i = calcOrthoDistance(segment_points[i], line_params);
      double d_increment = 0.0;
      if (i > 0)
      {
        d_increment = (segment_points[i] - segment_points[i-1]).norm();
      }
      if (err_i > e_max || d_increment > 0.5*dock_offset_)
      {
        // cout << "ortho error " << err_i << "/" << e_max << endl;
        valid_segment = false;
        break;
      }
    }
    // cout << "segment validation " << valid_segment << endl;
    // return false;

    // large distance: advance i_start
    if (!valid_segment)
    {
      i_start += 1;
      continue;
    }

    // small distance: find end of line segment
    while (i_end != i_start)
    {
      // cout << "find end, i_start/i_end " << i_start << "/" << i_end << endl;
      int i_prev_end = i_end;
      i_end += 1;
      i_end %= ranges.size();
      double d_increment = (ranges[i_end] - ranges[i_prev_end]).norm();
      // debug cycle
      // if (i_end < i_start)
      // {
      //   cout << "i_end: " << i_end << ", i_prev_end: " << i_prev_end << " points: ";
      //   cout << ranges[i_end].transpose() << ", " << ranges[i_prev_end].transpose();
      //   cout << " inc " << d_increment << endl;
      // }

      // Vector2d line_params_ = calcLineParams(i_start, i_end, ranges);
      double err_i = calcOrthoDistance(ranges[i_end], line_params);
      // cout << "Second loop; i_end " << i_end << endl;
      // each segment must be self-consistent and reasonably gapless
      if (err_i > e_max || d_increment > 0.5*dock_offset_)
      {
        i_end -= 1;
        if (i_end < 0)
          i_end += ranges.size();
        
        // populate segment cloud
        i_end_ = i_end;
        if (i_end_ < i_start)
          i_end_ += ranges.size();

        for (int i = i_start; i <= i_end_; i++)
        {
          Pt pt_i;
          int i_ = i % ranges.size();
          pt_i.x = ranges[i_](0);
          pt_i.y = ranges[i_](1);
          pt_i.z = 0.0;
          pt_i.intensity = segment_count;
          segment_cloud.points.push_back(pt_i);
        } 
        segment_count += 1;

        double final_segment_length = (ranges[i_end] - ranges[i_start]).norm();
        cout << segment_count-1 << " segment spec ";
        cout << ranges[i_start].transpose() << ", ";
        cout << ranges[i_end].transpose() << ", ";
        cout << "len: " << final_segment_length << "/" << dock_width_tolerance_*dock_width_ << ", ";
        // DEBUG: check maximum ortho error for the segment
        double max_segment_ortho_error = 0.0;
        for (int i = i_start; i <= (i_end + ranges.size())%ranges.size(); i++)
        {
          int i_ = i % ranges.size();
          double err = calcOrthoDistance(ranges[i_], line_params);
          if (err > max_segment_ortho_error)
            max_segment_ortho_error = err;
        }
        cout << "e_max: " << max_segment_ortho_error << endl;
        cout << "    --- " << i_start << ", " << i_end << ", ";
        cout << err_i << "/" << e_max << ", " << d_increment << "/" << 0.5*dock_offset_;
        cout << ", " << line_params.transpose() << endl;

        // segment just right: add to candidate dock
        if (final_segment_length < dock_width_ &&
            final_segment_length > dock_width_tolerance_*dock_width_) // tolerance because the laser won't hit precisely both ends of the docking plate
        {
          // check that segment is within max distance
          // and within allowed angle
          Vector2d drange = ranges[i_start] + ranges[i_end];
          double dee = (0.5*(drange)).norm();
          double angle_to_dock = atan2(drange(1), drange(0));
          bool angleOK = false;
          if (dock_backwards_) angleOK = true;
          else if (angle_to_dock < max_angle_to_dock_ && angle_to_dock > min_angle_to_dock_)
              angleOK = true;
          if (dee < max_dock_distance_ && angleOK)
              //angle_to_dock < max_angle_to_dock_ &&
              //angle_to_dock > min_angle_to_dock_)
          {
            vector<Vector2d> candidate_pair(3);
            candidate_pair[0] = ranges[i_start];
            candidate_pair[1] = ranges[i_end];
            candidate_pair[2] = line_params;
            dock_candidates.push_back(candidate_pair);

            vector<int> candidate_pair_indices(2);
            candidate_pair_indices[0] = i_start;
            candidate_pair_indices[1] = i_end;
            dock_candidate_indices.push_back(candidate_pair_indices);
          }
        }

        // advance i_start to after segment
        if (i_start < i_end)
          i_start = i_end+1;
        else
          i_start = ranges.size();
        break;
      }
    }
    // cout << "extension done" << endl;
  }
  cout << "dock candidates done; there are " << dock_candidates.size() << endl;
  visDockCandidates(dock_candidate_indices, ranges, scan);
  
  sensor_msgs::PointCloud2 segment_cloud_msg;
  pcl::toROSMsg(segment_cloud, segment_cloud_msg);
  segment_cloud_msg.header = scan.header;
  segments_pub_.publish(segment_cloud_msg);

  if (dock_candidates.size() == 0)
    return false;

  // pick candidate closest to laser origin while satisfying angle criteria
  double d_min = 1000.0;
  int idx_min = -1;
  for (int i = 0; i < dock_candidates.size(); i++)
  {
    cout << "candidate " << i << ": ";
    cout << dock_candidates[i][0].transpose() << ", " << dock_candidates[i][1].transpose() << endl;
    Vector2d pt_mid = 0.5*(dock_candidates[i][0] + dock_candidates[i][1]);
    double d_i = pt_mid.norm();
    if (d_i < d_min)
    {
      idx_min = i;
      d_min = d_i;
    }
  }
  
  if (idx_min < 0){ 
    return false;}

  // prepare pose message
  // pose is the robot's desired pose when it's docked
  // find docking plate centre
  Vector2d dock_pt = 0.5*(dock_candidates[idx_min][0] + dock_candidates[idx_min][1]);

  // find docking plate line normal
  double m = dock_candidates[idx_min][2](0);
  double theta_line;
  if (fabs(m) < 1.0e-3)
  {
    theta_line = 0.0;
  }
  else
  {
    theta_line = atan(-1.0/m);
  }
  Vector2d dock_normal;
  dock_normal << cos(theta_line), sin(theta_line);
  Vector2d dock_pose = dock_pt + dock_distance_*dock_normal;
  Vector2d dock_pose_alt = dock_pt - dock_distance_*dock_normal;
  dock_normal = dock_distance_*dock_normal;
  if (dock_pose_alt.squaredNorm() < dock_pose.squaredNorm())
    dock_pose = dock_pose_alt;
  else
    theta_line *= -1.0;

  outpose.position.x = dock_pose(0);
  outpose.position.y = dock_pose(1);
  outpose.position.z = 0.0;

  // double yaw = atan2(dock_pt(1), dock_pt(0));
  Vector2d ddock = dock_pt - dock_pose;
  double yaw = atan2(ddock(1), ddock(0));
  outpose.orientation.x = 0.0;
  outpose.orientation.y = 0.0;
  outpose.orientation.z = sin(yaw/2.0);
  outpose.orientation.w = cos(yaw/2.0);
  return true;
}

// Calculate line parameters from points with least squared error method
Vector2d DockDetector::calcLineParams(vector<Vector2d> points)
{
  // cout << "calculate line parameters" << endl;
  // allocate matrices and vectors
  int N = points.size();
  MatrixXd A(N, 2);
  VectorXd Y(N);

  for (int i = 0; i < N; i++)
  {
    A(i, 0) = points[i](0);
    A(i, 1) = 1.0;
    Y(i) = points[i](1);
  }
  // cout << A << endl;
  // cout << "---" << endl;
  // cout << Y << endl;

  // calculate LSE params
  VectorXd line_params = A.colPivHouseholderQr().solve(Y);
  // cout << "---" << endl;
  // cout << line_params << endl;
  return line_params;
}

Vector2d DockDetector::calcLineParams(int i_start, int i_end, vector<Vector2d> ranges)
{
  vector<Vector2d> points;
  if (i_end < i_start)
  {
    points.insert(points.begin(), ranges.begin(), ranges.begin()+i_end+1);
    i_end = ranges.size()-1;
  }
  points.insert(points.end(), ranges.begin()+i_start, ranges.begin()+i_end+1);
  return calcLineParams(points);
}

double DockDetector::calcOrthoDistance(Vector2d pt, Vector2d line_params)
{
  // move parameters to rho-theta space
  double m, c;
  m = line_params(0);
  c = line_params(1);

  double rho, theta;
  if (fabs(m) < 1.0e-3)
  {
    theta = 0.0;
  }
  else
  {
    theta = atan(-1.0/m);
  }
  rho = c*sin(theta);  

  // get two points on the line
  Vector2d pa;
  // pa << pt(0), line_params(0)*pt(0) + line_params(1);
  pa << rho*cos(theta), rho*sin(theta);
  Vector2d pb;
  // pb << pt(0)+1, line_params(0)*(pt(0)+1.0) + line_params(1);
  pb << rho*cos(theta) - sin(theta), rho*sin(theta) + cos(theta);

  Vector2d pta, pba;
  pta = pt - pa;
  pba = pb - pa;
  // cout << pa.transpose() << ", " << pb.transpose() << ", " << pt.transpose() << ", ";
  double pta_dot_pba = pta.dot(pba);
  double d_squared = pta.squaredNorm() - pta_dot_pba*pta_dot_pba/pba.squaredNorm();
  // cout << sqrt(d) << endl;
  return sqrt(d_squared);
}

void DockDetector::visDockCandidates(vector< vector<int> > &candidate_idx, 
                                     vector<Vector2d> &ranges, sensor_msgs::LaserScan scan)
{
  Cloud candidate_cloud;
  for (int i = 0; i < candidate_idx.size(); i++)
  {
    int idx_start = candidate_idx[i][0];
    int idx_end = candidate_idx[i][1];
    if (idx_end < idx_start)
      idx_end += ranges.size();

    for (int j = idx_start; j <= idx_end; j++)
    {
      int j_ = j % ranges.size();
      Pt pt_j;
      pt_j.x = ranges[j_](0);
      pt_j.y = ranges[j_](1);
      pt_j.z = 0.0;
      pt_j.intensity = i;

      candidate_cloud.points.push_back(pt_j);
    }
  }
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(candidate_cloud, cloud_msg);
  cloud_msg.header = scan.header;
  dock_candidates_pub_.publish(cloud_msg);
}
