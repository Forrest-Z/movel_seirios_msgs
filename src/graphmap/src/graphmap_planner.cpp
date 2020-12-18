#include <pluginlib/class_list_macros.h>
#include <graphmap/graphmap_planner.hpp>

PLUGINLIB_EXPORT_CLASS(graph_planner::GraphPlanner, nav_core::BaseGlobalPlanner)

geometry_msgs::PoseStamped bundle2pose(VxBundle bundle)
{
  geometry_msgs::PoseStamped outpose;
  outpose.pose.position.x = bundle.x;
  outpose.pose.position.y = bundle.y;
  outpose.pose.position.z = bundle.z;

  return outpose;
}

float getPoseDistanceSquared(const geometry_msgs::PoseStamped &pose1, const geometry_msgs::PoseStamped &pose2)
{
  float x0, y0, z0, x1, y1, z1;
  x0 = pose1.pose.position.x;
  y0 = pose1.pose.position.y;
  z0 = pose1.pose.position.z;

  x1 = pose2.pose.position.x;
  y1 = pose2.pose.position.y;
  z1 = pose2.pose.position.z;

  float dx, dy, dz, dd;
  dx = x1 - x0;
  dy = y1 - y0;
  dz = z1 - z0;

  dd = dx*dx + dy*dy + dz*dz;
  return dd;
}

float getPoseDistance(const geometry_msgs::PoseStamped &pose1, const geometry_msgs::PoseStamped &pose2)
{
  float dd = getPoseDistanceSquared(pose1, pose2);
  return sqrt(dd);
}

// Calculate the dot product of vectors (pose1 - pose0) and (pose2 - pose0)
float get3PtDotProduct(const geometry_msgs::PoseStamped &pose0, 
                       const geometry_msgs::PoseStamped &pose1,
                       const geometry_msgs::PoseStamped &pose2)
{
  float x0, y0, z0, x1, y1, z1, x2, y2, z2;
  x0 = pose0.pose.position.x;
  y0 = pose0.pose.position.y;
  z0 = pose0.pose.position.z;

  x1 = pose1.pose.position.x;
  y1 = pose1.pose.position.y;
  z1 = pose1.pose.position.z;

  x2 = pose2.pose.position.x;
  y2 = pose2.pose.position.y;
  z2 = pose2.pose.position.z;

  float dx10, dy10, dz10, dx20, dy20, dz20;
  dx10 = x1 - x0;
  dy10 = y1 - y0;
  dz10 = z1 - z0;

  dx20 = x2 - x0;
  dy20 = y2 - y0;
  dz20 = z2 - z0;

  return dx10*dx20 + dy10*dy20 + dz10*dz20;
}

namespace graph_planner
{

  GraphPlanner::GraphPlanner() : inited_(false), have_active_goal_(false), navfn_planner_()
  {
    // ROS_INFO("GraphPlanner default constructor");
    // costmap_2d::Costmap2DROS* costmap_ros(NULL);
    // std::string name = "GraphPlanner";
    
    // initialize(name, costmap_ros);
  }

  GraphPlanner::GraphPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros) 
  : inited_(false)
  , have_active_goal_(false)
  , navfn_planner_("GlobalPlanner", costmap_ros)
  {
    initialize(name, costmap_ros);
  }

  void GraphPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {
    if (!inited_)
    {
      int lic = checkForLicense();
      if (lic < 0)
      {
        ROS_INFO("License check failed");
        throw 99;
      }
      ROS_INFO("License OK");
      navfn_planner_.initialize("LOSPlanner", costmap_ros);
      ros::NodeHandle local_nh("~/" + name);
      // ROS_INFO("nodehandle %s", nh_.resolveName("graph_def", false).c_str());
      ROS_INFO("local nodehandle %s", local_nh.resolveName("graph_def", false).c_str());
      // ROS_INFO("name %s", name.c_str());
      inited_ = true;
      
      std::string graph_def;
      local_nh.getParam("graph_def", graph_def);
      ROS_INFO("graph definition in %s", graph_def.c_str());
      gm_.parseCsv(graph_def);

      goal_threshold_ = 0.5f;
      local_nh.getParam("goal_threshold", goal_threshold_);
      
      vis_graph_pub_ = local_nh.advertise<visualization_msgs::Marker>("map_graph", 1000);
      vis_plan_pub_ = local_nh.advertise<geometry_msgs::PoseArray>("waypoints_graph", 1);

      visGraph();
    }
  }

  bool GraphPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                              const geometry_msgs::PoseStamped &goal,
                              std::vector<geometry_msgs::PoseStamped> &plan)
  {
    // ROS_INFO("input plan is of size %lu", plan.size());
    visGraph();
    float x0, y0, z0, x1, y1, z1;
    x0 = start.pose.position.x;
    y0 = start.pose.position.y;
    z0 = start.pose.position.z;

    x1 = goal.pose.position.x;
    y1 = goal.pose.position.y;
    z1 = goal.pose.position.z;

    // check if we've actually arrived
    float dx, dy, dz;
    dx = x1 - x0;
    dy = y1 - y0;
    dz = z1 = z0;
    float dd = sqrt(dx*dx + dy*dy + dz*dz);
    if (dd < goal_threshold_)
    {
      plan.push_back(start);
      visPath(plan);
      return true;
    }

    // check if this is a replan with the same goal
    // ROS_INFO("active plan size %lu", active_plan_.size());
    geometry_msgs::PoseStamped active_goal;
    if (active_plan_.size() > 0)
    {
      dd = getPoseDistance(active_plan_[active_plan_.size()-1], goal);
      // ROS_INFO("new goal distance %f", dd);
      if (dd < goal_threshold_)
      {
        // Plan Truncation Algorithm
        // traverse pose in plan one by one
        // if plan pose is closer than goal_threshold_ to robot pose, skip it
        // else, check how far is the robot along the line between 
        //   the pose being considered and the next one
        // if negative, robot is behind current line, return path from current pose
        // if positive and < length of segment, return path from next pose
        // if positive and > length of segment, skip current pose
        // (next pose will be skipped by the positive condition too next iteration)
        // if pose under consideration is the last one, return path to it
        int skip_idx = -1;
        for (int i = 0; i < active_plan_.size()-1; i++)
        {
          dd = getPoseDistance(start, active_plan_[i]);
          if (dd < goal_threshold_)
          {
            skip_idx = i;
            continue;
          }
          
          float s, segment_length;
          segment_length = getPoseDistance(active_plan_[i], active_plan_[i+1]);
          s = get3PtDotProduct(active_plan_[i], active_plan_[i+1], start)
              / segment_length / segment_length;
          if (s < 0)
          {
            break;
          }
          else
          {
            skip_idx = i;
            if (s < 1.0)
              break;
          }
        }
        skip_idx = std::max(0, skip_idx);
        // ROS_INFO("skip until %d", skip_idx);
        // compose plan
        plan.push_back(start);
        for (int i = skip_idx+1; i < active_plan_.size(); i++)
        {
          plan.push_back(active_plan_[i]);
        }
        active_plan_ = plan;
        visPath(plan);
        return true;
      }
    }

    // general case, make path in graph, appending start and goal

    // expanded two-vertex rule with LOS
    // 1. add start and goal vertices
    // 2. get KNN from each
    // 3. check LOS
    // 4. connect only neighbour vertices with LOS
    // 5. do search with the Vx method
    // 6. convert output to pose

    bool success = false;

    Vx vx_start = gm_.addVertex(x0, y0, z0);
    Vx vx_goal = gm_.addVertex(x1, y1, z1);

    bool eg_start, eg_goal;
    eg_start = addEdgeLOS(vx_start);
    eg_goal = addEdgeLOS(vx_goal);

    std::vector<Vx> vx_path;
    std::vector<std::vector<float>> path;
    if (eg_start && eg_goal)
    {
      success = gm_.findPath(vx_start, vx_goal, vx_path);
      gm_.pathVx2Euclid(vx_path, path);
    }
    gm_.removeVertex(vx_goal);
    gm_.removeVertex(vx_start);
    
    // success = gm_.findPath(x0, y0, z0, x1, y1, z1, path);

    if (success)
    {
      for (int i = 0; i < path.size(); i++)
      {
        geometry_msgs::PoseStamped pose_i;
        pose_i.header.frame_id = "map";
        pose_i.header.stamp = ros::Time::now();
        pose_i.pose.position.x = path[i][0];
        pose_i.pose.position.y = path[i][1];
        pose_i.pose.position.z = path[i][2];

        // orientation 
        // for start and goal poses, keep given orientation
        // for intermediate poses, make orientation such that it faces
        // the next pose
        if (i == 0)
        {
          pose_i.pose.orientation = start.pose.orientation;
        }
        else if (i == path.size() - 1)
        {
          pose_i.pose.orientation = goal.pose.orientation;
        }
        else
        {
          float dx, dy;
          dx = path[i+1][0] - path[i][0];
          dy = path[i+1][1] - path[i][1];
          float th = atan2(dy, dx);
          pose_i.pose.orientation.x = 0.0;
          pose_i.pose.orientation.y = 0.0;
          pose_i.pose.orientation.z = sin(0.5*th);
          pose_i.pose.orientation.w = cos(0.5*th);
        }
        
        plan.push_back(pose_i);
      }
      active_plan_ = plan;
      visPath(plan);
      return true;
    }

    // failure case, stay where we are
    plan.push_back(start);
    active_plan_ = plan;
    return false;
  }

  void GraphPlanner::initGraph()
  {
  }

  void GraphPlanner::visGraph()
  {
    // publish vertices
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "vertices";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5;
    marker.lifetime = ros::Duration();
    marker.points.clear();

    std::vector<VxBundle> map_vertices;
    gm_.getVertices(map_vertices);
    for (int i = 0; i < map_vertices.size(); i++)
    {
      marker.pose.position.x = map_vertices[i].x;
      marker.pose.position.y = map_vertices[i].y;
      marker.pose.position.z = map_vertices[i].z;
      vis_graph_pub_.publish(marker);

      marker.id += 1;
    }

    int edge_id_offset = 100;
    while (edge_id_offset < map_vertices.size())
      edge_id_offset *= 10;

    // publish edges
    // prep common and initial parameters
    visualization_msgs::Marker eg_marker;
    eg_marker.header.frame_id = "map";
    eg_marker.header.stamp = ros::Time::now();
    eg_marker.ns = "edges";
    eg_marker.id = edge_id_offset;
    eg_marker.type = visualization_msgs::Marker::ARROW;
    eg_marker.action = visualization_msgs::Marker::ADD;
    eg_marker.scale.x = 0.05;
    eg_marker.scale.y = 0.1;
    eg_marker.color.r = 1.0f;
    eg_marker.color.g = 0.0f;
    eg_marker.color.b = 1.0f;
    eg_marker.color.a = 0.5;
    eg_marker.lifetime = ros::Duration();
    eg_marker.points.clear();

    // get endpoint coordinates of the map edges
    std::vector<std::vector<VxBundle>> map_edges;
    gm_.getEdgesEuclidean(map_edges);

    // publish edge by edge
    for (int i = 0; i < map_edges.size(); i++)
    {
      geometry_msgs::Point src, tgt;
      src.x = map_edges[i][0].x;
      src.y = map_edges[i][0].y;
      src.z = map_edges[i][0].z;

      tgt.x = map_edges[i][1].x;
      tgt.y = map_edges[i][1].y;
      tgt.z = map_edges[i][1].z;

      eg_marker.points.push_back(src);
      eg_marker.points.push_back(tgt);

      vis_graph_pub_.publish(eg_marker);
      eg_marker.points.clear();
      eg_marker.id += 1;
    }
  }

  void GraphPlanner::visPath(std::vector<geometry_msgs::PoseStamped> &plan)
  {
    geometry_msgs::PoseArray path;

    for (int i = 0; i < plan.size(); i++)
    {
      path.poses.push_back(plan[i].pose);
      path.poses[i].position.z = 0.5;
    }
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    
    vis_plan_pub_.publish(path);
  }

  // check if there is unobstructed "straight-line" path between src and tgt
  // if actual distance as returned by navfn is > los_factor * euclidean distance
  // then it's not straigh-line path, otherwise it is
  bool GraphPlanner::checkLOS(geometry_msgs::PoseStamped &src,
                              geometry_msgs::PoseStamped &tgt,
                              float los_factor)
  {
    float dd = getPoseDistance(src, tgt);
    std::vector<geometry_msgs::PoseStamped> plan;
    navfn_planner_.makePlan(src, tgt, plan);
    
    if (plan.size() <= 1)
      return true;

    float dnav = 0.0;
    for (int i = 0; i < plan.size()-1; i++)
    {
      float di = getPoseDistance(plan[i], plan[i+1]);
      dnav += di;
    }

    if (dnav/dd >= los_factor)
      return false;
    return true;
  }

  bool GraphPlanner::addEdgeLOS(Vx src, float los_factor)
  {
    int nbr_count = 2;
    int idx_check = 0;
    std::vector<Vx> src_nbrs;

    bool found_direct_nbr = false;
    while (!found_direct_nbr)
    {
      src_nbrs.clear();
      gm_.getKNN(src, nbr_count+1, src_nbrs); // plus one because the query is counted

      // std::cout << "neighbours of " << src << ": ";
      // for (int j = 0; j < src_nbrs.size(); j++)
      // {
      //   std::cout << src_nbrs[j] << ", ";
      // }
      // std::cout << std::endl;

      for (int i = idx_check+1; i < src_nbrs.size(); i++)
      {
        VxBundle vx_src, vx_tgt;
        vx_src = gm_.getBundle(src);
        vx_tgt = gm_.getBundle(src_nbrs[i]);

        geometry_msgs::PoseStamped los_src, los_tgt;
        los_src = bundle2pose(vx_src);
        los_tgt = bundle2pose(vx_tgt);
        los_src.header.frame_id = "map";
        los_tgt.header.frame_id = "map";

        bool los = checkLOS(los_src, los_tgt, los_factor);
        if (los)
        {
          gm_.addEdge(src, src_nbrs[i]);
          gm_.addEdge(src_nbrs[i], src);
          found_direct_nbr = true;
        }
      }

      if (nbr_count == gm_.size())
        break;

      idx_check = nbr_count;
      nbr_count *= 2;
      if (nbr_count >= gm_.size())
        nbr_count = gm_.size();
    }
    return found_direct_nbr;
  }
}; // namespace graph_planner