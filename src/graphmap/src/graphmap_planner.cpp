#include <pluginlib/class_list_macros.h>
#include <graphmap/graphmap_planner.hpp>
#include <movel_hasp_vendor/license.h>

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

  GraphPlanner::GraphPlanner() : inited_(false), have_active_goal_(false),tf_ear_(tf_buffer_),serv_request(false), navfn_planner_()
  {
    #ifdef MOVEL_LICENSE                                                                                                    
      MovelLicense ml(16);                                                                                                   
      if (!ml.login())                                                                                                      
        exit(0);                                                                                                           
    #endif

    // ROS_INFO("GraphPlanner default constructor");
    // costmap_2d::Costmap2DROS* costmap_ros(NULL);
    // std::string name = "GraphPlanner";
    
    // initialize(name, costmap_ros);
  }

  GraphPlanner::GraphPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros) 
  : inited_(false)
  , have_active_goal_(false)
  ,serv_request(false)
  ,tf_ear_(tf_buffer_)
  , navfn_planner_("GlobalPlanner", costmap_ros)
  {
    initialize(name, costmap_ros);
  }

  void GraphPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {
    if (!inited_)
    {
      navfn_planner_.initialize("LOSPlanner", costmap_ros);
      ros::NodeHandle local_nh("~/" + name);
      // ROS_INFO("nodehandle %s", nh_.resolveName("graph_def", false).c_str());
      ROS_INFO("local nodehandle %s", local_nh.resolveName("graph_def", false).c_str());
      // ROS_INFO("name %s", name.c_str());
      inited_ = true;
      make_plan_srv_ = nh_.advertiseService("/graph_map/make_plan", &GraphPlanner::make_plan_servicecb, this);
      path_distance_srv_ = nh_.advertiseService("/graph_map/path_distnace", &GraphPlanner::pathdist_servicecb, this);
      odom_sub_ = nh_.subscribe("/odom", 1, &GraphPlanner::odometryCb, this);
      edge_display_srv_ = nh_.advertiseService("/graph_map/edge_info", &GraphPlanner::edge_display_cb, this);
      change_graph_srv_ = nh_.advertiseService("/graph_map/change", &GraphPlanner::change_graph_servicecb, this);
      std::string graph_def;
      local_nh.getParam("graph_def", graph_def);
      ROS_INFO("graph definition in %s", graph_def.c_str());
      gm_.init_navfn(costmap_ros);
      bool check=gm_.parseCsv(graph_def);

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
    std::vector<geometry_msgs::PoseStamped> plan_check;
    navfn_planner_.makePlan(start, goal, plan_check);
    if (plan_check.size() == 0)
    {
      //plan.push_back(start);
      //visPath(plan);
      return false;
    } 
    //ROS_INFO("Hello");
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
      //ROS_INFO("Inside the active > 0");
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
      float segment_len = getPoseDistance(plan[plan.size()-2], plan[plan.size()-1]);
      float val = get3PtDotProduct(plan[plan.size()-2], plan[plan.size()-1], goal)/ segment_len;
      if (val>=0 && val <=1)
      {
        auto position = plan.end();
        position--;
        plan.erase(position);
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
    eg_start = addEdgeLOS(vx_start,true,start);
    eg_goal = addEdgeLOS(vx_goal,false,goal);
    std::vector<Vx> vx_path;
    std::vector<std::vector<float>> path;
    //ROS_INFO("Hello 123");
    if (eg_start && eg_goal)
    {
      success = gm_.findPath(vx_start, vx_goal, vx_path, path_list);
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
      float segment_len = getPoseDistance(plan[1], plan[2]);
      float val = get3PtDotProduct(plan[1], plan[2], start)/ segment_len;
      // ROS_INFO_STREAM("Inside check"<<val<<"\t"<<segment_len);

      if (val>=0 && val <=segment_len)
      {
        plan.erase(plan.begin()+1);
      }      


      segment_len = getPoseDistance(plan[plan.size()-3], plan[plan.size()-2]);
      val = get3PtDotProduct(plan[plan.size()-3], plan[plan.size()-2], goal)/ segment_len;
      // ROS_INFO_STREAM("Inside check"<<val<<"\t"<<segment_len);

      if (val>=0 && val <=segment_len)
      {
        plan.erase(plan.end()-2);
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

// Method called by the service call back to calculate distance and get plan  

bool GraphPlanner::makePlanServ(const geometry_msgs::PoseStamped &start,
                              const geometry_msgs::PoseStamped &goal,
                              std::vector<geometry_msgs::PoseStamped> &plan,std::list<Vx>& path_list_1)
  {
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
      return true;
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
    eg_start = addEdgeLOS(vx_start,true,start);
    eg_goal = addEdgeLOS(vx_goal,false,goal);
   // std::list<Vx> path_list_1;
    std::vector<Vx> vx_path;
    std::vector<std::vector<float>> path;
    if (eg_start && eg_goal)
    {
      success = gm_.findPath(vx_start, vx_goal, vx_path, path_list_1);
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
      return true;
    }

    // failure case, stay where we are
    plan.push_back(start);
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
    if (dnav - dd >= 0.30)
      return false;
    return true;
  }

  bool GraphPlanner::make_plan_servicecb(movel_seirios_msgs::get_graph_plan::Request& req, movel_seirios_msgs::get_graph_plan::Response& resp)
  {
    geometry_msgs::PoseStamped src;
    geometry_msgs::PoseStamped tgt;
    src = req.start;
    tgt = req.goal;
    std::vector<geometry_msgs::PoseStamped> plan;
    std::list<Vx> path_list_2;
    bool check =  makePlanServ(src,tgt,plan,path_list_2); 
    resp.plan.poses= plan;
    std::list<Vx>::iterator path_it = path_list_2.begin();
    for (; path_it != path_list_2.end(); path_it++)
    {
      resp.data.push_back(*path_it);
    }
    return true;
  }

  bool GraphPlanner::pathdist_servicecb(movel_seirios_msgs::pathdistance::Request& req, movel_seirios_msgs::pathdistance::Response& resp)
  {
    geometry_msgs::PoseStamped src;
    geometry_msgs::PoseStamped tgt;
    src = req.start;
    tgt = req.goal;
    std::vector<geometry_msgs::PoseStamped> plan;
    std::list<Vx> path_list_2;
    bool check =  makePlanServ(src,tgt,plan,path_list_2);
    float dnav = 0.0;
    for (int i = 0; i < plan.size()-1; i++)
    {
      float di = getPoseDistance(plan[i], plan[i+1]);
      dnav += di;
    }     
    resp.distance = dnav;
    return true;
  }

  bool GraphPlanner::change_graph_servicecb(movel_seirios_msgs::StringTrigger::Request& req, movel_seirios_msgs::StringTrigger::Response& resp)
  {
        if(gm_.parseCsv(req.input)){
        resp.success=true;
        resp.message="Success";
        }
        else {
        resp.success=false;
        resp.message="Failed";
    }
  }


  bool GraphPlanner::edge_display_cb(movel_seirios_msgs::edge_info::Request& req, movel_seirios_msgs::edge_info::Response& resp)
  {
    // Intial check to avoid failure
    if (active_plan_.size()>0)
    {   
        if (active_plan_.size()==2) // if plan size is two then the plan actually has only start and end point as the path
        {
          resp.v1[0]=active_plan_[0].pose.position.x;
          resp.v1[1]=active_plan_[0].pose.position.y;
          resp.v1[2]=path_list.front();
          resp.v2[0]=active_plan_[active_plan_.size()-1].pose.position.x;
          resp.v2[1]=active_plan_[active_plan_.size()-1].pose.position.y;
          resp.v2[2]=path_list.back();
        } else {
          geometry_msgs::PoseStamped pose_in_map;
          geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(active_plan_[0].header.frame_id, 
                                                                            latest_odom_.header.frame_id, ros::Time(0));
          tf2::doTransform(latest_odom_.pose.pose, pose_in_map.pose, transform);
          int min_idx = 0;
          float dd,min_dd ;
          min_dd=getPoseDistance(pose_in_map,active_plan_[0]);
          for(int i=1;i<active_plan_.size();i++){
            dd =  getPoseDistance(pose_in_map,active_plan_[i]);
            if (dd<min_dd){
              min_dd=dd;
              min_idx=i;
            }
          }

          if(min_idx==0){ // condition to check if the robot is closer to start point
            resp.v1[0]=active_plan_[0].pose.position.x;
            resp.v1[1]=active_plan_[0].pose.position.y;
            resp.v1[2]=path_list.front();
            resp.v2[0]=active_plan_[1].pose.position.x;
            resp.v2[1]=active_plan_[1].pose.position.y;
            auto l_front = path_list.begin();
            l_front++;
            resp.v2[2]=*l_front;
          } else if(min_idx==active_plan_.size()-1){// condition to check if the robot is closer to end point
            auto l_front = path_list.begin();
            // ROS_INFO_STREAM("L back 1 :"<<*l_front);
            std::advance(l_front, min_idx-1);
            // ROS_INFO_STREAM("L back 2:"<<*l_front);
            resp.v1[0]=active_plan_[active_plan_.size()-2].pose.position.x;
            resp.v1[1]=active_plan_[active_plan_.size()-2].pose.position.y;
            resp.v1[2]=*l_front;
            resp.v2[0]=active_plan_[active_plan_.size()-1].pose.position.x;
            resp.v2[1]=active_plan_[active_plan_.size()-1].pose.position.y;
            resp.v2[2]=path_list.back();
          } else { // dot product is used to find either the min vertices lies in front or back ward
            float d00,d01,modd0,v00,v01,d10,d11,r;
            d00 = active_plan_[min_idx+1].pose.position.x - active_plan_[min_idx].pose.position.x;
            d01 = active_plan_[min_idx+1].pose.position.y - active_plan_[min_idx].pose.position.y;
            modd0 = (sqrt(d00*d00+d01*d01));
            v00 = d00/modd0;
            v01 = d01/modd0;
            d10= pose_in_map.pose.position.x -  active_plan_[min_idx].pose.position.x;
            d11= pose_in_map.pose.position.y -  active_plan_[min_idx].pose.position.y;
            r=v00*d10+v01*d11;
            if (r<=modd0){
              auto l_front = path_list.begin();
              std::advance(l_front, min_idx);
              resp.v1[0]=active_plan_[min_idx].pose.position.x;
              resp.v1[1]=active_plan_[min_idx].pose.position.y;
              resp.v1[2]=*l_front;
              resp.v2[0]=active_plan_[min_idx+1].pose.position.x;
              resp.v2[1]=active_plan_[min_idx+1].pose.position.y;
              ++l_front;
              resp.v2[2]=*l_front;
            } else {
              auto l_front = path_list.begin();
              std::advance(l_front, min_idx-1);
              resp.v1[0]=active_plan_[min_idx-1].pose.position.x;
              resp.v1[1]=active_plan_[min_idx-1].pose.position.y;
              resp.v1[2]=*l_front;
              resp.v2[0]=active_plan_[min_idx].pose.position.x;
              resp.v2[1]=active_plan_[min_idx].pose.position.y;
              ++l_front;
              resp.v2[2]=*l_front;

            }
            


          }
        }
    }
    return true;
  }

  void GraphPlanner::odometryCb(nav_msgs::Odometry msg)
  {
    latest_odom_ = msg;
  }


  float GraphPlanner::getyawfromquaternion(const geometry_msgs::Quaternion msg)
  {
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    return yaw;


  }

  bool GraphPlanner::addEdgeLOS(Vx src,bool start,const geometry_msgs::PoseStamped &pose, float los_factor)
  {
    int nbr_count = 4;
    int idx_check = 0;
    std::vector<Vx> src_nbrs;

    bool found_direct_nbr = false;
    int count = 0;
    while (!found_direct_nbr)
    {
      src_nbrs.clear();
      gm_.getKNN(src, nbr_count+1, src_nbrs); // plus one because the query is counted

      std::cout << "neighbours of " << src << ": ";
      for (int j = 0; j < src_nbrs.size(); j++)
       {
         std::cout << src_nbrs[j] << ", ";
       }
      std::cout << std::endl;
      std::vector<int> disindx;
      std::vector<int> angindx;
      std::vector<float> dis;
      std::vector<float> angdiff;
      int angle_count=0;
      for (int i = idx_check+1; i < src_nbrs.size(); i++)
      {
        VxBundle vx_src, vx_tgt;
        //ROS_INFO_STREAM("source \t"<<src);
        //ROS_INFO_STREAM("src_nbrs \t"<<src_nbrs[i]);
        vx_src = gm_.getBundle(src);
        vx_tgt = gm_.getBundle(src_nbrs[i]);
        //ROS_INFO_STREAM("vx_src \t"<<src);
        //ROS_INFO_STREAM("vx_tgt \t"<<src_nbrs[i]);
        geometry_msgs::PoseStamped los_src, los_tgt;
        los_src = bundle2pose(vx_src);
        los_tgt = bundle2pose(vx_tgt);
        los_src.header.frame_id = "map";
        los_tgt.header.frame_id = "map";

        bool los = checkLOS(los_src, los_tgt, los_factor);
        if (los)
        {
          if (start && src_nbrs.size() >1)
          {
            float dx, dy;
            dx = los_tgt.pose.position.x - los_src.pose.position.x;
            dy = los_tgt.pose.position.y - los_src.pose.position.y;
            float th = atan2(dy, dx);
            geometry_msgs::Pose outpose;
            outpose.orientation.x = 0.0;
            outpose.orientation.y = 0.0;
            outpose.orientation.z = sin(0.5*th);
            outpose.orientation.w = cos(0.5*th);
            float yawsrctgt=getyawfromquaternion(outpose.orientation);
            float yawsrc=getyawfromquaternion(pose.pose.orientation);
            float yawdiff = yawsrctgt-yawsrc;
            if (yawdiff < 0 )
              yawdiff = yawdiff*-1;
            yawdiff= yawdiff*180/M_PI;
            yawdiff = 360 - yawdiff;
            if (yawdiff < 0 )
              yawdiff = yawdiff*-1;            
            if (yawdiff<=90 || yawdiff>=270){
              disindx.push_back(i);
              dis.push_back(getPoseDistance(los_src,los_tgt));
            } else {
              angdiff.push_back((yawdiff));
              angindx.push_back(i);
            }
          } else {
              disindx.push_back(i);
              dis.push_back(getPoseDistance(los_src,los_tgt));
          }
          //gm_.addEdge(src, src_nbrs[i]);
          //gm_.addEdge(src_nbrs[i], src);
          found_direct_nbr = true;
        }
      }
      float first, second;
      first = second = FLT_MAX;  
      int ind1,ind2;
      if (dis.size()>0) {
        for (int k=0;k<dis.size();k++)
          {
            if (dis[k] < first)  
            {  
           // second = first;  
            first = dis[k];
            ind1=k;  
            }  
           // else if (dis[k] < second && dis[k] != first)  
           //   second = dis[k];
           //   ind2=k;  
          }  
        gm_.addEdge(src, src_nbrs[disindx[ind1]]);
        gm_.addEdge(src_nbrs[disindx[ind1]], src);
      } else {
        for (int k=0;k<angdiff.size();k++)
          {
            if (angdiff[k] < first)  
            {  
           // second = first;  
            first = angdiff[k];
            ind1=k;  
            }  
           // else if (dis[k] < second && dis[k] != first)  
           //   second = dis[k];
           //   ind2=k;  
          }  
        gm_.addEdge(src, src_nbrs[angindx[ind1]]);
        gm_.addEdge(src_nbrs[angindx[ind1]], src);       
      }


      if (nbr_count == gm_.size())
        break;

      idx_check = nbr_count;
      nbr_count *= 2;
      if (nbr_count >= gm_.size())
        nbr_count = gm_.size();
      count ++;
    }
    return found_direct_nbr;
  }
}; // namespace graph_planner
