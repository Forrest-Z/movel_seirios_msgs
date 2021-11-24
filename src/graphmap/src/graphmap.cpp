#include "graphmap/graphmap.hpp"

using std::cout;
using std::endl;

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

geometry_msgs::PoseStamped bundle2pose(VxBundle bundle)
{
  geometry_msgs::PoseStamped outpose;
  outpose.pose.position.x = bundle.x;
  outpose.pose.position.y = bundle.y;
  outpose.pose.position.z = bundle.z;


  return outpose;
}

GraphMap::GraphMap() : vx_cloud_(new Cloud), octree_(0.05f)
{
  ros::NodeHandle local_nh;
  local_nh.param("/move_base/GraphPlanner/decimation_factor", decimation_factor, 1.2);
  cout << "graph map initialised" << endl;
}

bool GraphMap::parseCsv(std::string fpath)
{
  cout << "parsing map from " << fpath.c_str() << endl;
  std::ifstream f(fpath.c_str());
  if (!f.good()){
    return false;
  }
  std::fstream map_file;
  map_file.open(fpath, std::ios::in);

  std::string row, col;
  bool vx_mode = true;
  std::map<int, Vx> vertex_map;
  graphmap_.clear();
  vx_cloud_->points.clear();
  while (!map_file.eof())
  {
    std::getline(map_file, row);
    std::stringstream s(row);

    std::vector<std::string> rowvec;
    while (std::getline(s, col, ','))
    {
      rowvec.push_back(col);
    }

    if (rowvec.size() < 1)
      continue;

    if (rowvec[0] == "vertices")
    {
      cout << "begin parsing vertices" << endl;
      continue;
    }
    else if (rowvec[0] == "edges")
    {
      cout << "begin parsing edges" << endl;
      vx_mode = false;
      continue;
    }
    else if (rowvec[0] == "---")
    {
      cout << "---" << endl;
      // cout << "index map" << endl;
      // std::map<int, Vx>::iterator map_it;
      // map_it = vertex_map.begin();
      // for (; map_it != vertex_map.end(); map_it++)
      // {
      //   cout << map_it->first << ", " << map_it->second << endl;
      // }
      // cout << "---" << endl;
      continue;
    }

    if (vx_mode) 
    {
      // convert string from csv to appropriate types
      int idx;
      float x, y, z;
      idx = std::stoi(rowvec[0]);
      x = std::stof(rowvec[1]);
      y = std::stof(rowvec[2]);
      z = std::stof(rowvec[3]);

      // add vertex to graph, fill in properties
      Vx vx_i = add_vertex(graphmap_);
      graphmap_[vx_i].x = x;
      graphmap_[vx_i].y = y;
      graphmap_[vx_i].z = z;

      vertex_map[idx] = vx_i;

      // add vertex to point cloud, map cloud index to graph vertex descriptor
      Pt pt_i;
      pt_i.x = x;
      pt_i.y = y;
      pt_i.z = z;
      vx_cloud_->points.push_back(pt_i);
      int idx_pt_i = vx_cloud_->points.size() - 1;
      map_cloud2graph_[idx_pt_i] = vx_i;

      // for (int i = 0; i < rowvec.size(); i++)
      // {
      //   cout << rowvec[i] << ", ";  
      // }
      // cout << endl;
    }
    else
    {
      int idx_src, idx_tgt;
      std::string recip = "";

      idx_src = std::stoi(rowvec[0]);
      idx_tgt = std::stoi(rowvec[1]);
      recip = rowvec[2];

      Vx src, tgt;
      src = vertex_map[idx_src];
      tgt = vertex_map[idx_tgt];
      Eg eg_i;
      bool added = false;
      // cout << idx_src << ", " << idx_tgt << endl;
      // cout << src << ", " << tgt << ", " << recip << endl;
      //std::vector<geometry_msgs::PoseStamped> plan_check;
      //navfn_planner_.makePlan(bundle2pose(graphmap_[src]), bundle2pose(graphmap_[tgt]), plan_check);
      geometry_msgs::PoseStamped los_src, los_tgt;
      los_src = bundle2pose(graphmap_[src]);
      los_tgt = bundle2pose(graphmap_[tgt]);
      los_src.header.frame_id = "map";
      los_tgt.header.frame_id = "map";

      bool los_check = checkLOS(los_src, los_tgt);
      if(los_check)
      {
        float dis_cal;
        bool check_vertex = vertex_decimation_check(graphmap_[src],graphmap_[tgt],dis_cal);
        if (check_vertex)
        {
          Vx vx_i = add_vertex(graphmap_);
          graphmap_[vx_i].x = (graphmap_[src].x+ graphmap_[tgt].x)/2;
          graphmap_[vx_i].y = (graphmap_[src].y+ graphmap_[tgt].y)/2;
          graphmap_[vx_i].z = (graphmap_[src].z+ graphmap_[tgt].z)/2;
          int indx_mid=vertex_map.size();
          vertex_map[indx_mid] = vx_i;
          Pt pt_i;
          pt_i.x = (graphmap_[src].x+ graphmap_[tgt].x)/2;
          pt_i.y = (graphmap_[src].y+ graphmap_[tgt].y)/2;
          pt_i.z = (graphmap_[src].z+ graphmap_[tgt].z)/2;
          vx_cloud_->points.push_back(pt_i);
          int idx_pt_i = vx_cloud_->points.size() - 1;
          map_cloud2graph_[idx_pt_i] = vx_i;
          tie(eg_i, added) = add_edge(src, vx_i, graphmap_);
          graphmap_[eg_i].length = dis_cal/2;
          graphmap_[eg_i].weight = 1.0;
          tie(eg_i, added) = add_edge(vx_i, tgt, graphmap_);
          graphmap_[eg_i].length = dis_cal/2;
          graphmap_[eg_i].weight = 1.0;
          if (recip == "r")
          {
              tie(eg_i, added) = add_edge(tgt, vx_i, graphmap_);
              graphmap_[eg_i].length = dis_cal/2;
              graphmap_[eg_i].weight = 1.0;
              tie(eg_i, added) = add_edge(vx_i, src, graphmap_);
              graphmap_[eg_i].length = dis_cal/2;
              graphmap_[eg_i].weight = 1.0;
          }

        } else 
          {
            tie(eg_i, added) = add_edge(src, tgt, graphmap_);
            graphmap_[eg_i].length = dis_cal;
            graphmap_[eg_i].weight = 1.0;

            if (recip == "r")
            {
              tie(eg_i, added) = add_edge(tgt, src, graphmap_);
              graphmap_[eg_i].length = dis_cal;
              graphmap_[eg_i].weight = 1.0;
            }
          }
      }
      
      // for (int i = 0; i < rowvec.size(); i++)
      // {
      //   cout << rowvec[i] << ", ";
      // }
      // cout << endl;
    }
  }
  octree_=0.05f;
  // setup initial octree
  octree_.setInputCloud(vx_cloud_);
  octree_.addPointsFromInputCloud();

  cout << "initial cloud" << endl;
  for (int i = 0; i < vx_cloud_->points.size(); i++)
  {
    cout << i << ": " << vx_cloud_->points[i] << endl;
  }
 // printMap();
  return true;
}


bool GraphMap::vertex_decimation_check(VxBundle bundle1, VxBundle bundle2, float &dd)
{
  geometry_msgs::PoseStamped outpose1,outpose2;
  outpose1.pose.position.x = bundle1.x;
  outpose1.pose.position.y = bundle1.y;
  outpose1.pose.position.z = bundle1.z;

  float dx, dy, dz;
  dx =  bundle1.x -  bundle2.x;
  dy =  bundle1.y -  bundle2.y;
  dz =  bundle1.z -  bundle2.z;
  dd = sqrt(dx*dx + dy*dy + dz*dz);
  if (dd>decimation_factor)
    return true;
  else 
    return false;
  
}



bool GraphMap::checkLOS(const geometry_msgs::PoseStamped &src,
                              const geometry_msgs::PoseStamped &tgt,
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
    if (dnav/dd >los_factor)
      return false;
    return true;
  }

void GraphMap::printMap()
{
  cout << "here is the current state of the graph map" << endl;
  Graph::vertex_iterator vx_it, vx_ed;
  tie(vx_it, vx_ed) = vertices(graphmap_);
  for (; vx_it != vx_ed; vx_it++)
  {
    cout << *vx_it << ": ";
    Graph::out_edge_iterator eg_it, eg_ed;
    tie(eg_it, eg_ed) = out_edges(*vx_it, graphmap_);
    for (; eg_it != eg_ed; eg_it++)
    {
      cout << *eg_it << ", ";
    }
    cout << endl;
  }
}

bool GraphMap::findPath(Vx src, Vx tgt, std::vector<Vx>& path,std::list<Vx>& path_list_1)
{
  path.clear();
  path_list_1.clear();
  cout << "searching path from " << src << " to " << tgt << endl;
  // make heuristic
  Heuristic h(graphmap_, tgt);
  std::vector<Vx> preds(num_vertices(graphmap_));
  AStarVisitor vis(tgt);

  try
  {
    astar_search(graphmap_, src, h,
                predecessor_map(make_iterator_property_map(preds.begin(), get(vertex_index, graphmap_)))
                .weight_map(get(&EgBundle::length, graphmap_))
                .visitor(vis)
                );
  }
  catch (FoundGoal fg)
  {
    cout << "found path to goal! " << endl;

    // construct path sequence in list (because we want push_front)
    std::list<Vx> path_list;
    Vx vx = tgt;
    while (true)
    {
      path_list.push_front(vx);
      if (vx == preds[vx])
        break;
      vx = preds[vx];
    }
    // move path sequence to vector
    std::list<Vx>::iterator path_it = path_list.begin();
    for(; path_it != path_list.end(); path_it++)
    {
      path.push_back(*path_it);
    }

    // DEBUG: print path
    cout << "path length " << path_list.size() << endl;
    cout << src;
    path_it = path_list.begin();
    ++path_it;
    for (; path_it != path_list.end(); path_it++)
    {
      cout << " --> " << *path_it;
    }
    cout << endl;
    path_list_1 = path_list;

    return true;
  }
  return false;
}

bool GraphMap::findPath(float x0, float y0, float z0,
                        float x1, float y1, float z1, std::vector<Vx>& path,std::list<Vx>& path_list)
{
  // make source vertex
  // add source vertex to graph
  Vx src = addVertex(x0, y0, z0, 2);

  // make target vertex
  // add target vertex to graph
  Vx tgt = addVertex(x1, y1, z1, 2);

  // perform planning

  bool success = findPath(src, tgt, path,path_list);

  // remove temporary vertices
  // important to remove target and source in the *reverse* order they were added, 
  // because they are just indices, and if they are removed in the order they were added,
  // then the latter removal will be an out-of-bound index
  removeVertex(tgt);
  removeVertex(src);

  // report result
  return success;
}

bool GraphMap::findPath(float x0, float y0, float z0,
                        float x1, float y1, float z1,
                        std::vector< std::vector<float> >& path,std::list<Vx>& path_list)
{
  std::vector<Vx> path_graph;
  bool success = findPath(x0, y0, z0, x1, y1, z1, path_graph,path_list);
  if (success)
  {
    for (int i = 0; i < path_graph.size(); i++)
    {
      Vx vx_i = path_graph[i];
      std::vector<float> wp_i(3, 0.0);
      wp_i[0] = graphmap_[vx_i].x;
      wp_i[1] = graphmap_[vx_i].y;
      wp_i[2] = graphmap_[vx_i].z;

      path.push_back(wp_i);
    }
    return true;
  }
  return false;
}

// convert path in vertex descriptors to path in euclidean coordinates
void GraphMap::pathVx2Euclid(std::vector<Vx>& vx_path, 
                             std::vector< std::vector<float> >& ec_path)
{
  for (int i = 0; i < vx_path.size(); i++)
  {
    std::vector<float> path_i (3, 0.0);
    path_i[0] = graphmap_[vx_path[i]].x;
    path_i[1] = graphmap_[vx_path[i]].y;
    path_i[2] = graphmap_[vx_path[i]].z;

    ec_path.push_back(path_i);
  }
}

size_t GraphMap::size()
{
  return num_vertices(graphmap_);
}

Vx GraphMap::addVertex(float x, float y, float z, int k_connect)
{
  // add new vertex to graph
  Vx new_vx = add_vertex(graphmap_);
  graphmap_[new_vx].x = x;
  graphmap_[new_vx].y = y;
  graphmap_[new_vx].z = z;

  Pt pt_qry (x, y, z);
  // std::cout << "adding vertex " << pt_qry << std::endl;
  if (k_connect > 0)
  {
    // find k nearest neighbours in cloud
    std::vector<int> knn_indices;
    std::vector<float> knn_distances;
    if (octree_.nearestKSearch(pt_qry, k_connect, knn_indices, knn_distances))
    {
      // std::cout << k_connect << " nearest neighbours: " << std::endl;
      for (int i = 0; i < knn_indices.size(); i++)
      {
        // get vertex descriptor of neighbour
        Vx vx_nb = map_cloud2graph_[knn_indices[i]];

        // add edge between new vertex to neighbour
        add_edge(new_vx, vx_nb, graphmap_);
        add_edge(vx_nb, new_vx, graphmap_);

        // std::cout << knn_indices[i] << ", " << vx_nb << ", ";
        // std::cout << vx_cloud_->points[knn_indices[i]] << ", ";
        // std::cout << knn_distances[i] << std::endl;
      }    
    }
  }

  // add query point to octree (and cloud)
  octree_.addPointToCloud(pt_qry, vx_cloud_);
  // std::cout << "new cloud size " << vx_cloud_->points.size() << std::endl;
  int new_idx = vx_cloud_->points.size() - 1;
  map_cloud2graph_[new_idx] = new_vx;

  return new_vx;
}

void GraphMap::removeVertex(Vx vx)
{
  // remove edges
  clear_vertex(vx, graphmap_);
  clear_in_edges(vx, graphmap_);
  clear_out_edges(vx, graphmap_);

  // remove vertex
  remove_vertex(vx, graphmap_);

  // remove from octree/cloud
  Pt pt;
  pt.x = graphmap_[vx].x;
  pt.y = graphmap_[vx].y;
  pt.z = graphmap_[vx].z;
  
  std::vector<int> knn_indices;
  std::vector<float> knn_distances;
  if (octree_.nearestKSearch(pt, 1, knn_indices, knn_distances))
  {
    if (knn_distances[0] <= octree_.getResolution())
    {
      int pt_idx = knn_indices[0];
      octree_.deleteVoxelAtPoint(pt_idx);
      vx_cloud_->points.erase(vx_cloud_->points.begin() + pt_idx);
    }
  }
}

Eg GraphMap::addEdge(Vx src, Vx tgt)
{
  Eg new_eg;
  bool added = false;
  tie(new_eg, added) = add_edge(src, tgt, graphmap_);
  graphmap_[new_eg].length = getVxDistance(src, tgt);
  graphmap_[new_eg].weight = 1.0;

  return new_eg;
}

double GraphMap::getVxDistance(Vx src, Vx tgt)
{
  double x0, y0, z0, x1, y1, z1;
  x0 = graphmap_[src].x;
  y0 = graphmap_[src].y;
  z0 = graphmap_[src].z;

  x1 = graphmap_[tgt].x;
  y1 = graphmap_[tgt].y;
  z1 = graphmap_[tgt].z;

  double dx, dy, dz;
  dx = x1 - x0;
  dy = y1 - y0;
  dz = z1 - z0;

  return sqrt(dx*dx + dy*dy + dz*dz);
}

void GraphMap::getVertices(std::vector<VxBundle> &out_vertices)
{
  Graph::vertex_iterator vx_it, vx_end;
  tie(vx_it, vx_end) = vertices(graphmap_);
  out_vertices.clear();
  for (; vx_it != vx_end; vx_it++)
  {
    VxBundle vx_i;
    vx_i.x = graphmap_[*vx_it].x;
    vx_i.y = graphmap_[*vx_it].y;
    vx_i.z = graphmap_[*vx_it].z;
    out_vertices.push_back(vx_i);
  }
}

void GraphMap::getEdgesEuclidean(std::vector< std::vector<VxBundle> > &out_edges)
{
  Graph::edge_iterator eg_it, eg_end;
  tie(eg_it, eg_end) = edges(graphmap_);
  for (; eg_it != eg_end; eg_it++)
  {
    std::vector<VxBundle> endpoints;
    VxBundle pt_src, pt_tgt;
    Vx src, tgt;
    src = source(*eg_it, graphmap_);
    tgt = target(*eg_it, graphmap_);

    pt_src.x = graphmap_[src].x;
    pt_src.y = graphmap_[src].y;
    pt_src.z = graphmap_[src].z;

    pt_tgt.x = graphmap_[tgt].x;
    pt_tgt.y = graphmap_[tgt].y;
    pt_tgt.z = graphmap_[tgt].z;

    endpoints.push_back(pt_src);
    endpoints.push_back(pt_tgt);

    out_edges.push_back(endpoints);
  }
}

// get k nearest vertices
void GraphMap::getKNN(Vx qry, int k, std::vector<Vx> &neighbours)
{
  float x, y, z;
  x = graphmap_[qry].x;
  y = graphmap_[qry].y;
  z = graphmap_[qry].z;

  Pt pt_qry (x, y, z);

  std::vector<int> knn_indices;
  std::vector<float> knn_distances;
  if (octree_.nearestKSearch(pt_qry, k, knn_indices, knn_distances))
  {
    // std::cout << "getKNN, asked " << k << " got " << knn_indices.size() << std::endl;
    for (int i = 0; i < knn_indices.size(); i++)
    {
      neighbours.push_back(map_cloud2graph_[knn_indices[i]]);
    }
  }
}

void GraphMap::init_navfn(costmap_2d::Costmap2DROS *costmap_ros){
  navfn_planner_.initialize("LOS_val_Planner", costmap_ros);
}

VxBundle GraphMap::getBundle(Vx qry)
{
  return graphmap_[qry];
}

EgBundle GraphMap::getBundle(Eg qry)
{
  return graphmap_[qry];
}