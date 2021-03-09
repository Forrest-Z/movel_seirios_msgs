#ifndef graphmap_h
#define graphmap_h

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <fstream>
#include <iostream>
#include <pcl/octree/octree_search.h>
#include <navfn/navfn_ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

using namespace boost;

struct VxBundle
{
  float x;
  float y;
  float z;
};

struct EgBundle
{
  float length;
  float weight;
};

typedef adjacency_list<vecS, vecS, bidirectionalS, VxBundle, EgBundle> Graph;
typedef graph_traits<Graph>::vertex_descriptor Vx;
typedef graph_traits<Graph>::edge_descriptor Eg;

typedef pcl::PointXYZ Pt;
typedef pcl::PointCloud<Pt> Cloud;

class Heuristic : public astar_heuristic<Graph, float>
{
public:
  Heuristic(Graph& g, Vx goal): g_(g), goal_(goal) {}
  float operator() (Vx vx)
  {
    double dx, dy, dz, dd;
    dx = g_[vx].x - g_[goal_].x;
    dy = g_[vx].y - g_[goal_].y;
    dz = g_[vx].z - g_[goal_].z;

    dd = sqrt(dx*dx + dy*dy + dz*dz);

    return dd;
  }

private:
  Vx goal_;
  Graph g_;
};

struct FoundGoal {}; // thrown when A* finds the goal

class AStarVisitor : public default_astar_visitor
{
public:
  AStarVisitor(Vx goal) : goal_(goal) {}
  void examine_vertex(Vx vx, const Graph& g)
  {
    if (vx == goal_)
      throw FoundGoal();
  }

private:
  Vx goal_;
};

class GraphMap
{
private:
  Graph graphmap_; // underlying graph structure
  Cloud::Ptr vx_cloud_; // point cloud structure for nearest neighbour (euclidean)
  pcl::octree::OctreePointCloudSearch<Pt> octree_;
  std::map<int, Vx> map_cloud2graph_; // mapping between cloud index to graph index
  navfn::NavfnROS navfn_planner_;
  double decimation_factor;

public:
  GraphMap();

  bool parseCsv(std::string fpath);

  void printMap();

  bool findPath(Vx src, Vx tgt, std::vector<Vx>& path,std::list<Vx>& path_list);

  bool findPath(float x0, float y0, float z0,
                float x1, float y1, float z1,
                std::vector<Vx>& path, std::list<Vx>& path_list);

  bool findPath(float x0, float y0, float z0,
                float x1, float y1, float z1,
                std::vector< std::vector<float> >& path, std::list<Vx>& path_list);

  void pathVx2Euclid(std::vector<Vx>& vx_path, 
                     std::vector< std::vector<float> >& ec_path);

  size_t size();

  Vx addVertex(float x, float y, float z, int k_connect = 0);

  void removeVertex(Vx vx);

  Eg addEdge(Vx src, Vx tgt);

  double getVxDistance(Vx src, Vx tgt);

  void getVertices(std::vector<VxBundle> &out_vertices);

  void getEdgesEuclidean(std::vector< std::vector<VxBundle> > &out_eges);

  void getKNN(Vx qry, int k, std::vector<Vx> &neighbours);

  VxBundle getBundle(Vx qry);

  EgBundle getBundle(Eg qry);

  void init_navfn(costmap_2d::Costmap2DROS *costmap_ros);
  bool vertex_decimation_check(VxBundle bundle1, VxBundle bundle2, float &dd);

  bool checkLOS(const geometry_msgs::PoseStamped &src,
                const geometry_msgs::PoseStamped &tgt,
                float los_factor=1.5);
};

#endif