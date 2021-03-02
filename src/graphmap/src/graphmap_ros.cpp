#include <graphmap/graphmap.hpp>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

class GraphMapROS
{
public:
  GraphMapROS()
  {
    // parameters
    std::string graph_def;
    ros::param::param<std::string>("~graph_def", graph_def, "");
    if (graph_def.size() < 1)
    {
      ROS_INFO("Bad graph definition file. Try again.");
      return;
    }
    gm_.parseCsv(graph_def);
    gm_.printMap();

    // topics
    vis_pub_ = nh_.advertise<visualization_msgs::Marker>("map_graph", 1000);

    // visuals
    visGraph();

    // go!
    ros::spin();
  }

  ~GraphMapROS()
  {
  }

  void visGraph()
  {
    while (vis_pub_.getNumSubscribers() < 1)
    {
      if (!ros::ok)
        return;
      sleep(1);
    }

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
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    marker.points.clear();

    std::vector<VxBundle> map_vertices;
    gm_.getVertices(map_vertices);
    for (int i = 0; i < map_vertices.size(); i++)
    {
      // geometry_msgs::Point pt_i;
      // pt_i.x = map_vertices[i].x;
      // pt_i.y = map_vertices[i].y;
      // pt_i.z = map_vertices[i].z;

      // marker.points.push_back(pt_i);

      marker.pose.position.x = map_vertices[i].x;
      marker.pose.position.y = map_vertices[i].y;
      marker.pose.position.z = map_vertices[i].z;
      vis_pub_.publish(marker);
      
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
    eg_marker.color.a = 1.0;
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

      vis_pub_.publish(eg_marker);
      eg_marker.points.clear();
      eg_marker.id += 1;
    }
  }

private:
  ros::NodeHandle nh_;
  GraphMap gm_;

  ros::Publisher vis_pub_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "graphmap_ros_node");
  GraphMapROS gm;

  return 0;
}