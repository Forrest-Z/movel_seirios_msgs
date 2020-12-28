#ifndef VELOCITY_LIMITER_YAML_H
#define VELOCITY_LIMITER_YAML_H

#include "yaml-cpp/yaml.h"
#include <yaml_utils/yaml_utils.h>
#include <velocity_limiter/common.h>

/**
 * Classes and functions used by yaml-cpp.
 */
namespace YAML
{
/**
 * Provide ways to convert between Shape object and yaml node.
 */
template <>
struct convert<Shape>
{
  static Node encode(const Shape& shape)
  {
    Node node;
    std::vector<double> x_list;
    std::vector<double> y_list;
    for (int i = 0; i < shape.x_list.size(); ++i)
      x_list.push_back(shape.x_list[i]);
    for (int i = 0; i < shape.y_list.size(); ++i)
      x_list.push_back(shape.y_list[i]);
    node["x_coordinates"] = (x_list);
    node["y_coordinates"] = (y_list);
    return node;
  }

  static bool decode(const Node& node, Shape& shape)
  {
    if (!node.IsMap())
    {
      ROS_FATAL("Invalid shape format: node is not map");
      return false;
    }

    if (node.size() != 2)
    {
      ROS_FATAL("Invalid shape format: node size is not 2");
      return false;
    }

    std::vector<double> x_list;
    std::vector<double> y_list;

    for (int i = 0; i < node["x_coordinates"].size(); ++i)
      x_list.push_back(node["x_coordinates"][i].as<double>());
    for (int i = 0; i < node["y_coordinates"].size(); ++i)
      y_list.push_back(node["y_coordinates"][i].as<double>());

    shape.x_list = x_list;
    shape.y_list = y_list;

    return true;
  }
};

/**
 * Provide ways to convert between Frontier object and yaml node.
 */
template <>
struct convert<Frontier>
{
  static Node encode(const Frontier& frontier)
  {
    Node node;
    node["inclusion"] = (frontier.inclusion);
    node["value"] = (frontier.value);
    node["shape"] = (frontier.shape);
    return node;
  }

  static bool decode(const Node& node, Frontier& frontier)
  {
    if (!node.IsMap())
    {
      ROS_FATAL("Invalid frontier format: node is not map");
      return false;
    }

    if (node.size() != 3)
    {
      ROS_FATAL("Invalid frontier format: node size is not 3");
      return false;
    }

    frontier.inclusion = node["inclusion"].as<int>();
    frontier.value = node["value"].as<double>();
    frontier.shape = node["shape"].as<Shape>();
    return true;
  }
};

/**
 * Provide ways to convert between Zone object and yaml node.
 */
template <>
struct convert<Zone>
{
  static Node encode(const Zone& zone)
  {
    Node node;
    std::vector<Frontier> frontier_list;
    for (int i = 0; i < zone.frontier_list.size(); ++i)
      node["frontiers"][i] = (zone.frontier_list[i]);
    node["type"] = (zone.type);
    node["direction"] = (zone.direction);
    node["dimension"] = (zone.dimension);
    // node["frontiers"] = (frontier_list);

    return node;
  }

  static bool decode(const Node& node, Zone& zone)
  {
    if (!node.IsMap())
    {
      ROS_FATAL("Invalid zone format: node is not map");
      return false;
    }

    if (node.size() != 4)
    {
      ROS_FATAL("Invalid zone format: node size is not 4");
      return false;
    }

    std::vector<Frontier> frontier_list;

    for (YAML::const_iterator it = node["frontiers"].begin(); it != node["frontiers"].end(); ++it)
    {
      const YAML::Node& frontier = *it;
      frontier_list.push_back(frontier.as<Frontier>());
    }
    zone.type = node["type"].as<std::string>();
    zone.direction = node["direction"].as<std::string>();
    zone.dimension = node["dimension"].as<std::string>();
    zone.frontier_list = frontier_list;
    return true;
  }
};

/**
 * Provide ways to convert between Set object and yaml node.
 */
template <>
struct convert<Set>
{
  static Node encode(const Set& set)
  {
    Node node;
    std::vector<Zone> zone_list;
    for (int i = 0; i < set.zone_list.size(); ++i)
      zone_list.push_back(set.zone_list[i]);
    node["zones"] = zone_list;
    return node;
  }

  static bool decode(const Node& node, Set& set)
  {
    if (!node.IsMap())
    {
      ROS_FATAL("Invalid set format: node is not map");
      return false;
    }

    if (node.size() != 1)
    {
      ROS_FATAL("Invalid set format: node size is not 1");
      return false;
    }
    std::vector<Zone> zone_list;
    for (YAML::const_iterator it = node["zones"].begin(); it != node["zones"].end(); ++it)
    {
      const YAML::Node& zone = *it;
      zone_list.push_back(zone.as<Zone>());
    }

    set.zone_list = zone_list;
    return true;
  }
};

}  // namespace YAML

#endif
