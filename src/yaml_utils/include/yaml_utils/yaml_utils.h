#ifndef YAML_UTILS_H
#define YAML_UTILS_H

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

/*
 * test
 */
XmlRpc::XmlRpcValue loadFromParamServer(std::string& param_path)
{
  XmlRpc::XmlRpcValue xml;
  if (!ros::param::has(param_path))
  {
    ROS_ERROR_STREAM(param_path << " is not a valid path in the Parameter Server");
    return XmlRpc::XmlRpcValue::TypeInvalid;
  }
  ros::param::get(param_path, xml);
  return xml;
}

// Forward Declarations
YAML::Node parseArray(XmlRpc::XmlRpcValue& xml, std::string& name);
YAML::Node parseStruct(XmlRpc::XmlRpcValue& xml, std::string& name);
YAML::Node parseInt(XmlRpc::XmlRpcValue& xml, std::string& name);
YAML::Node parseDouble(XmlRpc::XmlRpcValue& xml, std::string& name);
YAML::Node parseString(XmlRpc::XmlRpcValue& xml, std::string& name);
YAML::Node parseBool(XmlRpc::XmlRpcValue& xml, std::string& name);
YAML::Node parseBase64(XmlRpc::XmlRpcValue& xml, std::string& name);
YAML::Node parseDateTime(XmlRpc::XmlRpcValue& xml, std::string& name);

YAML::Node parseArray(XmlRpc::XmlRpcValue& xml, std::string& name)
{
  YAML::Node yaml;
  for (int i = 0; i < xml.size(); ++i)
  {
    YAML::Node element;
    switch (xml[i].getType())
    {
      case 0:
        ROS_FATAL("%s", "XML of TypeInvalid detected. Exiting");
        break;
      case 1:
        element = parseBool(xml[i], name);
        break;
      case 2:
        element = parseInt(xml[i], name);
        break;
      case 3:
        element = parseDouble(xml[i], name);
        break;
      case 4:
        element = parseString(xml[i], name);
        break;
      case 5:
        element = parseDateTime(xml[i], name);
        break;
      case 6:
        element = parseBase64(xml[i], name);
        break;
      case 7:
        element = parseArray(xml[i], name);
        break;
      case 8:
        element = parseStruct(xml[i], name);
        break;
      default:
        ROS_FATAL("%s", "Unrecognized XML type detected. Exiting");
        break;
    }
    yaml.push_back(element);
  }
  return yaml;
}

YAML::Node parseStruct(XmlRpc::XmlRpcValue& xml, std::string& name)
{
  YAML::Node yaml;

  XmlRpc::XmlRpcValue::iterator iter;
  for (iter = xml.begin(); iter != xml.end(); ++iter)
  {
    YAML::Node branch;
    std::string key = iter->first;
    XmlRpc::XmlRpcValue value = iter->second;
    switch (value.getType())
    {
      case 0:
        ROS_FATAL("%s", "XML of TypeInvalid detected. Exiting");
        break;
      case 1:
        branch = parseBool(value, key);
        break;
      case 2:
        branch = parseInt(value, key);
        break;
      case 3:
        branch = parseDouble(value, key);
        break;
      case 4:
        branch = parseString(value, key);
        break;
      case 5:
        branch = parseDateTime(value, key);
        break;
      case 6:
        branch = parseBase64(value, key);
        break;
      case 7:
        branch = parseArray(value, key);
        break;
      case 8:
        branch = parseStruct(value, key);
        break;
      default:
        ROS_FATAL("%s", "Unrecognized XML type detected. Exiting");
        break;
    }
    yaml[key] = branch;
  }
  return yaml;
}

YAML::Node parseString(XmlRpc::XmlRpcValue& xml, std::string& name)
{
  YAML::Node yaml;
  yaml = static_cast<std::string>(xml);
  return yaml;
}

YAML::Node parseDouble(XmlRpc::XmlRpcValue& xml, std::string& name)
{
  YAML::Node yaml;
  // TODO: Unsure if check needs to be done for int->double. Current implementation throws the error somewhere else.
  yaml = static_cast<double>(xml);
  return yaml;
}

YAML::Node parseInt(XmlRpc::XmlRpcValue& xml, std::string& name)
{
  YAML::Node yaml;
  yaml = static_cast<int>(xml);
  return yaml;
}

YAML::Node parseBool(XmlRpc::XmlRpcValue& xml, std::string& name)
{
  YAML::Node yaml;
  yaml = static_cast<bool>(xml);
  return yaml;
}

YAML::Node parseDateTime(XmlRpc::XmlRpcValue& xml, std::string& name)
{
  YAML::Node yaml;
  ROS_FATAL("%s", "XML of TypeDateTime is currently unsupported.");
  ROS_INFO("%s", "Feel free to implement support for TypeDateTime");
  return yaml;
}

YAML::Node parseBase64(XmlRpc::XmlRpcValue& xml, std::string& name)
{
  YAML::Node yaml;
  ROS_FATAL("%s", "XML of TypeBase64 is currently unsupported.");
  ROS_INFO("%s", "Feel free to implement support for TypeBase64");
  return yaml;
}

namespace YamlUtils
{
YAML::Node xmlToYaml(XmlRpc::XmlRpcValue& xml, std::string& name)
{
  YAML::Node yaml;
  switch (xml.getType())
  {
    case 0:
      ROS_FATAL("%s", "XML of TypeInvalid detected. Exiting");
      break;
    case 1:
      yaml[name] = parseBool(xml, name);
      break;
    case 2:
      yaml[name] = parseInt(xml, name);
      break;
    case 3:
      yaml[name] = parseDouble(xml, name);
      break;
    case 4:
      yaml[name] = parseString(xml, name);
      break;
    case 5:
      yaml[name] = parseDateTime(xml, name);
      break;
    case 6:
      yaml[name] = parseBase64(xml, name);
      break;
    case 7:
      yaml[name] = parseArray(xml, name);
      break;
    case 8:
      yaml[name] = parseStruct(xml, name);
      break;
    default:
      ROS_FATAL("%s", "Unrecognized XML type detected. Exiting");
      break;
  }
  return yaml;
}

template <typename T>
T required(std::string param_path)
{
  // Cleaning input string
  std::size_t last_demarker = param_path.find_last_of("/");
  if (last_demarker == param_path.size() - 1)
  {
    param_path.pop_back();
    last_demarker = param_path.find_last_of("/");
  }

  std::string name = param_path.substr(last_demarker + 1);

  XmlRpc::XmlRpcValue xml = loadFromParamServer(param_path);
  if (xml.getType() == XmlRpc::XmlRpcValue::TypeInvalid)
    ROS_ERROR_STREAM("Error loading " << param_path << " from Parameter Server");

  // TODO: Use Function Pointers or functors.
  YAML::Node node = xmlToYaml(xml, name);
  // Uninitialized YAML::Node is of YAML::NodeType::Null
  if (node.Type() == YAML::NodeType::Null)
    ROS_ERROR_STREAM(param_path << " has been translated to YAML::NodeType::Null");

  T t;
  try
  {
    t = node[name].as<T>();
  }
  catch (...)
  {
    ROS_FATAL_STREAM(param_path << " from parameter server does not match specified data structure");
    ros::shutdown();
  }
  return t;
}

template <typename T>
T required(XmlRpc::XmlRpcValue xml)
{
  if (xml.getType() == XmlRpc::XmlRpcValue::TypeInvalid)
    ROS_ERROR_STREAM("Error loading " << xml);
  std::string name = static_cast<std::string>(xml);

  // TODO: Use Function Pointers or functors.
  YAML::Node node = xmlToYaml(xml, name);
  // Uninitialized YAML::Node is of YAML::NodeType::Null
  if (node.Type() == YAML::NodeType::Null)
    ROS_ERROR_STREAM(xml << " has been translated to YAML::NodeType::Null");

  T t;
  try
  {
    t = node[name].as<T>();
  }
  catch (...)
  {
    ROS_FATAL_STREAM(xml << " from parameter server does not match specified data structure");
    ros::shutdown();
  }
  return t;
}
}  // namespace YamlUtils

#endif  // YAML_UTILS_H
