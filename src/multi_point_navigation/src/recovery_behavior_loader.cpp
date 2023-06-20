#include <multi_point_navigation/recovery_behavior_loader.h>

namespace multi_point_navigation
{
RecoveryBehaviorLoader::RecoveryBehaviorLoader() : recovery_loader_("nav_core", "nav_core::RecoveryBehavior")
{
}

RecoveryBehaviorLoader::~RecoveryBehaviorLoader()
{
}

bool RecoveryBehaviorLoader::loadRecoveryBehaviors(
    const ros::NodeHandle& nh, tf2_ros::Buffer* tf_buffer, costmap_2d::Costmap2DROS* global_costmap,
    costmap_2d::Costmap2DROS* local_costmap,
    std::vector<boost::shared_ptr<nav_core::RecoveryBehavior>>& recovery_behaviors)
{
  recovery_behaviors.clear();
  XmlRpc::XmlRpcValue behavior_list;

  if (!getRecoveryBehaviorListFromRosParam(nh, behavior_list))
    return false;

  if (!validateRecoveryBehaviorList(behavior_list))
    return false;

  return createRecoveryBehaviors(behavior_list, tf_buffer, global_costmap, local_costmap, recovery_behaviors);
}

bool RecoveryBehaviorLoader::getRecoveryBehaviorListFromRosParam(const ros::NodeHandle& nh,
                                                                 XmlRpc::XmlRpcValue& behavior_list_out)
{
  if (!nh.getParam("recovery_behaviors", behavior_list_out))
  {
    ROS_WARN("[RecoveryBehaviorLoader] No recovery behaviors specified");
    return false;
  }
  return true;
}

bool RecoveryBehaviorLoader::validateRecoveryBehaviorList(const XmlRpc::XmlRpcValue& behavior_list)
{
  if (!behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_FATAL(
        "[RecoveryBehaviorLoader] Expected list of recovery behavior to be XmlRpcValue Type %d, instead received "
        "XmlRpcValue Type %d",
        XmlRpc::XmlRpcValue::TypeArray, behavior_list.getType());
    return false;
  }

  std::vector<std::string> behavior_names;
  for (int i = 0; i < behavior_list.size(); ++i)
  {
    if (!behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_FATAL("[RecoveryBehaviorLoader] Expected recovery behavior to be map, instead received XmlRpcValue Type %d",
                behavior_list[i].getType());
      return false;
    }

    if (!behavior_list[i].hasMember("name") || !behavior_list[i].hasMember("type"))
    {
      ROS_FATAL("[RecoveryBehaviorLoader] Recovery behaviors must have a name and a type but this one does not");
      return false;
    }

    std::string behavior_name = behavior_list[i]["name"];
    if (std::find(behavior_names.begin(), behavior_names.end(), behavior_name) != behavior_names.end())
    {
      ROS_FATAL("[RecoveryBehaviorLoader] A recovery behavior with the name %s already exists, this is not allowed",
                behavior_name.c_str());
      return false;
    }
    else
    {
      behavior_names.push_back(behavior_name);
    }
  }

  return true;
}

bool RecoveryBehaviorLoader::createRecoveryBehaviors(
    const XmlRpc::XmlRpcValue& behavior_list, tf2_ros::Buffer* tf_buffer, costmap_2d::Costmap2DROS* global_costmap,
    costmap_2d::Costmap2DROS* local_costmap,
    std::vector<boost::shared_ptr<nav_core::RecoveryBehavior>>& recovery_behaviors)
{
  for (int i = 0; i < behavior_list.size(); ++i)
  {
    std::string behavior_name = behavior_list[i]["name"];
    std::string behavior_type = behavior_list[i]["type"];
    try
    {
      // check existence of plugin class/type
      if (!recovery_loader_.isClassAvailable(behavior_type))
      {
        // cannot find plugin class name, check if a non-fully-qualified name has potentially been passed instead
        std::vector<std::string> plugin_classes = recovery_loader_.getDeclaredClasses();
        for (unsigned int j = 0; j < plugin_classes.size(); ++j)
        {
          if (behavior_type == recovery_loader_.getName(plugin_classes[i]))
          {
            // found the supposed plugin class
            ROS_WARN(
                "[RecoveryBehaviorLoader] Recovery behavior specifications should now include the package name. You "
                "are using a deprecated API. Please switch from %s to %s in your yaml file",
                behavior_type.c_str(), plugin_classes[i].c_str());
            behavior_type = plugin_classes[i];
            break;
          }
        }
      }

      boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_type));

      if (behavior.get() == NULL)
      {
        ROS_FATAL(
            "[RecoveryBehaviorLoader] The ClassLoader returned a null pointer without throwing an exception. This "
            "should not happen");
        return false;
      }

      behavior->initialize(behavior_name, tf_buffer, global_costmap, local_costmap);
      recovery_behaviors.push_back(behavior);
    }
    catch (pluginlib::PluginlibException& e)
    {
      ROS_FATAL("[RecoveryBehaviorLoader] Failed to load recovery behavior plugin %s: %s", behavior_name.c_str(),
                e.what());
      return false;
    }
  }

  return true;
}

}  // namespace multi_point_navigation