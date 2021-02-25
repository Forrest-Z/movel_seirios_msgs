#include <velocity_limiter/common.h>

bool checkType(XmlRpc::XmlRpcValue value, XmlRpc::XmlRpcValue::Type type, std::string name)
{
  if (value.getType() != type)
  {
    ROS_FATAL_STREAM("Invalid " << name << " type: " << value.getType() << " (expected " << type << ")");
    return false;
  }
  return true;
}
