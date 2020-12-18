#ifndef ROS_UTILS_H
#define ROS_UTILS_H

#include <ros/ros.h>
#include <string>
#include <limits>

namespace ros_utils
{
class ParamLoader
{
private:
  bool all_params_valid_ = true;
  ros::NodeHandle nh_;
  inline std::string type_check(XmlRpc::XmlRpcValue value_type);
  inline void param_check(std::string param_name);

public:
  ParamLoader();
  ParamLoader(const ros::NodeHandle &nh);

  template <class T>
  inline void get_required(const std::string &param_name, T &target);

  template <class T>
  inline void get_optional(const std::string &param_name, T &target, const T &default_value);

  bool params_valid();
};

// XmlRpcValue return value example: N6XmlRpc11XmlRpcValueE
// XmlRpcValue.getType() is an enum. For readability getType() is converted to strings.
inline std::string ParamLoader::type_check(XmlRpc::XmlRpcValue value_type)
{
  switch(value_type.getType())
  {
    case 0:
      return "TypeInvalid";
      break;
    case 1:
      return "TypeBoolean";
      break;
    case 2:
      return "TypeInt";
      break;
    case 3:
      return "TypeDouble";
      break;
    case 4:
      return "TypeString";
      break;
    case 5:
      return "TypeDateTime";
      break;
    case 6:
      return "TypeBase64";
      break;
    case 7:
      return "TypeArray";
      break;
    case 8:
      return "TypeStruct";
      break;
  }
}

inline void ParamLoader::param_check(std::string param_name)
{
  if (!nh_.hasParam(param_name))
  {
    ROS_ERROR_STREAM("Parameter: " << param_name << " not found");
    all_params_valid_ = false;
    return;
  }
}

// ROS cpp style guide of using ROS_ASSERT return in format below which gives unneeded information to end user.
// Opting towards if-else conditional checks
//
// ROSASSERT return message:
// [FATAL] [1553233763.681438798]: ASSERTION FAILED
//   file = /home/$USER/mov/catkin_ws/src/movel_utils/ros_utils/include/ros_utils/ros_utils.h
//   line = 117
//   cond = allowed_values.getType() == XmlRpc::XmlRpcValue::TypeBoolean
//

//template <typename T>
//inline void ParamLoader::get_required(const std::string &param_name, T &target)
//{
//  param_check(param_name);
//
//  nh_.getParam(param_name, target);
//}

template <>
inline void ParamLoader::get_required<std::string>(const std::string &param_name, std::string &target)
{
  bool valid = false;
  XmlRpc::XmlRpcValue received;
  XmlRpc::XmlRpcValue allowed_values;

  param_check(param_name);
  nh_.getParam(param_name, received);

  if (nh_.hasParam("validation/" + param_name + "/allowed"))
    nh_.getParam("validation/" + param_name + "/allowed", allowed_values);
  else
  {
  //   // ROS_ERROR_STREAM("Failed to load allowed validation values for " + param_name + ". Ensure that validation check is as follows:");
  //   // ROS_ERROR_STREAM("validation:");
  //   // ROS_ERROR_STREAM("  " + param_name + ":");
  //   // ROS_ERROR_STREAM("    allowed: ['VALUE_1' ,' VALUE_2']");
  //   // all_params_valid_ = false;
    target = static_cast<std::string>(received);
    return;
  }

  if (received.getType() != XmlRpc::XmlRpcValue::TypeString)
  {
    ROS_ERROR_STREAM("Parameter: " << param_name << " requires TypeString. Type received: " << type_check(received));
    all_params_valid_ = false;
    return;
  }

  if (allowed_values.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (int i = 0; i < allowed_values.size(); i++)
    {
      if (allowed_values[i].getType() != XmlRpc::XmlRpcValue::TypeString)
      {
        ROS_ERROR_STREAM("Parameter: " << param_name << " requires validation value(s) of TypeString. Validation value: (" << allowed_values[i] << ") type is " << type_check(allowed_values[i]));
        all_params_valid_ = false;
        return;
      }

      if (allowed_values[i] == received)
        valid = true;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Parameter: " << param_name << " requires TypeArray of TypeString. Type received: " << type_check(allowed_values));
    all_params_valid_ = false;
    return;
  };

  if (valid)
    target = static_cast<std::string>(received);
  else
  {
    all_params_valid_ = false;
    ROS_ERROR_STREAM("Parameter: " << param_name << " :(" << received << ") not among allowed values: " << allowed_values);
    return;
  }
}

template <>
inline void ParamLoader::get_required<bool>(const std::string &param_name, bool &target)
{
  bool valid = false;
  XmlRpc::XmlRpcValue received;
  XmlRpc::XmlRpcValue allowed_values;

  param_check(param_name);
  nh_.getParam(param_name, received);

  if (nh_.hasParam("validation/" + param_name + "/allowed"))
    nh_.getParam("validation/" + param_name + "/allowed", allowed_values);
  else
  {
    // ROS_ERROR_STREAM("Failed to load allowed validation values for " + param_name + ". Ensure that validation check is as follows:");
    // ROS_ERROR_STREAM("validation:");
    // ROS_ERROR_STREAM("  " + param_name + ":");
    // ROS_ERROR_STREAM("    allowed: ['VALUE_1' ,' VALUE_2']");
    // all_params_valid_ = false;
    target = static_cast<bool>(received);
    return;
  }

  if (received.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
  {
    ROS_ERROR_STREAM("Parameter: " << param_name << " requires TypeBoolean. Received: " << type_check(received));
    all_params_valid_ = false;
    return;
  }

  if (allowed_values.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (int i = 0; i < allowed_values.size(); i++)
    {
      if (allowed_values[i].getType() != XmlRpc::XmlRpcValue::TypeBoolean)
      {
        ROS_ERROR_STREAM("Parameter: " << param_name << " requires validation values of TypeBoolean. Validation value: (" << allowed_values << ") type is " << type_check(allowed_values[i]));
        all_params_valid_ = false;
        return;
      }

      if (allowed_values[i] == received)
        valid = true;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Parameter: " << param_name << " requires TypeArray of TypeBoolean. Type received: " << type_check(allowed_values));
    all_params_valid_ = false;
    return;
  };

  if (valid)
    target = static_cast<bool>(received);
  else
  {
    all_params_valid_ = false;
    ROS_ERROR_STREAM("Parameter: " << param_name << " (" << received << ") not among allowed values: " << allowed_values);
    return;
  }
}

template <>
inline void ParamLoader::get_required<int>(const std::string &param_name, int &target)
{
  XmlRpc::XmlRpcValue received;

  param_check(param_name);
  nh_.getParam(param_name, received);

  if (received.getType() != XmlRpc::XmlRpcValue::TypeInt)
  {
    ROS_ERROR_STREAM("Parameter: " << param_name << " requires TypeInt. Received: " << type_check(received));
    all_params_valid_ = false;
    return;
  }

  if (nh_.hasParam("validation/" + param_name + "/allowed"))
  {
    bool valid = false;
    XmlRpc::XmlRpcValue allowed_values;
    nh_.getParam("validation/" + param_name + "/allowed", allowed_values);

    if (allowed_values.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (int i = 0; i < allowed_values.size(); i++)
      {
        if (allowed_values[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
        {
          ROS_ERROR_STREAM("Parameter: " << param_name << " requires validation value(s) of TypeInt. Validation value: (" << allowed_values[i] << ") type is " << type_check(allowed_values[i]));
          all_params_valid_ = false;
          return;
        }
        if (allowed_values[i] == received)
          valid = true;
      }
    }
    else
    {
      ROS_ERROR_STREAM("Parameter: " << param_name << " validation requires TypeArray of TypeInt. Type received: " << type_check(allowed_values));
      all_params_valid_ = false;
      return;
    };

    if (valid)
      target = static_cast<int>(received);
    else
    {
      all_params_valid_ = false;
      ROS_ERROR_STREAM("Parameter: " << param_name << " :(" << received << ") not among allowed values: " << allowed_values);
      return;
    }
  }
  else
  {
    XmlRpc::XmlRpcValue min;
    XmlRpc::XmlRpcValue max;
    if (nh_.hasParam("validation/" + param_name + "/min"))
    {
      nh_.getParam("validation/" + param_name + "/min", min);
      if (min.getType() != XmlRpc::XmlRpcValue::TypeInt)
      {
        ROS_ERROR_STREAM(param_name << " validation max value requires TypeInt. Currently (" << min << ") type is " << type_check(min));
        all_params_valid_ = false;
        return;
      }
    }
    else
    {
      min = -INT_MAX;
      ROS_WARN_STREAM("Parameter: " << param_name << " validation missing a minimum value. Minimum set to: " << min);
    }

    if (nh_.hasParam("validation/" + param_name + "/max"))
    {
      nh_.getParam("validation/" + param_name + "/max", max);
      if (max.getType() != XmlRpc::XmlRpcValue::TypeInt)
      {
        ROS_ERROR_STREAM(param_name << " validation max value requires TypeInt. Currently (" << max << ") type is " << type_check(max));
        all_params_valid_ = false;
        return;
      }
    }
    else
    {
      max = INT_MAX;
      ROS_WARN_STREAM("Parameter: " << param_name << " validation missing a maximum value. Maximum set to: " << max);
    }

    int min_value = static_cast<int>(min);
    int max_value = static_cast<int>(max);
    int received_value = static_cast<int>(received);

    if (received_value <= max_value && received_value >= min_value)
      target = received_value;
    else
    {
      all_params_valid_ = false;
      ROS_ERROR_STREAM("Parameter " << param_name << " (" << received_value << ") out of range [" << min_value << ", " << max_value << "]");
      return;
    }
  }
}

template <>
inline void ParamLoader::get_required<float>(const std::string &param_name, float &target)
{
  ROS_WARN_STREAM("Due to lack of support for Float types in Parameter Server for ROS, " << param_name << " is not loaded. If float type is paramount, it is advised to load in with a double and doing a static_cast<float>(" << param_name << ").");

  all_params_valid_ = false;
  return;

  /*
  ROS_WARN_STREAM("Due to the lack of support for Float types in the Parameter Server for ROS,  static_cast<float>(" << param_name << ") was done to load. Some precision might have been lost.");

  XmlRpc::XmlRpcValue received;

  param_check(param_name);
  nh_.getParam(param_name, received);


  if (received.getType() != XmlRpc::XmlRpcValue::TypeDouble and \
      received.getType() != XmlRpc::XmlRpcValue::TypeInt)
  {
    ROS_ERROR_STREAM("Parameter: " << param_name << " requires TypeDouble. Received: " << type_check(received));
    all_params_valid_ = false;
    return;
  }

  if (nh_.hasParam("validation/" + param_name + "/allowed"))
  {
    bool valid = false;
    XmlRpc::XmlRpcValue allowed_values;
    nh_.getParam("validation/" + param_name + "/allowed", allowed_values);

    if (allowed_values.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (int i = 0; i < allowed_values.size(); i++)
      {
        if (allowed_values[i].getType() != XmlRpc::XmlRpcValue::TypeInt and \
            allowed_values[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
        {
          ROS_ERROR_STREAM("Parameter: " << param_name << " requires validation value(s) of TypeDouble. Validation value: (" << allowed_values[i] << ") type is " << type_check(allowed_values[i]));
          all_params_valid_ = false;
          return;
        }

        if (allowed_values[i] == received)
          valid = true;
      }
    }
    if (valid)
    {
      double ph = static_cast<double>(received);
      target = static_cast<float>(ph);
    }
    else
    {
      all_params_valid_ = false;
      ROS_ERROR_STREAM("Parameter: " << param_name << " :(" << received << ") not among allowed values: " << allowed_values);
      return;
    }
  }
  else
  {
    XmlRpc::XmlRpcValue min;
    XmlRpc::XmlRpcValue max;

    if (nh_.hasParam("validation/" + param_name + "/min"))
    {
      nh_.getParam("validation/" + param_name + "/min", min);
      if ((min.getType() != XmlRpc::XmlRpcValue::TypeDouble) and \
          (min.getType() != XmlRpc::XmlRpcValue::TypeInt))
      {
        ROS_ERROR_STREAM(param_name << " validation min value requires TypeDouble. Currently (" << min << ") type is " << type_check(min));
        all_params_valid_ = false;
        return;
      }
    }
    else
    {
      min = -FLT_MAX;
      ROS_WARN_STREAM("Parameter: " << param_name << " validation missing a minimum value. Minimum set to: " << min);
    }

    if (nh_.hasParam("validation/" + param_name + "/max"))
    {
      nh_.getParam("validation/" + param_name + "/max", max);
      if (max.getType() != XmlRpc::XmlRpcValue::TypeDouble and \
          max.getType() != XmlRpc::XmlRpcValue::TypeInt)
      {
        ROS_ERROR_STREAM(param_name << " validation max value requires TypeDouble. Currently (" << max << ") type is " << type_check(max));
        all_params_valid_ = false;
        return;
      }
    }
    else
    {
      max = FLT_MAX;
      ROS_WARN_STREAM("Parameter: " << param_name << " validation missing a maximum value. Maximum set to: " << max);
    }

    float min_value;
    float max_value;
    float received_value;

    if (min.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      int min_valuei = min;
      min_value = min_valuei;
    }
    else
    {
      double ph = static_cast<double>(min);
      min_value = static_cast<float>(ph);
    }

    if (max.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      int max_valuei = max;
      max_value = max_valuei;
    }
    else
    {
      double ph = static_cast<double>(max);
      max_value = static_cast<float>(ph);
    }

    if (received.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      int received_valuei = received;
      received_value = received_valuei;
    }
    else
    {
      double ph = static_cast<double>(received);
      received_value = static_cast<float>(ph);
    }

    if (received_value <= max_value && received_value >= min_value)
      target = received_value;
    else
    {
      all_params_valid_ = false;
      ROS_ERROR_STREAM("Parameter: " << param_name << " (" << received_value << ") out of range [" << min_value << ", " << max_value << "]");
    }
  }*/
}

template <>
inline void ParamLoader::get_required<double>(const std::string &param_name, double &target)
{
  XmlRpc::XmlRpcValue received;

  param_check(param_name);
  nh_.getParam(param_name, received);

  if (received.getType() != XmlRpc::XmlRpcValue::TypeDouble and \
      received.getType() != XmlRpc::XmlRpcValue::TypeInt)
  {
    ROS_ERROR_STREAM("Parameter: " << param_name << " requires TypeDouble. Received: " << type_check(received));
    all_params_valid_ = false;
    return;
  }

  if (nh_.hasParam("validation/" + param_name + "/allowed"))
  {
    bool valid = false;
    XmlRpc::XmlRpcValue allowed_values;
    nh_.getParam("validation/" + param_name + "/allowed", allowed_values);

    if (allowed_values.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (int i = 0; i < allowed_values.size(); i++)
      {
        if (allowed_values[i].getType() != XmlRpc::XmlRpcValue::TypeInt and \
            allowed_values[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
        {
          ROS_ERROR_STREAM("Parameter: " << param_name << " requires validation value(s) of TypeDouble. Validation value: (" << allowed_values[i] << ") type is " << type_check(allowed_values[i]));
          all_params_valid_ = false;
          return;
        }

        if (allowed_values[i] == received)
          valid = true;
      }
    }
    if (valid)
      target = static_cast<double>(received);
    else
    {
      all_params_valid_ = false;
      ROS_ERROR_STREAM("Parameter: " << param_name << " :(" << received << ") not among allowed values: " << allowed_values);
      return;
    }
  }
  else
  {
    XmlRpc::XmlRpcValue min;
    XmlRpc::XmlRpcValue max;

    if (nh_.hasParam("validation/" + param_name + "/min"))
    {
      nh_.getParam("validation/" + param_name + "/min", min);
      if ((min.getType() != XmlRpc::XmlRpcValue::TypeDouble) and \
          (min.getType() != XmlRpc::XmlRpcValue::TypeInt))
      {
        ROS_ERROR_STREAM(param_name << " validation min value requires TypeDouble. Currently (" << min << ") type is " << type_check(min));
        all_params_valid_ = false;
        return;
      }
    }
    else
    {
      min = -DBL_MAX;
      ROS_WARN_STREAM("Parameter: " << param_name << " validation missing a minimum value. Minimum set to: " << min);
    }

    if (nh_.hasParam("validation/" + param_name + "/max"))
    {
      nh_.getParam("validation/" + param_name + "/max", max);
      if (max.getType() != XmlRpc::XmlRpcValue::TypeDouble and \
          max.getType() != XmlRpc::XmlRpcValue::TypeInt)
      {
        ROS_ERROR_STREAM(param_name << " validation max value requires TypeDouble. Currently (" << max << ") type is " << type_check(max));
        all_params_valid_ = false;
        return;
      }
    }
    else
    {
      max = DBL_MAX;
      ROS_WARN_STREAM("Parameter: " << param_name << " validation missing a maximum value. Maximum set to: " << max);
    }

    double min_value;
    double max_value;
    double received_value;

    if (min.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      int min_valuei = min;
      min_value = min_valuei;
    }
    else
      min_value = static_cast<double>(min);

    if (max.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      int max_valuei = max;
      max_value = max_valuei;
    }
    else
      max_value = static_cast<double>(max);

    if (received.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      int received_valuei = received;
      received_value = received_valuei;
    }
    else
      received_value = static_cast<double>(received);

    if (received_value <= max_value && received_value >= min_value)
      target = received_value;
    else
    {
      all_params_valid_ = false;
      ROS_ERROR_STREAM("Parameter: " << param_name << " (" << received_value << ") out of range [" << min_value << ", " << max_value << "]");
    }
  }
}

template <>
inline void ParamLoader::get_required<std::vector<std::string> >(const std::string &param_name, std::vector<std::string> &target)
{
  std::vector<std::string> received_values;
  XmlRpc::XmlRpcValue received;

  param_check(param_name);
  nh_.getParam(param_name, received);

  if (received.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR_STREAM("Parameter: " << param_name << " requires TypeArray of TypeString. Type received: " << type_check(received));
    all_params_valid_ = false;
    return;
  }

  if (nh_.hasParam("validation/" + param_name + "/allowed"))
  {
    XmlRpc::XmlRpcValue allowed_values;
    nh_.getParam("validation/" + param_name + "/allowed", allowed_values);
    if (allowed_values.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (int i = 0; i < received.size(); i++)
      {
        if (received[i].getType() != XmlRpc::XmlRpcValue::TypeString)
        {
          ROS_ERROR_STREAM("Parameter: " << param_name << " requires validation value(s) of TypeString. Validation value: (" << received[i] << ") type is " << type_check(received[i]));
          all_params_valid_ = false;
          return;
        }
        for (int j = 0; j < allowed_values.size(); j++)
        {
          if (allowed_values[j].getType() != XmlRpc::XmlRpcValue::TypeString)
          {
            ROS_ERROR_STREAM("Parameter: " << param_name << " requires validation value(s) of TypeString. Validation value: (" << allowed_values[j] << ") type is " << type_check(allowed_values[j]));
            all_params_valid_ = false;
            return;
          }

          if (allowed_values[j] == received[i])
          {
            std::string ph = static_cast<std::string>(received[i]);
            received_values.push_back(ph);
            break;
          }
        }
      }
    }
    else
    {
      ROS_ERROR_STREAM("Parameter: " << param_name << " validation requires TypeArray of TypeString. Type received: " << type_check(allowed_values));
      all_params_valid_ = false;
      return;
    };
  }
  else
  {
    ROS_ERROR_STREAM("Failed to load required validation values. Ensure that values for " + param_name + " is as follows.");
    ROS_ERROR_STREAM("validation:");
    ROS_ERROR_STREAM("  " + param_name + ":");
    ROS_ERROR_STREAM("    allowed: ['VALUE_1' ,' VALUE_2']");

    all_params_valid_ = false;
    return;
  };

  if (received_values.size() == received.size())
    target = received_values;
  else
  {
    all_params_valid_ = false;
    ROS_ERROR_STREAM("Parameter: " << param_name << " :(" << received << ") not among validation values");
    return;
  }
}

template <>
inline void ParamLoader::get_required<std::vector<bool> >(const std::string &param_name, std::vector<bool> &target)
{
  std::vector<bool> received_values;
  XmlRpc::XmlRpcValue received;

  param_check(param_name);
  nh_.getParam(param_name, received);

  if (received.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR_STREAM("Parameter: " << param_name << " requires TypeArray of TypeBool. Type received: " << type_check(received));
    all_params_valid_ = false;
    return;
  }

  if (nh_.hasParam("validation/" + param_name + "/allowed"))
  {
    XmlRpc::XmlRpcValue allowed_values;
    nh_.getParam("validation/" + param_name + "/allowed", allowed_values);
    if (allowed_values.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (int i = 0; i < received.size(); i++)
      {
        if (received[i].getType() != XmlRpc::XmlRpcValue::TypeBoolean)
        {
          ROS_ERROR_STREAM("Parameter: " << param_name << " requires validation value(s) of TypeBool. Validation value: (" << received[i] << ") type is " << type_check(received[i]));
          all_params_valid_ = false;
          return;
        }
        for (int j = 0; j < allowed_values.size(); j++)
        {
          if (allowed_values[j].getType() != XmlRpc::XmlRpcValue::TypeBoolean)
          {
            ROS_ERROR_STREAM("Parameter: " << param_name << " requires validation value(s) of TypeBool. Validation value: (" << allowed_values[j] << ") type is " << type_check(allowed_values[j]));
            all_params_valid_ = false;
            return;
          }

          if (allowed_values[j] == received[i])
          {
            bool ph = static_cast<bool>(received[i]);
            received_values.push_back(ph);
            break;
          }
        }
      }
    }
    else
    {
      ROS_ERROR_STREAM("Parameter: " << param_name << " validation requires TypeArray of TypeBoolean. Type received: " << type_check(allowed_values));
      all_params_valid_ = false;
      return;
    };
  }
  else
  {
    ROS_ERROR_STREAM("Failed to load required validation values. Ensure that values for " + param_name + " is as follows.");
    ROS_ERROR_STREAM("validation:");
    ROS_ERROR_STREAM("  " + param_name + ":");
    ROS_ERROR_STREAM("    allowed: ['VALUE_1' ,' VALUE_2']");

    all_params_valid_ = false;
    return;
  };

  if (received_values.size() == received.size())
    target = received_values;
  else
  {
    all_params_valid_ = false;
    ROS_ERROR_STREAM("Parameter: " << param_name << " :(" << received << ") not among validation values");
    return;
  }
}

template <>
inline void ParamLoader::get_required<std::vector<int> >(const std::string &param_name, std::vector<int> &target)
{
  bool allowed = false;
  std::vector<int> received_values;
  XmlRpc::XmlRpcValue received;

  param_check(param_name);
  nh_.getParam(param_name, received);

  if (received.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR_STREAM("Failed to load values. Ensure that values for " + param_name + " is as follows.");
    ROS_ERROR_STREAM(param_name + ": ['VALUE_1' ,' VALUE_2']");

    all_params_valid_ = false;
    return;
  }

  if (nh_.hasParam("validation/" + param_name + "/allowed"))
    allowed = true;

  for (int i = 0; i < received.size(); i++)
  {
    if (received[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
    {
      ROS_ERROR_STREAM("Parameter: " << param_name << " requires validation value(s) of TypeInt. Validation value: (" << received[i] << ") type is " << type_check(received[i]));
      all_params_valid_ = false;
      return;
    }

    if (allowed)
    {
      XmlRpc::XmlRpcValue allowed_values;
      nh_.getParam("validation/" + param_name + "/allowed", allowed_values);

      if (allowed_values.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        for (int j = 0; j < allowed_values.size(); j++)
        {
          if (allowed_values[j].getType() != XmlRpc::XmlRpcValue::TypeInt)
          {
            ROS_ERROR_STREAM("Parameter: " << param_name << " requires validation value(s) of TypeInt. Validation value: (" << allowed_values[j] << ") type is " << type_check(allowed_values[j]));
            all_params_valid_ = false;
            return;
          }

          if (allowed_values[j] == received[i])
          {
            int ph = static_cast<int>(received[i]);
            received_values.push_back(ph);
            break;
          }
        }
      }
      else
      {
        ROS_ERROR_STREAM("Parameter: " << param_name << " validation requires TypeArray of TypeInt. Type received: " << type_check(allowed_values));
        all_params_valid_ = false;
        return;
      };
    }
    else
    {
      XmlRpc::XmlRpcValue min;
      XmlRpc::XmlRpcValue max;
      if (nh_.hasParam("validation/" + param_name + "/min"))
      {
        nh_.getParam("validation/" + param_name + "/min", min);
        if (min.getType() != XmlRpc::XmlRpcValue::TypeInt)
        {
          ROS_ERROR_STREAM(param_name << " validation max value requires TypeInt. Currently (" << min << ") type is " << type_check(min));
          all_params_valid_ = false;
          return;
        }
      }
      else
      {
        min = -INT_MAX;
        ROS_WARN_STREAM("Parameter: " << param_name << " validation missing a minimum value. Minimum set to: " << min);
      }

      if (nh_.hasParam("validation/" + param_name + "/max"))
      {
        nh_.getParam("validation/" + param_name + "/max", max);
        if (max.getType() != XmlRpc::XmlRpcValue::TypeInt)
        {
          ROS_ERROR_STREAM(param_name << " validation max value requires TypeInt. Currently (" << max << ") type is " << type_check(max));
          all_params_valid_ = false;
          return;
        }
      }
      else
      {
        max = INT_MAX;
        ROS_WARN_STREAM("Parameter: " << param_name << " validation missing a maximum value. Maximum set to: " << max);
      }

      int min_value = static_cast<int>(min);
      int max_value = static_cast<int>(max);

      int ph = static_cast<int>(received[i]);
      if (ph <= max_value && ph >= min_value)
        received_values.push_back(ph);
    }
  }

  if (received_values.size() == received.size())
    target = received_values;
  else
  {
    all_params_valid_ = false;
    ROS_ERROR_STREAM("Parameter: " << param_name << " :(" << received << ") not among validation values");
    return;
  }
}

template <>
inline void ParamLoader::get_required<std::vector<float> >(const std::string &param_name, std::vector<float> &target)
{

ROS_WARN_STREAM("Due to lack of support for Float types in Parameter Server for ROS, " << param_name << " is not loaded. If float type is paramount, it is advised to load in with a double and doing a static_cast<float>(" << param_name << ").");

  all_params_valid_ = false;
  return;

  /*
  ROS_WARN_STREAM("Due to the lack of support for Float types in the Parameter Server for ROS,  static_cast<float>(" << param_name << ") was done to load. Some precision might have been lost.");

  bool allowed = false;
  std::vector<float> received_values;
  XmlRpc::XmlRpcValue received;

  param_check(param_name);
  nh_.getParam(param_name, received);

  if (received.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR_STREAM("Failed to load values. Ensure that values for " + param_name + " is as follows.");
    ROS_ERROR_STREAM(param_name + ": ['VALUE_1' ,' VALUE_2']");

    all_params_valid_ = false;
    return;
  }

  if (nh_.hasParam("validation/" + param_name + "/allowed"))
    allowed = true;

  for (int i = 0; i < received.size(); i++)
  {
    if (received[i].getType() != XmlRpc::XmlRpcValue::TypeInt and \
        received[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
      ROS_ERROR_STREAM("Parameter: " << param_name << " requires validation value(s) of TypeDouble. Validation value: (" << received[i] << ") type is " << type_check(received[i]));
      all_params_valid_ = false;
      return;
    }

    if (allowed)
    {
      XmlRpc::XmlRpcValue allowed_values;
      nh_.getParam("validation/" + param_name + "/allowed", allowed_values);

      if (allowed_values.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        for (int j = 0; j < allowed_values.size(); j++)
        {
          if (allowed_values[j].getType() != XmlRpc::XmlRpcValue::TypeInt and \
              allowed_values[j].getType() != XmlRpc::XmlRpcValue::TypeDouble)
          {
            ROS_ERROR_STREAM("Parameter: " << param_name << " requires validation value(s) of TypeDouble. Validation value: (" << allowed_values[j] << ") type is " << type_check(allowed_values[j]));
            all_params_valid_ = false;
            return;
          }

          if (allowed_values[j] == received[i])
          {
            double ph = static_cast<double>(received[i]);
            received_values.push_back(static_cast<float>(ph));
            break;
          }
        }
      }
      else
      {
        ROS_ERROR_STREAM("Parameter: " << param_name << " validation requires TypeArray of TypeDouble. Type received: " << type_check(allowed_values));
        all_params_valid_ = false;
        return;
      };
    }
    else
    {
      XmlRpc::XmlRpcValue min;
      XmlRpc::XmlRpcValue max;

      if (nh_.hasParam("validation/" + param_name + "/min"))
      {
        nh_.getParam("validation/" + param_name + "/min", min);
        if ((min.getType() != XmlRpc::XmlRpcValue::TypeDouble) and \
            (min.getType() != XmlRpc::XmlRpcValue::TypeInt))
        {
          ROS_ERROR_STREAM(param_name << " validation min value requires TypeDouble. Currently (" << min << ") type is " << type_check(min));
          all_params_valid_ = false;
          return;
        }
      }
      else
      {
        min = -FLT_MAX;
        ROS_WARN_STREAM("Parameter: " << param_name << " validation missing a minimum value. Minimum set to: " << min);
      }

      if (nh_.hasParam("validation/" + param_name + "/max"))
      {
        nh_.getParam("validation/" + param_name + "/max", max);
        if (max.getType() != XmlRpc::XmlRpcValue::TypeDouble and \
            max.getType() != XmlRpc::XmlRpcValue::TypeInt)
        {
          ROS_ERROR_STREAM(param_name << " validation max value requires TypeDouble. Currently (" << max << ") type is " << type_check(max));
          all_params_valid_ = false;
          return;
        }
      }
      else
      {
        max = FLT_MAX;
        ROS_WARN_STREAM("Parameter: " << param_name << " validation missing a maximum value. Maximum set to: " << max);
      }

      float min_value;
      float max_value;
      float received_value;

      if (min.getType() == XmlRpc::XmlRpcValue::TypeInt)
      {
        int min_valuei = static_cast<int>(min);
        min_value = static_cast<float>(min_valuei);
      }
      else
      {
        double ph = static_cast<double>(min);
        min_value = static_cast<float>(ph);
      }

      if (max.getType() == XmlRpc::XmlRpcValue::TypeInt)
      {
        int max_valuei = static_cast<int>(max);
        max_value = static_cast<float>(max_valuei);
      }
      else
      {
        double ph = static_cast<double>(max);
        max_value = static_cast<float>(ph);
      }

      if (received[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
      {
        int received_valuei = received[i];
        received_value = static_cast<float>(received_valuei);
      }
      else
      {
        double ph = static_cast<double>(received[i]);
        received_value = static_cast<float>(ph);
      }

      if (received_value <= max_value && received_value >= min_value)
        received_values.push_back(received_value);
      else
      {
        all_params_valid_ = false;
        ROS_ERROR_STREAM("Parameter: " << param_name << " (" << received_value << ") out of range [" << min_value << ", " << max_value << "]");
      }
    }
  }

  if (received_values.size() == received.size())
    target = received_values;
  else
  {
    all_params_valid_ = false;
    ROS_ERROR_STREAM("Parameter: " << param_name << " :(" << received << ") not among validation values");
    return;
  }
  */
}

template <>
inline void ParamLoader::get_required<std::vector<double> >(const std::string &param_name, std::vector<double> &target)
{
  bool allowed = false;
  std::vector<double> received_values;
  XmlRpc::XmlRpcValue received;

  param_check(param_name);
  nh_.getParam(param_name, received);

  if (received.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR_STREAM("Failed to load values. Ensure that values for " + param_name + " is as follows.");
    ROS_ERROR_STREAM(param_name + ": ['VALUE_1' ,' VALUE_2']");

    all_params_valid_ = false;
    return;
  }

  if (nh_.hasParam("validation/" + param_name + "/allowed"))
    allowed = true;

  for (int i = 0; i < received.size(); i++)
  {
    if (received[i].getType() != XmlRpc::XmlRpcValue::TypeInt and \
        received[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
      ROS_ERROR_STREAM("Parameter: " << param_name << " requires validation value(s) of TypeDouble. Validation value: (" << received[i] << ") type is " << type_check(received[i]));
      all_params_valid_ = false;
      return;
    }

    if (allowed)
    {
      XmlRpc::XmlRpcValue allowed_values;
      nh_.getParam("validation/" + param_name + "/allowed", allowed_values);

      if (allowed_values.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        for (int j = 0; j < allowed_values.size(); j++)
        {
          if (allowed_values[j].getType() != XmlRpc::XmlRpcValue::TypeInt and \
              allowed_values[j].getType() != XmlRpc::XmlRpcValue::TypeDouble)
          {
            ROS_ERROR_STREAM("Parameter: " << param_name << " requires validation value(s) of TypeDouble. Validation value: (" << allowed_values[j] << ") type is " << type_check(allowed_values[j]));
            all_params_valid_ = false;
            return;
          }

          if (allowed_values[j] == received[i])
          {
            double received_values = static_cast<double>(received[i]);
            break;
          }
        }
      }
      else
      {
        ROS_ERROR_STREAM("Parameter: " << param_name << " validation requires TypeArray of TypeDouble. Type received: " << type_check(allowed_values));
        all_params_valid_ = false;
        return;
      };
    }
    else
    {
      XmlRpc::XmlRpcValue min;
      XmlRpc::XmlRpcValue max;

      if (nh_.hasParam("validation/" + param_name + "/min"))
      {
        nh_.getParam("validation/" + param_name + "/min", min);
        if ((min.getType() != XmlRpc::XmlRpcValue::TypeDouble) and \
            (min.getType() != XmlRpc::XmlRpcValue::TypeInt))
        {
          ROS_ERROR_STREAM(param_name << " validation min value requires TypeDouble. Currently (" << min << ") type is " << type_check(min));
          all_params_valid_ = false;
          return;
        }
      }
      else
      {
        min = -DBL_MAX;
        ROS_WARN_STREAM("Parameter: " << param_name << " validation missing a minimum value. Minimum set to: " << min);
      }

      if (nh_.hasParam("validation/" + param_name + "/max"))
      {
        nh_.getParam("validation/" + param_name + "/max", max);
        if (max.getType() != XmlRpc::XmlRpcValue::TypeDouble and \
            max.getType() != XmlRpc::XmlRpcValue::TypeInt)
        {
          ROS_ERROR_STREAM(param_name << " validation max value requires TypeDouble. Currently (" << max << ") type is " << type_check(max));
          all_params_valid_ = false;
          return;
        }
      }
      else
      {
        max = DBL_MAX;
        ROS_WARN_STREAM("Parameter: " << param_name << " validation missing a maximum value. Maximum set to: " << max);
      }

      double min_value;
      double max_value;
      double received_value;

      if (min.getType() == XmlRpc::XmlRpcValue::TypeInt)
      {
        int min_valuei = min;
        min_value = min_valuei;
      }
      else
        min_value = static_cast<double>(min);

      if (max.getType() == XmlRpc::XmlRpcValue::TypeInt)
      {
        int max_valuei = max;
        max_value = max_valuei;
      }
      else
        max_value = static_cast<double>(max);

      if (received[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
      {
        int received_valuei = received[i];
        received_value = static_cast<double>(received_valuei);
      }
      else
        received_value = static_cast<double>(received);

      if (received_value <= max_value && received_value >= min_value)
        received_values.push_back(received_value);
      else
      {
        all_params_valid_ = false;
        ROS_ERROR_STREAM("Parameter: " << param_name << " (" << received_value << ") out of range [" << min_value << ", " << max_value << "]");
      }
    }
  }
  if (received_values.size() == received.size())
    target = received_values;
  else
  {
    all_params_valid_ = false;
    ROS_ERROR_STREAM("Parameter: " << param_name << " :(" << received << ") not among validation values");
    return;
  }
}

template <typename T>
inline void ParamLoader::get_optional(const std::string &param_name, T &target, const T &default_value)
{
  if (!nh_.hasParam(param_name))
  {
	  ROS_DEBUG_STREAM("Parameter: " << param_name << " not found. Using default value.");
	  target = default_value;
  }
  else nh_.getParam(param_name, target);
};
} // namespace ros_utils

#endif //LOGGING_UTILS_H
