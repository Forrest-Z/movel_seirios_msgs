#include <task_supervisor/plugins/task_expander.h>

namespace task_supervisor
{
bool TaskExpander::initialize(ros::NodeHandle nh_supervisor, std::string name)
{
  nh_supervisor_ = nh_supervisor;
  name_ = name;
  nh_expander_ = ros::NodeHandle(nh_supervisor_, name_);

  if (!setupExpander())
  {
    ROS_FATAL("[%s] Error during expander setup. Shutting down.", name_.c_str());
    return false;
  }

  return true;
}

ReturnCode TaskExpander::expandTask(const movel_seirios_msgs::Task& task, std::vector<movel_seirios_msgs::Task>& subtasks,
                                    std::string& error_message)
{
  if (onExpandTask(task, subtasks, error_message))
    return ReturnCode::SUCCESS;
  else
    return ReturnCode::FAILED;
}

}  // namespace task_supervisor
