#ifndef TASK_SUPERVISOR_TASK_EXPANDER_H
#define TASK_SUPERVISOR_TASK_EXPANDER_H

#include <ros/ros.h>

#include <task_supervisor/common.h>

#include <movel_seirios_msgs/Task.h>

namespace task_supervisor
{
class TaskExpander
{
public:
  bool initialize(ros::NodeHandle nh_supervisor, std::string name);
  virtual ~TaskExpander(){};

  ReturnCode expandTask(const movel_seirios_msgs::Task& task, std::vector<movel_seirios_msgs::Task>& subtasks,
                        std::string& error_message);

protected:
  TaskExpander(){};

  std::string name_;
  ros::NodeHandle nh_supervisor_;  // node handle in the task supervisor node namespace
  ros::NodeHandle nh_expander_;    // node handle in the expander plugin namespace

  virtual bool setupExpander()
  {
    return true;
  }  // any specific setup goes here
  virtual bool onExpandTask(const movel_seirios_msgs::Task& task, std::vector<movel_seirios_msgs::Task>& subtasks,
                            std::string& error_message) = 0;
};

}  // namespace task_supervisor

#endif
