#ifndef TASK_SUPERVISOR_NAVIGATION_SMOOTH_WAYPOINT_HANDLER_H
#define TASK_SUPERVISOR_NAVIGATION_SMOOTH_WAYPOINT_HANDLER_H

#include <task_supervisor/plugins/base/navigation_handler_base.h>


namespace task_supervisor
{
class NavigationSmoothWaypointHandler : public NavigationHandlerBase
{
   /**
     * @brief Method called by task_supervisor when a navigation task is received
     * @param task Relevant task passed to handler by task supervisor
     * @param error_message Error message returned by this handler if execution fails
     * @return ReturnCode which indicates failure, cancellation or success
     */
  virtual ReturnCode runTask(movel_seirios_msgs::Task& task, std::string& error_message);
};

}  // namespace task_supervisor

#endif
