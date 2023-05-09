#ifndef TASK_SUPERVISOR_MULTI_POINT_LOCAL_PLANNER_NAVIGATION_HANDLER_H
#define TASK_SUPERVISOR_MULTI_POINT_LOCAL_PLANNER_NAVIGATION_HANDLER_H

#include <task_supervisor/plugins/base/multi_point_navigation_handler_base.h>
#include <nav_core/base_local_planner.h>
#include <multi_point_navigation/MultipointLocalPlannerConfig.h>
#include <dynamic_reconfigure/Reconfigure.h>

namespace task_supervisor
{

class MultiPointLocalPlannerNavigationHandler : public MultiPointNavigationHandlerBase
{
public:
    MultiPointLocalPlannerNavigationHandler();
    ~MultiPointLocalPlannerNavigationHandler();
    
    bool setupHandler();

private:
    bool navigateToPoint(const multi_point_navigation::Path& path, int goal_index);

    void stopNavigation();

    void reconfigureCb(multi_point_navigation::MultipointLocalPlannerConfig& config, uint32_t level);

    void setupDynamicReconfigure();
    
    bool loadParams();
    
    bool loadLocalPlanner(const std::string& planner, tf2_ros::Buffer& tf_buffer, costmap_2d::Costmap2DROS* costmap_ros);

    void publishZeroVelocity();

    void setupVelocitySetter();

    bool setVelocity(double linear_velocity, double angular_velocity);
    
    // Dynamic reconfigure
    std::shared_ptr<dynamic_reconfigure::Server<multi_point_navigation::MultipointLocalPlannerConfig>> dynamic_reconfigure_srv_;
    dynamic_reconfigure::Server<multi_point_navigation::MultipointLocalPlannerConfig>::CallbackType dynamic_reconfigure_cb_;

    // Local planner
    std::string local_planner_name_;
    boost::shared_ptr<nav_core::BaseLocalPlanner> local_planner_ptr_;
    pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;

    double controller_frequency_;
    double controller_patience_;
    ros::Time last_valid_control_;

    std::vector<geometry_msgs::PoseStamped> controller_plan_;

    // For velocity setter
    std::string local_planner_namespace_;
    std::string linear_vel_param_name_, angular_vel_param_name_;
    ros::ServiceClient vel_setter_client_;
};

}

#endif