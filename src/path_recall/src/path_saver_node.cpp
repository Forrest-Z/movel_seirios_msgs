/*!
 *  USAGE:
 *  1. Start service (/path_saver/start) : activates path saving and users can start teleop
 *  2. Stop service (/path_saver/stop)   : stops path saving
 *
 *  Alternative saving interface : Save service '/path_saver/save' (refer to SavePath.srv for message definition)
 */

#include <path_recall/path_saver.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros_utils/ros_utils.h>
#include <movel_hasp_vendor/license.h>

PathSaver Saver;

bool loadParams(ros::NodeHandle& nh_private_)
{
  ros_utils::ParamLoader loader(nh_private_);
  loader.get_required("yaml_path", Saver.yaml_path);
  loader.get_required("update_min_d", Saver.update_min_d);
  loader.get_required("update_min_a", Saver.update_min_a);
  return loader.params_valid();
}

int main(int argc, char** argv)
{
  #ifdef MOVEL_LICENSE                                                                                                    
  MovelLicense ml(7);                                                                                                   
  if (!ml.login())                                                                                                      
    return 1;                                                                                                           
  #endif

  std::string node_name_ = "path_saver";
  ros::init(argc, argv, node_name_);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_("~");

  ros::Time::waitForValid();
  if (!loadParams(nh_private_))
  {
    ROS_FATAL("Error during parameter loading. Shutting down.");
    #ifdef MOVEL_LICENSE
    ml.logout();
    #endif
    return 0;
  }
  ROS_INFO("All parameters loaded. Launching.");

  ros::Subscriber path_saver_sub_ = nh_.subscribe("/pose", 1, &PathSaver::onSave, &Saver);
  ros::ServiceServer start_srv_ = nh_private_.advertiseService("start", &PathSaver::onStart, &Saver);
  ros::ServiceServer stop_srv_ = nh_private_.advertiseService("stop", &PathSaver::onStop, &Saver);
  ros::ServiceServer save_srv_ = nh_private_.advertiseService("save", &PathSaver::savePath, &Saver);

  ros::spin();
  #ifdef MOVEL_LICENSE
  ml.logout();
  #endif
  return 0;
}
