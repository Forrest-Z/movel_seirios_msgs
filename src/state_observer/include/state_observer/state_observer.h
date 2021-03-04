#ifndef STATE_OBSERVER_H
#define STATE_OBSERVER_H

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros_utils/ros_utils.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

#include <movel_seirios_msgs/States.h>
#include <movel_seirios_msgs/DiagnosticState.h>
#include <movel_seirios_msgs/DiagnosticStateArray.h>


#include <math.h>

enum Level
{
  OK,
  WARN,
  ERROR,
  STALE
};

class StateObserver
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher state_pub_;

  ros::Subscriber sub1_;

  ros::Timer main_timer_;

  std::map<int, Level> level_map_;
  std::map<int, std::string> status_map_;

  /**
   * Latest diagnostic array received from the diagnostic aggregator.
   */
  diagnostic_msgs::DiagnosticArray new_diag_msg_;
  /**
   * Empty DiagnosticState message.
   * For clearing the content of a_node when a new message is received.
   */
	movel_seirios_msgs::DiagnosticState empty_node_;
  /**
   * The loop rate of the timer event.
   *
   * Parameter with default value 10.
   */
  double p_loop_rate_;
  /**
   * The name of the checker diagnostic task.
   * The purpose of the checker message is to check whether the node is launched.
   * The task with the name p_checker_ will not be present in the
   * DiagnosticState message for clearity.
   */
  std::string p_checker_;

  void initialize();
  bool loadParams();
  void setupTopics();

  void run(const ros::TimerEvent &e);

  void onReceiveDiag(const diagnostic_msgs::DiagnosticArray::ConstPtr &diag);


public:
	/**
	 * Constructor'
	 */
  StateObserver();
};

#endif // STATE_OBSERVER_H
