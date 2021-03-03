#include<state_observer/state_observer.h>

StateObserver::StateObserver() : nh_private_("~")
{
  initialize();
}

void StateObserver::initialize()
{
  ros::Time::waitForValid();
  if (!loadParams())
  {
    ROS_FATAL("Error during parameter loading. Shutting down.");
    return;
  }
  ROS_INFO("All parameters loaded. Launching.");
  setupTopics();

  main_timer_ = nh_.createTimer(ros::Duration(1.0 / p_loop_rate_), &StateObserver::run, this);
  ros::spin();
}
bool StateObserver::loadParams()
{
  ros_utils::ParamLoader loader(nh_private_);

  loader.get_required("loop_rate", p_loop_rate_);
  loader.get_required("checker_name", p_checker_);

  return loader.params_valid();
}

void StateObserver::setupTopics()
{
  sub1_ = nh_private_.subscribe("/diagnostics_agg", 10, &StateObserver::onReceiveDiag, this);

  state_pub_ = nh_.advertise<movel_seirios_msgs::DiagnosticStateArray>("/robot_state", 10);

  status_map_[0] = "Ok";
  status_map_[1] = "Warn";
  status_map_[2] = "Error";
  status_map_[3] = "Stale";

  level_map_[0] = OK;
  level_map_[1] = WARN;
  level_map_[2] = ERROR;
  level_map_[3] = STALE;
}

/**
 * Callback function of the subscriber sub1_.
 *
 * @param diag the diagnostic array message received.
 */
void StateObserver::onReceiveDiag(const diagnostic_msgs::DiagnosticArray::ConstPtr &diag)
{
	new_diag_msg_.status = diag->status;

}
/**
 * Convert messages from DiagnosticArray to DiagnosticStateArray.
 * DiagnosticArray is published by diagnostic_aggregator and subscribed
 * by the current node. It is consist of a header and an array of
 * DiagnosticStatus messages.
 * DiagnosticStateArray is published by the current node. It is consist of
 * a header and an array of DiagnosticState messages.
 */
void StateObserver::run(const ros::TimerEvent &e)
{
	movel_seirios_msgs::DiagnosticStateArray diag_msg; //message to be published
	movel_seirios_msgs::DiagnosticStateArray nodes; //temporary variable to store the nodes in each DiagnodticStatus message.
	movel_seirios_msgs::DiagnosticState a_node; //temporary variable to store the states in each node
  /**
   * Whether a piece of DiagnosticStatus message is converted to
   * DiagnosticState message.
   */
	bool first_time = true;

  // loop over the array of DiagnosticStatus messages
	for (const diagnostic_msgs::DiagnosticStatus &status_msg : new_diag_msg_.status)
	{
    // the array of DiagnosticStatus messages have the following pattern:
    //for one node, a DiagnosticStatus message that describe the node info and status will be published first,
    //following which messages that describe diagnostic tasks within the node will be published.
    // decide whether the DiagnosticStatus message contains info of the entire node or info of a diagnostic task.
		if (status_msg.name.find(":") == std::string::npos)
  	{
			if (!first_time)
			{
				// if the convertion is finished, append the message to the array
				nodes.status.push_back(a_node);
				first_time = true;
			}

			// Clear the content in a_node
			first_time = false;
			a_node = empty_node_;
	    //a_node.group = status_msg.name;
			if (level_map_[status_msg.level] != STALE)
			{
				// extract node name and status
   			a_node.node = status_msg.values[0].key.substr(0, status_msg.values[0].key.find(":"));
				a_node.status = status_map_[status_msg.level];

        if (level_map_[status_msg.level] == OK) a_node.message = "Everything is ok.";
  			else a_node.message = "";

			}
   		else
      {
        a_node.status = "Offline";
        a_node.message = "";
      }

			//specify the diagnostic task that is raising messages other than ok

 		}

		else if (status_msg.name.substr(status_msg.name.find(":")+2) != p_checker_)
   	{
			/**
       * Temporary variable to store the state of a diagnostic task.
       */
     	movel_seirios_msgs::States a_state;

			// extract name and status of the state
     	a_state.name = status_msg.name.substr(status_msg.name.find(":")+2);
     	a_state.status = status_map_[status_msg.level];

			// add error message to the node if status is not "ok"
			if (status_msg.level != 0)
			{
				std::string error_message = a_state.name + ": " + a_state.status + "; ";
				a_node.message += error_message;
			}
     	a_state.message = status_msg.message;
     	a_state.hardware_id = status_msg.hardware_id;

			// extract and pass key value pair
     	for (const diagnostic_msgs::KeyValue &key_value : status_msg.values)
     	{
       	diagnostic_msgs::KeyValue a_key_value;
       	a_key_value.key = key_value.key;
       	a_key_value.value = key_value.value;
       	a_state.values.push_back(a_key_value);
      }

     	a_node.states.push_back(a_state); // append the state message to the array of states in the node
  	}
	}
	nodes.status.push_back(a_node); // append the DiagnosticState message of the node to the array of DiagnosticState messages.
	for (movel_seirios_msgs::DiagnosticState &nodes_states : nodes.status) diag_msg.status.push_back(nodes_states);
	state_pub_.publish(diag_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_observer");
  StateObserver scm;
}
