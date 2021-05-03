#include <navigation_based_docking/navigation_based_docking.hpp>

using namespace std;


/**
 * @brief Docking::autoDock - service to move the robot to the docking poition
 * @param req - pallet number to go to and the operation being performed like pickup or drop
 * @param res - whether reached the docking position
 * @return
 */
bool Docking::autoDock(navigation_based_docking::StartAutoDock::Request  &req,
                       navigation_based_docking::StartAutoDock::Response &res)
{
    active_ = true;
    //clearing vector
    data.clear();    
    
    float a;
    std::stringstream sstream;
    sstream << path_of_waypoints_ << req.pallet_number << ".txt" ;
    std::fstream myfile(sstream.str());
    // std::fstream myfile("/home/robert/autodock_ws/waypoints_check.txt");

    // assign data
    while (myfile >> a)     
    {
        data.push_back(a);
        // std::cout<<a<<std::endl;
    }
    std::cout<<"Total size is "<<data.size()<<std::endl;

    nodeHandle_.getParam("/move_base/local_costmap/inflation/inflation_radius", inflation_temp_);
    nodeHandle_.getParam("/move_base/recovery_behavior_enabled", rotation_temp_);

    // Shrink inflation radius and Disable clearing rotation behavior
    reconfigureParams("inflation", "config");
    reconfigureParams("rotation", "config");

    // Loop for iterating points on the vector
    iteratePoints();

    std::cout<<"Reached position for "<<req.operation<<std::endl;

    // Revert the inflation_radius and clearinng rotaton behavior
    reconfigureParams("inflation", "revert");
    reconfigureParams("rotation", "revert");

    // Finished
    res.success = true;
    return true;
}

/**
 * @brief Docking::Docking -  constructor with arguments
 * @param nodeHandle
 * @param active_
 * @param paused_
 * @param navigating_
 * @param cancel_dock_
 */
Docking::Docking(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle), active_(false), paused_(false), navigating_(false), cancel_dock_(false)
{
    nodeHandle_.getParam("/navigation_based_docking/eps", eps_);
    nodeHandle_.getParam("/navigation_based_docking/waypoints_folder", path_of_waypoints_);
    nodeHandle_.getParam("/navigation_based_docking/param_name", param_name_);
    nodeHandle_.getParam("/navigation_based_docking/param_value", param_value_);
    nodeHandle_.getParam("/navigation_based_docking/service_call_topic", service_call_topic_);
}

/**
 * @brief Docking::iteratePoints - Procedur for iterating through points on the file
 */
void Docking::iteratePoints()
{
    // Assigning local variables
    float eps = eps_;
    int i = 0;
    vector<float> pt;
    geometry_msgs::Pose2D current_pose;
    bool first_and_last = true;

    navigating_ = true;

    // Move base action client pointer
    nav_ac_ptr_ = std::make_shared<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
    tf::TransformListener listener;
    tf::StampedTransform transform;
    while (ros::ok())
    {
        // std::cout<<"Point number:"<<( ceil( (i+1) / 4.0 ) )<<std::endl;
        // std::cout<<"Paused: "<<paused_<<", Navigating: "<<navigating_<<std::endl;

        listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(1.0)); // 5.0
        listener.lookupTransform("map", "base_link", ros::Time(0), transform);

        current_pose.x = transform.getOrigin().x();
        current_pose.y = transform.getOrigin().y();

        if(paused_ && navigating_)
        {
            nav_ac_ptr_->cancelGoal();
            ROS_INFO("Docking paused");
            navigating_ = false;
        }
        if (!paused_ && !navigating_)
        {
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            vector<float> point = getPointData(current_point_);

            goal.target_pose.pose.position.x = point[0];
            goal.target_pose.pose.position.y = point[1];
            goal.target_pose.pose.orientation.z = point[2];
            goal.target_pose.pose.orientation.w = point[3];

            nav_ac_ptr_->sendGoal(goal);
            ROS_INFO("Docking resumed");

            navigating_ = true;
        }
        if(!paused_ && (( current_pose.x - data[i-4])*( current_pose.x - data[i-4]) + ( current_pose.y  - data[i+1 - 4])*( current_pose.y  - data[i+1 - 4]) <= eps*eps) && i < data.size()-3)
        {
            std::cout<<"Point number:"<<i<<std::endl;
            current_point_ = i;
            pt = getPointData(current_point_);
            goto_docking_position(pt);
            i+=4;
        }

        if (!paused_ && first_and_last )
        {
            if (i == 0){
                std::cout<<"Point number:"<<i<<std::endl;
                current_point_ = i;
                pt = getPointData(current_point_);
                goto_docking_position(pt);
                first_and_last = false;
            }
            else if (i == data.size())
            {
                std::cout<<"Point number:"<<i-4<<std::endl;
                current_point_ = i-4;
                pt = getPointData(current_point_);
                goto_docking_position(pt);
                first_and_last = false;
            }
        }
        if(cancel_dock_)
        {
            ROS_INFO("Docking cancelled");
            navigating_ = false;
            nav_ac_ptr_->cancelGoal();
            
            std_msgs::UInt8 done;
            done.data = 3;
            status_pub_.publish(done);
            active_ = false;
            break;
        }

        // A block code to get the waiting function when proceeding the first and last point
        if (nav_ac_ptr_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            if(i == 0){
                i+=4;
                first_and_last = true;
            }
            else if ( i == data.size()){
                active_ = false;
                break;
            }
        }
        // std::cout<<(( current_pose.x - data[i-4])*( current_pose.x - data[i-4]) + ( current_pose.y  - data[i+1 - 4])*( current_pose.y  - data[i+1 - 4]))<<std::endl;
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
}

/**
 * @brief Docking::goto_docking_position - sending goal in the normal way
 * @param pt - pt to go to
 */
void Docking::goto_docking_position(vector<float> pt)
{
    while(!nav_ac_ptr_->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    std::cout<<"Sending goal "<<std::endl;
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = pt[0];
    goal.target_pose.pose.position.y = pt[1];
    goal.target_pose.pose.orientation.z = pt[2];
    goal.target_pose.pose.orientation.w = pt[3];

    nav_ac_ptr_->sendGoal(goal);
}

/**
 * @brief Docking::getPointData - retrieve desired point data to the vector
 * @param i - desired point
 */
std::vector<float> Docking::getPointData(int i)
{
    vector<float> point;
    for(int j =0 ; j < 4; j++)
    {
        point.push_back(data[i+j]);
    }
    return point;
}

/**
 * @brief Docking::pauseCb - Callback for pause
 * @param msg
 */
void Docking::pauseCb(std_msgs::Bool msg)
{
    paused_ = true;
}

/**
 * @brief Docking::resumeCb - Callback for resume
 * @param msg
 */
void Docking::resumeCb(std_msgs::Bool msg)
{
    paused_ = false;
}

/**
 * @brief Docking::cancelCb - Callback for cancel
 * @param msg
 */
void Docking::cancelCb(std_msgs::Bool msg)
{
    if (active_)                //cancel if this node is active
        cancel_dock_ = true;
}

/**
 * @brief Docking::reconfigureParams - reconfigure parameters
 * @param param - the name of the parameters
 * @param op - reconfig or revert
 */
void Docking::reconfigureParams(std::string param, std::string op)
{
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::Config conf;
    dynamic_reconfigure::Config reconf;
    std::string topic;

    if (param == "inflation")
    {
        dynamic_reconfigure::DoubleParameter inflation_param;
        inflation_param.name = param_name_;
        if (op == "config")
        {
            inflation_param.value = param_value_;

        }
        else        // revert
        {
            inflation_param.value = inflation_temp_;
        }
        conf.doubles.push_back(inflation_param); 
        srv_req.config = conf;
        topic = service_call_topic_;
    }
    else
    {
        dynamic_reconfigure::BoolParameter rotation_param;
        rotation_param.name = "recovery_behavior_enabled";
        if (op == "config")
        {
            rotation_param.value = false;
        }
        else        // revert
        {
            rotation_param.value = rotation_temp_;
        }
        conf.bools.push_back(rotation_param); 
        srv_req.config = conf;
        topic = "/move_base/set_parameters";
    }
    
    if (ros::service::call(topic, srv_req, srv_resp)) {
        ROS_INFO("call to set %s params succeeded", param.c_str());
    } else {
        ROS_INFO("call to set %s layer params failed", param.c_str());
    }
}