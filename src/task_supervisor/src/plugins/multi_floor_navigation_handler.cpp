#include <task_supervisor/plugins/base/navigation_handler_base.h>
#include <task_supervisor/plugins/multi_floor_navigation_handler.h>
#include <task_supervisor/json.hpp>
#include <pluginlib/class_list_macros.h>
#include <actionlib_msgs/GoalID.h>
#include <movel_seirios_msgs/GetReachableSubplan.h>
#include <type_traits>

PLUGINLIB_EXPORT_CLASS(task_supervisor::MultiFloorNavigationHandler, task_supervisor::NavigationHandlerBase);
PLUGINLIB_EXPORT_CLASS(task_supervisor::MultiFloorNavigationHandler, task_supervisor::TaskHandler);


using json = nlohmann::json;

namespace task_supervisor
{

MultiFloorNavigationHandler::MultiFloorNavigationHandler(){
  
}

bool MultiFloorNavigationHandler::setupHandler(){
  // Call setupHandler of NavigationHandler
  NavigationHandlerBase::setupHandler();

  ROS_INFO("[%s] MFN SETUP TEST 1 Reached", name_.c_str());
  if (!loadParams()) {
    ROS_FATAL("[%s] Error during parameter loading. Shutting down.", name_.c_str());
    return false;
  }
  mfn_map_change_server_ = nh_handler_.advertiseService("/mfn_change_map", &MultiFloorNavigationHandler::MFNChangeMapHandle, this);
  map_change_client_ = nh_handler_.serviceClient<nav_msgs::LoadMap>("/change_map");
  map_nav_change_client_ = nh_handler_.serviceClient<nav_msgs::LoadMap>("/change_map_nav");
  clear_costmap_client_ = nh_handler_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  initial_pose_pub_ = nh_handler_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
  map_changed_pub_ = nh_handler_.advertise<std_msgs::String>("map_changed", 10);

  ROS_INFO("[%s] MFN SETUP TEST 3 Reached", name_.c_str());

  return true;
}

// removed load param utils

bool MultiFloorNavigationHandler::loadParams(){
  ROS_INFO("[%s] MFN SETUP TEST 2 Reached", name_.c_str());
  ROS_WARN("[%s] Loading of plugin parameters by ros_utils has not been implemented. Loading directly from Parameter "
           "Server instead.",
           name_.c_str());
  if (!load_param_util("mfn_map_folder_path", p_map_folder_path_)) { return false; }
  if (!load_param_util("mfn_map_nav_folder_path", p_map_nav_folder_path_)) { return false; }
  if (!load_param_util("mfn_graph_folder_path", p_graph_folder_path_)) { return false; }
  if (!load_param_util("mfn_transit_folder_path", p_transit_folder_path_)) { return false; }
  return true;
}

// removed enableHumanDetectionCB
// removed humanDetectionCB
// removed enableBestEffortGoalCB
// removed robotPoseCB
// removed start_ActionClient
// removed startWithDetection
// removed navigationLoop
// removed navigationAttemptGoal
// removed navigationDirect
// removed navigationBestEffort
// removed runTaskChooseNav

bool MultiFloorNavigationHandler::MFNChangeMapHandle(nav_msgs::LoadMap::Request& req,nav_msgs::LoadMap::Response& res){
  if(changeMapFn(req.map_url)){
    ros::ServiceClient speed_zone_client = nh_handler_.serviceClient<movel_seirios_msgs::StringTrigger>("/mongo_bridge/get_speed_zones");
    ros::ServiceClient prohib_layer_client_ = nh_handler_.serviceClient<movel_seirios_msgs::StringTrigger>("/prohibition_layer_mongo/get_prohib_layer");
    movel_seirios_msgs::StringTrigger speed_zone_srv;
    speed_zone_srv.request.input = req.map_url;
    if(!speed_zone_client.call(speed_zone_srv))
    {
      ROS_ERROR("[%s] Failed to call /mongo_bridge/get_speed_zones service", name_.c_str());
    }

    if(!prohib_layer_client_.call(speed_zone_srv))
      {
        ROS_ERROR("[%s] Failed to call /prohibition_layer_mongo/get_prohib_layer service", name_.c_str());
      }
      
    res.result = 0;
  }
  else{
    res.result = 255;
  }
  return true;
}

bool MultiFloorNavigationHandler::changeMapFn(std::string new_map_name){
  // Check if file exists
  std::string loc_map_file_abs = p_map_folder_path_ + "/" + new_map_name + ".yaml";
  FILE* file_loc = fopen(loc_map_file_abs.c_str(), "r");
  if (file_loc == NULL)
  {
    ROS_ERROR("[%s] Map for localization: %s specified does not exist in %s or can't be opened", name_.c_str(), new_map_name.c_str(), p_map_folder_path_.c_str());
    return false;
  }

  std::string nav_map_file_abs = p_map_nav_folder_path_ + "/" + new_map_name + ".yaml";
  FILE* file_nav = fopen(nav_map_file_abs.c_str(), "r");
  if (file_nav == NULL)
  {
    ROS_ERROR("[%s] Navigation map not found. Using localization map for navigation.", name_.c_str());
    
    // If nav map can't be opened, use loc map instead
    nav_map_path_ = loc_map_file_abs;
  } 
  else 
    nav_map_path_ = nav_map_file_abs;

  // Assign loc map path
  loc_map_path_ = loc_map_file_abs; 

  // Assign nav map path if the file can be opened


  // Change map 
  nav_msgs::LoadMap change_map_msg;
  nav_msgs::LoadMap change_map_nav_msg;
  std_msgs::String map_changed_msg;
  change_map_msg.request.map_url = loc_map_path_;
  change_map_nav_msg.request.map_url = nav_map_path_;
  map_changed_msg.data = new_map_name;

  if(ros::service::waitForService("/change_map",ros::Duration(5.0)) && ros::service::waitForService("/change_map_nav",ros::Duration(5.0))){
    if(map_change_client_.call(change_map_msg) && map_nav_change_client_.call(change_map_nav_msg)){
      ROS_INFO("[%s] Map changed to : %s", name_.c_str(), new_map_name.c_str());
      // Publish that map has been changed 
      map_changed_pub_.publish(map_changed_msg);
      return true;
    }
    else{
      ROS_ERROR("[%s] Failed to call change_map or change_map_nav service", name_.c_str());
      return false;
    }
  }
  else{
    ROS_ERROR("[%s] change_map and/or change_map_nav service cannot be contacted", name_.c_str());
      return false;
  }
}

std::vector<float> MultiFloorNavigationHandler::getRobotPose(){
  std::vector<float> pose_to_return;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  try{
    ros::Duration(2.0).sleep();
    transformStamped = tfBuffer.lookupTransform("map", "base_footprint", ros::Time(0));
  }
  catch (tf::TransformException ex) {
    ROS_WARN("%s",ex.what());
  }

  tf::Quaternion q(
      transformStamped.transform.rotation.x,
      transformStamped.transform.rotation.y,
      transformStamped.transform.rotation.z,
      transformStamped.transform.rotation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  pose_to_return.push_back(transformStamped.transform.translation.x);
  pose_to_return.push_back(transformStamped.transform.translation.y);

  return pose_to_return;
}

bool MultiFloorNavigationHandler::clearCostmapFn(){
  if(ros::service::waitForService("/move_base/clear_costmaps",ros::Duration(2.0))){
    std_srvs::Empty clear_costmap_msg;
    clear_costmap_client_.call(clear_costmap_msg);
  }
  else{
    ROS_WARN("[%s] Could not contact clear_costmap service", name_.c_str());
    return false;
  }
  return true;
}

ReturnCode MultiFloorNavigationHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message){
  task_active_ = true;
  task_parsed_ = false;
  isHealthy_ = true;
  start_ = ros::Time::now();

  if (start_ActionClient()){
    ROS_INFO("[%s] Task payload %s", name_.c_str(), task.payload.c_str());
    json payload = json::parse(task.payload);
    ROS_INFO("[%s] MFN RUN TEST 0 Reached", name_.c_str());

    if (payload.find("to_map") != payload.end()){
      if(getNodeNames()){
        if(graphGenerationHandle()){
          ROS_INFO("[%s] MFN RUN TEST 1 Reached", name_.c_str());

          // Calling clear costmap
          clearCostmapFn();

          // Input final goal to reach within the goal room
          geometry_msgs::Pose final_goal_pose;
          final_goal_pose.position.x = payload["position"]["x"].get<float>();
          final_goal_pose.position.y = payload["position"]["y"].get<float>();
          final_goal_pose.position.z = payload["position"]["z"].get<float>();
          final_goal_pose.orientation.x = payload["orientation"]["x"].get<float>();
          final_goal_pose.orientation.y = payload["orientation"]["y"].get<float>();
          final_goal_pose.orientation.z = payload["orientation"]["z"].get<float>();
          final_goal_pose.orientation.w = payload["orientation"]["w"].get<float>();
          
          int map_counter = 0;
          std::string from_room = payload["from_map"].get<std::string>();
          std::string to_room = payload["to_map"].get<std::string>();

          // Variables for Path generation
          std::vector<float> start_robot_pose = getRobotPose();
          std::vector<float> robot_coords;
          std::vector<float> goal_coords;
          robot_coords.push_back(start_robot_pose[0]);
          robot_coords.push_back(start_robot_pose[1]);
          goal_coords.push_back(final_goal_pose.position.x);
          goal_coords.push_back(final_goal_pose.position.y);

          ROS_INFO("[%s] MFN RUN TEST 2 Reached", name_.c_str());

          // Generate shortest path
          if (pathGenerationHandle(from_room, to_room, robot_coords, goal_coords)){
            ROS_INFO("[%s] MFN RUN TEST 3 Reached", name_.c_str());

            // Call each instance of the path one-by-one

            for(int i = 0; i <= path_to_follow_.size()-2; i++)
            {
              // Read transit_point file for instance goal pose
              float t_coord;
              std::vector<float> t_points_data, i_points_data;
              std::stringstream t_sstream;
              t_sstream << p_transit_folder_path_ + "/" + path_to_follow_[i] + "_" + path_to_follow_[i+1];
              std::fstream t_coord_file(t_sstream.str());
              while (t_coord_file >> t_coord)
              {
                t_points_data.push_back(t_coord);
              }

              // Read transit_point file for publishing initial pose in the new map
              float i_coord;
              std::stringstream i_sstream;
              i_sstream << p_transit_folder_path_ + "/" + path_to_follow_[i+1] + "_" + path_to_follow_[i];
              std::fstream i_coord_file(i_sstream.str());
              while (i_coord_file >> i_coord)
              {
                i_points_data.push_back(i_coord);
              }

              // Rotate init co-ordinates by 180 deg
              tf2::Quaternion q_og_tf, q_rotate, q_new;
              geometry_msgs::Quaternion q_og;
              q_og.x = 0.0;
              q_og.y = 0.0;
              q_og.z = i_points_data[5];
              q_og.w = i_points_data[6];
              tf2::convert(q_og , q_og_tf);
              q_rotate.setRPY(0, 0, 3.14159);
              q_new = q_rotate*q_og_tf;
              q_new.normalize();

              // Save initial pose
              geometry_msgs::PoseWithCovarianceStamped init_pose_msg;
              geometry_msgs::Quaternion q_send;

              init_pose_msg.header.frame_id = "map";
              init_pose_msg.pose.pose.position.x = i_points_data[0];
              init_pose_msg.pose.pose.position.y = i_points_data[1];
              q_send = tf2::toMsg(q_new);
              init_pose_msg.pose.pose.orientation.z = q_send.z;
              init_pose_msg.pose.pose.orientation.w = q_send.w;

              // Make instance goal pose
              geometry_msgs::Pose instance_goal_pose;
              instance_goal_pose.position.x = t_points_data[0];
              instance_goal_pose.position.y = t_points_data[1];
              instance_goal_pose.position.z = t_points_data[2];
              instance_goal_pose.orientation.x = t_points_data[3];
              instance_goal_pose.orientation.y = t_points_data[4];
              instance_goal_pose.orientation.z = t_points_data[5];
              instance_goal_pose.orientation.w = t_points_data[6];

              ROS_INFO("[%s] Calling goal for map change : %s -> %s",name_.c_str(),path_to_follow_[i].c_str(),path_to_follow_[i+1].c_str());

              // Call Navigation Function
              runTaskChooseNav(instance_goal_pose);

              // Change map and map_nav
              if(!task_cancelled_ && code_ != ReturnCode::FAILED && changeMapFn(path_to_follow_[i+1])){
                // Publish initial pose
                initial_pose_pub_.publish(init_pose_msg);
                ROS_INFO("[%s] Initial pose published", name_.c_str());
                ros::Duration(1.0).sleep();
                // Clear cost map
                clearCostmapFn();
                // Update how many rooms have been traversed
                map_counter++;
              }
              else{
                setMessage("Failure in map changing");
                error_message = message_;
                setTaskResult(false);
              }
            }
            // If all room goals done, call final goal within goal room
            if(map_counter==path_to_follow_.size()-1){
              ROS_INFO("[%s] Calling final goal within goal room",name_.c_str());

              // Call Navigation Function
              runTaskChooseNav(final_goal_pose);
            }
            else{
              setMessage("Could not reach goal room, not providing final goal");
              error_message = message_;
              setTaskResult(false);
            }
          }
          else{
            setMessage("Failed to call Path Generation");
            error_message = message_;
            setTaskResult(false); 
          }
        }
        else{
          setMessage("Failed to call Graph Generation");
          error_message = message_;
          setTaskResult(false); 
        }
      }
      else{
        setMessage("Failed to get map node names");
        error_message = message_;
        setTaskResult(false);
      }
    }
    else {
      setMessage("Malformed payload Example: {\"from_map\":\"room1\", \"to_map\":\"room2\", \"position\":{\"x\":-4,\"y\":0.58,\"z\":0}, "
                 "\"orientation\":{\"x\":0,\"y\":0,\"z\":0.71,\"w\":0.69}}");
      error_message = message_;
      setTaskResult(false);
    }
  }
  else {
    setMessage("Unable to talk to Move Base action server on " + p_navigation_server_);
    error_message = message_;
    setTaskResult(false);
  }

  return code_;
}

// removed cancelTask
// removed locReportingCB


bool MultiFloorNavigationHandler::getNodeNames(){
  map_nodes_.clear();
  try{
    // Load map names file node_names.txt
    std::string node_name;
    std::stringstream sstream_node_names;
    sstream_node_names << p_graph_folder_path_ + "/node_names.txt";
    std::fstream node_names_file(sstream_node_names.str());
    std::cout<<"Map node names :"<<std::endl;
    while (node_names_file >> node_name){
      map_nodes_.push_back(node_name);
      std::cout<<node_name<<std::endl;
    }
  }
  catch(...){
    return false;
  }
  return true;
}


/////// Path Generation //////////////////////////////////////
//////////////////////////////////////////////////////////////

bool MultiFloorNavigationHandler::pathGenerationHandle(std::string from_room, std::string to_room, std::vector<float> robot_coords, std::vector<float> goal_coords){
  path_to_follow_.clear();
  try{
    int src=999,dest=999;
    std::string to_print_path = " ";
    std::vector<std::vector<float>> rcvd_coords {{},{}};
    
    for(int i = 0; i<map_nodes_.size();i++)
    {
        if(map_nodes_[i]==from_room.c_str())src = i;
        if(map_nodes_[i]==to_room.c_str())dest = i;
    }
    if(src==999 || dest==999)
    {
        ROS_ERROR("[%s] Error interpreting inputs : Map/Room names supplied do not match with node_names.txt",name_.c_str());
        return false;
    }

    if(robot_coords.size()>=2)
    {
        rcvd_coords[0].push_back(robot_coords[0]);
        rcvd_coords[0].push_back(robot_coords[1]);
    }
    if(goal_coords.size()>=2)
    {
        rcvd_coords[1].push_back(goal_coords[0]);
        rcvd_coords[1].push_back(goal_coords[1]);
    }

    std::vector<int> inverted_path = DijkstraAlgo(src, dest, rcvd_coords);

    while(inverted_path.size()>0)
    {
      path_to_follow_.push_back(map_nodes_[inverted_path.back()]);
      to_print_path = to_print_path + map_nodes_[inverted_path.back()] + " ";
      inverted_path.pop_back();
    }
    ROS_INFO("[%s] Path :%s", name_.c_str(),to_print_path.c_str());
  }
  catch(...){
    return false;
  }
    
  return true;
}

std::vector<int> MultiFloorNavigationHandler::DijkstraAlgo(int src, int dest, std::vector<std::vector<float>> rcvd_coords){
  int distance[map_nodes_.size()];                           
  bool Tset[map_nodes_.size()];
  int prev[map_nodes_.size()];


  //////////////////////////////////////////////
  float robot_x, robot_y, goal_x, goal_y;
  bool rcvd_goal_coords;

  if(rcvd_coords.size()==1)
  {
      rcvd_goal_coords = false;
      robot_x = rcvd_coords[0][0];
      robot_y = rcvd_coords[0][1];
  }
  else if(rcvd_coords.size()==2 && rcvd_coords[1].size()<2)
  {
      rcvd_goal_coords = false;
      robot_x = rcvd_coords[0][0];
      robot_y = rcvd_coords[0][1];
  }
  else if(rcvd_coords.size()==2 && rcvd_coords[1].size()>=2)
  {
      rcvd_goal_coords = true;
      robot_x = rcvd_coords[0][0];
      robot_y = rcvd_coords[0][1];
      goal_x = rcvd_coords[1][0];
      goal_y = rcvd_coords[1][1];
  }
  else{ROS_ERROR("Incorrect no. of co-ordinates received");}
  //////////////////////////////////////////////
  
    
  for(int k = 0; k<map_nodes_.size(); k++)
  {
      distance[k] = INT_MAX;
      Tset[k] = false;    
  }
  
  distance[src] = 0;             
  prev[src] = 0;
  for(int k = 0; k<map_nodes_.size(); k++)                           
  {
      int m=miniDist(distance,Tset); 
      Tset[m]=true;
      for(int k = 0; k<map_nodes_.size(); k++)                  
      {
          if(m == dest && graph_[m][k])
          {
              if(!Tset[k] && graph_[m][k] && distance[m]!=INT_MAX && distance[m]+graph_[m][k]<distance[k]){
                  distance[k]=distance[m]+graph_[m][k];
                  prev[k] = m;}
          }

          else if(m == src && k == dest && graph_[m][k])
          {
              std::vector<float> robotroom_switchpoint_for_k, goalroom_switchpoint;
              robotroom_switchpoint_for_k = getSwitchPoint(src, k);
              goalroom_switchpoint = getSwitchPoint(k, m);

              if(!rcvd_goal_coords)
              {
                  goal_x = goalroom_switchpoint[0] + 0.05;
                  goal_y = goalroom_switchpoint[1] + 0.05;
              }

              float eucl_dist = euclidDistance(robot_x,robot_y,robotroom_switchpoint_for_k[0],robotroom_switchpoint_for_k[1]) + euclidDistance(goal_x,goal_y,goalroom_switchpoint[0],goalroom_switchpoint[1]);
              

              if(!Tset[k] && graph_[m][k] && distance[m]!=INT_MAX && distance[m]+graph_[m][k]+ eucl_dist <distance[k]){
                  distance[k]=distance[m]+graph_[m][k]+eucl_dist;
                  prev[k] = m;}
          }
          
          else if(m == src && k != dest && graph_[m][k])
          {
              std::vector<float> robotroom_switchpoint_for_k;
              robotroom_switchpoint_for_k = getSwitchPoint(src, k);

              float eucl_dist = euclidDistance(robot_x,robot_y,robotroom_switchpoint_for_k[0],robotroom_switchpoint_for_k[1]);
              

              if(!Tset[k] && graph_[m][k] && distance[m]!=INT_MAX && distance[m]+graph_[m][k]+ eucl_dist <distance[k]){
                  distance[k]=distance[m]+graph_[m][k]+eucl_dist;
                  prev[k] = m;}
          }     
          
          else if(m != src && k == dest && graph_[m][k])
          {
              std::vector<float> goalroom_switchpoint,first_coords, second_coords;
              first_coords = getSwitchPoint(m,prev[m]);
              second_coords = getSwitchPoint(m,k);
              goalroom_switchpoint = getSwitchPoint(k, m);

              if(!rcvd_goal_coords)
              {
                  goal_x = goalroom_switchpoint[0] + 0.05;
                  goal_y = goalroom_switchpoint[1] + 0.05;
              }

              float eucl_dist = euclidDistance(goal_x,goal_y,goalroom_switchpoint[0],goalroom_switchpoint[1]) + euclidDistance(first_coords[0], first_coords[1], second_coords[0], second_coords[1]);
              

              if(!Tset[k] && graph_[m][k] && distance[m]!=INT_MAX && distance[m]+graph_[m][k]+ eucl_dist <distance[k]){
                  distance[k]=distance[m]+graph_[m][k]+eucl_dist;
                  prev[k] = m;}
          }
          
          
          else if(m != src && k != dest && graph_[m][k])
          {
              std::vector<float> first_coords, second_coords;
              first_coords = getSwitchPoint(m,prev[m]);
              second_coords = getSwitchPoint(m,k);
              float eucl_dist = euclidDistance(first_coords[0], first_coords[1], second_coords[0], second_coords[1]);
              

              if(!Tset[k] && graph_[m][k] && distance[m]!=INT_MAX && distance[m]+graph_[m][k]+ eucl_dist <distance[k]){
                  distance[k]=distance[m]+graph_[m][k]+eucl_dist;
                  prev[k] = m;}
          }
      }
  }
  return getQueue(prev,src,dest);
}

std::vector<int> MultiFloorNavigationHandler::getQueue(int prev[],int src,int dest){
  std::vector<int> midQ;
  int head = dest;
  midQ.push_back(head);
  while(head!=src)
  {
      head = prev[head];
      midQ.push_back(head);
  }
  return midQ;
}

std::vector<float> MultiFloorNavigationHandler::getSwitchPoint(int switch_from_room, int switch_to_room){
    float coord;
    std::vector<float> file_points_data;
    std::vector<float> return_data;
    std::stringstream sstream;
    sstream << p_transit_folder_path_ + "/" + map_nodes_[switch_from_room] + "_" + map_nodes_[switch_to_room];
    std::fstream coord_file(sstream.str());
    while (coord_file >> coord){
        file_points_data.push_back(coord);
    }
    return_data.push_back(file_points_data[0]);
    return_data.push_back(file_points_data[1]);

    return return_data;
}

float MultiFloorNavigationHandler::euclidDistance(float x1,float y1, float x2, float y2){
    return sqrt((pow(x2-x1,2))+(pow(y2-y1,2)));
}

int MultiFloorNavigationHandler::miniDist(int distance[], bool Tset[]){
    int minimum=INT_MAX,ind;
              
    for(int k=0;k<map_nodes_.size();k++){
        if(Tset[k]==false && distance[k]<=minimum){
            minimum=distance[k];
            ind=k;
        }
    }
    return ind;
}

//------------------------------------------------------------
//------------------------------------------------------------


/////// Graph Generation //////////////////////////////////////
///////////////////////////////////////////////////////////////

void MultiFloorNavigationHandler::printGraphConnections(std::vector<std::vector<std::string>> vector_list){
  std::cout << std::endl << "Connections found : " << std::endl;
  for(int i = 0; i<vector_list.size() ; i++){
      std::cout << vector_list[i][0] << " <-|-> " << vector_list[i][1] << std::endl;
  }
  std::cout << std::endl;
}

void MultiFloorNavigationHandler::printGraph(std::vector<std::vector<int>> graph){
  std::cout << "Graph : " << std::endl;
  for(int i = 0; i < graph.size(); i++ ){
      for(int j = 0; j < graph[i].size(); j++ ){
          std::cout << graph[i][j] << " ";
      }
      std::cout<<std::endl;
  }
  std::cout<<std::endl;
}

std::vector<std::vector<std::string>> MultiFloorNavigationHandler::getTransitConnections(){
  std::vector<std::string> file_list;
  std::vector<std::vector<std::string>> connections_list;
  file_list = getTransitFiles();

  for(int i = 0; i < file_list.size(); i++){
      std::string map_name_1, map_name_2;
      size_t underscore = file_list[i].find("_");
      for(int j = 0; j < file_list[i].size(); j++){
          if(j<underscore){
              map_name_1.push_back(file_list[i][j]);
          }
          else if(j>underscore){
              map_name_2.push_back(file_list[i][j]);
          }
      }
      auto map_it_1 =  find(map_nodes_.begin(), map_nodes_.end(), map_name_1);
      auto map_it_2 =  find(map_nodes_.begin(), map_nodes_.end(), map_name_2);
      if(map_it_1 != map_nodes_.end() && map_it_2 != map_nodes_.end()){
        connections_list.push_back({map_name_1, map_name_2});
      }
      else{
        ROS_WARN("[%s] Discarded transit connection between - %s & %s : Atleast one map name not found in node_names.txt file", name_.c_str(), map_name_1.c_str(), map_name_2.c_str());
      }
  }
  return connections_list;
}

std::vector<std::string> MultiFloorNavigationHandler::getTransitFiles(){
  std::string transit_path = p_transit_folder_path_ + "/";
  std::vector <std::string> file_list;
  for (const auto & transit_file : directory_iterator(transit_path))
  {
      file_list.push_back(transit_file.path().filename());
  }
  return file_list;
}

void MultiFloorNavigationHandler::buildSaveGraph(std::vector<std::vector<std::string>> connections_list, std::string graph_file_name){
  int num_of_nodes = map_nodes_.size();
  std::vector<int> in_graph(num_of_nodes, 0);
  std::vector<std::vector<int>> graph(num_of_nodes, in_graph);

  for(int i = 0; i < connections_list.size(); i++)
  {
      auto map_it_1 = find(map_nodes_.begin(), map_nodes_.end(), connections_list[i][0]);
      int index_1 = map_it_1 - map_nodes_.begin();
      auto map_it_2 = find(map_nodes_.begin(), map_nodes_.end(), connections_list[i][1]);
      int index_2 = map_it_2 - map_nodes_.begin();
      graph[index_1][index_2] = 1;
  }

  printGraph(graph);

  graph_ = graph;

  for(int i = 0; i < graph.size(); i++ )
  {
      for(int j = 0; j < graph[i].size(); j++ )
      {
          graph_file_ << graph[i][j] << std::endl;
      }
  }

  graph_file_ << std::endl;
  ROS_INFO("[%s] Graph saved : %s", name_.c_str(), graph_file_name.c_str());
}

bool MultiFloorNavigationHandler::graphGenerationHandle(){
  try{
    std::string graph_file_name = p_graph_folder_path_ + "/graph.txt";
    graph_file_.open(graph_file_name);

    std::vector<std::vector<std::string>> connections_list;
    connections_list = getTransitConnections();
    printGraphConnections(connections_list);

    buildSaveGraph(connections_list, graph_file_name);
  }
  catch(...){
    return false;
  }
  return true;
}

//------------------------------------------------------------
//------------------------------------------------------------

}  // namespace task_supervisor
