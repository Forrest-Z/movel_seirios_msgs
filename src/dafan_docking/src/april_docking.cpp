#include <dafan_docking/april_docking.h>

AprilDocking::AprilDocking() : nh_private_("~"), tfListener_(tfBuffer_)
{
  initialize();
}

/**
 * INITIALIZATION
 */

void AprilDocking::initialize()
{
  if(!loadParams())
  {
    ROS_FATAL("[april_docking] Failed to load params. Shutting down.");
    return;
  }
  setupTopics();

  //Init dynamic reconfigure server
  dyn_server_.reset(new dynamic_reconfigure::Server<dafan_docking::april_dockingConfig>(own_mutex_, nh_private_)); 
  dynamic_reconfigure::Server<dafan_docking::april_dockingConfig>::CallbackType f; 
  f = boost::bind(&AprilDocking::reconfigureCB, this, _1, _2 ); 
  dyn_server_ -> setCallback(f); 

  //publish docking goal at 10Hz
  pubGoalTimer = nh_.createTimer(ros::Duration(0.1), boost::bind(&AprilDocking::publishDockGoal, this, _1));

  while(ros::ok()){
    ros::spinOnce();
  }
}

bool AprilDocking::loadParams()
{
  if (nh_private_.hasParam("goal_x_offset1")
      and nh_private_.hasParam("goal_y_offset1")
      and nh_private_.hasParam("goal_yaw_offset1")
      and nh_private_.hasParam("skip_phase_0")
      )
  {
    nh_private_.getParam("goal_x_offset1", goal_x_offset1_);
    nh_private_.getParam("goal_y_offset1", goal_y_offset1_);
    nh_private_.getParam("goal_yaw_offset1", goal_yaw_offset1_);
    nh_private_.getParam("skip_phase_0", skip_phase_0_);

  }
  else
    return false;

  if (nh_private_.hasParam("id_array1") 
      and nh_private_.hasParam("checkpoint_thresh1")
      )
  {
    nh_private_.getParam("id_array1", id_array1_);
    nh_private_.getParam("checkpoint_thresh1", checkpoint_thresh1_);
  }
  else{
    id_array1_ = "";
    checkpoint_thresh1_ = 0.1;
  }
  dist_to_dock_ = 1.0;

  stage1a_ = true;
  stage1b_ = false;

  start_pub_ = false;

  goal_x_offset_now_ = goal_x_offset1_;
  goal_y_offset_now_ = goal_y_offset1_;
  goal_yaw_offset_now_ = goal_yaw_offset1_;

  genIDVectors(id_array1_, id_array_vec_);

  enable_docking_ = skip_phase_0_;

  // ROS_INFO("Parameters set: id_array1(%s), id_array2(%s), \
  //           checkpoint_thresh1(%f), checkpoint_thresh2(%f), run_avg_n(%f)", \
  //           id_array1_, id_array2_, \
  //           checkpoint_thresh1_, checkpoint_thresh2_, run_avg_n_);
  return true;
}

void AprilDocking::setupTopics()
{
  //Publishers
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/pid_goal", 1, true);
  stop_pub_ = nh_.advertise<std_msgs::Bool>("/stop_now", 1);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  docking_status_pub_ = nh_.advertise<std_msgs::Int8>("/docking_status", 1);

  //Subscribers
  transform_april_sub_ = nh_.subscribe("/tag_detections", 10, &AprilDocking::transformAprilBundleCB, this);
  dock_dist_sub_ = nh_.subscribe("/dist_to_dock", 1, &AprilDocking::distDockCB, this);
  reached_sub_ = nh_.subscribe("/goal/status", 1, &AprilDocking::goalReachedCB, this);
  // batt_sub_ = nh_.subscribe("/batt_status", 1, &AprilDocking::battCB, this);

  //services
  toggle_device_client_ = nh_.serviceClient<dafan_msgs::ToggleDevices>("toggle_devices");
  toggle_docking_server = nh_.advertiseService("toggle_docking", &AprilDocking::start_dockingSRV, this);

}


bool AprilDocking::start_dockingSRV(dafan_docking::ToggleDocking::Request &req,
                                dafan_docking::ToggleDocking::Response &res){
  if (req.toggle){
    ROS_INFO("Starting Autonomous Docking");
    enable_docking_ = true;
    res.april_detected = april_detected_; //return true if there are april tags detected
    t_start_ = ros::Time::now();
  }
  else{
    ROS_INFO("Stopping Autonomous Docking");
    std_msgs::Bool msg;
    msg.data = true;
    stop_pub_.publish(msg);

    enable_docking_ = false;
    res.april_detected = april_detected_; //return true if there are april tags detected
  }

  return true;
}

/**
 * TIMED FUNCTIONS
 */

void AprilDocking::publishDockGoal(const ros::TimerEvent& event){
  if (!enable_docking_){
    return;
  }

  if (!april_detected_)
  {
    if ((ros::Time::now() - t_start_).toSec() > 10.0)
    {
      ROS_INFO("[april_docking] No marker detected after 10 [s]");

      std_msgs::Int8 status_msg;
      status_msg.data = -1;
      docking_status_pub_.publish(status_msg);
    }
    return;
  }

  //3RD STAGE
  if (stage1b_){
    // ROS_INFO("Stage 1b: Going towards dock goal 1");
    //Do not publish any goal

    //Robot should be moving towards docking goal
    //goalReachedCB will be changing the stage when it has reached docking goal 2
    // ROS_INFO("Stage 1b: Distance to docking goal: %f", dist_to_dock_);
  }
  //2ND STAGE (RUN ONCE)
  else if (dist_to_dock_ <= checkpoint_thresh1_ && stage1a_) {
    ROS_INFO("Reached Checkpoint A. Freezing dock goal 1");

    //Stop planner adjuster
    std_msgs::Bool msg;
    msg.data = true;
    stop_pub_.publish(msg);
    //sleep for 2 seconds and publish april pose
    ros::Rate r(0.25);
    r.sleep();
    pose_pub_.publish(pose_);

    //Start planner adjuster
    msg.data = false;
    stop_pub_.publish(msg);

    stage1b_ = true;
    ROS_INFO("Stage 1a");
    toggleCameraLED(0);

  }
  //1ST STAGE 
  else {
    //Before reaching checkpoint A
    if (start_pub_){
      pose_pub_.publish(pose_);
      // ROS_INFO("Stage 0: Distance to docking goal: %f", dist_to_dock_);
    }
  }

  // ROS_INFO("  Dock_goal: pos.x(%f), pos.y(%f), ori.z(%f), ori.w(%f)", 
  //           pose_.pose.position.x, pose_.pose.position.y,
  //           pose_.pose.orientation.z, pose_.pose.orientation.w);
  // ROS_INFO("goal offset: x(%f), y(%f), yaw(%f)", goal_x_offset_now_, goal_y_offset_now_, goal_yaw_offset_now_);

  return;
}

/**
 * CALLBACKS
 */

void AprilDocking::genIDVectors(std::string& id_array, std::vector<int32_t>& id_array_vec){
  std::string error_msg = "";
  parseVVF(id_array, error_msg, id_array_vec);

  if (error_msg != ""){
    ROS_ERROR("%s",error_msg.c_str());
  }
}


void AprilDocking::transformDockGoal(const geometry_msgs::Pose& april_pose, 
                                  const std::string& msg_frame, 
                                  std::string april_pose_frame, std::string dock_frame,
                                  double& goal_x_offset, double& goal_y_offset,
                                  double& goal_yaw_offset,
                                  geometry_msgs::PoseStamped& pose_saved) {
  ros::Time stamp_now = ros::Time::now();
  //send april_marker_selected transform relative to original april frame
  geometry_msgs::TransformStamped april_tf;
  april_tf.header.stamp = stamp_now;
  april_tf.header.frame_id = msg_frame;
  april_tf.child_frame_id = april_pose_frame;
  poseToTransform(april_pose , april_tf);
  br_.sendTransform(april_tf);

  //send dock_goal transform relative to april_marker_selected
  //We also translate and rotate the pose to the desired offset position
  geometry_msgs::TransformStamped april_to_dock_tf;
  april_to_dock_tf.header.stamp = stamp_now;
  april_to_dock_tf.header.frame_id = april_pose_frame;
  april_to_dock_tf.child_frame_id = dock_frame;
  //MULTISTAGE
  april_to_dock_tf.transform.translation.z = goal_x_offset;
  april_to_dock_tf.transform.translation.x = goal_y_offset;
  april_to_dock_tf.transform.rotation.x = 0;
  april_to_dock_tf.transform.rotation.y = 0;
  april_to_dock_tf.transform.rotation.z = 0;
  april_to_dock_tf.transform.rotation.w = 1;
  rotateQuaternionMsgRPY(april_to_dock_tf.transform.rotation, -M_PI/2, 0.0, 0.0);
  rotateQuaternionMsgRPY(april_to_dock_tf.transform.rotation, 0.0, 0.0, M_PI/2);
  rotateQuaternionMsgRPY(april_to_dock_tf.transform.rotation, 0.0, 0.0, goal_yaw_offset);
  br_.sendTransform(april_to_dock_tf);

  //Publish pose to "/pid_goal"
  geometry_msgs::TransformStamped pose_tf;
  try
  {
    pose_tf = tfBuffer_.lookupTransform("map", dock_frame, ros::Time(0), ros::Duration(0.5));
    
    // geometry_msgs::PoseStamped dock_pose;
    pose_saved.header.frame_id = "map";
    pose_saved.pose.position.x = pose_tf.transform.translation.x;
    pose_saved.pose.position.y = pose_tf.transform.translation.y;
    pose_saved.pose.position.z = 0; //pose should be on the parallel z plane with base link

    double norm = sqrt(pow(pose_tf.transform.rotation.z, 2) + pow(pose_tf.transform.rotation.w, 2));
    pose_saved.pose.orientation.x = 0;
    pose_saved.pose.orientation.y = 0;
    pose_saved.pose.orientation.z = pose_tf.transform.rotation.z / norm;
    pose_saved.pose.orientation.w = pose_tf.transform.rotation.w / norm;

  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
  }
}

void AprilDocking::transformAprilBundleCB(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
  bool bundle_got_elem;
  //if message is not empty
  if(msg->detections.size() > 0) 
  {
    for (size_t i = 0; i < msg->detections.size(); i++)
    {
      //VECTOR 
      if (msg->detections[i].id.size() > 0 and msg->detections[i].id.size() == id_array_vec_.size()){
        //check if each id in the bundle matches the user defined ids
        for (int id_msg : msg->detections[i].id)
        {
          bundle_got_elem = false;
          for (int id_def : id_array_vec_)
          {
            //break out of looping through user defined ids
            if (id_def == id_msg){
              bundle_got_elem = true;
              break;
            }
          }
          if (!bundle_got_elem){ //if none of the ids in the bundle match the user input
            break;
          }
        }
        if (!bundle_got_elem){ //if none of the ids in the bundle match the user input
          continue;
        }
        transformDockGoal(msg->detections[i].pose.pose.pose,  msg->header.frame_id, 
                          "april_bundle", "dock_goal", 
                          goal_x_offset_now_, goal_y_offset_now_, goal_yaw_offset_now_,
                          pose_);
        april_detected_ = true;
        start_pub_ = true;
      }
    }
  }
}

void AprilDocking::distDockCB(const std_msgs::Float32::ConstPtr& dock_dist){ 
  dist_to_dock_ = dock_dist->data;
  return;
}  

// void AprilDocking::battCB(const dafan_msgs::BatteryStatus& msg){
//   // Callback only does work during stage 3
//   if (stage3_){
//     geometry_msgs::Twist cmd_vel; 
//     if(msg.is_charging == 0 || msg.is_charging == 1){
//       ROS_INFO("  DOCKED: Stopped movement.");
//       //stop the robot
//       cmd_vel_pub_.publish(cmd_vel);
//       stage3_ = false;
//     }
//     else{
//       ROS_INFO("  DOCKING: Moving forward at 0.02 m/s.");
//       cmd_vel.linear.x = 0.02;
//       cmd_vel_pub_.publish(cmd_vel);
//     }
//   }
//   return;
// }

void AprilDocking::goalReachedCB(const std_msgs::Bool& goal_status_){
  if(goal_status_.data){
    //Robot has reached goal 1
    if (stage1a_){
      //disable activating the functions for stage1 in the timer
      stage1a_ = false;
      stage1b_ = false;
      start_pub_ = false;
    }
  }
  else{
    ROS_INFO("Invalid message from topic /goal/status");
  }
}

void AprilDocking::reconfigureCB(dafan_docking::april_dockingConfig &config, uint32_t level) 
{ 
  if (level == 0xFFFFFFFF) {
      return;
  }

  goal_x_offset_now_ = config.goal_x_offset1;
  goal_y_offset_now_ = config.goal_y_offset1;
  goal_yaw_offset_now_ = config.goal_yaw_offset1;

  checkpoint_thresh1_ = config.checkpoint_thresh1;

  ROS_INFO("April Pose parameters have been dynamically reconfigured");
}

/**
 * HELPER FUNCTIONS
 */

void AprilDocking::toggleCameraLED(const int& toggle_){
  //service call to turn on/off LED lights
  dafan_msgs::ToggleDevices srv_;
  srv_.request.dev_num = 9; //Camera LED lights
  srv_.request.toggle = toggle_; //Toggle ON

  if (toggle_device_client_.call(srv_)){
    ROS_INFO("Toggled lights with %d", srv_.response.success);
  }
  else{
    ROS_ERROR("Service call failed to toggle lights");
  }

}


void AprilDocking::rotateQuaternionMsgRPY(geometry_msgs::Quaternion& q_msg, const float& r, const float& p, const float& y){

  tf2::Quaternion q_rot, q_new;
  tf2::convert(q_msg, q_new);
  q_rot.setRPY(r, y, p);
  q_new = q_rot * q_new;
  q_new.normalize();
  tf2::convert(q_new, q_msg);
}

void AprilDocking::poseToTransform(const geometry_msgs::Pose& pose, geometry_msgs::TransformStamped& tf){
  
  //Translation
  tf.transform.translation.x = pose.position.x;
  tf.transform.translation.y = pose.position.y;
  tf.transform.translation.z = pose.position.z;

  //Orientation
  tf.transform.rotation.x = pose.orientation.x;
  tf.transform.rotation.y = pose.orientation.y;
  tf.transform.rotation.z = pose.orientation.z;
  tf.transform.rotation.w = pose.orientation.w;
}

// void AprilDocking::calcRunningAvg(const geometry_msgs::PoseStamped& pose_){

//   double sum_pos_x = 0.0, sum_pos_y = 0.0;
//   double sum_ori_z = 0.0, sum_ori_w = 0.0;

//   // ROS_INFO("itr_avg(%d)", itr_avg_);

//   //if the pose_avg_array_ is not completely filled yet
//   if (itr_avg_ < run_avg_n_){
//     itr_avg_++;
//     pose_avg_array_[idx_replace_] = pose_;
//     // ROS_INFO("  not filled: idx_replace_(%d)", idx_replace_);

//   }
//   else{
//     init_dock_goal_ = true;

//     //replace the oldest element with the latest one
//     pose_avg_array_[idx_replace_] = pose_;

//     //calculate average
//     for (int i = 0; i < run_avg_n_; i++){
      
//       sum_pos_x += pose_avg_array_[i].pose.position.x;
//       sum_pos_y += pose_avg_array_[i].pose.position.y;

//       sum_ori_z += pose_avg_array_[i].pose.orientation.z;
//       sum_ori_w += pose_avg_array_[i].pose.orientation.w;

//       // ROS_INFO("    single: pos.x(%f), pos.y(%f), ori.z(%f), ori.w(%f)", 
//       //         sum_pos_x, sum_pos_y,
//       //         sum_ori_z, sum_ori_w);

//     }

//     pose_avg_.header.frame_id = "map";
//     pose_avg_.pose.position.x = sum_pos_x/run_avg_n_;
//     pose_avg_.pose.position.y = sum_pos_y/run_avg_n_;
//     pose_avg_.pose.position.z = 0.0;

//     pose_avg_.pose.orientation.x = 0.0;
//     pose_avg_.pose.orientation.y = 0.0;
//     pose_avg_.pose.orientation.z = sum_ori_z/run_avg_n_;
//     pose_avg_.pose.orientation.w = sum_ori_w/run_avg_n_;

//     // ROS_INFO("  filled: idx_replace_(%d)", idx_replace_);

//     // ROS_INFO("    AVERAGE: pos.x(%f), pos.y(%f), ori.z(%f), ori.w(%f)", 
//     //           pose_avg_.pose.position.x, pose_avg_.pose.position.y,
//     //           pose_avg_.pose.orientation.z, pose_avg_.pose.orientation.w);
//   }

//   //reset to position if it exceeds the size of the array
//   // if(idx_replace_%(run_avg_n_-1) == 0){
//   //   idx_replace_ = 0;
//   // }
//   // idx_replace_++;
  
// }

// void AprilDocking::getNextID(std::string& id_array_){
//   std::string error_msg = "";
//   parseVVF(id_array_, error_msg, id_array_vec_);

//   if (error_msg != ""){
//     ROS_ERROR("%s",error_msg.c_str());
//   }
// }

void AprilDocking::parseVVF(const std::string& input, std::string& error_return, std::vector<int>& result)
{
  result.clear();
  std::stringstream input_ss(input);
  int depth = 0;
  while (!!input_ss && !input_ss.eof())
  {
    switch (input_ss.peek())
    {
    case EOF: //end of file
      break;
    case '[': //start of array
      depth++;
      if (depth > 1)
      {
        error_return = "Array depth greater than 1";
        return;
      }
      input_ss.get(); //skip this character
      break;
    case ']': //end of array
      depth--;
      if (depth < 0)
      {
        error_return = "More close ] than open [";
        return;
      }
      input_ss.get(); //skip this character
      break;
    case ',':
    case ' ':
    case '\t':
      input_ss.get();
      break;
    default:  // All other characters should be part of the numbers.
      if (depth != 1)
      {
        std::stringstream err_ss;
        err_ss << "Numbers at depth other than 1. Char was '" << char(input_ss.peek()) << "'.";
        error_return = err_ss.str();
        return;
      }
      int32_t value;
      input_ss >> value;
      if (!!input_ss){
        result.push_back(value);
      }
      break;
    }
  }

  if (depth != 0)
  {
    error_return = "Unterminated vector string.";
  }
  else
  {
    error_return = "";
  }

  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "april_pose");
  AprilDocking ar;
}


