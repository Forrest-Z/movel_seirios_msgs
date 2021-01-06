
#include <ipa_room_exploration/room_exploration_client.h>
#include <cstdint>

// overload of << operator for geometry_msgs::Pose2D to wanted format
std::ostream& operator<<(std::ostream& os, const geometry_msgs::Pose2D& obj)
{
  std::stringstream ss;
  ss << "[" << obj.x << ", " << obj.y << ", " << obj.theta << "]";
  os << ss.rdbuf();
  return os;
}

bool ExplorationClient::pathPlan(ipa_room_exploration_msgs::RoomExplorationClient::Request& req,
                                 ipa_room_exploration_msgs::RoomExplorationClient::Response& res)
{
  map_path = req.path_to_cropped_map;
  std::string points_file_path = req.path_to_coordinates_txt;
  std::cout << points_file_path << std::endl;

  std::string line;
  std::ifstream myfile(points_file_path);
  if (myfile.is_open())
  {
    myfile >> origin[0] >> origin[1];
    myfile >> start_pos[0] >> start_pos[1];
  }
  myfile.close();

  if (start_pos.size() != 3)
  {
    ROS_FATAL("starting_position must contain 3 values");
    return -1;
  }

  ROS_INFO("Waiting for action server to start.");

  // wait for the action server to start for 10 sec
  if (!ac.waitForServer(ros::Duration(10.0)))
  {
    res.success = false;
    std::cout << "nothing received from server" << std::endl;
    return true;
  }
  ROS_INFO("Action server started, sending goal.");

  requested = true;
  res.success = true;
  return true;
}

bool ExplorationClient::planPath()
{
  //	actionlib::SimpleActionClient<ipa_building_msgs::RoomExplorationAction> ac("room_exploration_server", true);
  cv::Mat map_flipped = cv::imread(map_path, 0);
  cv::Mat map;
  cv::flip(map_flipped, map, 0);
  // make non-white pixels black
  for (uint16_t y = 0; y < map.rows; y++)
  {
    for (uint16_t x = 0; x < map.cols; x++)
    {
      // find not reachable regions and make them black
      if (map.at<unsigned char>(y, x) < 250)
      {
        map.at<unsigned char>(y, x) = 0;
      }
      // else make it white
      else
      {
        map.at<unsigned char>(y, x) = 255;
      }
    }
  }
  std::cout << "map-size: " << map.rows << "x" << map.cols << std::endl;

  sensor_msgs::Image labeling;
  cv_bridge::CvImage cv_image;
  //	cv_image.header.stamp = ros::Time::now();
  cv_image.encoding = "mono8";
  cv_image.image = map;
  cv_image.toImageMsg(labeling);

  geometry_msgs::Pose map_origin;
  map_origin.position.x = origin[0];
  map_origin.position.y = origin[1];
  map_origin.position.z = origin[2];

  geometry_msgs::Pose2D starting_position;
  starting_position.x = start_pos[0];
  starting_position.y = start_pos[1];
  starting_position.theta = start_pos[2];

  std::vector<geometry_msgs::Point32> fov_points(4);
  fov_points[0].x = 0.04035;  // this field of view represents the off-center iMop floor wiping device
  fov_points[0].y = -0.136;
  fov_points[1].x = 0.04035;
  fov_points[1].y = 0.364;
  fov_points[2].x = 0.54035;  // todo: this definition is mirrored on x (y-coordinates are inverted) to work properly
                              // --> check why, make it work the intuitive way
  fov_points[2].y = 0.364;
  fov_points[3].x = 0.54035;
  fov_points[3].y = -0.136;
  //	int planning_mode = 2;	// viewpoint planning

  int planning_mode = 1;  // footprint planning
  geometry_msgs::Point32 fov_origin;
  fov_origin.x = 0.;
  fov_origin.y = 0.;

  ipa_building_msgs::RoomExplorationGoal goal;
  goal.input_map = labeling;
  goal.map_resolution = resolution;
  goal.map_origin = map_origin;
  goal.robot_radius = robot_radius;        // turtlebot, used for sim 0.177, 0.4
  goal.coverage_radius = coverage_radius;  // by default it is being read as 0.25 in config file. the thing is it will
                                           // move in y dirn next y will be ycur + 0.25 and so on
  goal.field_of_view = fov_points;
  goal.field_of_view_origin = fov_origin;
  goal.starting_position = starting_position;
  goal.planning_mode = planning_mode;

  ac.waitForServer(ros::Duration());
  ac.sendGoalAndWait(goal);
  ipa_building_msgs::RoomExplorationResultConstPtr action_result = ac.getResult();

  std::cout << "Got a path with " << action_result->coverage_path.size() << " nodes." << std::endl;

  // display path
  const double inverse_map_resolution = 1. / goal.map_resolution;
  cv::Mat path_map = map.clone();
  for (size_t point = 0; point < action_result->coverage_path.size(); ++point)
  {
    const cv::Point point1((action_result->coverage_path[point].x - map_origin.position.x) * inverse_map_resolution,
                           (action_result->coverage_path[point].y - map_origin.position.y) * inverse_map_resolution);
    cv::circle(path_map, point1, 2, cv::Scalar(128), -1);
    if (point > 0)
    {
      const cv::Point point2(
          (action_result->coverage_path[point - 1].x - map_origin.position.x) * inverse_map_resolution,
          (action_result->coverage_path[point - 1].y - map_origin.position.y) * inverse_map_resolution);
      cv::line(path_map, point1, point2, cv::Scalar(128), 1);
    }
    // std::cout << "coverage_path[" << point << "]: x=" << action_result->coverage_path[point].x << ", y=" <<
    // action_result->coverage_path[point].y << ", theta=" << action_result->coverage_path[point].theta << std::endl;
  }
  // cv::imwrite("/home/sudhakar/room_1.png", path_map);
  //    cv::imshow("path", path_map);
  //    cv::waitKey();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "room_exploration_client");
  ros::NodeHandle priv_nh("~");
  ros::NodeHandle nh;

  //     DynamicReconfigureClient drc_exp(nh, "room_exploration_server/set_parameters",
  //     "room_exploration_server/parameter_updates");
  //     drc_exp.setConfig("room_exploration_algorithm", 8);
  //     drc_exp.setConfig("execute_path", false);

  // read params

  double resolution;
  priv_nh.param("resolution", resolution, 0.05);
  std::vector<double> origin = { -10, -10, 0 };
  priv_nh.param("origin", origin, origin);
  double robot_radius;
  priv_nh.param("robot_radius", robot_radius, 0.3);
  double coverage_radius;
  priv_nh.param("coverage_radius", coverage_radius, 1.0);
  std::vector<double> start_pos = { 0, 0, 0 };  //{-3, 1, 0};//world partition{-2, 1, 0};//{-3, 1, 0}; // R3{-5, 3, 0};
                                                //// R2{3, 0, 0};// R1{-3, 1, 0};
  priv_nh.param("starting_position", start_pos, start_pos);
  ExplorationClient excl(resolution, origin, robot_radius, coverage_radius, start_pos);
  // excl.displayParams();
  excl.service = priv_nh.advertiseService("start", &ExplorationClient::pathPlan, &excl);

  ros::Rate r(10);
  while (ros::ok())
  {
    if (excl.get_requested())
    {
      excl.planPath();
    }
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
