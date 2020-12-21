## Camera Lidar Docking

The package is responsible for docking the robot to the charging station. We use aruco markers detected by the camera and a lidar point cloud. The code searches for an ruco marker in the camera image and also simultaneously searches for the cluster of the charging dock taking into consideration the intensity values from the charging station and its geometrical length. When both the marker and charging station are detected by camera and lidar simultaneously, only then the navigation starts. The robot tries to align itself in the middle of the charging station which also considers that the aruo marker centre and the charger centre coincides. 

Obstacle detection is also taken care of by the code. If any obstacle is present in front of the robot given that the robot is away from the marker, it will stop and wait for the obstacle to clear. If the obstacle doesn't move, the program terminates after waiting for a specific period of time, a parameter that can be changed in the yaml file present in the config folder.


Tunable parameters are

* dist_to_dock_station - distance from the station  to stop the robot just before the aruco  marker used for docking

* marker_slope_lidar_threshold  - slope of the marker wrt lidar used for docking 

* marker_intensity - intensity of lidar rays when they strike the aruco marker or the docker/charger

* makrer_intensity_threshold - error value wrt marker intensity i.e  marker intensity +- marker intensity threshold 

* cluster_length - length of the aruco marker / charging station in use for docking which will form a cluster in the point cloud

* cluster_length_threshold - error value wrt cluster length i.e cluster length +- cluster_length_threshold

* dist_camera_to_lidar_switch - distance from the charging station given by the lidar at which the robot uses lidar and not camera for docking. The camera is not able to see the aruco marker when it is closer than 0.25m

* speed_translation_max - maximum permissible translation / forward speed of the robot

* speed_angular_max -  maximum permissible angular speed of the robot

* max_delta_error_slope - max permissible error in slope of the marker from the required slope of the marker in the charger.yaml file

* robot_width - width of the robot

* robot_length - length of the robot

* object_distance_threshold -  distance of obstacle at which the robot comes to a halt

* min_points_obstacle -  minimum number of points within a given thresold distance to qualify a point as an obstacle

* max_err_y -  max error limit outside which the revovery will start i.e if err_y > max_err_y -> start recovery which is to go to the docking position

* kp - 0.05 kp parameter

* kd - 0.05 kd parameter

* time_for_obstacle_clearance - time to wait for obstacle clearance in sec before aborting the task


charger.ymal in the config folder contains the coordinates of the charging station


### Usage

roslaunch camera_lidar_docking camera_lidar_docking.launch 

rosservice call /camera_lidar_docking/StartAutoDocking "x: 0.0
y: 0.0
theta: 0.0"

You need to define the location from where the docking will start i.e 1 m away from the charger with the orientation facing towards the charger

After that both the marker and the charging station are detected in camera and lidar simultaneously, the robot starts to move
