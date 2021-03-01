# Camera Lidar Docking

This code has been modified in order to merge the perception and navigation based. So here is the explanation of these two functions.

## FIRST FUNCTION : Camera-navigation-based docking (Current)
This code considers the inputs from camera. The transformation of the pallet wrt map / odom is considered to move the robot to the near pallet. This code excecutes the movement of the robot by taking the tf from camera_docking package. After the code finished the camera-navigation, the code calls the service from navigation_based_docking package to move the robot under the pallet.

This packages doesn't need the another lidar-based packages (concatenate_point_cloud or pointcloud_to_laserscan) because it doesn't use lidar-dock.

To avoid the obstacle, robot uses the move_base planner by using the local costmap.

The code can be used again by call the rosservice again.

### Usage
You need to call the service by the following command
##### rosservice call /camera_lidar_docking/StartAutoDocking "x: 0.0 y: 0.0 theta: 0.0 operation: ''"

x - x coordinate of the pt 2 m away from pallet
y - x coordinate of the pt 2 m away from pallet
theta - orientation at which the tobot should stop
operation - pickup / drop 

#### To run the rospackage, you need to run the following command

##### roslaunch camera_lidar_docking camera_lidar_docking.launch


* nb : to make it works, you should run the camera_docking, navigation_based_docking, move_base package first.

## SECOND FUNCTION : Camera-lidar-based docking

This code considers the inputs from both camera and lidar.The transformation of the pallet wrt map / odom is considered along with the concatenated point cloud for computing a navigable path for going under the pallet. After attaining the transformation of the pallet, the robot naviagtes to the centre of the front legs of the pallet, 1m away from it. Thereafter, the lidar takes over as the aruco markers are not visible to the camera. Lidar detects the legs of the pallet by forming clusters of point cloud points segregated on the basis of intensity.
Then the algorithm finds the pair of cluster legs closest to the robot and navigation follows suit on the basis of following the mid point of the cluster pairs.

A service needs to be called that gives the robot the desired location 2m awy from the pallet. Then the robot searches for the markers and the pipeline runs.

* The code for this second function is labeled upper the function definition by : /* -------------- Lidar-based docking ------------- */

### Usage 
You need to call the service by the following command
##### rosservice call /camera_lidar_docking/StartAutoDocking "x: 0.0 y: 0.0 theta: 0.0 operation: ''"

x - x coordinate of the pt 2 m away from pallet
y - x coordinate of the pt 2 m away from pallet
theta - orientation at which the tobot should stop
operation - pickup / drop 


#### To run the rospackage, you need to run the following command

##### roslaunch camera_lidar_docking camera_lidar_docking.launch

* nb : to make it works, you should run the camera_docking,concatenate_point_cloud, pointcloud_to_laserscan, move_base package first.

## Parameters
You can tune the parameters by editing the my_params.yaml file in the config folder

1. pallet_orientation_angle: -172      - angle required to maintain orientation with the pallet legs

2. pallet_intensity: 100    - intensity value received from the pallet legs

3. pallet_length: 2   length of the pallet. Tghe centre of the pallet will lie at 1m away from the front legs

4. kp_orientation: 0.01  kp value for tuning the orientation wrt pallet legs

5. kd_orientation: 0.0   kd value for tuning the orientation wrt pallet legs

6. kp_movement: 0.8      kp tuning in maintaining the robot at the centre wrt the pallet legs

7. kd_movement: 0.1      kd tuning in maintaining the robot at the centre wrt the pallet legs

8. max_linear_speed: 0.05  max linear speed the robot can attain

9. max_angular_speed: 0.1  max angular speed the robot can attain to rotate

This package represents perception based approach for docking under the pallet

