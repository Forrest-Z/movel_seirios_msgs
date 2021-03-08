# AutoDock - Charging Station #

## Description
* This packages is used to move the robot to the docking station. This packages consist of :
    
    - camera_lidar_docking_pallet :  The transformation of the pallet wrt map / odom is considered along with the concatenated point cloud for computing a navigable path for going under the pallet

## Set-up

### Launch :
#### roslaunch camera_lidar_docking_charging_station camera_lidar_docking.launch

### Service call :
#### rosservice call /camera_lidar_docking_charging_station/StartAutoDocking "x: 2.0 y: 4.5 theta: 0.0 operation: 'pickup'"

*nb the value of x, y, theta, operation can be arbitrary according to the position of the docking position


