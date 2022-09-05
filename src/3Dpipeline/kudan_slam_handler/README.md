# Kudan Slam Handler Plugin
A task\_supervisor plugin for starting and stopping Kudan Mapping.

## Prerequisites
* kdlidar_ros package

## task\_supervisor Config Setup

* Add kudan\_slam\_handler to 'plugins' section in task\_supervisor.yaml with class : task\_supervisor::KudanSlamHandler and comment if there's any plugin with the same type. Example:

```
  - {name: kudan_slam_handler, type: 32, class: 'task_supervisor::KudanSlamHandler'}
```

* Add a 'kudan_slam_handler' section with the following parameters:
```
kudan_slam_handler:
  watchdog_rate: 2.0
  watchdog_timeout: 0.0
  save_timeout: 20.0
  map_topic: "/map"
  kudan_slam_launch_package: "kdlidar_ros"
  kudan_slam_launch: "kdlidar_ros_pcl_mapping.launch"
  use_dynamic_2d: true
  kudan_map_saver_package: "task_supervisor"
  kudan_map_saver_launch: "map_saver.launch"
  three_to_two_package: "movel_octomap_server"
  three_to_two_launch: "pointcloud_grid.launch"
  temp_map_name: "/home/movel/.config/movel/maps/temp_rtabmap_save_"
```
## Kudan Setup
* Install kdlidar_ros package into /home/$USER folder

* Change the permission of kdlidar_ros, so it can execute in docker
```
chmod -R +x /home/$USER/kdlidar_ros/
```

* Configure Kudan TF parameters in the mapping launch file (Lidar to Base_Link) (/home/$USER/kdlidar_ros/install/share/kdlidar_ros/launch)
```
<rosparam param="platform_to_lidar0_rot">
            [1, 0,  0,
            0,  1,  0,
            0, 0,  1]
      </rosparam>
      <rosparam param="platform_to_lidar0_tran">
            [0.032, 0.000, -0.178]
            <!-- [0, 0, 1.73] -->
      </rosparam>
```

## Docker Setup
* Under seirios-ros volumes, add the following line:
```
- /home/$USER/kdlidar_ros:/home/movel/kdlidar_ros:rw 
- /home/$USER/ros_entrypoint.sh:/ros_entrypoint.sh:rw  
```

* Edit ros_entry point by adding the following line:
```
source /home/movel/kdlidar_ros/install/setup.bash
```
