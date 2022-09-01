# Kudan Slam Handler Plugin
A task\_supervisor plugin for starting and stopping Kudan Localization.

## Prerequisites
* kdlidar_ros package

## task\_supervisor Config Setup

* Add kudan\_localizations\_handler to 'plugins' section in task\_supervisor.yaml with class : task\_supervisor::KudanLocalizationHandler, and comment if there's any plugin with the same type. Example:

```
  - {name: kudan_localization_handler, type: 31, class: 'task_supervisor::KudanLocalizationHandler'}
```

* Add a 'kudan_localization_handler' section with the following parameters:
```
kudan_localization_handler:
  watchdog_rate: 2.0
  watchdog_timeout: 0
  kudan_localization_launch_package: "kdlidar_ros"
  kudan_localization_launch_file: "kdlidar_ros_pcl_localise.launch"
  kudan_navigation_launch_file: "move_base.launch" 
  localization_map_dir: "/home/movel/.config/movel/maps"
  navigation_map_dir: "/home/movel/.config/movel/maps/nav"
  kudan_localization_launch_nodes: "/obs_cloud_to_scan /move_base /kdlidar_ros_pcl /velocity_limiter /anti_shin_buster_node /rgbd_to_base /velocity_setter_node /plan_inspector"
  temp_map_name: "/home/movel/.config/movel/maps/temp_rtabmap_save_"
  update_param_launch_file: "update_param_lpm.launch"
```
## Kudan Setup
* Install kdlidar_ros package into /home/$USER folder
* Change the permission of kdlidar_ros, so it can execute in docker
```
chmod -R +x /home/$USER/kdlidar_ros/
```

* Configure Kudan TF parameters in the localise launch file (Lidar to Base_Link) (/home/$USER/kdlidar_ros/install/share/kdlidar_ros/launch)
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
