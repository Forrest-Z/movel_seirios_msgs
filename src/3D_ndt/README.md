# Ndt_mappingLocalization
3D mapping and localization with ndt of autoware
 
for simulation check on 3DMLNC package of this github to get the 3d environment and also it will be compatible with ndt package

# for mapping 
``` 
 roslaunch diff_wheeled_robot_gazebo diff_wheeled_gazebo_full.launch
 roslaunch lidar_localizer ndt_mapping.launch 
 roslaunch diff_wheeled_robot_control keyboard_teleop.launch
 ```
 # To save
 ```
   rostopic pub /config/ndt_mapping_output autoware_config_msgs/ConfigNDTMappingOutput "header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: ''
  filename: '/home/navnaeeth/auto_ws/finalNdtMapPT2.pcd'
  filter_res: 0.2"
 ```
# for Localization
```
roslaunch diff_wheeled_robot_gazebo diff_wheeled_gazebo_full.launch
 roslaunch lidar_localizer ndt_matching.launch
```
# For Navigation
  see the document in 3DMLNC package
 
  
