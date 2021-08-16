# RTAB-Map Handler Plugin

A task\_supervisor plugin for starting and stopping mapping with rtabmap using camera(s). Once mapping is started, stopping the mapping requires that the *save_map* service be called, or a goal cancellation is issued to the task supervisor.

**Task type: 9**

## Prerequisites

* rtabmap\_ros\_multi ([rtabmap\_ros](http://wiki.ros.org/rtabmap_ros) with multi camera support)

## task\_supervisor Config Setup
* Add rtabmap\_handler to 'plugins' section in task\_supervisor.yaml with class : rtabmap\_handler::RtabmapHandler. Example:

```
  - {name: rtabmap_handler, type: 9, class: 'rtabmap_handler::RtabmapHandler'}
```

* Add a 'rtabmap\_handler' section with the following format:

```
rtabmap_handler:
  watchdog_rate: 2.0
  watchdog_timeout: 1000.0
  mapping_launch_package: "movel"
  mapping_launch_file: "rtabmap_multi_mapping.launch"
  camera_names: ["<camera 1>", "<camera 2>", ...]
```

* In 'localization\_handler' section, set the following parameter as shown below:

```
  localization_launch_file: "move_base_rtabmap_multi.launch"
```

## Additional Config Setup

* In base\_local\_planner\_params.yaml, set the following parameter:

```
  odom_topic: /rtabmap/odom
```

## Usage

The handler's function is to start and stop RTAB-Map mapping. Once mapping is started, stopping it requires that the *save_map* service be called, or a goal cancellation issued to task\_supervisor.

### Task Payload Format

This task handler takes no payload argument.

### Services

* \_~/rtabmap\_handler/save\_map

This service is only available once mapping has started. Example input argument:

```
'/home/movel/.config/movel/maps/test'
```
Map will then be saved to */home/movel/.config/movel/maps/test.pgm* and */home/movel/.config/movel/maps/test.yaml*, while corresponding RTAB-Map database is saved to */home/movel/.config/movel/maps/test.db*

Saving will be done by map\_server's **map_saver** node. Example:

      rosservice call /task_supervisor/rtabmap_handler/save_map

* \_~/rtabmap\_handler/save\_map\_async

This service is the same as *save_map* above except this service will not stop mapping once saving is complete.

## Parameters

***Required***

* *mapping_launch_package*

Package of mapping launch file (specified in param below) to be launched for mapping.

* *mapping_launch_file*

Launch file in the specified mapping_launch_package to launch for rtabmap mapping.

* *camera_names*

List of camera names (up to 4).

***Optional***

* *loop_rate (default: 5Hz)*

Determines the rate at which handler will check if map has been saved, or if cancellation has been triggered by task_supervisor

* *save_timeout (default: 5s)*

Sets the number of seconds before map saving times out. Map saving time is dependent on map size

* *map_topic (default: /map)*

Topic name to save map when *save_map* service is called

## Camera Topic Setup

For each camera, remap its published topics as follows if needed:

Color image topic: /\<camera name\>/color/image_raw

Depth image topic: /\<camera name\>/aligned\_depth\_to\_color/image\_raw

Camera calibration data topic: /\<camera name\>/color/camera_info

## Hardware Driver Setup

### Intel RealSense Camera

In [realsense-ros](https://github.com/IntelRealSense/realsense-ros) repository, realsense2\_camera provides [opensource\_tracking.launch](https://github.com/IntelRealSense/realsense-ros/blob/development/realsense2_camera/launch/opensource_tracking.launch) for launching necessary nodes for RTAB-Map.

In opensource\_tracking.launch, only keep the sections to launch from realsense2\_camera and imu\_filter\_madgwick packages while commenting out the rest as they will be launched separately.

Reference: [https://github.com/IntelRealSense/realsense-ros/wiki/SLAM-with-D435i](https://github.com/IntelRealSense/realsense-ros/wiki/SLAM-with-D435i)

### ZED Stereo Camera

With [zed_ros_wrapper](https://github.com/stereolabs/zed-ros-wrapper), run:

```
roslaunch zed_wrapper zed.launch
```

* Set the tf w.r.t. base_link in zed.launch

* Important parameters to be set in zed_wrapper/params/common.yaml:
  * depth\_confidence: 100
  * depth\_texture\_conf: 90
  * pub\_frame\_rate
  * general/resolution
  * depth/quality
  * pos\_tracking/publish\_tf: false
  * pos\_tracking/publish\_map\_tf: false
