# RTAB-Map Handler Plugin

A task\_supervisor plugin for starting and stopping mapping with rtabmap using camera. Once mapping is started, stopping the mapping requires that the *save_map* service be called, or a goal cancellation is issued to the task supervisor.

**Task type: 9**

## Prerequisites

* [rtabmap\_ros](http://wiki.ros.org/rtabmap_ros)

## task\_supervisor Config Setup
* Add rtabmap\_handler to 'plugins' section in task\_supervisor.yaml with class : rtabmap\_handler::RtabmapHandler. Example:

```
  - {name: rtabmap_handler, type: 9, class: 'rtabmap_handler::RtabmapHandler'}
```

* Add a 'rtabmap\_handler' section as follows:

```
rtabmap_handler:
  watchdog_rate: 2.0
  watchdog_timeout: 1000.0
  mapping_launch_package: "movel"
  mapping_launch_file: "rtabmap_mapping.launch"
  rgb_topic: "/camera/color/image_raw"
  depth_topic: "/camera/aligned_depth_to_color/image_raw"
  camera_info_topic: "/camera/color/camera_info"
```

* In 'localization\_handler' section, set the following parameter as shown below:

```
  localization_launch_file: "move_base_rtabmap.launch"
```

## Additional Config Setup

* In base\_local\_planner\_params.yaml, set the following parameter:

```
  odom_topic: /rtabmap/odom
```

## Usage

The handler's function is to start and stop RTAB-Map mapping. Once mapping is started, stopping it requires that the *save_map* service be called, or a goal cancellation issued to task\_supervisor.

### Task Payload Format

This task handler takes a single string argument with a full file path as payload in '/task_supervisor/goal' topic. Full path must have no extension. Example:

```
..., payload: '/home/movel/.config/movel/maps/test', ...
```

Map will then be saved to */home/movel/.config/movel/maps/test.pgm* and */home/movel/.config/movel/maps/test.yaml*, while corresponding RTAB-Map database is saved to */home/movel/.config/movel/maps/test.db*

### Services

* \_~/rtabmap\_handler/save\_map

This service is only available once mapping has started. No input arguments are required.

Saving will be done by map\_server's **map_saver** node. Example:

      rosservice call /task_supervisor/rtabmap_handler/save_map

* \_~/rtabmap\_handler/save\_map\_async

This service is the same as *save_map* above except this service will not stop mapping once saving is complete.

## Parameters

***Required***

* *mapping_launch_package*

Package of mapping launch file (specified in param below) to be launched for mapping.

* *mapping_launch_file*

Launch file in the specified mapping_launch_package to launch for mapping. This launch file should **only launch a rtabmap node** and load its relevant configuration parameters.

* *rgb_topic*

Name of RGB image topic published by camera.

* *depth_topic*

Name of depth image topic published by camera.

* *camera_info_topic*

Name of camera info topic published by camera.

***Optional***

* *loop_rate (default: 5Hz)*

Determines the rate at which handler will check if map has been saved, or if cancellation has been triggered by task_supervisor

* *save_timeout (default: 5s)*

Sets the number of seconds before map saving times out. Map saving time is dependent on map size

* *map_topic (default: /map)*

Topic name to save map when *save_map* service is called

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
