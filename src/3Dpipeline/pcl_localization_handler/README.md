### PCL Localization Handler

**Task type: 31**

PCL Localization handler starts and stops localization with PCL, and can be given a PCD file to be loaded for use during localization.

## Prerequisites

- PCL
- OpenMP

The following ros packages are required:
- pcl_ros
- [ndt_omp](https://github.com/koide3/ndt_omp)
- [fast_gicp](https://github.com/SMRT-AIST/fast_gicp)
- [hdl_global_localization](https://github.com/koide3/hdl_global_localization)

## task\_supervisor setup
* Add pcl_localization_handler to 'plugins' section in task\_supervisor.yaml with class : task\_supervisor::PCLLocalizationHandler

* Add a 'pcl_localization\_handler' section with the following parameters:
*   watchdog_rate: 2.0
*   watchdog_timeout: 0
*   pcl_localization_launch_package: "movel"
*   pcl_localization_launch_file: "3D_move_base.launch"
*   map_dir: "/home/movel/.config/movel/maps"

**Task Payload Format**

**Services**

* *~/pcl_localization_handler/start*

This service provides an option for users to start localization directly. Input arguments is a string indicating the full path to map that should be loaded, if any. Example:

	rosservice call /task_supervisor/pcl_localization_handler/start "input: '/home/map_office.yaml'"

Start localization with the PCD map at the full path given. PCD Map will be loaded using HDL Localization and 2D map will be laoded using map_server.

* *~/pcl_localization_handler/stop*

Stop localization, takes in no arguments.

**Topics**

***Published***

* *~/pcl_localization_handler/localizing*

Latching *std_msg/Bool* indicating whether localization is running. True when localization is running.

***Subscribed***

* */map*

Subscription to this topic is setup the moment localization_handler is loaded by task_supervisor.

Localization handler subscribes to this topic for cases when localization is started immediately after mapping is done, and no map file is specified to be loaded by map_server on start of localization.

Using this topic, localization_handler will pickup from where mapping stopped, and continue using the map received to do localization.
***Map topic name configurable in params***

**Parameters**

***Required***

* pcl_localization_launch_package*

Package of pcl_localization launch file (specified in the param below) to be launched for pcl_localization

* *pcl_localization_launch_file*

PCL Launch file in the specified pcl_localization_launch_package to launch for pcl_localization. This launch file should **only launch a PCL localization node** and load its relevant configurations.

***Optional***

* *loop_rate (default: 5Hz)*

Determines the rate at which handler will check if map has been saved, or if cancellation has been triggered by task_supervisor

* *set_map_timeout (default: 10s)*

This timeout is enforced when no map is loaded on start of localization. amcl provides a service */set_map* that can be used to set map and initial pose for amcl. If no map is loaded, this */set_map* service will be called to continue localization from where mapping left off.

* *map_topic (default: /map)*

Topic name to subscribe to for continuing localization if no map file is loaded.

* *map_frame (default: map)*

Specify name of map frame. Map frame is used to get pose from mapping node's TF broadcasters. Default transformation to get pose if from map -> base_link.

* *base_link_frame (default: base_link)*

Specify name of base link frame. Used to get pose of robot in map_frame. Default transformation to get pose if from map -> base_link.

* *set_map_service (default:/set_map)*

This service will depend on localization package used. For amcl package that was used for development of this handler, amcl provides a service */set_map* to set the initial pose and map occupancy grid.

If any other localization packages are used and a similar service is available, change this name. Service should be of the same form as amcl's */set_map* service.
