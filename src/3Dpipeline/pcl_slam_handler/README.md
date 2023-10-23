### PCL Mapping Handler
**Task type: 30**

PCL Mapping handler's function is to start and stop pcl mapping (currently implementing HDL_Slam). Once mapping is started, stopping the mapping requires that the *save_map* service be called, or a goal cancellation is issued to the task supervisor.

First a PCD map will be saved, before being converted to a 2D impression by slicing the PCD at a given height as configured in the config files. The slicing is done via octomaps tools. The 2D impression is then saved by map_server's map_saver.

## Prerequisites

- OpenMP
- PCL
- g2o
- suitesparse

The following ROS packages are required:

- geodesy
- nmea_msgs
- pcl_ros
- [ndt_omp](https://github.com/koide3/ndt_omp)
- [fast_gicp](https://github.com/SMRT-AIST/fast_gicp)

## task\_supervisor setup
* Add pcl_mapping_handler to 'plugins' section in task\_supervisor.yaml with class : task\_supervisor::PCLSlamHandler

* Add a 'pcl_mapping\_handler' section with the following parameters:
  *	watchdog_rate: 2.0
  *	watchdog_timeout: 0.0
  *	save_timeout: 10.0
  *	map_topic: "/map"
  *	pcl_slam_launch_package: "hdl_graph_slam"
  *	pcl_slam_launch: "hdl_graph_slam_501.launch"
  *	pcl_map_saver_package: "task_supervisor"
  *	pcl_map_saver_launch: "map_saver.launch"
  *	three_to_two_package: "movel_octomap_server"
  *	three_to_two_launch: "pointcloud_grid.launch"

**Task Payload Format**

This task handler does not take any payload arguments.

**Services**

* _~/pcl_mapping_handler/save_pcl_map

This service is only available once pcl mapping has started. A single string argument with a full path can be sent to this service to set pcl map save location. Full path must have no extension.

Saving will be done by map_server's **map_saver** node. Example:

	rosservice call /task_supervisor/pcl_mapping_handler/save_pcl_map "input: '/home/map'"

2D Map will then be saved to */home/map.pgm* and */home/map.yaml*
3D PCL map will then be saved to /home/map.pcd

**Parameters**

***Required***

* *pcl_slam_launch_package*

Package of pcl_slam launch file (specified in param below) to be launched for pcl_mapping

* *pcl_slam_launch*

Launch file in the specified pcl_slam_launch_package to launch for pcl_mapping. This launch file currently **only launch HDL Slam nodes** and load its relevant configuration parameters.

* *pcl_map_saver_package*

Package of pcl_map_saver file (specified in param below) to be launched for pcl_map saving.

* *pcl_map_saver_launch*

Launch file in the specified pcl_map_saver_launch_package to launch for pcl_map saving. This launch file currently **only launch nodes required for saving with HDL Slam**, and load its relevant configuration parameters.

* *three_to_two_package*

Package of three_to_two launch file (specified in param below) to be launched for converting PCD to PGM/Yaml pair.

* *three_to_two_launch*

Launch file in the specified three_to_two package to launch for 3D to 2D map conversion. This launch file currently **only launch Octomap Tools to convert 3D to 2D maps** and load its relevant configuration parameters.

***Optional***

* *loop_rate (default: 5Hz)*

Determines the rate at which handler will check if map has been saved, or if cancellation has been triggered by task_supervisor

* *save_timeout (default: 5s)*

Sets the number of seconds before map saving times out. Map saving time is dependent on map size

* *map_topic (default: /map)*

Topic name to save map when *save_map* service is called

* *resolution (default: /0.05)*

Resolution of each grid in the resultant 2D pgm map.
