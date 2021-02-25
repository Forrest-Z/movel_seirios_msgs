## Overview

This package is an implementation of Olsen’s real-time correlative scan matching [paper](https://april.eecs.umich.edu/pdfs/olson2009icra.pdf).

In the paper, Olsen first generates probability grids from a reference scan. The cells corresponding to hits in the scan are given the maximum probability in the grid, and the cells around these hits are awarded decreasing probability based on a Gaussian noise model. Two probability grids are generated, one low-resolution and one high-resolution grid.

For subsequent scans, for a given search space each point in the scan is checked against the low-resolution probability grid to find the voxel with the highest probability. This voxel is then searched using the high-resolution probability grid to find the best matching pose. Running the match through the low-resolution grid first reduces the search space for the high-resolution grid and hence reduces computation time.

For our implementation, rather than using a reference scan, we are using a map to generate the probability grids. As this limits the probability grid resolution to the resolution of the map, raycasting was added to maximize probability at the edges of obstacles. Our implementation also adds an average filter to the pose estimates as the estimates tend to jump due to laser noise and possibly multiple estimates with equally high probability.

## Quick Launch

```
roslaunch correlative_scan_matcher real_time_csm.launch scan_topic:=<scan topic> map_topic:=<map_topic>
```

## Subscribed Topics

*~map* (nav_msgs/OccupancyGrid)

* If use_map_topic is true, CSM subscribes to this topic for the input map used for probability grid generation.

*~scan* (sensor_msgs/LaserScan)

* Input laser scans for scan matching.

## Published Topics
*initialpose* (geometry_msgs/PoseWithCovarianceStamped)

* If *publish_initial_pose* is true, CSM publishes its pose estimates to this topic.

*~raycast_output* (sensor_msgs/LaserScan)

* If *enable_raycast* is true, CSM publishes the raycast scan used for probability grid generation to this topic.

*~low_res_grid* (nav_msgs/OccupancyGrid)

* If *publish_tables* is true, CSM publishes the generated low-resolution grid on this topic.

*~high_res_grid* (nav_msgs/OccupancyGrid)

* If *publish_tables* is true, CSM publishes the generated high-resolution grid on this topic.

## Available Services
*set_csm_map* (correlative_scan_matcher/SetMap)

* If *use_map_topic* is false, the input map for probability grid generation can be set through this service.

*enable_csm* (std_srvs/SetBool)

* Enable CSM. CSM is disabled by default.

*pause_csm* (std_srvs/SetBool)

* Pause CSM. Pausing stops matching but still publishes the latest valid transform.

## Transforms

CSM publishes the following transform.

**reference_frame <- matched_frame**

Setting *reference_frame* to the odometry frame allows localization to continue using odometry when there is no sufficient estimate. In that case, CSM effectively acts as a correction for odometry drift, similar to AMCL.

If *invert_transform* is true, *matched_frame* is the corrected map frame, otherwise it is the corrected *base_frame*.

## Parameters

*~sigma* (double, default: 0.05 meters)

* Standard deviation for the Gaussian noise model used for probability grid generation. Since this models the laser noise, this should match the standard deviation of the laser.

*~search_x* (double, default: 0.1 meters)

* Search space in x. The search space is defined with respect to the *base_frame*, but is ignored if *do_global_search* is true.

*~search_y* (double, default: 0.1 meters)

* Search space in y. The search space is defined with respect to the *base_frame*, but is ignored if *do_global_search* is true.

*~search_a* (double, default: 0.35 radians)

* Search space in theta. The search space is defined with respect to the *base_frame*, but is ignored if *do_global_search* is true.

*~min_score* (double, default: 0.5, [0.0, 1.0])

* Minimum score for a candidate to be considered. Increasing the score will prune poor matches early but too high a score will result in no suitable matches.

*~high_grid_resolution* (double, default: 0.03 meters)

* Resolution of the high-resolution probability grid when using raycasting. Otherwise, the map resolution is used. The low-resolution grid always has 10x the resolution of the high-resolution grid.

*~enable_raycast* (bool, default: false)

* Whether to perform a raycast from the initial position estimate to generate the probability grids. Set to false for global localization.

*~raycast_lin_thres* (double, default: 0.5 meters)

* The minimum linear distance from the last raycasted pose to re-trigger a raycast.

*~raycast_ang_thres* (double, default: 1.57 radians)

* The minimum angular distance from the last raycasted pose to re-trigger a raycast.

*~base_frame* (string, default: ‘base_link’)

* The robot base frame. Used to get the initial pose estimate in the map frame.

*~reference_frame* (string, default: ‘map’)

* The published transform is of the form *reference_frame* <- *matched_frame* (*matched_frame* in *reference_frame*). Set this to odom for CSM to effectively act as a correction for odometry drift.

*~matched_frame* (string, default: ‘csm/base_link’)

* The name of the corrected frame. Which frame is corrected is determined by *invert_transform*.

*~show_corrected_scan* (bool, default: ~true~)

* Disabled. Due to a bug, the corrected scan is published with the wrong transforms.

*~publish_initial_pose* (bool, default: false)

* Whether to publish the *matched map* <- *base_frame* transform to */initialpose* to reset AMCL. The current implementation is not friendly to AMCL as it publishes every time it gets a pose estimate, which does not allow AMCL to properly build its filter.

*~do_global_search* (bool, default: false)

* Whether to do a global search. Sets *search_x* and *search_y* to the size of the map, and *search_a* to 360 degrees. Set to true for global localization.

*~use_map_topic* (bool, default: true)

* Whether to get the input map from the topic or the service.

*~first_map_only* (bool, default: true)

* If *use_map_topic* is true, whether to only process the first map received on the topic.

*~publish_last_valid_tf* (bool, default: false)

* Whether to publish the last valid transform when no new valid match is found. When used together with odom as *reference_frame*, allows localization to continue using odometry when there is no new estimate.

*~publish_tables* (bool, default: false)

* Whether to publish the probability grids. Slows down processing as populating the occupancy grid message requires iterating through the probability grids. Modify *getOccupancyGrid* in *real_time_csm_node.cpp* to choose whether to publish the obstacle map or the probabilities.

*~wait_for_transform* (double, default: 0.05 seconds)

* How long to wait for a transform to be available. If the value needs to be increased significantly, it could be a sign of a slow processor, connection issues or timing issues between multiple processors.

*~invert_transform* (bool, default: false)

* If true, transform published will be

  **reference_frame <- corrected map frame** (with name from matched_frame)
  
  else,
  
  **reference_frame <- corrected base_frame** (with name from matched_frame)

*~filter_window_size* (int, default: 1)

* Size of the average filter for pose estimates. For any value < 1 no filtering is performed.

*~use_last_match* (bool, default: false)

* If true, uses the last pose estimate rather than the map frame <- *base_frame* transform as the next initial estimate. The last pose estimate is reset whenever CSM is disabled.

*~min_valid_points* (int, default: 100)

* Minimum number of valid points in the scan for the scan to be used for matching.
