# The obstacle_detector package 

**Resource from :`https://github.com/tysik/obstacle_detector`**

The `obstacle_detector` package provides utilities to detect and track obstacles from data provided by 2D laser scanners. Detected obstacles come in a form of line segments or circles. The package was designed for a robot equipped with two laser scanners therefore it contains several additional utilities. The working principles of the method are described in an article provided in the `resources` folder.

The package requires [Armadillo C++](http://arma.sourceforge.net) library for compilation and runtime.

-----------------------

![picture](https://user-images.githubusercontent.com/1482514/27595825-0abe4338-5b5e-11e7-8438-ffdeec4e9cef.png)

Fig. 1. Visual example of obstacle detector output.

-----------------------
0. Set-up
1. The obstacle_extractor 
2. The messages
3. Launch files
4. The displays
5. Delete points
6. Final Preview


## 0. Set-up

To get this package work, you should `make` this package with **movel_seirios_msgs** and both/either **path_recall** and **plan_inspector**. 

For **movel** purposes, this package will publish a *obstacle_extractor/obstacles* topic with a circle consist of its radius, and its center.

To show the circle on Rviz, the terminal that you want to open RViz should source the workspace which consists of this package.

## 1. The obstacle_extractor node 

This node converts messages of type `sensor_msgs/LaserScan` from topic `scan` into obstacles which are published as messages of custom type `obstacles_detector/Obstacles` under topic `/obstacle_detector/obstacle`.

-----------------------

![picture](https://user-images.githubusercontent.com/1482514/27595822-0aa50ab2-5b5e-11e7-8061-1da4b947b617.gif)

Fig. 2. Visual example of `obstacle_extractor` output.

-----------------------

The input points are firstly grouped into subsets and marked as visible or not (if a group is in front of neighbouring groups, it is visible. Otherwise it is assumed to be occluded). The algorithm extracts segments from each points subset. Next, the segments are checked for possible merging between each other. The circular obstacles are then extracted from segments and also merged if possible. Resulting set of obstacles can be transformed to a dedicated coordinate frame.

The node is configurable with the following set of local parameters:

* `~use_split_and_merge` (`bool`, default: `true`) - choose wether to use Iterative End Point Fit (false) or Split And Merge (true) algorithm to detect segments,
* `~circles_from_visibles` (`bool`, default: `true`) - detect circular obstacles only from fully visible (not occluded) segments,
* `~discard_converted_segments` (`bool`, default: `true`) - do not publish segments, from which the circles were spawned,
* `~transform_coordinates` (`bool`, default: `true`) - transform the coordinates of obstacles to a frame described with `frame_id` parameter,
* `~min_group_points` (`int`, default: `5`) - minimum number of points comprising a group to be further processed,
* `~max_group_distance` (`double`, default: `0.1`) - if the distance between two points is greater than this value, start a new group,
* `~distance_proportion` (`double`, default: `0.00628`) - enlarge the allowable distance between points proportionally to the range of point (use scan angle increment in radians),
* `~max_split_distance` (`double`, default: `0.2`) - if a point in group lays further from a leading line than this value, split the group,
* `~max_merge_separation` (`double`, default: `0.2`) - if distance between obstacles is smaller than this value, consider merging them,
* `~max_merge_spread` (`double`, default: `0.2`) - merge two segments if all of their extreme points lay closer to the leading line than this value,
* `~max_circle_radius` (`double`, default: `0.6`) - if a circle would have greater radius than this value, skip it,
* `~radius_enlargement` (`double`, default: `0.25`) - artificially enlarge the circles radius by this value,
* `~frame_id` (`string`, default: `map`) - name of the coordinate frame used as origin for produced obstacles (used only if `transform_coordinates` flag is set to true).
* `~debug_scan` (`bool`, default: `false`) - whether to show the filtered laserscan or not.


## 2. The messages

The package provides three custom message types. All of their numerical values are provided in SI units.

* `CircleObstacle`
    - `geometry_msgs/Point center` - center of circular obstacle,
    - `geometry_msgs/Vector3 velocity` - linear velocity of circular obstacle,
    - `float64 radius` - radius of circular obstacle with added safety margin,
    - `float64 true_radius` - measured radius of obstacle without the safety margin.
* `SegmentObstacle`
    - `geometry_msgs/Point first_point` - first point of the segment (in counter-clockwise direction),
    - `geometry_msgs/Point last_point` - end point of the segment.
* `Obstacles`
    - `Header header`
    - `obstacle_detector/SegmentObstacle[] segments`
    - `obstacle_detector/CircleObstacle[] circles`

## 3. The launch files

Provided launch file is a good example of how to use `obstacle_detector` package.
* `nodes_pcl.launch` - Runs the node with its parameters set in obstacle_detector.yaml

## 4. The displays

For better visual effects, appropriate Rviz display for `Obstacles` messages was prepared. Via its properties, one can change the colors of the obstacles.


## 5. Delete points if the lidar scan lies on the wall/seen object from mapping (Added Feature from Movel AI)

For Movel AI purposes, circle should be generated just for unseen object. In order to that, we see the `occupancyGrid` on the lidar scan coordinate from `map` frame whether it's occupied space, free space, or unknown space. If the cell is a occupied space, we don't use the points.

Params: 
* `~num_neighbors_check` (`int`, default: `2`) - number of neighborhood cells around lidar scan points.

