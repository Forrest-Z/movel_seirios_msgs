## Overview

This package is used to limit the velocity of the robot when there are obstacles near its vicinity. Values of velocity limitation are calculated for each cell in the velocity grid, which covers user defined areas. Point cloud data that represents the obstacles is read. If a point falls into a position with velocity limit, the velocity of the robot will take the limited values.


## Quick Launch

```
roslaunch velocity_limiter velocity_limiter.launch
```

## Subscribed Topics

*~cmd_vel* (geometry_msgs/Twist)

* Input velocity before velocity limiting.

*~cloud* (sensor_msgs/PointCloud2)

* Point cloud data representing the surrounding obstacles.

*~clicked_point* geometry_msgs/PointStamped)

* The point clicked in the rviz software. velocity limit values of the point will be shown for debugging.

## Published Topics

*~cmd_vel/capped* (geometry_msgs/Twist)

* After receiving the subscribed topic /cmd_vel, the velocity limit will be calculated and the capped velocity will be published.

*~velocity_grid* (nav_msgs/OccupancyGrid)

* If the service ```publish_velocity_grid``` is called, the velocity grid showing limit values is published.

*~velocity_frontiers* (geometry_msgs/PolygonStamped)

* If the service ```publish_limit_zones``` is called, the polygons representing the user-defined frontiers are published.

*~cloud/persisted* (sensor_msgs/PointCloud2)

* Publish the filtered point cloud data that contains only persistent points. Data points that appear less than a certain time are removed.

## Available Services

*enable_velocity_limiter* (std_srvs/SetBool)

* Enable velocity_limiter. velocity_limiter is enabled by default.

*switch_limit_set* (velocity_limiter/SwitchLimitSet)

* Switch limit_set. Switch to another limit_set that defines another set of velocity limit regions.

*publish_limit_zones* (std_srvs/Trigger)

* Publish the velocity limit zones of the current set.

*publish_velocity_grid* (velocity_limiter/PublishGrid)

* Publish the velocity gird that shows calculated velocity limit values.
## Parameters

*~base_frame* (string, default: ‘base_link’)

* The robot base frame.

*~merging_frame* (string, default: ‘odom’)

* The frame that the point cloud data will be merged in.

*~publish_pcl* (bool, default: true)

* Whether to publish the topic ```cloud/persisted```.

*~grid_resolution* (double, default: 0.02 meters)

* Resolution of the velocity grid, defining the size of each grid cell.

*~cloud_persistence* (double, default: 0.5 seconds)

* If the point loud data appears for less that this period, the data will be discard to avoid disturbance.

*~initial_limit_set* (string, default: ‘without_rack’)

* The initial set of velocity limit regions.

*~zone_sets* (map)

* A list of the sets of velocity limit regions.

*~zones* (map)

A list of zones in one limit set.

*~type* (string, allowed: "linear" and "angular")

Type of the velocity that the zone is applied to.

*~direction* (string, allowed: "positive" and "negative")

Direction of the velocity that the zone is applied to.

*~dimension* (string, allowed: "x" for linear and "z" for angular)

Dimension of the velocity that the zone is applied to.

*~frontiers* (map)

A list of frontiers in one zone.

*~inclusion* (int, allowed: 0, 1, 2, 3...)

Order of the frontiers. A frontier with larger inclusion number should contain the one with smaller inclusion  number entirely.** Note**: A frontier of inclusion 0 must exist, serving as critical frontier. Velocity within the critical frontier is always zero.

*~value* (double)

The velocity limit value of the frontier. If an obstacle is positioned right on the frontier, the velocity limit would be this value.

*~shape* (map)

The shape of the frontier. 

*~x_coordinates* and *~y_coordinates* (array of doubles)

Coordinates of the shape of the frontier. ** Notes**: The origin of the frame is the centre of the robot. The first and last element of the array must be equal.


## Tips on defining frontiers

In one limit set, there should be 4 zones representing 4 different velocity situations: linear positive velocity, linear negative velocity, angular positive velocity and angular negative velocity. 

Each zone has several frontiers defining the velocity limit. On the frontier line, velocity will be limited to the frontier value. Between 2 frontiers, the velocity limit values are in a gradient. 

A sample limit set definition is in the image below: 
![Zone3.jpeg](https://bitbucket.org/repo/nk4rgX9/images/279281985-Zone3.jpeg)

It is recommended that the distance between the outermost frontier and the robot outer frame should be the maximum safety distance. Also, the velocity limit value of the outermost frontier should be the maximum velocity of the robot.

For example, if a robot has a maximum linear positive velocity of 0.5 m/s, and the maximum distance of deceleration in this case is 10 m, the value of the outermost frontier should be 0.5m/s. The distance between the outermost frontier and the robot should roughly be 10m.
