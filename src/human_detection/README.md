# human_detection

This node detects human-like objects using point cloud input from 3D lidar. It searches for objects with human-like dimensions in the point cloud, and compares them with the map of the robot's surroundings to detect dynamic obstacles which are likely to be humans.

## Prerequisites

1. 3D lidar point cloud input

2. Map input

3. A node that broadcasts the current pose of the robot in real-time in geometry_msgs::Pose format to a ROS topic.

4. 2D laser scan converted from 3D point cloud (usually converted with pointcloud\_to\_laserscan package)

5. *ros_utils* package for parameter loading

## Launch

Edit the launch file so that required parameters are loaded (refer to 'config' folder for default parameters).

## Usage

1. **/human_detection/detection**        : Detection value ranging from 0 to 1, where 0 is no detection, and 1 is detection with absolute certainty

2. **/human\_detection/human\_clusters** : Visualize bounding boxes of all detected human-like objects

## Debugging

1. **/human\_detection/filtered\_cloud** : Pre-processed input point cloud

2. **/human_detection/crop**             : Visualize crop edges for point cloud input and map

3. **/human\_detection/map\_cloud**      : Point cloud representation of cropped map

4. **/human\_detection/scan\_cloud**     : Point cloud representation of laser scan data converted from 3D lidar point cloud data

## Methodology

**1. Pre-processing**

* Crop point cloud input to only the area near the robot for faster processing

* Remove ground and ceiling from point cloud input


**2. Adaptive clustering**

Segment the point cloud into clusters where each cluster represents an object. The code is incorporated from https://github.com/yzrobot/adaptive_clustering


**3. 2D laser scan filter**

Filter for point cloud clusters with bounding boxes that contain points from the laser scan point cloud. This is required because this package filters for human clusters based on map that is created with laser scan converted from 3D lidar point cloud data, or directly from a 2D lidar.


**4. Map filter**

Compare the remaining point cloud clusters to the map input and filter out static obstacles that are marked in the map. Before filtering with map, point cloud is transformed to map frame from 3D lidar frame.


**5. Object dimensions filter**

Narrow down the number of point cloud clusters to only those with human-like dimensions based on bounding box dimensions. Desired dimensions can be configured in YAML file.


**6. History averaging**

Keep track of a user-defined number of past detection results, and output the average detection value of all tracked results. For each detection result, 0 represents no detection, and 1 represents detection. Hence, the average detection value ranges between 0 and 1.

## Citation for adaptive clustering
```
@article{yz19auro,
   author = {Zhi Yan and Tom Duckett and Nicola Bellotto},
   title = {Online learning for 3D LiDAR-based human detection: Experimental analysis of point cloud clustering and classification methods},
   journal = {Autonomous Robots},
   year = {2019}
}
```
