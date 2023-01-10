# Anti Shin Buster

## Config Tuning




### anti_shin_buster.yaml

- topic_depth_pointcloud (points topic name of depth cloud input for obstacle detection)
    - Note: if there is nothing in /obstacles_scan, edit this to match the pointcloud topic from the rgbd camera, usually /d435/depth/color/points

- min_z,max_z: Filterning pointcloud in z axis, min height and max height set based on the height of the robot. Reference frame is from base_link

### pointcloud_to_laserscan.yaml

* `angle_min` (double, default: -π) - The minimum scan angle in radians.
* `angle_max` (double, default: π) - The maximum scan angle in radians.
* `angle_increment` (double, default: π/180) - Resolution of laser scan in radians per ray.
* `range_min` (double, default: 0.0) - The minimum ranges to return in meters.
* `range_max` (double, default: 1.8e+308) - The maximum ranges to return in meters.