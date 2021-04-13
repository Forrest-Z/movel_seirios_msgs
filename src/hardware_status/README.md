# hardware\_status

This is a package for monitoring hardware status based on published topics by hardware drivers or ROS nodes of hardware drivers. List of tracked hardware can be configured in the config file.

## Prerequisites

This package requires ros\_utils package for parameter loading.

## Launch

Edit the launch file so that it loads the correct yaml file.

## Usage

Service: '/hardware\_status/get\_status'

Hardware states: **0** (**Inactive**: No data received from hardware), **1** (**Active**: Received data from hardware)

## Hardware list

### Motor

Subscribes to nav_msgs/Odometry topic published by motor driver.

### 2d lidar

Subscribes to sensor_msgs/LaserScan topic published by lidar driver.

### 3d lidar

Subscribes to sensor_msgs/PointCloud2 topic published by lidar driver.

### Camera

Subscribes to sensor_msgs/Image topic published by camera driver.

### Other hardware

State of other hardware not mentioned above is determined by checking whether the respective driver nodes are running.
