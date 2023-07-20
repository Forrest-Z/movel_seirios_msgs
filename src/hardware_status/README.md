# hardware\_status

This is a package for monitoring hardware status based on published topics by hardware drivers or ROS nodes of hardware drivers. List of tracked hardware can be configured in the config file.

## Prerequisites

This package requires ros\_utils package for parameter loading.

## Launch

Edit the launch file so that it loads the correct yaml file.

## Usage

Service: '/movel\_hardware\_status/get\_status'

Hardware states: **0** (**Inactive**: No data received from hardware), **1** (**Active**: Received data from hardware)

## Hardware list

### Motor

Subscribes to nav\_msgs/Odometry topic published by motor driver.

### 2D Lidar

Subscribes to sensor\_msgs/LaserScan topic published by lidar driver.

### 3D Lidar

Subscribes to sensor\_msgs/PointCloud2 topic published by lidar driver.

### Camera

Subscribes to sensor\_msgs/Image topic published by camera driver.

### Other Hardwares

Subscribes to topics of any type published by other hardware drivers not mentioned above.
