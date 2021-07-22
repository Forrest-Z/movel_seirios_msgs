# pointcloud\_mux

This is a package for selecting a pointcloud to be used among pointclouds from multiple cameras based on direction of motion.

## Config Setup

1. Set the pointcloud topics representing the front, rear, left and right views of the robot.

2. Set the angular speed threshold to trigger the switch to left/right views from front/rear views.

## Usage

Published topic: '/movel\_pointcloud\_mux/pointcloud'
