# path_recall

This is a package for providing functions on handling the saving and loading of robot navigation paths in a 2d plane using ROS.

## Prerequisites

This package requires the following:

1. ros_utils

2. move_base ([link](http://wiki.ros.org/move_base))

3. A node that broadcasts the current pose of the robot in real-time in geometry_msgs::Pose format to a ROS topic.

## Launch

Edit the launch files for the nodes so that:

1. Required parameters are loaded (refer to 'config' folder for default parameters)

2. Directory for saving and loading path files is specified in the parameter named 'yaml_path'

## Usage

### path_saver

This is a path saving node that saves each path in individual yaml files in the specified directory.

##### Services

1. Recording by teleoperation

    * Start service '/path_saver/start' : activates path saving and users can start teleop

    * Stop service '/path_saver/stop'   : stops path saving

2. Direct saving by passing path data through service call

    * Save service '/path_saver/save'

### path\_load\_segments

This is a path loading node that loads paths from yaml files and executes the path by sending waypoints of the path in sequence to move_base node while skipping waypoints blocked by obstacles as a form of obstacle avoidance.

##### Services

1. Load service by path name '/path_load/load' : loads path from yaml file with path name as input

2. Direct load service '/path\_load/path\_input' : directly loads path given in the service call

3. Cancel service '/path_load/cancel'          : cancels and stops path following

4. Pause service '/path_load/pause'            : pauses path following to be resumed later

5. Resume service '/path_load/resume'          : resumes paused path following

### path_recovery

This code checks for uncovered path segments after path loading is done and uses path\_saver to save the segments as a path in a yaml file to be loaded later if needed. **It must be launched alongside path_saver and path\_load\_segments.**

##### Services

1. Recovery service '/path_recovery/recovery' : loads recovery path for uncovered segments of original path with the name of original path as input

