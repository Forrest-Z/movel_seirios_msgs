# pallet_detection

This node detects pallets using point cloud input from depth camera (ifm camera preferred), and sends the goal for initial phase docking.

## Prerequisites

1. Point cloud input

2. PCL library

3. planner_adjuster / move_base (for docking)

4. ros_utils

## Launch

Tune the pallet detection parameters in pallet_detection.yaml (refer to 'config' folder for default parameters).

## Usage

### Input topics

1. **/camera/cloud** : Point cloud input

2. Docking completion check: **/goal/status** (planner_adjuster) or **/move_base/result** (move_base)

### Output topics

1. **/pallet_detection/current_goal** : Publish real time updated docking goal based on real time detection

2. Goal output: **/pid_goal** (planner_adjuster) or **/move_base_simple/goal** (move_base)

## Debugging

1. **/pallet\_detection/filtered\_cloud** : Pre-processed input point cloud

2. **/pallet\_detection/pallets** : Display estimated pallet position

3. **/pallet\_detection/centroids** : Display centroids of segmented clusters

## Reference

[https://mediatum.ub.tum.de/doc/1453019/document.pdf](https://mediatum.ub.tum.de/doc/1453019/document.pdf)


# pallet_docking

This node is for the final phase docking or undocking with forward/backward motion only.

## Prerequisites

1. Odometry source

2. planner_adjuster/move_base to trigger this node

3. ros_utils

## Launch

Edit the launch file so that required parameters are loaded (refer to 'config' folder for default parameters).

## Usage

### Input topics

1. **/odom** : Odometry data

2. Trigger : **/pid_goal** (planner_adjuster) or **/move_base_simple/goal** (move_base)

3. **/pallet_detection/current_goal** : Check goal deviation during docking

### Output topics

1. **/cmd_vel_mux/autonomous** : Velocity command output

2. **/pallet_docking/success** : Signals end of docking while returning success/failure