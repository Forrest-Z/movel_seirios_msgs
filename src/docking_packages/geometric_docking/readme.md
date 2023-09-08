# Geometric Docking

For docking to plates of set width at set distance from a wall.

## Usage

- Launch geometric_docking.launch
- Move robot to within distance of docking plate
- publish message of type std_msgs/Empty to topic /start_dock
- wait for docking to complete

## Topics

Subscribes to:

- `scan` (sensor_msgs/LaserScan) for laser scan to find the docking plate
- `task_supervisor/cancel`  (actionlib_msgs/GoalID) task cancelling.
- `goal/status` (std_msgs/Bool) docking controller status, expected from package planner_adjuster

Publishes to:

- `dock_pose` (geometry_msgs/PoseStamped) shows the desired robot docking pose
- `pid_goal` (geometry_msgs/PoseStamped) pose to move the robot to for docking, expected by package planner_adjuster
- `stop_now` (std_msgs/Bool) to stop the docking controller, expected by package planner_adjuster
- `dock_candidates` (sensor_msgs/PointCloud2) visualises the dock candidate line segments
- `line_segments` (sensor_msgs/PointCloud2) visualises all line segments found in the laser scan
- `status`  (std_msgs/UInt8) Docking Status

As server for:

- `startDocking` (std_srvs/Trigger) to trigger finding dock plate and docking
- `cancelDocking` (std_srvs/Trigger) to cancel docking exercise


## Parameters

- dock_width: 0.55, width of docking plate
- dock_offset: 0.15, distance of docking plate to wall
- dock_distance: 0.50, desired distance between robot and docking plate
- dock_width_tolerance: 0.67, 0 to 1, ratio of dock candidate with actual width to accept
- max_dock_distance: 1.5 
- laser_skip: 0
- line_consistency_threshold: 0.050, how far a point can defiate from its line; must be smaller than 1/2 of dock_offset
docking_frame: odom, what frame the docking goal should be given at; detection is done in laser frame

## Brief Explanation

This package finds dock in a laser scan, then dispatch reference pose to a PID controller.

Docking plate is of specified width and distance from wall. To detect it, we find line segments in the laser scan, and keep the ones that are within range of the dock width. We then pick the closest one as the desired dock target.

Line segments are detected by:
- take a sequence of ranges whose width is of a set fraction of the dock width,
- find a least-square best fit line parameters for them
- check that the orthogonal distance of every point in above line is lower than a threshold
- extend that line with subsequent points from the scan, break the segment if orthogonal error is larger than a threshold
- upon segment breaking, if its length is within thresholds of actual docking plate dimension, add it as a dock candidate
- when all points in scan have been evaluated, pick dock candidate closest to laser origin as the target dock, where distance is calculated from centre of docking candidate to laser origin.