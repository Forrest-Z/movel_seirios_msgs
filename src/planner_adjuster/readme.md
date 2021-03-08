# Simple PID controller for last centimetre control

## Usage

- Launch planner_adjuster.launch
- Publish message of type geometry_msgs/Pose to topic /pid_goal (be sure to fill in the frame_id field in the header!)
- Wait for terminal to say "REACH REACH REACH"

Only use this if the goal is very close (< 1 m). It doesn't consider obstacles at all!

## Topics

Subscribes to /odom and /pid_goal. Remap as necessary.

Publishes directly to /cmd_vel. Remap if you filter or multiplex cmd_vel.

## Parameters

See config/planner_adjuster.yaml. You can set angle and distance gains and tolerances.

## Brief Explanation

Control happens in three stages:

- orient robot towards goal position
- move robot towards the goal position (angle is still adjusted)
- orient robot towards goal orientation

## Suggested Improvements

- Success/failure feedback
- Failure mode detection
- PID gain and control saturation/clamping