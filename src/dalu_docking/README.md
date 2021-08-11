# Docking with robots that allow movements in x & y directions

## Usage

- Launch dalu_docking.launch
- Publish message of type geometry\_msgs/PoseStamped to topic /movel\_dalu\_docking/goal (for goal input) or call /movel\_dalu\_docking/run service with argument 'true' (for apriltag)
- Wait for message in /movel\_dalu\_docking/success to be published which indicates docking ended

Only use this if the goal is very close. It doesn't consider obstacles at all!

## Topics/Service

### tag_offset node

Subscribes to /tag_detections and publish transform between apriltag and dock position.

### dalu_docking node

* Published topics

/cmd\_vel\_mux/autonomous: To drive robot while docking

/movel\_dalu\_docking/success: To indicate docking success

* Subscribed topic

/movel\_dalu\_docking/goal: Start docking with goal input

* Service

/movel\_dalu\_docking/run: Start docking with apriltag (argument: true) or stop docking (argument: false)

## Parameters

### tag_offset node

* tag_id: ID number of apriltag
* apriltag\_x\_offset: Offset in x direction of dock position from apriltag
* apriltag\_y\_offset: Offset in y direction of dock position from apriltag

### dalu_docking node

* xy_tolerance: Distance tolerance from dock position
* yaw_tolerance: Yaw angle tolerance from dock position
* frames_tracked: Number of past dock positions taken to get average dock position
* max\_linear\_vel: Maximum linear speed while docking
* min\_linear\_vel: Minimum linear speed while docking
* max\_turn\_vel: Maximum turning speed while docking
* min\_turn\_vel: Minimum turning speed while docking

## Brief Explanation

Control happens in three stages:

1. Orient robot to match yaw of goal position
2. Move robot in y direction until within tolerance
3. Move robot in x direction until within tolerance
