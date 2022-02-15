# Aruco AMCL # 

## Prerequisites

sudo apt-get install ros-noetic-fiducials
sudo apt-get install ros-noetic-fiducials-msgs

This package is meant to complement 2D localization by AMCL. The idea here is to use vanilla AMCL localization when a fiducial is not present, and to improve localization using fiducials when the camera detects a fiducial. 

There is code for 2 nodes - aruco_saver and aruco_amcl. Both would need aruco_detect node to be running, and subscribe to the /fiducial_transforms published by /aruco_detect node. aruco_saver is meant to be running when mapping, whereas aruco_amcl is meant to be running when navigating(move_base) and localizing (amcl).

### aruco_saver

fiducial_len: Length of aruco marker(m). If possible, we would prefer bigger markers since they can be detected consistently from further away.
correction_range: Maximum Euclidean distance from robot base to fiducial marker for us to save poses (m). This is because the fiducial detections tend to be noisier when too far away.

The fiducial_transforms subscribed by aruco_saver are in one of the camera' frames. We would want to transform them into a fixed frame - frame that does not move in the world, i.e. map or odom frame. 
The odom frame can drift, thus we choose to transform to the map frame. If the fiducial marker is within the correction range, we save its pose in the map frame in hashtable data structure. If the pose of a fiducial has been saved in the hashtable before, we update the hashtable's pose with history averaging. We proceed to create visualization markers to publish to RVIZ for visualization.

When a service to save aruco is called, we save the aruco poses in the hashtable to a file. Buffering the poses in memory using a hashtable reduces the number of disk I/O operations, and also gives a chance for history averaging of poses. 

### aruco_amcl

Max error of amcl pose to start the algo making new pose candidate.
    max_error_x: 0.5
    max_error_y: 0.5
    max_error_theta: 0.5
fiducial_len: Length of aruco marker(m).
correction_range: Maximum Euclidean distance from robot base to fiducial marker for us to consider it for calculating robot's pose (m). 
cooldown_time: Time to wait between calculating /initialpose to publish.

aruco_amcl helps amcl by using fiducials to calculate the robot's estimated pose in the map frame, and publishes this to amcl.
After this node is launched, it needs to load a file containing corresponding aruco poses via a service, in order for this node to be initialized.
The poses in the file will be buffered in memory using a hashtable (aruco_map_).
As mentioned earlier, the fiducials detected are in one of the camera's frames. 
If a fiducial id of a fiducial detection matches the fiducial id of a hashtable entry and is within the correction range, we proceed to transform its pose to the map frame.
If its pose in map frame is significantly different from the corresponding pose in aruco_map_, most likely amcl localization is not accurate. We save it in another hashtable called aruco_flush_. If an id has been saved in aruco_flush multiple times, we transform the pose to get the pose of robot in map frame, and publish it as "/initialpose".

### Recommendations

We recommend that users put multiple aruco markers since this package is useful only when a fiducial marker is within the camera's FOV.

### Areas for Improvement 

A weakness with running aruco_saver during mapping is that the mapping algorithm may have loop closure, which changes the map coordinates somewhat. Since the coordinates saved by aruco_saver would be still based on the coordinates prior, it would be inaccurate. Thus, for the time being, we recommend this package only for indoor environments where loop closure is unlikely to be significant.