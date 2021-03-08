# Camera docking #

The camera docking node is primarily responsible for finding aruco markers in the image, compute the centre of the two markers thereby estimating the centre of the pallet/object. This position of the pallet defines the transformation between the camera and the pallet thereby estimating the transformation between odom with the pallet. The search for markers begin once the camera_lidar_docking code sends a signal to search for markers. If the camera_docking node doesn't find 2 markers, it starts to rotate until it finds the requisite number of markers i.e 2

To change the length of the aruco marker in use in the code, you change the "fiducial_len" param in the launch file. The parameters related to camera topic and info need to be changed from inside the code.

This node shouold be used when you are using perception based docking process i.e robot should make use of camera and lidar for docking under the pallet. 

## Usage
There should be 2 markers in the simulation world to get this package works

roslaunch camera_docking camera_docking.launch 

## Output
Transformation published between camera and pallet/object 

To look at the main code from where the code has been taken, follow this link [ros aruco_detect](https://github.com/UbiquityRobotics/fiducials/tree/kinetic-devel/aruco_detect)
