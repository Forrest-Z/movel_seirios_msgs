# dafan_docking #

## Introduction

The purpose of this node is to autonomously navigate the robot to the front of the charging station and dock it using the pose estimation from fiducial markers. This is done in 4 phases:
1. Phase 0: Pre-docking. The robot will move from elsewhere in the map to the front of the docking station.
2. Phase 1: April docking. The robot will move to a goal offset from the pose of april markers.
3. Phase 2: Orientate Plane (optional). The robot will correct its orientation according to the normal of the choosen plane. 
4. Phase 3: The robot will move forward until it registers that the battery is charging, in which case the robot will have completed the docking procedure.

## Prerequisite Packages

1. **apriltag** (The native apriltag package)
```
git clone https://github.com/AprilRobotics/apriltag.git
```

2. **apriltag_ros** (Provides ROS API for the april tag package)
```
git clone https://github.com/AprilRobotics/apriltag_ros.git
```

## Quick Start

Configure the parameters. Refer to **Configuration instructions** for more information. If you are using bundle calibration and you are setting up the april markers for the first time, refer to **Apriltag bundle calibration**

Make sure there are no obstacles in a 1 meter distance from the front of the docking station.

**Launch docking node**: 
This will start phase 0 to 3.
```
roslaunch dafan_docking dafan_docking.launch
```

## Apriltag bundle calibration
- More information on april tag bundle calibration can be found at http://wiki.ros.org/apriltag_ros/Tutorials/Bundle%20calibration . Once you have familiarised yourself with the concept above, you can proceed to do the following

- If you are using a bundle of april tags arranged in an arbitrary manner, then make use of the following launch file to launch everything required for recording the april tag poses for use in calibration. Make sure that all april markers are in view throughout the calibration and their estimated poses look correct.
```
roslaunch dafan_docking calibration.launch
```

- The rosbag file from calibration.launch should be saved as *./calib_scripts/data/calibration.bag*, proceed to the MATLAB script at *./calib_scripts/calibrate_bundle.m* to obtain the poses of the markers relative to each other.

- Put the obtained configuration parameters into *./config/april_tags.yaml* 

## Usage

**Services**
- */toggle_docking* : Service advertised by april_docking(PHASE 1), it is to be called by pre_docking(PHASE 0) when it has successfully reached the pre-docking goal.

**Published Topics**
- */cmd_vel* : The command velocity sent to the motor drivers
- */pid_goal* : The phase 1 goal sent to the planner_adjuster node
- */pid_goal2* : The phase 2 goal sent to the planner_adjuster node
- */dist_to_dock* : The distance to the docking goal
- */goal/status* : Status on whether the goal has been reached by planner_adjuster

**Subscribed Topics**
- */odom* : Odometry message
- */batt_status* : Status of battery charging, used to determine when to stop in phase 3. Once the battery is either charging/full, then the dafan robot will stop moving and end the autonomous docking nodes.
- */stop_now* : Takes in a bool to tell planner_adjuster to stop.

## Configuration instructions

1. Phase 0: pre-docking [config/phase0_pre_docking.yaml]
```
#All coordinates are relative to map frame
#pre-docking position (meters)
g_x: -0.270
g_y: 0.680
g_z: 0.0 
#pre-docking orientation (quaternions)
goal_qx: 0.0 
goal_qy: 0.0 
goal_qz: 0.9747
goal_qw: 0.2234
```

2. Phase 1: April Docking [config/phase1_april_docking.yaml]
```
#whether to skip phase 0 and start phase 1 straight away
skip_phase_0: true 

#bundle of april tags to track
id_array1 : "[0, 1, 2]" 

#goal offset from the april marker
goal_x_offset1: 0.50 #meters
goal_y_offset1: 0.0  #meters
goal_yaw_offset1: 0.0 #radians

#distance from the april tag to "save" the goal
checkpoint_thresh1: 0.06 #meters
```

3. Phase 2: Orientate Plane [config/phase2_orientate_plane.yaml]
```
#display debug information
print_debug: false

#Goal offset (meters), should be set to 0
goal_offset_x: 0.0 
goal_offset_y: 0.0

#passthrough filter (z and x are the 
#height and distance normal to camera plane respectively)
min_x: 0.0 #minimum width
max_x: 1.1 #maximum width
min_z: 0.18 #minimum height
max_z: 0.4 #maximum height

#voxel parameters
leaf_size: 0.01 #in meters
min_points_voxel: 1 #minimum number of point clouds to constitute a voxel

#Statistical Outlier Removal
sor_knn: 30 #k nearest neigbours to consider
sor_stddev: 1.0 #standard deviation

#Radius Outlier removal
# outrem_rad_search: 0.04 
# outrem_min_neigh: 10

#Planar segmentation
seg_max_itr: 100      #number of iterations to refine
seg_dist_thresh: 0.01 #maximum distance between points on the plane

#euclidean cluster extraction
ec_tol: 0.02          #if too high, multiple objects seen as one cluster
ec_min_size: 50       #minimum no. of points required to constitute a cluster
ec_max_size: 2500     #maximum no. of points required to constitute a cluster
```

4. Phase 3: Robot docking into charging station *No configurable parameters* 
But it can be disabled in planner_adjuster.yaml

5. planner_adjuster (PID Controller) [config/planner_adjuster.yaml]
```
#disables phase 3 (when the robot is docking into the charging station)
disable_phase_3: true

#Parameters are in the following form [P, I, D]
angle_gains_init: [0.05, 0.0, 0.0]  #Stage 0
angle_gains_final: [0.05, 0.0, 0.0] #Stage 2
dist_gains: [0.35, 0.1, 0.6]        #Stage 1

angle_tol: 0.0436332    #Tolerance for stage 0
dist_tol: 0.01          #Tolerance for stage 1
final_angle_tol: 0.025  #Tolerance for stage 2
```

5. april tag ID settings [config/april_tags.yaml]
```
standalone_tags:
  [
    # {id: 0, size: 0.08},
    # {id: 1, size: 0.07},
    # {id: 2, size: 0.07}
  ]

tag_bundles:
  [
    {
      name: 'docking_goal',
      layout:
        [
          {id: 0, size: 0.0800, x: 0.0000, y: 0.0000, z: 0.0000, qw: 1.0000, qx: 0.0000, qy: 0.0000, qz: 0.0000},
          {id: 2, size: 0.0700, x: 0.0772, y: -0.0058, z: 0.0218, qw: 0.9495, qx: 0.0013, qy: -0.3136, qz: -0.0073},
          {id: 1, size: 0.0700, x: -0.0778, y: -0.0036, z: 0.0230, qw: 0.9562, qx: -0.0060, qy: 0.2926, qz: 0.0070}
        ]
    }
  ]
```

5. april tag ros settings [config/april_settings.yaml]
Does not need to be modified unless there are severe performance issues

## Additional Resources:
- calib_scripts contains scripts for calibrating bundles of april tags
- *april_tags_images* folder contains some images that can be readily printed for pose estimation

## Known Issues
1. Pose estimation based on april tag can be very noisy, turning on the camera LED light helps to increase the contrast which reduces the noise. 
2. April tag markers need to be VERY stable, they cannot be moved after calibration or the accuracy will suffer.

## Future improvements
1. Recovery mechanism: There needs to be a recovery behaviour in the event that the april tags go out of view. This can be a simple rotation on the spot until the april tags are registered.

## External Contributions

The planner_adjuster module has been taken from the planner_adjuster package and repurposed in this docking module. 

## License

All rights reserved by Movel AI