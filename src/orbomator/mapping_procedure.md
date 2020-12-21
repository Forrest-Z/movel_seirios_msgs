# Creating and using supplementary orb-slam map

This instruction shall tell you how to create an orb-slam2 map and use it along your grid map. We assume that you already have a grid map (e.g., from gmapping).

## Map creation

### Before mapping run
1. setup orb_slam2 and camera parameters in `orb_slam2_d435_rgbd_map.launch`
2. run orb_slam2 mapping node with `$ roslaunch orb_slam2_ros orb_slam2_d435_rgbd_map.launch`
3. run in another terminal `$ roslaunch orbomator map_transform_monitor.launch`
    the printout from this node will be used to align the grid and orb maps later, so make sure the terminal setup will make it convenient to do so
4. run rviz with the included orbslam2.rviz configuration

### During mapping run
5. localise the robot manually (with UI or rviz)
5. drive the robot with teleop **slowly**. 
    If rviz is loaded with the recommended configuration, the top right image would show tracking points. If all tracking points disappear, turn slowly to a previous pose until you can reacquire tracking. If you cannot reacquire tracking, restart this procedure from step 2
6. stop driving when you have covered the region you would like to map. Coverage can be judged by the distribution of the orb points in the map. Do not kill the nodes launched above yet.

### After mapping run
7. save the orb_slam2 map
8. save the printout from `map_transform_monitor`. You can either take the last line or average a range of lines. We recommend plotting the entire printout, then average a span of time where the plot is stable. You can now kill the mapping nodes from above
9. enter the transfrom from step 8 to the static transform in `static_map_alignment.launch`

## Map usage

1. setup orb_slam2 and camera parameters in `orb_slam2_d435_rgbd_loc.launch`
    They should be harmonised with the mapping counterpart, except the map name should be set to the one saved during map creation
2. run orb_slam2 localisation node with `$ roslaunch orb_slam2_ros orb_slam2_d435_rgbd_loc.launch`
3. run map alignment node with `$ roslaunch orbomator static_map_alignment.launch`
4. run orb localiser with `$roslaunch orbomator orbomator.launch`
    if the allow_reinit parameter is set to true, the robot's amcl pose will be reinitialised to the most recent orb_slam2 pose
5. if you need to force amcl reinitialisation to orb_slam2 pose estimate, publish a std_msgs/Empty message to /reinit_amcl

## Troubleshooting

- orb_slam2 map orientation does not match grid map orientation
    - verify the transform between camera frame and base_link

- orb_slam2 map has different scale from the grid map
    - in `orb_slam2_d435_rgbd_{loc, map}`, adjust the `depth_map_factor` parameter
    - if orb_slam2 is too large, increase the parameter, otherwise decrease it
