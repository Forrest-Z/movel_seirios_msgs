# Orbomator

Unifying 2D grid map with ORB-SLAM2 map

## Usage

### Creating maps

- setup static tf with `launch/static_tf_corridor.launch` or your own setup
- run `orbomator map_transform_monitor`
- set orb-slam2 parameters to publish transform in `orb_map` and `orb_camera` frames, these parameters specifically
```
    <param name="pointcloud_frame_id" type="string" value="orb_map" />
    <param name="camera_frame_id" type="string" value="orb_camera_link" />
```
- run your 2D mapping and orb-slam2 nodes
- play back your rosbag, or start mapping with your robot
- save your maps (grid and orb-slam2) when ready
- save map alignment tf from `map_transform_monitor` output

### Localising with previously-created maps

- setup static tf
- setup map alignment tf (if not included in static tf)
- start 2D localisation (e.g. amcl)
- start orb-slam2 in localise-only mode (verify that the correct map is loaded)
- start orbomator
- let the robot localize (or start your bag)

## Output

### Map Transform Monitor

- the `map_transform_monitor` node publishes transform between `orb_map` to `map`
- it also prints out the transform in terminal, and the difference between consecutive transforms
    - if the transform is stable, the difference would be very small

#### Using the transform

The simplest way to use the output is to put the resulting transform to a `static_tf_publisher` node when performing localisation. When robot poses from orb-slam2 and amcl disagree, do corrective action, like reinitialising amcl to orb-slam2 estimate (after transfer to map frame).

It is important to accurately set the transfrom between `camera_link` and `base_link`, so that there is no residual rotation error between the two maps after alignment.

### Camera Attitude Monitor

- estimate of camera roll and pitch, based on relative orientation to floor (detected in this node)
- use this to fill in `camera_link` to `base_link` transform if not known
- you still need to estimate camera yaw by other means (trial and error is good)
