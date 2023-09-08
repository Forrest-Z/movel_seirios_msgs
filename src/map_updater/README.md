# Map Updater #

### Description ###
This package is used to update the map by subscribing to the static map topic. The map will be updated when the robot is moving. The map will be reverted to the static map when the `/restore_map` service is called. Might be useful to update only parts of a big map in a often-changing environment.

### Set-up ###
`global_costmap_params.yaml`
subscribe_to_updates: true; 
first_map_only: true

### How to run ###

`roslaunch map_updater map_updater.lanch`

### Parameters ###

* subscribe_only_once : Subscribe static map only on the first time
* local_threshold : The grid value under this threshold will be freed
* global_threshold : The grid value above this threshold will be freed
* update_width : Width for update area
* update_height : Height for update area

### How to revert map to static map ###

`rosservice call /restore_map "{}"`