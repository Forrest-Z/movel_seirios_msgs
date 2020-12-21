# Map Updater #

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