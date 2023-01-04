# Movel_costmaps
add - {name: costmap_prohibition_layer,       type: "movel_costmap_2d::CostmapProhibitionLayer"} to relevant costmap config files.

# Costmap Prohibition Layer

ROS package that implements a costmap layer to add prohibited areas to the costmap 2D by user config. 
Source: https://travis-ci.org/rst-tu-dortmund/costmap_prohibition_layer

## USAGE 

1. Mount the plugin - {name: costmap_prohibition_layer, type: "movel_costmap_2d::CostmapProhibitionLayer"} under plugins in global_costmap_params.yaml. (And local_costmap_params.yaml too if you want to block in the local costmap.)


2. Do a rosservice call to the service /move_base/global_costmap/costmap_prohibition_layer/nogo_zone

`Example service call:`

```
rosservice call /move_base/global_costmap/costmap_prohibition_layer/nogo_zone "header:
 seq: 0
 stamp:
   secs: 0
   nsecs: 0
 frame_id: ''
zone_data:
- polygons:
   points:
   - x: 1.0
     y: 4.0
     z: 0.0
   - x: 0.0
     y: 2.0
     z: 0.0
   - x: 2.0
     y: 2.0
     z: 0.0
 labels: 0
 percentage_reduction: 0.0"

```

- Then check rviz under global costmap topic (or local costmap, if you enabled it there). You should see the area being marked by a polygon. If you don’t, please debug.
