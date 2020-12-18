# Graph Map and Planner

# Brief

This package implements graph map and graph planner. The graph map containes much fewer states than the grid map, and so planning is simpler. Graph map also allows one to add semantically meaningful routes on the grid map, such as multi-lane corridors, one-way roads, and roundabouts.

# non-ROS Dependencies

- Boost Graph Library (BGL)
- Point Cloud Library (PCL)

# Building

This is a ROS package, put it in your catkin workspace and use `catkin_make`.

# Usage

## Defining Map

Graph map is defined in a csv. The definition includes vertex coordinates in map frame and edges (by vertex indices). See included examples for formatting.

- Vertex defintion column order: index, x, y, z, (the rest are ignored and can be used for comments)
- Edge definition column order: source index, destination index, 'r' if bidirectional (leave blank if unidirectional, do not use for comments)

## Setting up Planner

Graph planner is a plugin for nav_core. Set the `base_global_planner` parameter in `move_base` to `graphmap_planner/GraphPlanner`


```
# in move_base_params.yaml

base_global_planner: "graphmap_planner/GraphPlanner"
```

Specify the graph map definition csv under the GraphPlanner namespace, and load inside `move_base` in your launch file (the parameter will resolve to move_base/GraphPlanner/graph_def). Use full path in the yaml.

## 

# See Also

- [Design document](https://drive.google.com/open?id=1avnAzn2I9khviUgs-vU4yPOsvY2shNbtTsjj_6piKMY)