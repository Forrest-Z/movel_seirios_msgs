# Map Edtor #

Map Editor is a ROS package that edit navigation map and save it again with no-go-zone addition on it. 
Works well with localization handler in feature/no-go-zones branch of task_supervisor

### Requirements ###

- OpenCV
- Localization_handler (task_supervisor) with no-go-zones features
- movel_seirios_msgs/DrawMultiPolygons.srv

### Setup ###

This package needs two different kind of map, **Localization map** and **Navigation map**. The localization map should be located at the usual directory that the map will be saved. The navigation map will be located inside the localization map directory named "/nav" (seirios web should have saved the map in both place when the user save a map in mapping process). 

### How it works ###

The package receives a service with a `Polygon` type. This service consist current map name, and a group of points. It can makes:
- Line            (2 points)
- Triangle        (3 points)
- Other polygons  (>3 points)

The minimum number of points that will be accepted by the package is two (to make a single line). The points will be a vertices of a line or geometry things. The order of the points is not important, but it's better to make it ordered. 

When there's an instruction to update the map, this package edit the map to add polygons and then it calls a service in the task_supervisor to relaunch the map_server.

The map_editor can draw:
- Black area/line
- White area/line

***note**, e.g. if you want to make a triangle, just pass 3 points, not 4 (don't pass the same points at the beginning and at the end).

Beside **make polygons to the map**, this package can **revert** edited-navigation map to the original map using services.

### How to run ###

`roslaunch map_editor map_editor.launch`

### Services ###

**Update map*: 

* Array of Polygons
```
rosservice call /map_editor/multi_updates "filename: 'fullhouse'               
polygons:
- pixels:
  - x: 213 
    y: 130
  - x: 213
    y: 235
  - x: 246
    y: 235
  - x: 246
    y: 130
  is_eraser: false
  line_width: 2
- pixels:
  - x: 213
    y: 191
  - x: 170
    y: 235
  is_eraser: false
  line_width: 5
- pixels:
  - x: 163
    y: 130
  - x: 163
    y: 235
  - x: 196
    y: 235
  is_eraser: false
  line_width: 2
  "
```
**Revert map*:

```
rosservice call /map_editor/restore "filename: 'fullhouse'
polygons:
- pixels:
  - x: 0
    y: 0
  is_eraser: false
  line_width: 0
  "
```

### Parameters ###

* navigation_map_path : Path of navigation map
* localization_map_path : Path of localization map
