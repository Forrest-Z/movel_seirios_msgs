# Dynamic Obstacle Layer Plugin

A costmap\_2D plugin to inflate dynamic obstacles in costmap.

## Prerequisites

* current robot pose info with "/pose" topic

* map topic

* [move\_base](http://wiki.ros.org/move_base)

## Config Setup
* Add dynamic\_obstacle\_layer to 'plugins' section in global\_costmap\_params.yaml using the following line. Add this line after the obstacle layers and before inflation layer.

```
- {name: dynamic_obstacle_layer, type: "dynamic_obstacle_layer::DynamicLayer"}
```

* In costmap\_common\_params.yaml:
   * Copy over the following static_layer parameters if there are any:
      * map\_topic
      * first\_map\_only
      * track\_unknown\_space
      * lethal\_cost\_threshold
      * unknown\_cost\_value
      * trinary\_costmap

   * Additional parameters
      * **enabled**: To enable/disable this layer
      * **map\_tolerance**: Distance in meters from static obstacles where obstacles within this distance are filtered out
      * **footprint\_radius**: Distance in meters to inflate dynamic obstacles
      * **range**: Distance from current position of robot to apply the inflation

* Example dynamic_obstacle_layer section in costmap\_common\_params.yaml:
```
dynamic_obstacle_layer:
    track_unknown_space: true #Copied over from static_layer section
    enabled: true
    map_tolerance: 0.2
    footprint_radius: 0.2
    range: 1.0
```

## Parameter Tuning

* If robot has bad localization, increase map\_tolerance to filter out static obstacles that do not match the map closely.

* Increase footprint\_radius for bigger inflation and vice versa

* Increase range value to inflate dynamic obstacles that are further away from robot
