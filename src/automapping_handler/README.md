# Automapping Handler Plugin

A task\_supervisor plugin for mapping with autonomous exploration.

**Task type: 7**

## Prerequisites

* [explore\_lite](http://wiki.ros.org/explore_lite)

* [move\_base](http://wiki.ros.org/move_base)

* SLAM package (e.g. [gmapping](http://wiki.ros.org/gmapping))

* [map\_server](https://wiki.ros.org/map_server)

## task\_supervisor setup

1. gmapping

* Add automapping\_handler to 'plugins' section in task\_supervisor.yaml as follows:
   * - {name: automapping\_handler, type: 7, class: 'task_supervisor::MappingHandler'}

* Add a 'automapping\_handler' section and copy over the configs in the 'mapping_handler' section. After that, add the following line 
   * auto: true

2. RTAB-Map

* Add automapping\_handler to 'plugins' section in task\_supervisor.yaml as follows:
   * - {name: automapping\_handler, type: 7, class: 'rtabmap_handler::RtabmapHandler'}

* Add a 'automapping\_handler' section and copy over the configs in the 'rtabmap_handler' section. After that, add the following line 
   * auto: true

## Usage

The handler's function is to start and stop automapping. Once automapping is started, stopping it requires that the *save_map* service be called, or a goal cancellation issued to task\_supervisor.

### Task payload format

This task handler does not take any payload arguments.

### Services

* \_~/automapping\_handler/save\_map

This service is only available once mapping has started. A single string argument with a full path can be sent to this service to set map save location. Full path must have no extension.

Saving will be done by map\_server's **map_saver** node. Example:

      rosservice call /task_supervisor/automapping_handler/save_map "input: '/home/map'"

Map will then be saved to */home/map.pgm* and */home/map.yaml*

* \_~/automapping\_handler/save\_map\_async

This service is the same as *save_map* above except this service will not stop mapping once saving is complete.

### Published ROS topic

* \_~/automapping\_handler/stopped

ROS topic with message type std\_msgs/Empty. The handler publishes to this topic when automapping is complete.
