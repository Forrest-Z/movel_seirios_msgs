# Automapping Handler Plugin

A task\_supervisor plugin for mapping with autonomous exploration.

**Task type: 7**

## Prerequisites

* [explore\_lite](http://wiki.ros.org/explore_lite)

* [move\_base](http://wiki.ros.org/move_base)

* SLAM package (e.g. [gmapping](http://wiki.ros.org/gmapping))

* [map\_server](https://wiki.ros.org/map_server)

## task\_supervisor setup
* Add automapping\_handler to 'plugins' section in task\_supervisor.yaml with class : task\_supervisor::AutomappingHandler

* Add a 'automapping\_handler' section with the following parameters:
   * automapping\_launch\_package: "automapping\_handler"
   * automapping\_launch\_file: "automapping.launch"

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
