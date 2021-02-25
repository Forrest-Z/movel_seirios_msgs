# Task Supervisor
Task supervisor is an action server which can receive goals, which are task lists, through the topic _/task_supervsisor/goal_.

_/task_supervsisor/goal_ has a message type of */task_supervisor/RunTaskListActionGoal*.

At the end of each goal message, there is a list of tasks:

  * task_supervisor/Task[] tasks
	* uint16 id
	* string name
	* uint8 type
	* string payload

## Dependency

Needs yaml-cpp package (from https://github.com/jbeder/yaml-cpp, or libyaml-cpp-dev from apt)

***Usage***

Task supervisor **requires** launch_manager to be running. Use the task_supervisor.launch in task_supervisor package.

	roslaunch task_supervisor task_supervisor.launch

***Issuing a goal***

Publish a message to */task_supervisor/goal* with the required task item fields filled. Required task fields are **type**, and **payload** depending on task type.

***Canceling goal***

Publish a message to */task_supervisor/cancel*

*Task scheduling is not implemented yet*

## Task Handlers
Each *Task* in the goal message sent to task supervisor will have a type number associated with it; type number is an *unsigned 8 bit integer*. Each task type corresponds to a specific task handler.

### Mapping Handler
**Task type: 2**

Mapping handler's function is to start and stop mapping. Once mapping is started, stopping the mapping requires that the *save_map* service be called, or a goal cancellation is issued to the task supervisor. 
The choice of mapping node to launch can be configured using **task_supervisor.yaml** in the config folder of task_supervisor package. 

**Task Payload Format**

This task handler does not take any payload arguments.

**Services**

* _~/mapping_handler/save_map

This service is only available once mapping has started. A single string argument with a full path can be sent to this service to set map save location. Full path must have no extension.

Saving will be done by map_server's **map_saver** node. Example:
	
	rosservice call /task_supervisor/mapping_handler/save_map "input: '/home/map'"
	
Map will then be saved to */home/map.pgm* and */home/map.yaml*
	
	
* _~/mapping_handler/save_map_async

This service is the same as *save_map* above. Difference between this and *save_map* is that this service will not stop mapping once saving is complete.

**Parameters**

***Required***

* *mapping_launch_package*

Package of mapping launch file (specified in param below) to be launched for mapping

* *mapping_launch_file*

Launch file in the specified mapping_launch_package to launch for mapping. This launch file should **only launch a gmapping node** and load its relevant configuration parameters. Example: *gmapping.launch*

***Optional***

* *loop_rate (default: 5Hz)*

Determines the rate at which handler will check if map has been saved, or if cancellation has been triggered by task_supervisor

* *save_timeout (default: 5s)*

Sets the number of seconds before map saving times out. Map saving time is dependent on map size

* *map_topic (default: /map)*

Topic name to save map when *save_map* service is called


### Localization Handler

Localization handler starts and stops localization, and can be given a map file to be loaded for use during localization.

**Task type: 1**

**Task Payload Format**

Payload of this task handler should be in one of 3 forms:

* *start*

Start localization without loading any map. Map used will be from what was received on */map* topic, as well as most recent pose from *mapping*.

* *start /home/map.yaml*

Start localization with the map at the full path given. Map will be loaded using map_server.

* *stop*

Stop localization.

**Services**

* *~/localization_handler/start*

This service provides an option for users to start localization directly. Input arguments is a string indicating the full path to map that should be loaded, if any. Example:

	rosservice call /task_supervisor/localization_handler/start "input: '/home/map.yaml'" 

*or if no map is to be loaded*

	rosservice call /task_supervisor/localization_handler/start "input: ''" 

* *~/localization_handler/stop*

Stop localization, takes in no arguments.

**Topics**

***Published***

* *~/localization_handler/localizing*

Latching *std_msg/Bool* indicating whether localization is running. True when localization is running.

***Subscribed***

* */map*

Subscription to this topic is setup the moment localization_handler is loaded by task_supervisor.

Localization handler subscribes to this topic for cases when localization is started immediately after mapping is done, and no map file is specified to be loaded by map_server on start of localization.

Using this topic, localization_handler will pickup from where mapping stopped, and continue using the map received to do localization. 
***Map topic name configurable in params***

**Parameters**

***Required***

* *localization_launch_package*

Package of localization launch file (specified in the param below) to be launched for localization

* *localization_launch_file*

Launch file in the specified localization_launch_package to launch for localization. This launch file should **only launch a localization node** and load its relevant configurations. Example: *amcl.launch*

***Optional***

* *loop_rate (default: 5Hz)*

Determines the rate at which handler will check if map has been saved, or if cancellation has been triggered by task_supervisor

* *set_map_timeout (default: 10s)*

This timeout is enforced when no map is loaded on start of localization. amcl provides a service */set_map* that can be used to set map and initial pose for amcl. If no map is loaded, this */set_map* service will be called to continue localization from where mapping left off.

* *map_topic (default: /map)*

Topic name to subscribe to for continuing localization if no map file is loaded.

* *map_frame (default: map)*

Specify name of map frame. Map frame is used to get pose from mapping node's TF broadcasters. Default transformation to get pose if from map -> base_link.

* *base_link_frame (default: base_link)*

Specify name of base link frame. Used to get pose of robot in map_frame. Default transformation to get pose if from map -> base_link.

* *set_map_service (default:/set_map)*

This service will depend on localization package used. For amcl package that was used for development of this handler, amcl provides a service */set_map* to set the initial pose and map occupancy grid. 

If any other localization packages are used and a similar service is available, change this name. Service should be of the same form as amcl's */set_map* service.

### Cleaning Handler

**Task type: 4**

Cleaning handler integrates functions from path_recall, crop_map, and coverage_planner to allow a user to request a cleaning task to be completed by the robot. THe cleaning handler can execute previously planned cleaning path, or plan a new cleaning path from a map and a polygonal boundary for the cleaning area.

***The process of cleaning handler is as follows:***

(This is the most general case, where a new cleaning path is planned and immediately executed)

1. User issues a task of type 4, and a payload that points to a text file which contains the coordinates of the polygon shape that is to be cropped from the big map. Path to big map is set in *task_supervisor*'s config file and can be overridden with the payload, details can be found below under the **Parameters** and **Task Payload Format** sections.

2. Task supervisor calls cleaning handler to run, which causes cleaning handler to launch all required launch files, which are: ipa_room_exploration_client, ipa_room_exploration_server, move_base, crop_map, path_recovery, path_load, path_saver. If you would like to launch move_base independently, you can set the `move_base_launch` parameter to noop.launch.

3. Handler crops the map to the specified polygon with the CropMap object included from the crop_map package. Map centre and resolution are deduced from the `big_map_path` parameter, since that parameter is the .pgm filename, the same filename with .yaml extension is used to deduce the parameters. Make sure they are in the same folder.

4. Handler then calls *room_exploration_client/start* service to request for a plan. This call is non-blocking. If planning exceeds a specified timeout, then planning will be cancelled and the task will fail altogether. Planning timeout can be specified in config file.

5. room_exploration_server calls *path_saver/save*'s service to save the generated path into a yaml file. Save path is determined by param *yaml_path*, which can be configured in the config file. The path is first saved into an intermediate file `coverage_path.yaml` (set both in ipa_room_exploration_server.launch and task_supervisor.yaml *they have to match*). Immediately after, the path is renamed into the same name as its polygon text file. For instance, if a path is planned with polygon `room_a.txt`, then it will be saved in `yaml_path/room_a.yaml`. The stem name `room_a` can be used to recall the path later without repeating the planning step. *Future improvement* associate path name with its source map.

6. Upon completion of path saving and generation, handler will then call *path_load/check*'s service to check if the current location of robot is within distance threshold from the first way point of generated path. Param *start_distance_threshold* can be set in the config. If robot is too far, task is cancelled. If checking is successful, *path_load/load* service is called by handler, which will then start the execution of the generated path through the use of move_base. Current implementation of path_load requires GlobalPlanner and TebLocalPlanner (local planner can be swapped out, but not global planner).

7. When all way points have been traversed, the handler will mark this task as complete, all previously launched files are shutdown, and control is returned to task_supervisor.

**Polygon Text File Format**

Format is in x-y pairs, and last pair is robot location. The coordinates are specified in *pixels*, and (0, 0) is the top left of the pgm image. Current implementation does not do anything with specified robot location. Example:

	 0 0
	 10 0
	 10 10
	 0 10
	 5 5

There must be at least 3 points to form a polygon, and it can either be clockwise or anti-clockwise. The above example forms a square with length of 10 units.

**Task Payload Format**

Payload is yaml string with these keys: name, poly, bigmap, and run.

- `name` is the name of a previously saved path. If it is not specified, then the cleaning handler will fall back to planning a new path with a supplied polygon. If a `name` is specified but the path file associated with it does not exist, the task will fail. If `name` is specified, then the associated cleaning path will be loaded and run, the other keys are ignored.
- `poly` is the full path to a polygon specification for path planning, see section **Polygon Text File Format** above for form. If a `poly` is specified but the associated file does not exist, the task will fail. The next two keys only apply when `poly` is specified. The filename of the polygon specification file is used as the path name for later reuse (e.g. polygon `room_one.txt` yields `room_one` path, saved in `room_one.yaml` file).
- `bigmap` is the full path to the map file (.pgm) to be used for path planning, if different from the one in `config/task_supervisor.yaml`. If it is specified but cannot be loaded for any reason, the cleaning handler will fall back to the big map path in `config/task_supervisor.yaml`.
- `run` is whether to run the newly planned path immediately or not. Possible values are `true` and `false`.

Example payloads (use these in the payload field of the Task msg, e.g. the task list in /task_supervisor/goal):

- plan new path, do not run it
> `payload: '{poly: /path/to/poly/new_room.txt, run: false}'`

- plan new path in a different map, run it
> `payload: '{poly: /path/to/poly/new_room.txt, bigmap: /path/to/bigmap/map.pgm, run: true}'`

- run a previously saved path
> `payload: '{name: old_path}'`

**Topics**

***Subscribed***

* */path_load/start*

This topic is subscribed to once cleaning handler is actively running to get the state of path_load. If path_load is actively executing a path and sending way points to move_base, this topic will publish true. Cleaning handler uses this topic to see if the path has been completed or cancelled. 

* */room_exploration_server/coverage_path*

Momentarily subscribed to by cleaning handler to know if path planning has been completed. 

**Parameters**

***Required***

* *move_base_package*

Package of move_base launch file (specified in the param below) to be launched for navigation

* *move_base_launch*

Launch file in the specified move_base_package to launch for navigation. This launch file should **only launch a move_base node** and load its relevant param configurations, including planners. Example: *move_base.launch*

Alternatively, you can launch move_base independently of task_supervisor, and set `move_base_package` to `task_supervisor` and `noop.launch` to `move_base_launch`. If you use move_base for other tasks and do not need to set special parameters for move_base in cleaning_handler, use this method.

* *start_distance_thresh (meters)*

Distance threshold that planned path's first way point and current location of robot must fulfill before path will be executed.

* *yaml_path*

Full path to a directory that generated path should be saved. Example: 
	
	/home/yaml_path/
	
**Note that the path must end with a forward slash.**

* *big_map_path*

Full path to .pgm file that will be used for cropping. Example: 
	
	/home/map.pgm

* *cropped_map_path*

Full path to .png file for saving by crop_map. Example:
	
	/home/cropped_map.png

* *cropped_coordinates_path*

Full path to .txt file for saving map origin coordinates by crop_map. Example:
	
	/home/coordinates.txt

* *planned_path_name*

Full path to .yaml file where generated path should be saved and loaded from. Example:

	/home/coverage_path.yaml

***Optional***

* *loop_rate (default: 5Hz)*

Determines the rate at which handler will check if map has been saved, or if cancellation has been triggered by task_supervisor

* *planning_timeout (default: 10s)*

Sets the time allowed for path planning to run. If planning exceeds this time, cleaning handler will stop and task is set as failed.

### Human Detection Handler

Human detection handler starts and stops human detection.

**Task type: 5**

**Task Payload Format**

Payload of this task handler should be in one of 2 forms:

* *start*

Start human detection.

* *stop*

Stop human detection.

**Services**

* *~/human_detection_handler/start*

Start human detection takes in no arguments.

* *~/localization_handler/stop*

Stop human detection, takes in no arguments.

**Required Parameters**

* *human_detection_launch_package*

Package of human detection launch file (specified in the param below) to be launched for human detection

* *human_detection_launch_file*

Launch file in the specified human_detection_launch_package to launch for human detection. This launch file should launch human detection node and load its relevant configurations.

* *human_detection_start_log_msg*

Log message to be shown when starting human detection

* *human_detection_stop_log_msg*

Log message to be shown when stopping human detection

* *human_detection_start_error_msg*

Error message when human detection failed to start

* *human_detection_stop_error_msg*

Error message when human detection failed to stop

**Navigation with Human Detection**

Human detection node publishes a 0 to 1 detection score to be used for tasks such as navigation, which is handled by navigation_handler.

***Navigation Handler Service for Human Detection***

* *~/navigation_handler/enable_human_detection*

Enable or disable human detection based navigation in which robot stops when it detects human(s). Input argument is true/false to enable/disable. Example:

	rosservice call /task_supervisor/navigation_handler/enable_human_detection "data: true" 
	
***Navigation Handler Parameters for Human Detection***

* *human_detection_min_score*

Minimum detection score to consider that the robot detected human(s)

* *human_detection_topic*

Topic where human detection score is published

* *enable_human_detection_msg*

Log message when navigation with human detection is enabled

* *disable_human_detection_msg*

Log message when navigation with human detection is disabled

### Path Handler
**Task type: 6**

Path handler's function is to execute path following. Once path following is started, stopping it requires that a goal cancellation is issued to the task supervisor. 
The choice of path following node to launch can be configured using **task_supervisor.yaml** in the config folder of task_supervisor package. 

**Task Payload Format**

```
{
	"path": [{
		"position": {
			"x": 0,
			"y": 0,
			"z": 0
		},
		"orientation": {
			"x": 0,
			"y": 0,
			"z": 0,
			"w": 1
		}
	}, {
		"position": {
			"x": 0,
			"y": 0,
			"z": 0
		},
		"orientation": {
			"x": 0,
			"y": 0,
			"z": 0,
			"w": 1
		}
	}, ... ]
}
```

**Parameters**

***Required***

* *path_load_launch_package*

Package of path loading launch file (specified in param below) to be launched for path following

* *path_load_launch_file*

Launch file in the specified path_load_launch_package to launch for mapping. This launch file should **only launch a path loading node** and load its relevant configuration parameters. Example: *path_load_segments.launch*

***Optional***

* *loop_rate (default: 5Hz)*

Determines the rate at which handler will check if task has been cancelled