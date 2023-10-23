# Geometric Docking Handler Plugins #
### for task_supervisor ###

One of task_supervisor's plugin for docking to plates of set width at set distance from a wall.

### How to setup ###
* Make sure you have this package: geometric_docking, task_supervisor package, planner_adjuster
* Add the plugins at task_supervisor.yaml with type : geometric_docking_handler::GeometricDockingHandler
* Add some parameters

### How to run ###
- `catkin_make`
- launch task_supervisor
- run docking task with payload, example: {"start"}

### Plugin in action ###
This package finds dock in a laser scan, then dispatch reference pose to a PID controller.

Docking plate is of specified width and distance from wall. To detect it, we find line segments in the laser scan, and keep the ones that are within range of the dock width. We then pick the closest one as the desired dock target.

### Task Supervisor Config ###
For plugins:
`- {name: geometric_docking_handler, type: 8, class: 'geometric_docking_handler::GeometricDockingHandler'}`

`geometric_docking_handler:`

 ` watchdog_rate: 2.0`

` watchdog_timeout: 0`

` geometric_launch_package: "geometric_docking"`

` geometric_launch_file: "geodock.launch"`

` geometric_launch_nodes: "/geometric_docking_node /planner_adjuster"```