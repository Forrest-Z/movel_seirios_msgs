# Dalu Docking Handler Plugin

A task\_supervisor plugin for starting and stopping docking/undocking task for dalu robot.

**Task type: 12**

## Prerequisites

* dalu_docking package

## task\_supervisor Config Setup

* Add dalu\_docking\_handler to 'plugins' section in task\_supervisor.yaml with class : dalu\_docking\_handler::DaluDockingHandler. Example:

```
  - {name: docking, type: 15, class: 'dalu_docking_handler::DaluDockingHandler'}
  - {name: undocking, type: 16, class: 'dalu_docking_handler::DaluDockingHandler'}
```

* Add a 'docking' and 'undocking' sections as follows:

```
docking:
  watchdog_rate: 2.0
  watchdog_timeout: 0
  dalu_docking_launch_package: "dalu_docking"
  dalu_docking_launch_file: "dalu_docking.launch"
  camera_name: "camera"
  loop_rate: 10.0
  battery_status_timeout: 15.0
  dock: true

undocking:
  watchdog_rate: 2.0
  watchdog_timeout: 0
  undocking_distance: 1.0
  undocking_speed: 0.1
  loop_rate: 10.0
  battery_status_timeout: 15.0
  dock: false
```

## Usage

The handler's function is to start and stop docking/undocking for dalu robot. Once task is complete, clean up is done automatically to exit the task.

### Task Payload Format

No payload required for docking with apriltag. Docking with localization requires payload with the same format as navigation_handler in task_supervisor.

## Parameters

### Common parameters

* *loop_rate*

Rate to run docking/undocking loop.

* *battery_status_timeout*

Timeout to wait for battery status.

* *dock*

Set true for docking, and false for undocking.

### Docking parameters

* *dalu_docking_launch_package*

Package of docking launch file to be launched for docking.

* *dalu_docking_launch_file*

Launch file in the specified package to launch for docking.

* *camera_name*

Name assigned to the camera used to detect AprilTag marker. Used in:

1. Color image topic: /\<camera name\>/color/image_raw

2. Camera info topic: /\<camera name\>/color/camera_info

3. Camera frame: \<camera name\>_color_optical_frame

### Undocking parameters

* *undocking_distance*

Distance travelled during undocking.

* *undocking_speed*

Max speed during undocking.

### Optional parameter

* *use_apriltag*

Set true for apriltag docking, false for docking with localization.