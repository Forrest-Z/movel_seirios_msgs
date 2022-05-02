# General Docking Handler Plugin

A task\_supervisor plugin for starting and stopping docking/undocking task for dalu robot.

## task\_supervisor Config Setup

* Add general\_docking\_handler to 'plugins' section in task\_supervisor.yaml with class : general\_docking\_handler::GeneralDockingHandler. Example:

```
  - {name: docking, type: 15, class: 'general_docking_handler::GeneralDockingHandler'}
  - {name: undocking, type: 16, class: 'general_docking_handler::GeneralDockingHandler'}
```

* Add a 'docking' and 'undocking' sections as follows:

```
docking:
  watchdog_rate: 2.0
  watchdog_timeout: 0.0
  loop_rate: 10
  odom_topic: "/odom"
  disable_smoother: true
  use_external_service: false
  #external_service: ""       # uncomment if use_external_service is true
  #external_cancel_topic: ""  # uncomment if use_external_service is true
  dock: true
  use_external_feedback: false
  #feedback_timeout: 5.0      # uncomment if use_external_feedback is true
  #external_topic: ""         # uncomment if use_external_feedback is true
  internal_topic: "/movel_diff_drive_docking/success"
  pause_service: "/movel_diff_drive_docking/pause"
  docking_launch_package: "dalu_docking"
  docking_launch_file: "diff_drive_docking.launch"
  camera_name: "camera"
  enable_retry: false
  undocking_distance: 0.3
  undocking_speed: -0.1

undocking:
  watchdog_rate: 2.0
  watchdog_timeout: 0.0
  loop_rate: 10
  odom_topic: "/odom"
  disable_smoother: true
  use_external_service: false
  #external_service: ""       # uncomment if use_external_service is true
  #external_cancel_topic: ""  # uncomment if use_external_service is true
  dock: false
  undocking_distance: 1.0
  undocking_speed: -0.1
```

## Usage

The handler's function is to start and stop docking/undocking. After docking completes or before undocking starts, it provides an option to trigger external process through ROS service. Once task is complete, clean up is done automatically to exit the task.

### Task Payload Format

No payload required for docking with apriltag or undocking. Docking with localization requires payload with the same format as navigation_handler in task_supervisor.

## Parameters

### Common parameters

* *loop_rate*

Rate to run docking/undocking loop.

* *odom_topic*

ROS topic name in which odometry data is published. (nav_msgs/Odometry)

* *disable_smoother*

Whether to disable velocity_smoother while docking/undocking

* *use_external_service*

Whether to trigger external service after docking or before undocking.

* *external_service*

Name of ROS service to trigger external process. (std_srvs/Trigger)

* *external_cancel_topic*

Name of topic to publish to for cancelling external process. (std_msgs/Empty)

* *dock*

Set true for docking, and false for undocking.

* *undocking_distance*

Distance travelled during undocking. (for undocking, undock before docking retry)

* *undocking_speed*

Max speed during undocking. Positive value for forward direction, negative value for backward direction. (for undocking, undock before docking retry)

### Docking parameters

* *use_external_feedback*

Whether to use sensor feedback to determine if docking is finished

* *feedback_timeout*

Time period to wait for external feedback once docking motion is complete. If timeout exceeded, docking task fails.

* *external_topic*

ROS topic name to get sensor feedback (std_msgs/Bool)

* *internal_topic*

ROS topic name to get feedback from docking node (std_msgs/Bool)

* *pause_service*

Name of ROS service to pause/resume docking (std_srvs/SetBool)

* *docking_launch_package*

Package of docking launch file to be launched for docking.

* *docking_launch_file*

Launch file in the specified package to launch for docking.

* *camera_name*

Name assigned to the camera used to detect AprilTag marker. Used in:

1. Color image topic: /\<camera name\>/color/image_raw

2. Camera info topic: /\<camera name\>/color/camera_info

3. Camera frame: \<camera name\>_color_optical_frame

* *enable_retry*

Whether to retry docking if it fails.