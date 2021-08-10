# Dalu Docking Handler Plugin

A task\_supervisor plugin for starting and stopping docking/undocking task for dalu robot.

**Task type: 12**

## Prerequisites

* dalu_docking package

## task\_supervisor Config Setup

* Add dalu\_docking\_handler to 'plugins' section in task\_supervisor.yaml with class : dalu\_docking\_handler::DaluDockingHandler. Example:

```
  - {name: dalu_docking_handler, type: 12, class: 'dalu_docking_handler::DaluDockingHandler'}
```

* Add a 'dalu\_docking\_handler' section as follows:

```
dalu_docking_handler:
  watchdog_rate: 2.0
  watchdog_timeout: 1000.0
  dalu_docking_launch_package: "dalu_docking"
  dalu_docking_launch_file: "dalu_docking.launch"
  loop_rate: 10.0
  undocking_distance: 1.0
  undocking_speed: 0.1
  battery_status_timeout: 15.0
```

## Usage

The handler's function is to start and stop docking/undocking for dalu robot. Once task is complete, clean up is done automatically to exit the task.

### Task Payload Format

1. Dock with april tags

```
..., payload: 'dock', ...
```

2. Undock

```
..., payload: 'undock', ...
```

## Parameters

* *dalu_docking_launch_package*

Package of docking launch file to be launched for docking.

* *dalu_docking_launch_file*

Launch file in the specified package to launch for docking.

* *loop_rate*

Rate to run docking/undocking loop.

* *undocking_distance*

Distance travelled during undocking.

* *undocking_speed*

Max speed during undocking.

* *battery_status_timeout*

Timeout to wait for battery status.
