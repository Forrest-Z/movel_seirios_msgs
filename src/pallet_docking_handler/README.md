# Pallet Docking Handler Plugin

A task\_supervisor plugin for starting and stopping pallet docking/undocking task.

## Prerequisites

* pallet_detection package

## task\_supervisor Config Setup

* Add pallet\_docking\_handler to 'plugins' section in task\_supervisor.yaml with class : pallet\_docking\_handler::PalletDockingHandler. Example:

```
  - {name: docking, type: 15, class: 'pallet_docking_handler::PalletDockingHandler'}
  - {name: undocking, type: 16, class: 'pallet_docking_handler::PalletDockingHandler'}
```

* Add a 'docking' and 'undocking' sections as follows:

```
docking:
  watchdog_rate: 2.0
  watchdog_timeout: 0
  dock: true
  loop_rate: 10
  pallet_docking_launch_package: "pallet_detection"
  pallet_docking_launch_file: "pallet_detection.launch"  # "pallet_detection_move_base.launch" to use move_base
  use_move_base: false                                   # true to use move_base

undocking:
  watchdog_rate: 2.0
  watchdog_timeout: 0
  dock: false
  loop_rate: 10
  pallet_docking_launch_package: "pallet_detection"
  pallet_docking_launch_file: "pallet_detection.launch"
```

## Usage

The handler's function is to start and stop docking/undocking. Once task is complete, clean up is done automatically to exit the task.

### Task Payload Format

No payload required.

## Parameters

### Common parameters

* *loop_rate*

Rate to run docking/undocking loop.

* *dock*

Set true for docking, and false for undocking.

* *pallet_docking_launch_package*

Package of docking launch file to be launched for docking.

* *pallet_docking_launch_file*

Launch file in the specified package to launch for docking.

### Docking parameters

* *use_move_base*

Set true for move_base docking, false for planner_adjuster docking.

* *spot_turn_vel*

Angular velocity for turning on the spot to search for pallet.

* *detection_timeout*

Timeout to stop task when waiting for pallet to be detected.

* *xy_tolerance*

move_base TEB config for linear distance tolerance.

* *yaw_tolerance*

move_base TEB config for yaw tolerance.