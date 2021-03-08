# Wall Inspection Handler Plugin

A task\_supervisor plugin for autonomous wall inspection.

**Task type: 8**

## Prerequisites

* wall\_inspection package

## task\_supervisor setup
* Add wall\_inspection\_handler to 'plugins' section in task\_supervisor.yaml with class : task\_supervisor::WallInspectionHandler

* Add a 'wall\_inspection\_handler' section with the following parameters:
   * wall\_inspection\_launch\_package: "wall\_inspection"
   * wall\_inspection\_launch\_file: "wall\_inspection.launch"

## Usage

The handler's function is to start and stop wall inspection. Once wall inspection starts, it ends when inspection is complete, or when a goal cancellation is issued to task\_supervisor.

### Task payload format

This task handler does not take any payload arguments.
