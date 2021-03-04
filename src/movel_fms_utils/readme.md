# Movel FMS Utils

Utilities for Fleet Management System FMS

## Requirements

- `movel_seirios_msgs`

## Plan Measurer

### Description

Measures plan length given waypoints (and implicitly, robot pose). Considers map and current state of the costmap. 

Implemented in two services:

- plan_measurer_node/measure_plan_simple
- plan_measurer_node/measure_plan

The former takes a single `geometry_msgs/Pose` and returns the plan length between the current robot position and the given pose. The latter takes a series of `geometry_msgs/Pose` and returns the plan length starting with the current robot position, visiting the given poses in order, and ending at the last pose in the input.

If no valid plan is found, the returned plan length is -1. For the general case with multiple poses, intermediate poses that are obstructed will be skipped and the plan length calculation continues with the next unobstructed pose. Except, if the final pose is obstructed, then the plan is deemed invalid and -1 will be returned.

### Usage

Launch `plan_measurer.launch` or include it in one of the setup launches. It will wait until `/move_base/make_plan` service is available before making its own services available. Consequently, localisation task must be active for this utility to work.

### Services

- plan_measurer_node/measure_plan_simple (type movel_seirios_msgs/GetPlanLengthSimple)
- plan_measurer_node/measure_plan (type movel_seirios_msgs/GetPlanLength)