# Plan Inspector

Cancellation feature of navigation action when blocked by previously unseen obstacle.

## Use Case

Suppose you allow global plan to happen exactly once, and you want your robot to stop navigating when it sees a surprise obstacle, without recovery behaviour, and without the local planner trying its best to still follow the global plan.

Another example, suppose you want a behaviour where your robot stops immediately when it sees an unexpected obstacle, but proceed normally for previously seen obstacles. Unexpected obstacle is, for instance, when someone places a block behind your robot when it was not looking.

If you set your robot to replan periodically, then some obstructed plans will get cleared by replan. This package can still help by shorting the controls in the time before a new plan is ready.

The package also has a functionality to make the robot stop at some distance in front of the obstacle, rather than just stop immediately when it sees an unexpected obstacle. The user can specify the distance at the config file

## Working Principle

This node listens for global plan and local costmap. Whenever they are updated, it checks if the plan intersects occupied area in the local costmap. If it is, it will short the controls to zero, and wait for a set period. If after the period the obstruction is still around, it asks the action server to cancel its task. The package also updates the move_base params to deactivate the robot rotary behavior, replan when the path intersect the costmap, and clearing behavior.

*note: if you want to restart the package, please disable the plan_inspector first by using `rosservice call /enable_plan_inspector "data: false"` first to prevent misleading saved initial configuration.

## Requirements

- `cmd_vel_mux`

## Usage

- fill in config/plan_inspector.yaml with your robot-specific information
- launch plan_inspector.launch

## Setup Tips

Make sure inflation radius in both local and global costmaps are larger than robot radius (or footprint). If they are exactly equal, the edges of the costmap will have value of 99, and you will get spurious action cancellation. If they are larger, the values will drop off more gradually and your plans will survive, as long as they are actually viable. 

This can be visually checked in rviz. Set your local costmap to "costmap" color scheme, and make sure that cells within the inflation radius have more than one colors (not including the obstacle cells).

When using Ideaspark robot using DWA Local Planner, the size of local costmap should be increased in order to make the Planner won't replan the global path. The big local costmap size will make the planner know there is still a valid trajectory in the existing global path. Because the longest robot footprint is 2.2m, the size must be at least 4x bigger than the robot. *10m x10m is preferable.
