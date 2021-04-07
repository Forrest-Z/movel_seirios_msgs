# velocity_setter

This node sets the velocity of the robot through dynamic reconfigure of the velocity parameter of move_base node.

## Prerequisites

This node requires move_base node ([link](http://wiki.ros.org/move_base)).

## Launch

Set up the 'velocities' parameters in yaml file in the following format:

    velocities:
      cleaning: 0.3
      cruising: 0.5

where 'cleaning' and 'cruising' are examples of names of velocities to be requested, and the velocity values are in meters per second (m/s).

There is no limit on the number of velocities to be set up, and the velocity names can be any string.

## Usage

Set velocity service '/velocity\_setter/set\_velocity': Take velocity name (eg 'cruising', 'cleaning') as input and set velocity according to yaml file
