# move\_base\_state

This is a package for monitoring move\_base log messages for determining the state of move\_base node.

## Prerequisites

This package requires ros\_utils package for parameter loading.

## Launch

Edit the launch file so that it loads the correct yaml file.

## Usage

Published topic: '/move\_base\_state/state'

Output: 0, 1, 2, 3

| State       | Description                 |
| ----------- | --------------------------- |
| 0           | Normal operation            |
| 1           | Failed path planning        |
| 2           | Started recovery behavior   |
| 3           | Aborting goal               |
