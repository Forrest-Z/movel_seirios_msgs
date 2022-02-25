# odom\_3dof

This is a package for filtering out z height, roll and pitch angles from raw odometry source.

## Prerequisites

1. ros\_utils

2. Raw odometry input

## Launch

Edit the launch file so that it loads the correct yaml file.

## Usage

1. Subscribes to raw odometry with *input_topic* as topic name.

2. Publishes filtered odometry with *output_topic* as topic name.

3. Provides tf from "odom" to child link with *child_frame_id* as frame id of child link (typically "base_link").