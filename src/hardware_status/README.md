# hardware\_status

This is a package for monitoring hardware status based on published topics by hardware drivers or ROS nodes of hardware drivers. List of tracked hardware can be configured in the config file.

## Prerequisites

This package requires ros\_utils package for parameter loading.

## Launch

Edit the launch file so that it loads the correct yaml file.

## Usage

Service: '/hardware\_status/get\_status'

States: 0(Inactive: No data received from hardware), 1(Active: Received data from hardware)
