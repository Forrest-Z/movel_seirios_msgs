### Introduction

This is a modified version of the laser_filters package (https://github.com/ros-perception/laser_filters) that allows reconfiguration of laser filter parameters at runtime by switching between multiple filter chains (known as profiles). This allows dynamic laser filter setups for specific scenarios (Navigation in environments with specular surfaces, removal of box footprints, being surrounded by rack legs etc.). 

The latest addition is the LaserMultiFilter node, which is an all-in-one node for filtering multiple laserscan topics from multiple lidars and merging them into a single laserscan topic. More details can be found at https://movel.atlassian.net/wiki/spaces/EG/pages/98041857/Laser+pipeline+laserscan+multi+filter?atlOrigin=eyJpIjoiYmY2MzQ5M2M2YmNlNDA4ZmFkODNkMWE1OTdhOWQxNzAiLCJwIjoiYyJ9

### LaserMultiFilter Quick Start

The LaserMultiFilter node has 3 primary functions:

1. Filtering multiple laserscan topics, each with their own unique filter chain, and then publishing their individual outputs. Example: for input scan topics “/scan_1”, “/scan_2”, … . The published filtered scan topics would be “/scan_1_filtered”, “/scan_2_filtered”.

2. Merging multiple filtered laserscan topics into a single node, and publishing it. Example: input scan topics “/scan_1”, “/scan_2”, … would be merged into “/scan_merged”.

3. Updating the filter chains of each laserscan topic through dynamic reconfigure. Through a ROSService call, we can update the filter chain of each individual laserscan topic at runtime by specifying 2 strings:  Scan topic (“/scan_i”) and Profile number (“profilei”) where i is an arbitrary number.

We assign profiles, a filter chain we want to apply, to each scan topic. The first profile will be applied to the scan topic by default, with the other profiles being alternative configurations which we can switch to later on.

#### Step 0: Scan topics remapping
We follow the convention that the scan topics are named: /scan_front, /scan_left, /scan_rear and /scan_right (instead of /scan_i as indicated in the rest of the document). If there is a deviation from this convention, please change the `laserscan_topics` parameter in the launch file `launch/laserscan_multi_filter.launch`, and change the names (scan_1, scan_2, ...) in the config file `config/profile_examples.yaml`.

#### Step 1: Configuration parameters
The structure of the YAML configuration file is as such - We have scan topics at the highest level, followed by the profiles under each scan topic, and then the filter parameters under each profile. A full example of this is provided in `config/profile_examples.yaml`.

We define the laser filter profiles as required for each scan topic. The format of the profile.yaml parameter is as follows:
```
scan_1: 
  profile1: 
    tf_message_filter_target_frame: base_link
    scan_filter_chain:
      -name: filter1
      -type: laser_filters/filtertype1
      -params: 
        param1: 3
        param2: 0.5
        ...
        switch_: True
        
      -name: filter2
      -type: laser_filters/filtertype2
      -params: 
        param1: -0.5
        ...
        switch_: False
      ...

  profile2:
    tf_message_filter_target_frame: base_link
    scan_filter_chain:
      -filter 1
      -filter 2
      ...
      
scan_2:
  profile1:
    ...

scan_3:
  ...
```

#### Step 2: Launch LaserScanMultiFilter node
We launch the LaserScanMultiFilter node from `launch/laserscan_multi_filter.launch` which is achieved through the following command:
```
roslaunch laser_filters laserscan_multi_filter.launch
```
This should result in the publishing of the filtered scan topics (/scan_1_filtered, /scan_2_filtered, ...) and the merged scan (/scan_merged).

If we look at `launch/laserscan_multi_filter.launch`. We find the following launch parameters:
```
<node pkg="laser_filters" type="laserscanmultifilter_node" output="screen" name="laserscanmultifilter_node">
    <rosparam command="load" file="$(find laser_filters)/config/profiles_test.yaml" />
    <param name="laserscan_topics" value="/scan_front /scan_rear"/>
    <param name="destination_frame" value="base_link"/>  
    <param name="merged_destination_topic" value="/scan_merged"/>
    <param name="angle_min" value="-3.14159"/>
    <param name="angle_max" value="3.14159"/>
    <param name="angle_increment" value="0.0058"/>
    <param name="scan_time" value="0.0333333"/>
    <param name="range_min" value="0.25"/>
    <param name="range_max" value="25.0"/>
</node>
```
- laserscan_topics: These are the scan topics that we will be filtering and merging. They must already be present in the rostopic server or they will not be processed by the node. You can add as many as needed as long as they are on the same plane.
- destination_frame: This is the frame that the merged laser scan will be transformed relative to.
- merged_destination_topic: The topic name of the published merged laser scan.
- The other parameters are self explanatory

#### Step 3: Switch between profiles at runtime
To switch between different profiles at runtime, we use a ROS Service call in the client `profile_update_client` located in `src/profile_update_client.cpp`. 
In the following command, replace `[scan_i]` and `[profileid]` with the desired fields (which must already be present in the YAML configuration file, see Step 1).
```
rosrun laser_filters profile_update_client [scan_i] [profileid]
```

### Documentation

For more detailed documentation on the individual filter types available, please visit:
https://movel.atlassian.net/wiki/spaces/EG/pages/44662785/laser+filters+1.0.4?atlOrigin=eyJpIjoiN2I5YTcxMTg3NTM5NDY0NWE1NTkyZjI0NTRmZDNmZTUiLCJwIjoiYyJ9

### Known Issues

There is a few issues that might (or might not) require fixes:

- Do not launch multiple instances of the same filter using the dynamic laser filter node. If you require 2 box filters, use LaserScanBoxArrayFilter, where you can specify an array of boxes (Please refer to the detailed documentation for usage). Having two instance of the same filter in the same profile will only activate a single dynamic reconfigure client for one of them.

- Merging filter scans make them lose their intensity values. This is because there is a conversion from laserscan → point cloud → laserscan, thereby losing information in intensity in the process.

### Unknown Issues

- The LaserMultiFilter node has not been tested on any hardware yet! 

### License
Original laser_filters package is BSD licensed (https://opensource.org/licenses/BSD-3-Clause)
