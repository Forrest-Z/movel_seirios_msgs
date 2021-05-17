# Changelog
All notable changes to this project will be documented in this file.

## [1.0.4] - 2020-07-30
### Added
- Main addition is LaserScanMultiFilter which aims to provide a all-in-one node for filtering multiple laserscan topics from multiple lidars and merging them into a single laserscan topic. Please refer to README for trying out this node.
- Two header files each with their own distinct functions: LaserScanMultiFilter.h and LaserScanProfileUpdateServer. Please refer to the detailed documentation: https://movel.atlassian.net/l/c/znuPx8Rb
- One source file: laserscanmultifilter_node.cpp

### Changed
- Cleaned up some of the include folders 

### Removed
- Some files that have been used for node testing and are no longer required due to architectural changes.

## [1.0.3] - 2020-06-04
### Added
- New laser filter type Box Array Filter , which is able to define an arbitrary number of boxes to filter out. Adapted from polygon filters, box filters, and rack filters. Only issue is that it can be made more user friendly in terms of paramter input. 
- Comments to scanblobfilter and scanmaskfilter for future documentation

### Changed
- LaserScanCircularFilter now works.

### Removed
- Removed /launch folder as it is redundant, all launch examples are contained in /examples folder.


## [1.0.2] - 2020-05-31
### Added
- New laser filter types updated from the latest official package: ScanBlobFilter, LaserScanSpeckleFilter and LaserScanPolygonFilter. 
- Dynamic reconfigure server for all laser filters (except scanblobfilter, maskfilter and arrayfilter)
- ROSINFO messages to show which filter types have started up and configured successfully.
- Added 'dynamic_laser_client_node' which
- ROS Service to switch between multiple profiles: "profile_update"
- Added safeguard against int to double conversion when reconfiguring parameters.
- PolygonStamped Message for box_filter.

### Changed 
- Changed all laser_filters to include switches to enable/disable the filter using dynamic reconfigure/ROSParam server

### Removed
- Removed all changes in [1.0.1] as using ROSParam is a less robust solution to the runtime reconfiguration issues at hand.

## [1.0.1] - 2020-05-15
### Added
- Added 2 launch files in /launch: "test_dynamic_laser.launch" and "rviz_setup.launch".
- Added 2 nodes in /src: "test_scan_to_scan.cpp" and "test_laser_dynamic.cpp".
- Added 3 laser filter config files in /cfg: "filter_testset1.yaml", "filter_testset2.yaml", "filter_testset3.yaml".
- Added rviz config in /cfg: "mov_nav.rviz".

### Changed 
- Changed header files of the different filter types to update itself with the latest parameters from the parameter server. The files changed are "angular_bounds_filter.h",  "angular_bounds_filter_in_place.h", "box_filter.cpp", "box_filter.h", "intensity_filter.h", "range_filter.h", "scan_shadows_filter.h".    
-All changes in code can by found by searching [1.0.1] in the aforementioned files

### Removed
- None

