cmake_minimum_required(VERSION 3.16.3)
project(points_preprocessor_usi LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  pcl_conversions
  velodyne_pointcloud
  velodyne_pcl
)

catkin_package(CATKIN_DEPENDS
	sensor_msgs
	velodyne_pointcloud
	velodyne_pcl
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(Eigen3 REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})

#Ground Plane Fitter
find_package(PCL 1.7.2 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

add_executable(groundplanfit
	nodes/ground_filter/groundplanfit.cpp
)
add_executable(runtime_groundplanfit
	nodes/ground_filter/runtime_groundplanfit.cpp
)


target_include_directories(groundplanfit PUBLIC
	${PCL_INCLUDE_DIRS}
)

target_link_libraries(groundplanfit
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

target_include_directories(runtime_groundplanfit PUBLIC
	${PCL_INCLUDE_DIRS}
)

target_link_libraries(runtime_groundplanfit
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

#Scan Line Run
add_executable(scanlinerun
	nodes/clustering/scanlinerun.cpp
)

target_include_directories(scanlinerun PUBLIC
	${PCL_INCLUDE_DIRS}
)

target_link_libraries(scanlinerun
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(TARGETS groundplanfit scanlinerun runtime_groundplanfit
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
