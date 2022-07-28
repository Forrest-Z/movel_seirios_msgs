cmake_minimum_required(VERSION 3.0.2)
project(movel_costmaps)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  roscpp
)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11) 
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()



## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/CostmapProhibitionLayer.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES costmap_prohibition_layer
  CATKIN_DEPENDS costmap_2d dynamic_reconfigure
  DEPENDS
)


include_directories(${catkin_INCLUDE_DIRS} include)
include_directories(${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})

## Declare a C++ library

add_library(costmap_prohibition_layer src/costmap_prohibition_layer.cpp)
target_link_libraries(costmap_prohibition_layer ${catkin_LIBRARIES})

# Dynamic reconfigure: make sure configure headers are built before any node using them
add_dependencies(costmap_prohibition_layer ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if (CONFIGURE_DEBIAN_PACKAGE)
  # copy python scripts to destination and mark as executables
  install(PROGRAMS scripts/prohib_mongo_bridge.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else() # will run if user invokes catkin_make or colcon witout -DCONFIGURE_DEBIAN_PACKAGE=ON flag
  catkin_install_python(
    PROGRAMS scripts/prohib_mongo_bridge.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()


## Mark executables and/or libraries for installation
install(TARGETS costmap_prohibition_layer
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   #FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


option(CONFIGURE_DEBIAN_PACKAGE "Configure debian package" OFF)
if (CONFIGURE_DEBIAN_PACKAGE)
  string(REGEX MATCH "^.*/seirios_ros" SEIRIOS_ROOT_DIR ${PROJECT_SOURCE_DIR})
  include(${SEIRIOS_ROOT_DIR}/debian.cmake)
endif()