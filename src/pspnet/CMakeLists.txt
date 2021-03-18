cmake_minimum_required(VERSION 3.16.3)
project(pspnet LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "-std=c++14")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

# set(CMAKE_MODULE_PATH "/home/autolab/libtorch")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  movel_hasp_vendor
  message_generation
)

if ("$ENV{MOVEL_LICENSE}" STREQUAL "KQAYZ")
      add_definitions(-DMOVEL_LICENSE)
endif()

if ("$ENV{MOVEL_LICENSE}" STREQUAL "RWBBX")
      add_definitions(-DMOVEL_LICENSE)
endif()



## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
# find_package(Torch REQUIRED)

generate_messages(
  DEPENDENCIES
  std_msgs
 # Or other packages containing msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   # INCLUDE_DIRS include
    LIBRARIES pspnet
    CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs 
#  DEPENDS system_lib
    CATKIN_DEPENDS message_runtime
)



###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(indoor_seg src/indoor_seg.cpp)
#target_link_libraries(indoor_seg ${catkin_LIBRARIES} ${OpenCV_LIBS} ${TORCH_LIBRARIES})
target_link_libraries(indoor_seg ${catkin_LIBRARIES} ${OpenCV_LIBS} ${TORCH_LIBRARIES})
add_dependencies(indoor_seg pspnet_generate_messages_cpp)
# set_property(TARGET indoor_seg PROPERTY CXX_STANDARD 14)

install(TARGETS indoor_seg
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)

