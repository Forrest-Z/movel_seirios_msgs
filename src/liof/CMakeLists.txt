cmake_minimum_required(VERSION 3.16.3)
project(liof LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(OpenCV REQUIRED)
# find_package(OpenCV 4.1.1 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES liof
  CATKIN_DEPENDS roscpp
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(liof src/liof.cpp)
target_link_libraries(liof
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS liof
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

add_executable(example src/example.cpp)
target_link_libraries(example
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  liof
)
