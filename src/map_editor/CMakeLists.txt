cmake_minimum_required(VERSION 3.16.3)
project(map_editor LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_srvs
    geometry_msgs
    std_msgs
    message_generation
    movel_hasp_vendor
    movel_seirios_msgs
)

if ("$ENV{MOVEL_LICENSE}" STREQUAL "true")
  add_definitions(-DMOVEL_LICENSE)
endif()

find_package(OpenCV REQUIRED)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp std_srvs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(map_editor src/map_editor.cpp)
target_link_libraries(map_editor ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(map_editor_node src/map_editor_node.cpp)
target_link_libraries(map_editor_node map_editor ${catkin_LIBRARIES} ${OpenCV_LIBS})


install(TARGETS map_editor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS map_editor_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


