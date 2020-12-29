cmake_minimum_required(VERSION 3.16.3)
project(robot_center LANGUAGES CXX)

include(cmake/colorize.cmake)
include(cmake/cxxStandard.cmake)

# Allows target_link_libraries() to be called from any directory to add
# dependencies and interface libraries to targets created in other directories
cmake_policy(SET CMP0079 NEW)

find_package(catkin REQUIRED COMPONENTS
)

find_package(Boost REQUIRED)
find_package(libmongocxx REQUIRED)
find_package(libbsoncxx REQUIRED)
find_package(g3log CONFIG REQUIRED)
find_package(cxxopts CONFIG REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
	${catkin_INCLUDE_DIRS}
    ${G3LOG_INCLUDE_DIRS}
    ${LIBMONGOCXX_INCLUDE_DIRS}
    ${LIBBSONCXX_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
    src/robot_center/robot_center.cpp
    src/sm/state_init.cpp src/sm/state_idle.cpp src/sm/state_error.cpp src/sm/state_map.cpp src/sm/state_mission.cpp src/sm/state_prep.cpp src/sm/sm.cpp
    src/helpers/fs_helper.cpp src/helpers/mongo_helper.cpp src/helpers/rabbit_helper.cpp src/helpers/ros_helper.cpp
)

target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
    ${LIBMONGOCXX_LIBRARIES}
    ${G3LOG_LIBRARIES}
    cxxopts::cxxopts
    ${LIBBSONCXX_LIBRARIES}
    uv
    )

#add_executable(dependency
#    src/examples/dependency.cpp
#)
#
#target_link_libraries(dependency
#    ${LIBMONGOCXX_LIBRARIES}
#    ${G3LOG_LIBRARIES}
#    ${LIBBSONCXX_LIBRARIES}
#    ${G3LOG_LIBRARIES}
#    uv
#    )
