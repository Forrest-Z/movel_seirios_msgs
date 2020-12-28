cmake_minimum_required(VERSION 3.16.3)
project(robot_center LANGUAGES CXX)

include(cmake/colorize.cmake)
include(cmake/cxxStandard.cmake)

# Allows target_link_libraries() to be called from any directory to add
# dependencies and interface libraries to targets created in other directories
cmake_policy(SET CMP0079 NEW)

# Opting for this over ExternalProject_Add() as the latter populates, builds,
# and install in the main project's Build step.
message("${BoldCyan}Configuring G3Log\n\n${ColourReset}")
include(FetchContent)
FetchContent_Declare(
  g3log
  GIT_REPOSITORY https://github.com/KjellKod/g3log.git
  GIT_TAG        1.3.4
  SOURCE_SUBDIR ${g3log_SOURCE_DIR}
)
FetchContent_GetProperties(g3log)
if(NOT g3log_POPULATED)
  FetchContent_Populate(g3log)
  add_subdirectory(${g3log_SOURCE_DIR} ${g3log_BINARY_DIR})
endif()
#FetchContent_MakeAvailable(g3log)

message("${BoldCyan}Configuring CXXOPTS\n\n${ColourReset}")
FetchContent_Declare(
  cxxopts
  GIT_REPOSITORY https://github.com/jarro2783/cxxopts.git
  GIT_TAG        v2.2.1
)
FetchContent_GetProperties(cxxopts)
if(NOT cxxopts_POPULATED)
  FetchContent_Populate(cxxopts)
  add_subdirectory(${cxxopts_SOURCE_DIR} ${cxxopts_BINARY_DIR})
endif()
set(CXXOPTS_BUILD_EXAMPLES OFF)
set(CXXOPTS_BUILD_TESTS OFF)
set(ADD_FATAL_EXAMPLE OFF)
set(ADD_G3LOG_UNIT_TEST OFF)
FetchContent_MakeAvailable(cxxopts)

#message("${BoldCyan}Configuring MongoCXX\n\n${ColourReset}")
#FetchContent_Declare(
#  mongocxx
#  GIT_REPOSITORY https://github.com/mongodb/mongo-cxx-driver.git
#  GIT_TAG        r3.4.2
#)
#FetchContent_GetProperties(mongocxx)
#if(NOT mongocxx_POPULATED)
#  FetchContent_Populate(mongocxx)
#  #add_subdirectory(${mongocxx_SOURCE_DIR} ${mongocxx_BINARY_DIR})
#endif()


#message("${BoldYellow}Building G3Log\n\n${ColourReset}")
#include(ExternalProject)
#ExternalProject_Add(g3log
#    SOURCE_DIR ${g3log_SOURCE_DIR}
#    BINARY_DIR ${g3log_BINARY_DIR}
#    CMAKE_ARGS -DADD_FATAL_EXAMPLE=OFF -DCMAKE_BUILD_TYPE:STRING=Release
#    BUILD_ALWAYS TRUE
#)
#
#message("${BoldYellow}Building CXXOPTS\n\n${ColourReset}")
#ExternalProject_Add(cxxopts
#    SOURCE_DIR ${cxxopts_SOURCE_DIR}
#    BINARY_DIR ${cxxopts_BINARY_DIR}
#    CMAKE_ARGS -DCXXOPTS_BUILD_EXAMPLES=OFF -DCXXOPTS_BUILD_TESTS=OFF
#    BUILD_ALWAYS TRUE
#)

#message("${BoldYellow}Building MongoCXX\n\n${ColourReset}")
#ExternalProject_Add(mongocxx
#    SOURCE_DIR ${mongocxx_SOURCE_DIR}
#    BINARY_DIR ${mongocxx_BINARY_DIR}
#    CONFIGURE_COMMAND ""
#    CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release -DENABLE_TESTS=OFF
#    COMMAND ${CMAKE_COMMAND} --build . --target EP_mnmlstc_core
#    COMMAND ${CMAKE_COMMAND} --build . --target install
#    INSTALL_COMMAND ""
#    BUILD_ALWAYS TRUE
#)

#
#find_package(Boost REQUIRED)
#find_package(cxxopts REQUIRED)
#find_package(libmongocxx REQUIRED)
#find_package(libbsoncxx REQUIRED)
#
#message(
#"g3log package:
#    found: ${G3LOG_FOUND}
#    include dir: ${G3LOG_INCLUDE_DIRS}
#    libraries: ${G3LOG_LIBRARIES}
#    "
#)
#
#include_directories(
#    ${CMAKE_CURRENT_SOURCE_DIR}/include
#    ${G3LOG_INCLUDE_DIRS}
#    ${LIBMONGOCXX_INCLUDE_DIRS}
#    ${LIBBSONCXX_INCLUDE_DIRS}
#)
#
#add_executable(${PROJECT_NAME}
#    src/robot_center/robot_center.cpp
#    src/sm/state_init.cpp src/sm/state_idle.cpp src/sm/state_error.cpp src/sm/state_map.cpp src/sm/state_mission.cpp src/sm/state_prep.cpp src/sm/sm.cpp
#    src/helpers/fs_helper.cpp src/helpers/mongo_helper.cpp src/helpers/rabbit_helper.cpp src/helpers/ros_helper.cpp
#)
#
#target_link_libraries(${PROJECT_NAME} PUBLIC
#    ${LIBMONGOCXX_LIBRARIES}
#    ${G3LOG_LIBRARIES}
#    ${LIBBSONCXX_LIBRARIES}
#    ${G3LOG_LIBRARIES}
#    uv
#    )

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
