cmake_minimum_required(VERSION 3.16.3)
project(robot_center LANGUAGES CXX)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" AND "${CMAKE_CXX_COMPILER_VERSION}" VERSION_LESS 6.0.0)
  message(FATAL_ERROR "sml requires GCC >= 6.0.0")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" AND "${CMAKE_CXX_COMPILER_VERSION}" VERSION_LESS 3.5.0)
  message(FATAL_ERROR "sml requires Clang >= 3.5.0")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.0)
  message(FATAL_ERROR "sml requires Visual Studio 14 2015 at least")
endif()

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-std=c++14 HAS_CXX14_FLAG)
check_cxx_compiler_flag(-std=c++17 HAS_CXX17_FLAG)
check_cxx_compiler_flag(-std=c++2a HAS_CXX20_FLAG)

if(HAS_CXX20_FLAG)
  set(CMAKE_CXX_STANDARD 20)
elseif(HAS_CXX17_FLAG)
  set(CMAKE_CXX_STANDARD 17)
elseif(HAS_CXX14_FLAG)
  set(CMAKE_CXX_STANDARD 14)
else()
  message(FATAL_ERROR "sml requires c++14")
endif()

message(WARNING "Using CXX: " ${CMAKE_CXX_STANDARD})
if(NOT (DEFINED CMAKE_CXX_STANDARD) OR CMAKE_CXX_STANDARD STREQUAL "" OR CMAKE_CXX_STANDARD LESS 14)
    message(FATAL_ERROR "sml requires c++14")
endif()

set(IS_COMPILER_GCC_LIKE FALSE)
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR
    "${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang" OR
    "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" )

  set(IS_COMPILER_GCC_LIKE TRUE)
endif()

set(CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  g3log
  GIT_REPOSITORY https://github.com/KjellKod/g3log.git
  GIT_TAG        1.3.4
)
FetchContent_GetProperties(g3log)
if(NOT g3log_POPULATED)
  FetchContent_Populate(g3log)
  add_subdirectory(${g3log_SOURCE_DIR} ${g3log_BINARY_DIR})
endif()

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

FetchContent_Declare(
  mongocxx
  GIT_REPOSITORY https://github.com/mongodb/mongo-cxx-driver.git
  GIT_TAG        r3.4.2
)
FetchContent_GetProperties(mongocxx)
if(NOT mongocxx_POPULATED)
  FetchContent_Populate(mongocxx)
  add_subdirectory(${mongocxx_SOURCE_DIR} ${mongocxx_BINARY_DIR})
endif()

include(ExternalProject)
ExternalProject_Add(ep_g3log
	DOWNLOAD_COMMAND _deps/g3log
    CMAKE_ARGS -DADD_FATAL_EXAMPLE=OFF -DCMAKE_BUILD_TYPE:STRING=Release
    BUILD_ALWAYS TRUE
)

ExternalProject_Add(ep_cxxopts
	DOWNLOAD_COMMAND _deps/cxxopts
    CMAKE_ARGS -DCXXOPTS_BUILD_EXAMPLES=OFF -DCXXOPTS_BUILD_TESTS=OFF
    BUILD_ALWAYS TRUE
)

ExternalProject_Add(ep_mongocxx
	DOWNLOAD_COMMAND _deps/mongocxx
    CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release -DENABLE_TESTS=OFF
    COMMAND ${CMAKE_COMMAND} --build . --target EP_mnmlstc_core
    COMMAND ${CMAKE_COMMAND} --build . --target install
    BUILD_ALWAYS TRUE
)

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
