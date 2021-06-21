cmake_minimum_required(VERSION 3.16.3)
project(movel_hasp_vendor LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
)
if (NOT ("$ENV{BUILD_AARCH}" STREQUAL "arm64"))
  if ("$ENV{MOVEL_LICENSE}" STREQUAL "KQAYZ")
    add_library(${PROJECT_NAME} include/ src/errorprinter.cpp src/haspbase.cpp src/haspimpl.cpp src/haspdiag.cpp src/haspenc64.cpp src/haspfeature.cpp src/haspfile.cpp src/hasphandle.cpp src/hasphasp.cpp src/haspinfo.cpp src/hasplegacy.cpp src/hasplock.cpp src/haspmap.cpp src/haspmain.cpp src/hasptime.cpp src/haspversion.cpp src/license_91368.cpp)
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/libhasp_linux_x86_64_91368.a -lpthread)
  elseif ("$ENV{MOVEL_LICENSE}" STREQUAL "RWBBX")
    add_library(${PROJECT_NAME} include/ src/errorprinter.cpp src/haspbase.cpp src/haspimpl.cpp src/haspdiag.cpp src/haspenc64.cpp src/haspfeature.cpp src/haspfile.cpp src/hasphandle.cpp src/hasphasp.cpp src/haspinfo.cpp src/hasplegacy.cpp src/hasplock.cpp src/haspmap.cpp src/haspmain.cpp src/hasptime.cpp src/haspversion.cpp src/license_34404.cpp)
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/libhasp_linux_x86_64_34404.a -lpthread)
  else ()
    add_library(${PROJECT_NAME} include/ src/errorprinter.cpp src/haspbase.cpp src/haspimpl.cpp src/haspdiag.cpp src/haspenc64.cpp src/haspfeature.cpp src/haspfile.cpp src/hasphandle.cpp src/hasphasp.cpp src/haspinfo.cpp src/hasplegacy.cpp src/hasplock.cpp src/haspmap.cpp src/haspmain.cpp src/hasptime.cpp src/haspversion.cpp src/license_34404.cpp)
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/libhasp_linux_x86_64_34404.a -lpthread)
  endif()
else ()
  if ("$ENV{MOVEL_LICENSE}" STREQUAL "KQAYZ")
    add_library(${PROJECT_NAME} include/ src/errorprinter.cpp src/haspbase.cpp src/haspimpl.cpp src/haspdiag.cpp src/haspenc64.cpp src/haspfeature.cpp src/haspfile.cpp src/hasphandle.cpp src/hasphasp.cpp src/haspinfo.cpp src/hasplegacy.cpp src/hasplock.cpp src/haspmap.cpp src/haspmain.cpp src/hasptime.cpp src/haspversion.cpp src/license_91368.cpp)
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/libhasp_linux_arm64_91368.a -lpthread)
  elseif ("$ENV{MOVEL_LICENSE}" STREQUAL "RWBBX")
    add_library(${PROJECT_NAME} include/ src/errorprinter.cpp src/haspbase.cpp src/haspimpl.cpp src/haspdiag.cpp src/haspenc64.cpp src/haspfeature.cpp src/haspfile.cpp src/hasphandle.cpp src/hasphasp.cpp src/haspinfo.cpp src/hasplegacy.cpp src/hasplock.cpp src/haspmap.cpp src/haspmain.cpp src/hasptime.cpp src/haspversion.cpp src/license_34404.cpp)
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/libhasp_linux_arm64_34404.a -lpthread)
  else ()
    add_library(${PROJECT_NAME} include/ src/errorprinter.cpp src/haspbase.cpp src/haspimpl.cpp src/haspdiag.cpp src/haspenc64.cpp src/haspfeature.cpp src/haspfile.cpp src/hasphandle.cpp src/hasphasp.cpp src/haspinfo.cpp src/hasplegacy.cpp src/hasplock.cpp src/haspmap.cpp src/haspmain.cpp src/hasptime.cpp src/haspversion.cpp src/license_34404.cpp)
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/libhasp_linux_arm64_34404.a -lpthread)
  endif()
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

option(CONFIGURE_DEBIAN_PACKAGE "Configure debian package" OFF)
if (CONFIGURE_DEBIAN_PACKAGE)
  string(REGEX MATCH "^.*/seirios_ros" SEIRIOS_ROOT_DIR ${PROJECT_SOURCE_DIR})
  include(${SEIRIOS_ROOT_DIR}/debian.cmake)
endif()