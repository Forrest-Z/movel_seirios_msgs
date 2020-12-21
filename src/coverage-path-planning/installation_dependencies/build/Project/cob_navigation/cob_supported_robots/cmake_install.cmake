# Install script for directory: /home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_navigation/cob_supported_robots

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/install/Project")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/cob_navigation/cob_supported_robots/catkin_generated/installspace/cob_supported_robots.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cob_supported_robots/cmake" TYPE FILE FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/cob_navigation/cob_supported_robots/catkin_generated/installspace/cob_supported_robots-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cob_supported_robots/cmake" TYPE FILE FILES
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/cob_navigation/cob_supported_robots/catkin_generated/installspace/cob_supported_robotsConfig.cmake"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/cob_navigation/cob_supported_robots/catkin_generated/installspace/cob_supported_robotsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cob_supported_robots" TYPE FILE FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_navigation/cob_supported_robots/package.xml")
endif()

