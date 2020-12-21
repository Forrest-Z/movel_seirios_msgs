# Install script for directory: /home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_common/cob_srvs

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cob_srvs/srv" TYPE FILE FILES
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_common/cob_srvs/srv/SetFloat.srv"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_common/cob_srvs/srv/SetInt.srv"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_common/cob_srvs/srv/SetString.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cob_srvs/cmake" TYPE FILE FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/cob_common/cob_srvs/catkin_generated/installspace/cob_srvs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/include/cob_srvs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/roseus/ros/cob_srvs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/common-lisp/ros/cob_srvs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/gennodejs/ros/cob_srvs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/lib/python3/dist-packages/cob_srvs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/lib/python3/dist-packages/cob_srvs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/cob_common/cob_srvs/catkin_generated/installspace/cob_srvs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cob_srvs/cmake" TYPE FILE FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/cob_common/cob_srvs/catkin_generated/installspace/cob_srvs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cob_srvs/cmake" TYPE FILE FILES
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/cob_common/cob_srvs/catkin_generated/installspace/cob_srvsConfig.cmake"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/cob_common/cob_srvs/catkin_generated/installspace/cob_srvsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cob_srvs" TYPE FILE FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_common/cob_srvs/package.xml")
endif()

