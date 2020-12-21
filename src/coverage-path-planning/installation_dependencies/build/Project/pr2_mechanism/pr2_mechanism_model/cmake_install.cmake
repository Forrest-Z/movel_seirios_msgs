# Install script for directory: /home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/pr2_mechanism/pr2_mechanism_model

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/pr2_mechanism/pr2_mechanism_model/catkin_generated/installspace/pr2_mechanism_model.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pr2_mechanism_model/cmake" TYPE FILE FILES
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/pr2_mechanism/pr2_mechanism_model/catkin_generated/installspace/pr2_mechanism_modelConfig.cmake"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/pr2_mechanism/pr2_mechanism_model/catkin_generated/installspace/pr2_mechanism_modelConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pr2_mechanism_model" TYPE FILE FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/pr2_mechanism/pr2_mechanism_model/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pr2_mechanism_model" TYPE DIRECTORY FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/pr2_mechanism/pr2_mechanism_model/include/pr2_mechanism_model/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpr2_mechanism_model.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpr2_mechanism_model.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpr2_mechanism_model.so"
         RPATH "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/install/Project/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/lib/libpr2_mechanism_model.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpr2_mechanism_model.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpr2_mechanism_model.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpr2_mechanism_model.so"
         OLD_RPATH "/opt/ros/noetic/lib::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"
         NEW_RPATH "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/install/Project/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpr2_mechanism_model.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pr2_mechanism_model" TYPE FILE FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/pr2_mechanism/pr2_mechanism_model/transmission_plugins.xml")
endif()

