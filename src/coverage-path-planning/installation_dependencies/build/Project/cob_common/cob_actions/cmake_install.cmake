# Install script for directory: /home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_common/cob_actions

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cob_actions/action" TYPE FILE FILES
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_common/cob_actions/action/SetBool.action"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_common/cob_actions/action/SetFloat.action"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_common/cob_actions/action/SetInt.action"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_common/cob_actions/action/SetString.action"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cob_actions/msg" TYPE FILE FILES
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetBoolAction.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetBoolActionGoal.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetBoolActionResult.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetBoolActionFeedback.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetBoolGoal.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetBoolResult.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetBoolFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cob_actions/msg" TYPE FILE FILES
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetFloatAction.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetFloatActionGoal.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetFloatActionResult.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetFloatActionFeedback.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetFloatGoal.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetFloatResult.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetFloatFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cob_actions/msg" TYPE FILE FILES
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetIntAction.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetIntActionGoal.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetIntActionResult.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetIntActionFeedback.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetIntGoal.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetIntResult.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetIntFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cob_actions/msg" TYPE FILE FILES
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetStringAction.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetStringActionGoal.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetStringActionResult.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetStringActionFeedback.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetStringGoal.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetStringResult.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_actions/msg/SetStringFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cob_actions/cmake" TYPE FILE FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/cob_common/cob_actions/catkin_generated/installspace/cob_actions-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/include/cob_actions")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/roseus/ros/cob_actions")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/common-lisp/ros/cob_actions")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/gennodejs/ros/cob_actions")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/lib/python3/dist-packages/cob_actions")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/lib/python3/dist-packages/cob_actions")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/cob_common/cob_actions/catkin_generated/installspace/cob_actions.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cob_actions/cmake" TYPE FILE FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/cob_common/cob_actions/catkin_generated/installspace/cob_actions-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cob_actions/cmake" TYPE FILE FILES
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/cob_common/cob_actions/catkin_generated/installspace/cob_actionsConfig.cmake"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/cob_common/cob_actions/catkin_generated/installspace/cob_actionsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cob_actions" TYPE FILE FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_common/cob_actions/package.xml")
endif()

