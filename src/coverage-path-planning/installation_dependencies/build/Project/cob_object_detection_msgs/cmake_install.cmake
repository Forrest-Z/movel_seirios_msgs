# Install script for directory: /home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_object_detection_msgs

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cob_object_detection_msgs/action" TYPE FILE FILES
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_object_detection_msgs/action/AcquireObjectImage.action"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_object_detection_msgs/action/DetectObjects.action"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_object_detection_msgs/action/TrainObject.action"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cob_object_detection_msgs/msg" TYPE FILE FILES
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_object_detection_msgs/msg/AcquireObjectImageAction.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_object_detection_msgs/msg/AcquireObjectImageActionGoal.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_object_detection_msgs/msg/AcquireObjectImageActionResult.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_object_detection_msgs/msg/AcquireObjectImageActionFeedback.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_object_detection_msgs/msg/AcquireObjectImageGoal.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_object_detection_msgs/msg/AcquireObjectImageResult.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_object_detection_msgs/msg/AcquireObjectImageFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cob_object_detection_msgs/msg" TYPE FILE FILES
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_object_detection_msgs/msg/DetectObjectsAction.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_object_detection_msgs/msg/DetectObjectsActionGoal.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_object_detection_msgs/msg/DetectObjectsActionResult.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_object_detection_msgs/msg/DetectObjectsActionFeedback.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_object_detection_msgs/msg/DetectObjectsGoal.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_object_detection_msgs/msg/DetectObjectsResult.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_object_detection_msgs/msg/DetectObjectsFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cob_object_detection_msgs/msg" TYPE FILE FILES
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_object_detection_msgs/msg/TrainObjectAction.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_object_detection_msgs/msg/TrainObjectActionGoal.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_object_detection_msgs/msg/TrainObjectActionResult.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_object_detection_msgs/msg/TrainObjectActionFeedback.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_object_detection_msgs/msg/TrainObjectGoal.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_object_detection_msgs/msg/TrainObjectResult.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/cob_object_detection_msgs/msg/TrainObjectFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cob_object_detection_msgs/msg" TYPE FILE FILES
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_object_detection_msgs/msg/Detection.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_object_detection_msgs/msg/DetectionArray.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_object_detection_msgs/msg/Mask.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_object_detection_msgs/msg/MaskArray.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_object_detection_msgs/msg/PoseRT.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_object_detection_msgs/msg/Rect.msg"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_object_detection_msgs/msg/RectArray.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cob_object_detection_msgs/srv" TYPE FILE FILES
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_object_detection_msgs/srv/AcquireObjectImage.srv"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_object_detection_msgs/srv/BaTestEnvironment.srv"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_object_detection_msgs/srv/BagTrainObject.srv"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_object_detection_msgs/srv/DetectObjects.srv"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_object_detection_msgs/srv/SaveRecordedObject.srv"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_object_detection_msgs/srv/StartObjectRecording.srv"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_object_detection_msgs/srv/StopObjectRecording.srv"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_object_detection_msgs/srv/TrainObject.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cob_object_detection_msgs/cmake" TYPE FILE FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/cob_object_detection_msgs/catkin_generated/installspace/cob_object_detection_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/include/cob_object_detection_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/roseus/ros/cob_object_detection_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/common-lisp/ros/cob_object_detection_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/share/gennodejs/ros/cob_object_detection_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/lib/python3/dist-packages/cob_object_detection_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/devel/lib/python3/dist-packages/cob_object_detection_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/cob_object_detection_msgs/catkin_generated/installspace/cob_object_detection_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cob_object_detection_msgs/cmake" TYPE FILE FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/cob_object_detection_msgs/catkin_generated/installspace/cob_object_detection_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cob_object_detection_msgs/cmake" TYPE FILE FILES
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/cob_object_detection_msgs/catkin_generated/installspace/cob_object_detection_msgsConfig.cmake"
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/cob_object_detection_msgs/catkin_generated/installspace/cob_object_detection_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cob_object_detection_msgs" TYPE FILE FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_object_detection_msgs/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cob_object_detection_msgs" TYPE PROGRAM FILES "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/cob_object_detection_msgs/catkin_generated/installspace/detection_component_testing.py")
endif()

