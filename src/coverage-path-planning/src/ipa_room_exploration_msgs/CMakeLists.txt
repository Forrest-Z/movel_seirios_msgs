cmake_minimum_required(VERSION 2.8.3)
project(ipa_room_exploration_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	message_generation
    nav_msgs
	std_msgs
)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
	DIRECTORY
		msg
	FILES
        PathInfo.msg
)

## Generate services in the 'srv' folder
add_service_files(
	DIRECTORY
		srv
	FILES
        RoomExplorationClient.srv
        SavePath.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
		std_msgs
        nav_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	CATKIN_DEPENDS
		message_runtime
        nav_msgs
		std_msgs
)

include_directories(
	${catkin_INCLUDE_DIRS}
)
