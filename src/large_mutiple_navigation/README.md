# README #

This README would normally document whatever steps are necessary to get your application up and running.

### What is this repository for? ###

* This package consist of multi map loader and multi map navigation.
* Version 1.0
* [Learn Markdown](https://bitbucket.org/tutorials/markdowndemo)

### How do I get set up? ###

* Summary of set up
	This package consist of a complete workspace just create a workspace and build the package in it 
* How to run tests
    The package can be runned using the following commands
    ```
    roslaunch diff_wheeled_robot_gazebo diff_wheeled_gazebo_full.launch
    roslaunch diff_wheeled_robot_gazebo amcl.launch
     roslaunch map_swapper map_swapper.launch
    ```
* The map_swapper package consist of a node which can help in swapping the map during navigation. Inside the package it consist of an yaml file which will help in loading the the map. example model was given below
```
 - id: 0 
   name: hall
   floor: 0  
   pos: [ [[-4.320,-6.811],[-4.981,-7.416]], [[-4.223,1.616],[-4.851,0.584]], [[-4.4353,5.8762],[-4.975,4.747]],[[7.916,-0.249],[6.309,0.222]],[[3.017,-2.748],[3.605,-3.669]] ]
   map_location: "/home/navaneeth/Documents/practice_ws/ros_navigation_ws/src/diff_wheeled_robot_gazebo/maps/fivemap/hall.yaml"

```

### Algorithm ###
* The map_swapper was build by using two box method for swapping the map. The box can be given using two diagonal points of the region.with the id as mensioned above when ever the robot reaches that zone based on the id the map will be swapped.
* The Multi_map server uses a node with a service called /load_map . with this service the location of the map will be feed in the map_url as a request and the server will chage the map and return sucess or fail based on the swap as a response

### Who do I talk to? ###

* Repo owner or admin
* Other community or team contact