# Docking Plugins #

### How to setup ###
* Make sure you have this package: autodock-pallet package (newest with navigation_based_docking), task_supervisor package
* Add the plugins at task_supervisor.yaml with type : docking_handler::DockingHandler
* Add some parameters

### How to run ###
- `catkin_make`
- launch task_supervisor
- run docking task with payload, example: {\"x\": 1.5, \"y\": 6.0, \"theta\": 0.0, \"operation\": \"pickup\" ,\"id_pallet\": 0}

### Plugin in action ###
- Robot will go to the given goal from payload
- Robot will calculate the middle point of the pallet using its camera and go to that point
- Robot will go to the docking position by using navigation-based-docking 