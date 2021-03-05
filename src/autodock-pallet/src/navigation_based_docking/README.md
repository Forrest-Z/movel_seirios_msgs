# Navigation Based Docking 

Navigation based doocking gives the capability to record waypoints and make the robot naviagte on those waypoints. The waypoints are saved in a txt file. The user can save the text file with a name pertaining to a pallet number like waypoints_1.txt.

## Usage
### Record Wayoints
rosrun navigation_based_docking record_waypoints 

You can change the name of the file in which you want to save waypoints in the record_waypoints.yaml file in the config folder

### Run the robot on saved waypoints

##### roslaunch navigation_based_docking navigation_based_docking.launch 

Along with  this command you will have to call a service StartAutoDocking using the following command

##### rosservice call /navigation_based_docking/StartAutoDocking "pallet_number: 0 operation: ''" 
* this command will read waypoints_0.txt

The pallet number needs to be mentioned where the robot needs to go. The pallet number should correspond to the specific waypoint file stored i.e if pallet number is 1 then it will correspond to waypoints_1.txt file.
operation can be pickup or drop. Though no specific functionality has been implemented yet. The developer can change the code if the client requies any specific functionlity in genral. Like this code can also be used for docking to charging station. Therefore, charging can also become an operation.

### The following params can be set in the navigation_based_docking code for running the robot on the saved waypoints

1) eps: 0.6    - the distance to the next waypoint so that move base considers the next waypoint as the desired location. The code will take the square root of it i.e sqrt(0.6)

2) waypoints_folder: /home/sudhakar/pallet_docking/waypoints_ - waypoint file. The pallet number would be picked up from the service and subsequently appended to the foldername path i.e if pallet number is 1 in the service call, then it will correspond to waypoints_1.txt file.

3) param_name: inflation_radius - the parameter you would like to change dynamically . The inflation radius value in the local costmap should change as the robot will be very close to the pallet while docking

4) param_value: 1.0 -  the value of the parameter you have defined earlier

5)  service_call_topic: /move_base/local_costmap/inflation_layer/set_parameters  - To change the above mentioned param, you need to tell which service call would update the same. Set parameters is the one that updates the values. 


