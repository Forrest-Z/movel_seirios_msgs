# Aux Task Manager
https://docs.google.com/document/d/1W8DJe40wU3rdGmeF_ROb2Af_slhKbVS1iKbn97uU3xs/edit
Manages starting, cancelling and monitoring auxiliary tasks with ros topics.

## How it works
- aux_task_manager_node subscribes to topic /aux_task_manager/request published by web.
- Then it processes the request. After which it reports the status by publishing the topic /aux_task_manager/status.
- While alive, aux_task_manager continuously monitors running tasks and publish status to update.

## Topics
- Subscribes to `/aux_task_manager/request` 
- Publishes to `/aux_task_manager/status`


## Usage    

- Bash script/Launch file is required for the aux task 

- Sample Bash script below , save it as sample.bash inside catkin_ws/movel_ai/aux_tasks/ and make it as executable

```
#!/bin/bash

function proc_start {
    rostopic pub -1 /seirios_bridge/switch std_msgs/Bool "data: true"

    while true
    do
        sleep 10
    done
}

function proc_exit {
    rostopic pub -1 /seirios_bridge/switch std_msgs/Bool "data: false"
    exit 0
}

trap proc_exit TERM INT
proc_start

```
- sample.bash needs to be mounted inside the docker. Change the main docker-compose.yaml volume mounting under seirios-ros as below
 
 ` /home/$USER/amgen_ws/movel_ai/aux_tasks:/home/movel/.config/movel/aux_tasks:rw `