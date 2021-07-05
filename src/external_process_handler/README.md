# Launch your own Plugins #


# PARAM and Plugin setup #

* - {name: task_name, type: task_type, class: 'external_process_handler::ExternalProcessHandler'}


* task_name:
    - watchdog_rate: 2.0
    - watchdog_timeout: 0
    - service_req: true #bool type required field need to mention true or false based on how the user wanna run the external process
    - launch_req: true # type required field need to mention true or false based on how the user wanna run the external process
    - service_start: "/client/process/start" #required if service_req is true
    - service_start_msg: "START" #optional  if there is any particular msg needs to sent or else empty string 
    - service_stop: "/client/process/stop" #required if service_req is true
    - service_start_msg: "STOP" #optional  if there is any particular msg needs to sent or else empty string 
    - launch_package: "pkg name " #required if launch_req is true
    - launch_file: "launch file"  #required if launch_req is true
    - launch_nodes: " nodes launched" #required if launch_req is true
    - topic_cancel_req: true # required if you wanna publish some topic to cancel. 
    - topic_process_cancel: "/client/node/cancel" # opt name publishes bool true if UI pressed cancel. This works if topic process cancel req is true.
    - client_status: /client/node/status # optional if only launch file then - need provide status to this topic if 2 is passed task is success and 3   is task failed

### How to setup ###

* external package needs to be catkin_make install and should be mounted to seirios_ws/. or any other path also fine unless it is sourced.  
* Add the plugins at task_supervisor.yaml with type : external_process_handler::ExternalProcessHandler
* Add the plugin and param as descibed above

### How to run ###
* launch task_supervisor
* called based on the task type


### Plugin in action ###
- Will launch the file req or call the service req or both.
- Will kill tbe launch file or stop the service req or both.
