#!/usr/bin/python3

import rospy
import json
import sys
import threading
import subprocess
import os
import platform
from std_msgs.msg import String


class AuxTaskManager:

    def __init__(self):
        self.request_sub = rospy.Subscriber("/aux_task_manager/request", String, self.CB_request)
        self.status_pub = rospy.Publisher("/aux_task_manager/status", String, queue_size=20)
        self.running_tasks = {}
        self.running_cancel_threads = {}
        self.lock = threading.RLock()


    def CB_request(self, msg):
        # check for request_type 
        # start the relevant function based on request_type 
        #   start, cancel, cancel_all, poll, poll_all
        rospy.loginfo("Receiving request msg")
        d = json.loads(msg)
        req_type = d["request_type"]
        
        # task_id = d["task_id"]
        # payload = d["payload"] # type dict

        if req_type == "start":
            task_id = d["task_id"]
            payload = d["payload"] # type dict
            self.__process_start_request(task_id, payload)
        elif req_type == "cancel":
            task_id = d["task_id"]
            self.__process_cancel_request(task_id)
        elif req_type == "cancel_all":
            self.__process_cancel_all_request()
        elif req_type == "poll":
            task_id = d["task_id"]
            self.__process_poll_request(task_id)
        elif req_type == "poll_all":
            self.__process_poll_all_request()
        else:
            err = "Unknown request type" + " " + req_type
            self.pub_status(task_id, "not_running", err)
            #raise Exception("Unknown request type", req_type)

    def pub_status(self, task_id, status, error_msg=None):
        # format the details into a JSON and publish
        x = {"task_id": task_id, "status": status, "error_msg": error_msg}
        formated_msg = json.dumps(x)
        self.status_pub.publish(formated_msg)

    def monitor_loop(self):
        d = rospy.Duration(5.0)
        while not rospy.is_shutdown():
            
            # self.running_tasks status check
            task_status = {}
            with self.lock:
                # loop through self.running_tasks track status in task_status
                for task_id in self.running_tasks:
                    self.running_tasks[task_id].poll()

                    if self.running_tasks[task_id].poll() is None:
                        task_status[task_id] = "running"
                    else:
                        task_status[task_id] = "not_running"
            
            # loop through task_status
                for task_id in task_status:
                    if task_status[task_id] == "running":
                        self.pub_status(task_id, task_status[task_id])
                    elif task_status[task_id] == "not_running":
                        self.__process_cancel_request(task_id) 

            
            # self.running_cancel_threads status check
            with self.lock:
                # loop through self.running_cancel_threads and remove if thread is not is_alive()
                self.__clean_dead_threads(task_id)
                    
            rospy.sleep(d)


    def __clean_dead_threads(self, task_id):
        for task_id in self.running_cancel_threads:
            if not self.running_cancel_threads[task_id].is_alive():
                #self.running_cancel_threads[task_id].join()
                self.running_cancel_threads.pop(task_id)


    def __del__(self):
        # clean up of self.running_tasks
        self.__process_cancel_all_request()  

        # waiting for theads in self.running_cancel_threads to join before exiting main 
        for task_id in self.running_cancel_threads:
            self.running_cancel_threads[task_id].join()
        
        os.kill(os.getpid(), signal.SIGINT)



    ### process request functions

    def __process_start_request(self, task_id, payload):
        # start popen process based on roslaunch, rosrun, or executable
        #       format cmd args based on payload
        # how to handle start if task_id already exists?
        # register task to self.running_tasks
        # self.pub_status()
        # TODO: how to implement start timeout?

        # running_tasks - keys: task_id, values: popen_object
        # if task_id in self.running_tasks.keys():
        if task_id in self.running_tasks:
            err = f"Duplicate task_id: {task_id}"
            self.pub_status(task_id, "not_running", err)
            raise Exception(err) 
            # add early exit
        
        #popen_obj = None
        launch_type = payload["launch_type"]

        # package = payload["package"]
        # launch_file = payload["launch_file"]
        # args = payload["args"]
        # executable = payload["executable"]

        if launch_type == "roslaunch":
            # 4 cases:
            # all roslaunch fields != ""
            # args == ""
            # package == ""
            # args and package == ""
            package = payload["package"]
            launch_file = payload["launch_file"]
            args = payload["args"]

            cmd = [launch_type]
            if package is not None:
                cmd.append(package)
            cmd.append(launch_file) # must append launch file
            if args is not None:
                cmd.append(args)
            
            if launch_file is None
                self.pub_status(task_id, "not_running", "Malformed roslaunch")
            else:
                popen_obj = subprocess.Popen(cmd)


            # if package is not None and args is not None:
            #     popen_obj = subprocess.Popen([launch_type, package, launch_file, args])
            # elif package is not None and args is None:
            #     popen_obj = subprocess.Popen([launch_type, package, launch_file])
            # # IF there's no package given
            # # is is assumed that the launch file will give the full path??
            # elif package is None and args is not None:
            #     popen_obj = subprocess.Popen([launch_type, launch_file, args])
            # elif package is None and args is None:
            #     popen_obj = subprocess.Popen([launch_type, launch_file])
            # else:
            #     self.pub_status(task_id, "not_running", "Malformed roslaunch")
            #     #raise Exception("Malformed roslaunch")

        elif launch_type == "rosrun":
            package = payload["package"]
            executable = payload["executable"]       
            if package is None or executable is None:
                self.pub_status(task_id, "not_running", "Malformed rosrun")
                #raise Exception("Malformed rosrun")
            else:
                popen_obj = subprocess.Popen([launch_type, package, executable])
        
        elif launch_type == "executable":
            args = payload["args"]
            executable = payload["executable"]
            
            if executable is None:
                self.pub_status(task_id, "not_running", "Missing executable")
                #raise Exception("No executable found")
            else:    
                # assumption that executable is the /full/path/to/file.sh
                if args == "":
                    popen_obj = subprocess.Popen([executable])
                else:
                    popen_obj = subprocess.Popen([executable, args])             
        # else:
        #     self.pub_status(task_id, "not_running", "Unrecognized launch type")
        #     #raise Exception("Unrecognized launch type")

        with self.lock:
            self.running_tasks[task_id] = popen_obj
    
        self.pub_status(task_id, "running")


        


    def __process_cancel_request(self, task_id):
        t = threading.Thread(target=self.__cancel_thread, args=(task_id))
        # register cancel thread
        with self.lock:
            if task_id not in self.running_cancel_threads:
                self.running_cancel_threads[task_id] = t
            else: 
                # TODO: error handling?
                pass
        t.start()            

    
    def __process_cancel_all_request(self):
        # loop through self.running_tasks and self.__process_cancel_request(task_id)
        for task_id in self.running_tasks:
            self.__process_cancel_request(task_id)


    def __process_poll_request(self, task_id):
        with self.lock:
            # get status from self.running_tasks
            popen_obj = self.running_tasks.get(task_id)
            popen_obj.poll()
       
        if popen_obj.poll() is None:
            self.pub_status(task_id, "running")
        elif popen_obj.poll() == 0:
            self.pub_status(task_id, "not_running", "Task finished successfully")
        else: # other RETURNCODE
            self.pub_status(task_id, "not_running", "Task failed")
        

    def __process_poll_all_request(self):
        # loop through self.running_tasks and self.__process_poll_request(task_id)
        for task_id in self.running_tasks:
            self.__process_poll_request(task_id)


    ### utility functions

    def __cancel_thread(self, task_id):
        # terminate #popen_obj and wait for the process to terminate cleanly
        popen_obj = self.running_tasks.get(task_id)
        popen_obj.terminate()
        popen_obj.wait()   # wait/block until terminate
        
        # unregister task from running tasks
        with self.lock:
            #self.running_tasks.pop(task_id , None)
            self.running_tasks.pop(task_id)
        
        self.pub_status(task_id, "not_running", "Task cancelled")
        
        sys.exit()   # exit thread



if __name__ == "__main__":
    rospy.init_node('aux_task_manager_node')
    
    aux_task_manager = AuxTaskManager()
    aux_task_manager.monitor_loop()

    rospy.spin()