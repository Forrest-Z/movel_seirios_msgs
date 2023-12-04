#!/usr/bin/python3
import rospy
import json
import sys
import threading
import subprocess
import os, signal
import platform
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray

class AuxTaskManager:

    def __init__(self):
        self.print_name = "[aux_task_manager_node]"
        self.request_sub = rospy.Subscriber("/aux_task_manager/request", String, self.CB_request)
        self.status_pub = rospy.Publisher("/aux_task_manager/status", String, queue_size=20)
        self.feedback_pub = rospy.Publisher("/aux_task_manager/feedback", String, queue_size=20)
        self.running_tasks = {}
        self.running_cancel_threads = {}
        self.lock = threading.RLock()

        self.monitor_duration = rospy.get_param("monitor_duration", 5.0)
        self.start_time = 0

    def CB_request(self, msg):
        self.__loginfo("Receive request msg")
        d = json.loads(msg.data)
        req_type = d["RequestType"]

        # check for requestType 
        # start the relevant function based on request_type 
        #   start, cancel, cancel_all, poll, poll_all
        if req_type == "start":
            start_time = rospy.Time.now()
            self.__loginfo(f"start time, {start_time}")

            task_id = d["TaskId"]
            launch_type = d["LaunchType"]
            payload = d["Payload"] # type dict
            self.__process_start_request(task_id, launch_type, payload)
        
        elif req_type == "cancel":
            task_id = d["TaskId"]
            self.__process_cancel_request(task_id)
        
        elif req_type == "cancel_all":
            self.__process_cancel_all_request()
        
        elif req_type == "poll":
            task_id = d["TaskId"]
            self.__process_poll_request(task_id)
        
        elif req_type == "poll_all":
            self.__process_poll_all_request()
        
        else:
            err = "Unknown request type" + " " + req_type
            self.__logerror(err)
            self.pub_status(task_id, "not_running", err)
            raise Exception("Unknown request type", req_type)

    def pub_status(self, task_id, status, error_msg=None):
        # format the details into a JSON and publish
        x = {"TaskId": task_id, "Status": status, "ErrorMsg": error_msg}
        formated_msg = json.dumps(x)
        self.status_pub.publish(formated_msg)
    
    def pub_feedback(self, task_id, feedback, error_msg=None):
        # format the details into a JSON and publish
        x = {"TaskId": task_id, "\nFeedback": feedback, "\nErrorMsg": error_msg}
        formated_msg = json.dumps(x)
        self.feedback_pub.publish(formated_msg)

    def monitor_loop(self):
        d = rospy.Duration(self.monitor_duration)
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
                    self.__loginfo(f"[task status] {task_id} is alive")
                    self.pub_feedback(task_id, task_status[task_id])
                elif task_status[task_id] == "not_running":
                    self.__loginfo(f"[task status] {task_id} is not alive")
                    self.__process_cancel_request(task_id) 

            # self.running_cancel_threads status check
            with self.lock:
                # loop through self.running_cancel_threads and remove if thread is not is_alive()
                self.__clean_dead_threads()
            rospy.sleep(d)

    def __clean_dead_threads(self):
        with self.lock:
            # prevent dictionary changed size over iteration error
            dead_threads = [task_id for task_id, thread in self.running_cancel_threads.items() if not thread.is_alive()]
            for task_id in dead_threads:
                self.running_cancel_threads.pop(task_id)
                self.__loginfo(f"dead thread of {task_id} removed")

    def __del__(self):
        # clean up of self.running_tasks
        with self.lock:
            task_ids = list(self.running_tasks.keys())
            for task_id in task_ids:
                popen_obj = self.running_tasks.get(task_id)
                try:
                    os.killpg(os.getpgid(popen_obj.pid), signal.SIGTERM)
                    popen_obj.wait()   # wait/block until terminate
                    self.running_tasks.pop(task_id)

                    print(f"{self.print_name} __del__(): removed task {task_id}")
                except ProcessLookupError:
                    pass
                
        print(f"{self.print_name} __del__(): all tasks removed")

    ### process request functions
    def __process_start_request(self, task_id, launch_type, payload):
        # start popen process based on roslaunch, rosrun, or executable
        # TODO: how to implement start timeout?
        # running_tasks - keys: task_id, values: popen_object
        if task_id in self.running_tasks:
            err = f"Duplicate task_id: {task_id}"
            self.__logerror(err)
            self.pub_status(task_id, "not_running", err)
            raise Exception(err) 
        # start new aux task
        #launch_type = payload["launch_type"]

        while task_id not in self.running_tasks:
            running_time = rospy.Time.now()
            self.__loginfo(f"running time, {running_time}")

            # timeout = payload["Timeout"]
            timeout = 0.1

            if launch_type == "roslaunch":
                # 4 cases:
                # all roslaunch fields != ""
                # args == ""
                # package == ""
                # args and package == ""            
                package = payload["Pkg"]
                launch_file = payload["Launchfile"]
                args = payload["Args"]
                # sanity check
                if launch_file is None:
                    err = "Malformed roslaunch"
                    self.__logerror(err)
                    self.pub_status(task_id, "not_running", err)
                    raise Exception(err) 
                # build cmd 
                cmd = [launch_type]
                if package is not None:
                    cmd.append(package)
                cmd.append(launch_file) # must append launch file
                if args is not None:
                    cmd.append(args)
                cmd = " ".join(cmd)
                popen_obj = subprocess.Popen(cmd, shell=True, start_new_session=True)
            
            elif launch_type == "rosrun":
                package = payload["Pkg"]
                executable = payload["Executable"] 
                # sanity check      
                if package is None or executable is None:
                    err = "Malformed rosrun"
                    self.__logerror(err)
                    self.pub_status(task_id, "not_running", err)
                    raise Exception(err)         
                cmd = " ".join([launch_type, package, executable])
                popen_obj = subprocess.Popen(cmd, shell=True, start_new_session=True)
            
            elif launch_type == "executable":
                args = payload["Args"]
                executable = payload["Executable"]
                # sanity check
                if executable is None:
                    err = "Missing executable"
                    self.__logerror(err)
                    self.pub_status(task_id, "not_running", err)
                    raise Exception(err) 
                # assumption that executable is the /full/path/to/file.sh
                cmd = [executable]
                if args is not None:
                    cmd.extend(args)
                cmd = " ".join(cmd) # needed a string, but gave a list
                popen_obj = subprocess.Popen(cmd, shell=True, start_new_session=True)
        
            # register task
            with self.lock:
                self.running_tasks[task_id] = popen_obj
                break
            
            dt = running_time - self.start_time
            self.__loginfo(dt)
            if dt.to_sec() > timeout:
                self.__process_cancel_request(task_id)
                break
        
        # status
        self.__loginfo(f"task with {task_id} is running")
        self.pub_status(task_id, "running")

    def __process_cancel_request(self, task_id):
        self.__loginfo(f"processing cancel request for task {task_id}")
        if task_id not in self.running_tasks:
            err = f"Task with {task_id} not found"
            self.__logerror(err)
            self.pub_status(task_id, "not_running", "Task id not found")   
            raise Exception(err)
        # start a thread to cancel, then,
        # register the thread
        t = threading.Thread(target=self.__cancel_thread, args=(task_id,))
        with self.lock:
            if task_id not in self.running_cancel_threads:
                self.running_cancel_threads[task_id] = t
            else: 
                # TODO: error handling?
                pass
        t.start()            

    def __process_cancel_all_request(self):
        if not bool(self.running_tasks):
            err = f"Nothing to cancel"
            self.__logerror(err)
            raise Exception(err)
        with self.lock:
            for task_id in self.running_tasks:   
                self.__process_cancel_request(task_id)
        
    def __process_poll_request(self, task_id):
        if task_id not in self.running_tasks:
            err = f"[Poll] Task with {task_id} not found"
            self.__logerror(err)
            self.pub_status(task_id, "not_running", err)   
            raise Exception(err)    

        with self.lock:
            # get status from self.running_tasks
            popen_obj = self.running_tasks.get(task_id)
        if popen_obj is None:
            err = f"{task_id} not found"
            self.__logerror(err)
            self.pub_status(task_id, "not_running", err)                
            raise Exception(err)   # early exit
        
        poll_status = popen_obj.poll()
        if poll_status is None:
            self.__loginfo(f"{task_id} is running")
            self.pub_status(task_id, "running")
        elif poll_status == 0:
            self.__loginfo(f"{task_id} finished successfully")
            self.pub_status(task_id, "not_running", "Task finished successfully")
        else: # other RETURNCODE
            self.__logerror(f"{task_id} failed")
            self.pub_status(task_id, "not_running", "Task failed")
        
    def __process_poll_all_request(self):
        if not bool(self.running_tasks):
            err = f"Nothing to poll"
            self.__logerror(err)
            raise Exception(err)
        with self.lock:
            tasks_ids = self.running_tasks.keys()
        for task_id in tasks_ids:
            self.__process_poll_request(task_id)

    ### utility functions
    def __cancel_thread(self, task_id):
        self.__loginfo(f"canceling task {task_id}")
        # terminate #popen_obj and wait for the process to terminate cleanly
        popen_obj = self.running_tasks.get(task_id)
        try:
            os.killpg(os.getpgid(popen_obj.pid), signal.SIGTERM)
            popen_obj.wait()   # wait/block until terminate
        except ProcessLookupError:
            self.__logerror(f"No process associated with task {task_id}. Nothing to cancel.")
            self.pub_status(task_id, "not_running", "[CANCEL] Nothing to cancel")
        # unregister task from running tasks
        with self.lock:
            self.__loginfo(f"task {task_id} removed from running tasks")
            self.running_tasks.pop(task_id)
        self.pub_status(task_id, "not_running", "Task cancelled")
        sys.exit()   # exit thread

    ## logging functions
    def __loginfo(self, msg):
        rospy.loginfo(f"{self.print_name} {msg}")

    def __logerror(self, msg):
        rospy.logerr(f"{self.print_name} {msg}")


if __name__ == "__main__":
    rospy.init_node('aux_task_manager_node')
    
    aux_task_manager = AuxTaskManager()
    aux_task_manager.monitor_loop()
    rospy.spin()