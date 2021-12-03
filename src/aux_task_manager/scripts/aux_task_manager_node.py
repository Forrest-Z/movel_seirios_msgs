#!/usr/bin/python3

import rospy
import json
import sys
import threading

from std_msgs.msg import String


class AuxTaskManager:

    def __init__(self):
        self.request_sub = rospy.Subscriber("/aux_task_manager/request", String, self.CB_request)
        self.status_pub = rospy.Publisher("/aux_task_manager/status", String, queue_size=20)
        self.running_tasks = {}
        self.running_cancel_threads = {}
        self.lock = threading.RLock()


    def CB_request(self, msg):
        # TODO: check for request type 
        # TODO: start the relevant function based on request type 
        #       start, cancel, cancel all, poll, poll all
        pass


    def pub_status(self, task_id, status, error_msg=None):
        # TODO: format the details into a JSON and publish
        # self.status_pub.publish( #formated_msg )
        pass


    def monitor_loop(self):
        d = rospy.Duration(5.0)
        while not rospy.is_shutdown():
            
            # TODO: self.running_tasks status check
            # task_status = {}
            with self.lock:
                # loop through self.running_tasks track status in task_status
                pass
            # loop through task_status
                # if #running:
                    # self.pub_status()
                # else:   #(not running)
                    # self.__process_cancel_request(task_id)
            
            # TODO: self.running_cancel_threads status check
            with self.lock:
                # loop through self.running_cancel_threads and remove if thread is not is_alive()
                pass
                    
            rospy.sleep(d)


    def __del__(self):
        # TODO: clean up of self.running_tasks
        # TODO: waiting for theads in self.running_cancel_threads to join before exiting main 
        pass


    ### process request functions

    def __process_start_request(self, task_id, payload):
        # TODO: start popen process based on roslaunch, rosrun, or executable
        #       format cmd args based on payload
        
        # TODO: register task to self.running_tasks
        # with self.lock:
        #     self.running_tasks[ #task_id ] = #popen_obj

        # TODO: self.pub_status()

        # TODO: how to implement start timeout?
        # TODO: how to handle start if task_id already exists?

        pass


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
        # TODO: loop through self.running_tasks and self.__process_cancel_request(task_id)
        pass


    def __process_poll_request(self, task_id):
        with self.lock:
            # TODO: get status from self.running_tasks
            pass
        # TODO: self.pub_status()


    def __process_poll_all_request(self):
        # TODO: loop through self.running_tasks and self.__process_poll_request(task_id)
        pass


    ### utility functions

    def __cancel_thread(self, task_id):
        # TODO: terminate #popen_obj and wait for the process to terminate cleanly
        # popen_obj.terminate()
        # popen_obj.wait()   # wait/block until terminate
        
        # TODO: unregister task from running tasks
        # with self.lock:
        #     self.running_tasks.pop( #task_id , None)
        # TODO: self.pub_status()
        
        sys.exit()   # exit thread



if __name__ == "__main__":
    rospy.init_node('aux_task_manager_node')
    
    aux_task_manager = AuxTaskManager()
    aux_task_manager.monitor_loop()

    rospy.spin()