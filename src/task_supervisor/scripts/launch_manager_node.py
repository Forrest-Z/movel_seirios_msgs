#!/usr/bin/env python

import roslaunch, sys, rospy, rosnode, rosgraph
from optparse import OptionParser
from roslaunch import rlutil
from movel_seirios_msgs.srv import StartLaunch, StartLaunchResponse, StopLaunch, StopLaunchResponse
from movel_seirios_msgs.srv import LaunchExists, LaunchExistsResponse
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String
import signal
import os

#Override option parser to raise exception
class ModifiedParser(OptionParser):
    def error(self, msg):
        raise Exception("Argument format incorrect")
    
class LaunchManager():

    def __init__(self):
        self.launch_dict = {}            #dictionary of launch index and launch var
        self.argv_dict = {}              #dicitionar of args that will be used when starting launch
        self.config_dict = {}            #dictionary of roslaunch config for node info
        self.status_dict = {}            #dictionary of ready status of launched nodes
        self.nodes_dict = {}             #dictionary of nodes that supposed to be active / launched
        self.unlaunched_list = []        #list of unlaunched index
        self.nodes = []                  #list of nodes in last launch called
        self.exists_check = False
        self.exists_id = 0
        self.exists = False
        self.avail_id_list = []
        self.shutdown_flag = False
        self.ping_block = False

        #Initialize all values for launch_ids
        #8 bit index, 0 reserved for null
        for i in range(1,256):
            self.avail_id_list.append(i)
        
        # rospy.on_shutdown(self.signal_handler)
        
        rospy.loginfo("[Launch Manager] Launch Manager initializing")
        self.launch_manager()

    #Callback function for start launch
    def start_launch(self,req):
        self.ping_block = True #Block ping while launching
        #Check if any available id's remaining
        if len(self.avail_id_list) == 0:
            rospy.logerr("[Launch Manager] Unable to launch, all unique id's used")
            return StartLaunchResponse(0)

        #Get index for launch id
        index = self.avail_id_list.pop()

        #Create launch var, append args if exist
        args = ["", req.package, req.launch_file]

        if req.args:
            args_split = req.args.split()
            for i in args_split:
                args.append(i)

        argv = args

        #Create parser object and then change it to modified parser
        #Modified parser raises exception if arguments are invalid
        parser = roslaunch._get_optparse()
        parser.__class__ = ModifiedParser


        #Catch if launch file or package does not exist
        t_start = rospy.Time.now()
        try:
            (options, args) = parser.parse_args(args[1:])

            #Get full path of package and launch file
            
            stream = os.popen('rospack find '+ req.package)
            args=[str((stream.read()).strip() + '/launch/' + req.launch_file)]
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

            #Check if args are valid
            roslaunch._validate_args(parser,options,args)

            argstring = ' '.join(args)
            rospy.loginfo("[Launch Manager] attempting launch with args: %s", argstring)
            self.launch_dict[index] = roslaunch.parent.ROSLaunchParent(uuid, args)   #add launch to dictionary along with index
            self.argv_dict[index] = argv
            config = roslaunch.config.load_config_default(args, 0)
            self.config_dict[index] = config
            self.status_dict[index] = False

        #Return index of 0 if no such such launch file self.exists
        except roslaunch.core.RLException as e:
            rospy.logerr(str(e))
            return StartLaunchResponse(0)

        except Exception as e:
            rospy.logerr(str(e))
            return StartLaunchResponse(0)

        self.unlaunched_list.append(index)                                   #add index to unlaunched list for main loop to launch
        loop_count = 0
        while len(self.unlaunched_list) != 0:                                #block until launched in main loop
            loop_count += 1
            rospy.sleep(0.033)
            pass
        dt = (rospy.Time.now() - t_start).to_sec()
        rospy.loginfo("[Launch Manager] start_launch %s with launch_id %d complete. It took %d loops and %f s", req.launch_file, index ,loop_count, dt)
        launched_nodes = roslaunch.node_args.get_node_list(self.config_dict.get(index))
        launched_nodes = [str(node) for node in launched_nodes]
        for node in launched_nodes:
            self.nodes_dict[node] = True
        self.ping_block = False #Unblock ping after launching
        return StartLaunchResponse(index)

    #Callback function for stop launch
    def stop_launch(self,req):
        self.ping_block = True #Block ping while stopping
        rospy.loginfo("[Launch Manager] stopping launch %d", req.launch_id)
        exceed_timeout = False

        #Clear any dead launches
        self.launch_exists(req)

        #Check if launch_id self.exists before killing
        if req.launch_id in self.launch_dict:

            lnch = self.launch_dict.get(req.launch_id)       #Get launch item from dictionary
            lnch.shutdown()                             #Shutdown the launch
            shutdown_start_time = rospy.Time.now().to_sec()

            nodes_to_be_stopped = roslaunch.node_args.get_node_list(self.config_dict.get(req.launch_id))
            nodes_to_be_stopped = [str(node) for node in nodes_to_be_stopped]

            for node in nodes_to_be_stopped:
                while rosnode.rosnode_ping(node, 1, False) and not exceed_timeout: #Wait for node to shutdown
                    if (rospy.Time.now().to_sec() - shutdown_start_time) > self.stop_launch_timeout: #Check if timeout exceeded
                        exceed_timeout = True
                        rospy.logwarn("[launch manager] Stop launch exceeded timeout")
                        break
                del self.nodes_dict[node] # Delete inactive node from dictionary
            rosnode.cleanup_master_blacklist(rosgraph.Master(nodes_to_be_stopped[0]), nodes_to_be_stopped)

            self.launch_dict.pop(req.launch_id)              #Remove the launch from dictionary
            self.avail_id_list.append(req.launch_id)         #Add launch id back to avail list
            return StopLaunchResponse(True)

        rospy.logwarn("[Launch Manager] Unable to stop, index does not correspond to a launch")
        self.ping_block = False #Unblock ping after stopping
        return StopLaunchResponse(False)

    #Callback function for checking if launch self.exists
    def launch_exists(self,req):
        rospy.loginfo("[Launch Manager] performing exist check %d", req.launch_id)
        
        # Prevent multiple self.exists checks
        while self.exists_check:
            pass

        response = LaunchExistsResponse()

        #Check if launch_id is in dictionary
        if req.launch_id in self.launch_dict:
            self.exists_id = req.launch_id
            self.exists_check = True

            #Wait for check to be done on main thread
            if not self.exists_id in self.launch_dict.keys() or\
                self.launch_dict[self.exists_id].runner is None:
                self.exists_check = False
            t_check_start = rospy.Time.now()
            while self.exists_check:
                if self.launch_dict[self.exists_id].runner is None:
                    self.exists_check = False
                dt = (rospy.Time.now() - t_check_start).to_sec()
                if (dt > 10):
                    self.exists = False
                    break
                rospy.sleep(0.066)
                pass
            rospy.loginfo("[Launch Manager] exist check done")

            if self.exists:
                self.exists = False                          #Reset flag
                response.exists = True
                return response

            else:
                #Remove launch from list of launch dict
                if (self.config_dict.get(req.launch_id) is not None):
                    nodes_to_be_removed = roslaunch.node_args.get_node_list(self.config_dict.get(req.launch_id))
                    nodes_to_be_removed = [str(node) for node in nodes_to_be_removed]
                    for node in nodes_to_be_removed:
                        if node in self.nodes_dict:
                            del self.nodes_dict[node]
                    rosnode.cleanup_master_blacklist(rosgraph.Master(nodes_to_be_removed[0]), nodes_to_be_removed)
                self.config_dict.pop(req.launch_id)
                self.launch_dict.pop(req.launch_id)
                self.avail_id_list.append(req.launch_id)
                response.exists = False
                rospy.logwarn("[Launch Manager] Clear launch_id %s from list", req.launch_id)
                return response

        else:
            response.exists = False
            rospy.logerr("[Launch Manager] launch_id %s does not correspond to a launch", req.launch_id)
            return response


    #Check launch status
    def launch_status(self,req):        
        response = LaunchExistsResponse()
        if req.launch_id in self.launch_dict and self.status_dict.get(req.launch_id):
            ready = True
            nodes_to_be_checked = roslaunch.node_args.get_node_list(self.config_dict.get(req.launch_id))
            nodes_to_be_checked = [str(node) for node in nodes_to_be_checked]
            node_names = ""
            for node in nodes_to_be_checked:
                if not rosnode.rosnode_ping(node, 1, False):
                    ready = False
                    node_names = node_names + " " + node

            response.exists = ready
            if not ready:
                rospy.logerr("[Launch Manager] Nodes not running:%s", node_names)
                response.message = "Nodes not running:" + node_names
            return response
        else:
            response.exists = False
            rospy.logerr("[Launch Manager Status] launch_id %s does not correspond to a launch", req.launch_id)
            return response

    # Ping nodes that are supposed to be active
    def ping_routine_cb(self,event):
        try:
            for node in self.nodes_dict.keys():
                t_start_ping = rospy.Time.now()
                while (not rosnode.rosnode_ping(node, 1, False)) and (not self.shutdown_flag) and (not self.ping_block):
                    if (self.nodes_dict[node] is None):
                        break
                    rospy.loginfo_throttle(1, "[Launch Manager] Can't ping node %s", node)
                    dt = (rospy.Time.now() - t_start_ping).to_sec()
                    if dt > 1.0 and (node in self.timeoutable_nodes):
                        rospy.loginfo("[Launch Manager] %s ping timeout %5.2f", node, dt)
                        break
                    rospy.sleep(0.02)
                rospy.loginfo_once("[Launch Manager]: %s node ready", node)
                if (self.shutdown_flag) or (self.ping_block):
                    break
        except Exception as e:
            rospy.logwarn("[Launch Manager] Ping routine: %s", e)

    # This is a workaround to force shutdown launch_manager node, the message is published by task_master
    def kill_cb(self, msg):
        if (msg.data == "/launch_manager"):
            rospy.logwarn("[Launch Manager] Received kill signal from task_master")
            self.shutdown_flag = True
            rospy.signal_shutdown("Received kill signal from task_master")

    def launch_manager(self):

        self.stop_launch_timeout = rospy.get_param('~stop_launch_timeout', 0)
        self.timeoutable_nodes = rospy.get_param('~timeoutable_nodes', [])
        self.node_ping_rate = rospy.get_param('~node_ping_rate', 300000) # 300 ms

        self.sub_kill = rospy.Subscriber("/kill_node", String, self.kill_cb)

        self.start_launch_service = rospy.Service("launch_manager/start_launch", StartLaunch, self.start_launch)
        self.stop_launch_service = rospy.Service("launch_manager/stop_launch", StopLaunch, self.stop_launch)
        self.launch_exists_service = rospy.Service("launch_manager/launch_exists", LaunchExists, self.launch_exists)
        self.launch_status_service = rospy.Service("launch_manager/launch_status", LaunchExists, self.launch_status)

        self.node_ping_routine = rospy.Timer(rospy.Duration(nsecs=self.node_ping_rate), self.ping_routine_cb)
        #Loop while ros core is running, launch vars must start() in main function
        while not rospy.is_shutdown():
            try:
                #Check if there are any unlaunched launch files, if yes then launch
                if len(self.unlaunched_list) != 0:
                    for i in self.unlaunched_list:               #Run through all unlaunched items to start()
                        t_start_launch = rospy.Time.now()
                        lnch = self.launch_dict.get(i)           #Get launch corresponding to index i
                        sys.argv = self.argv_dict.get(i)         #Set roslaunch args before calling start, which refers to sys.argv
                        rospy.loginfo("[Launch Manager] Starting launch")
                        lnch.start()                        #Start launch, must be done in main function
                        while not lnch.runner.spin_once():  #Wait for launch to start
                            rospy.loginfo("[Launch Manager] waiting for launch")
                            pass
                        self.unlaunched_list.remove(i)           #Remove index from unlaunched list

                        rospy.loginfo("[Launch Manager] Launch started")
                        self.nodes = roslaunch.node_args.get_node_list(self.config_dict.get(i))
                        self.nodes = [str(node) for node in self.nodes]
                        self.status_dict[i] = True

                #Check if there are any self.exists check requests
                if self.exists_check and self.exists_id:
                    #Prevent race condition when check is issued before launch is fully started
                    rospy.loginfo("[Launch Manager] check existence of %d, %d", self.exists_id, self.exists_id in self.launch_dict.keys())
                    if not self.exists_id in self.launch_dict.keys():
                        self.exists_check = False
                    elif self.launch_dict[self.exists_id].runner is not None:
                        self.exists = self.launch_dict[self.exists_id].runner.spin_once() #Spin must be called in main function
                        self.exists_check = False
                rospy.sleep(0.033)

            except AttributeError as e:
                rospy.logwarn("[Launch Manager] exception")
                rospy.logwarn(str(e))
            except roslaunch.core.RLException as e:
                rospy.logwarn("[Launch Manager] exception")
                rospy.logwarn(str(e))
            except Exception as e:
                rospy.logwarn("[Launch Manager] exception")
                rospy.logwarn(str(e))
                pass

        #Shutdown all launch when this node shuts down
        rospy.logwarn("[Launch Manager] Shutting down all launches")
        for index in list(self.launch_dict.keys()):
            try:
                self.launch_dict[index].shutdown()
            except:
                pass

            self.launch_dict.pop(index)

def main():
    rospy.init_node("launch_manager")
    launch_manager = LaunchManager()
    rospy.spin()
    
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("[Launch Manager] KeyboardInterrupt")
        rospy.signal_shutdown("Launch Manager shutting down")
        pass