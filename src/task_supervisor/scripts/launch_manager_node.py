#!/usr/bin/env python

import roslaunch, sys, rospy, rosnode, rosgraph
from optparse import OptionParser
from roslaunch import rlutil
from movel_seirios_msgs.srv import StartLaunch, StartLaunchResponse, StopLaunch, StopLaunchResponse
from movel_seirios_msgs.srv import LaunchExists, LaunchExistsResponse
from std_srvs.srv import Trigger, TriggerResponse
import tf2_ros
import os

launch_dict = {}            #dictionary of launch index and launch var
argv_dict = {}              #dicitionar of args that will be used when starting launch
config_dict = {}            #dictionary of roslaunch config for node info
status_dict = {}            #dictionary of ready status of launched nodes
unlaunched_list = []        #list of unlaunched index
exists_check = False
exists_id = 0
exists = False

avail_id_list = []

#Override option parser to raise exception
class ModifiedParser(OptionParser):
    def error(self, msg):
        raise Exception("Argument format incorrect")

#Callback function for start launch
def start_launch(req):
    #Check if any available id's remaining
    if len(avail_id_list) == 0:
        rospy.logerr("[Launch Manager] Unable to launch, all unique id's used")
        return StartLaunchResponse(0)

    #Get index for launch id
    index = avail_id_list.pop()

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
        #args = rlutil.resolve_launch_arguments(args)
        
        stream = os.popen('rospack find '+ req.package)
        args=[str((stream.read()).strip() + '/launch/' + req.launch_file)]
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

        #Check if args are valid
        roslaunch._validate_args(parser,options,args)

        argstring = ' '.join(args)
        rospy.loginfo("[Launch Manager] attempting launch with args: %s", argstring)
        launch_dict[index] = roslaunch.parent.ROSLaunchParent(uuid, args)   #add launch to dictionary along with index
        argv_dict[index] = argv
        config = roslaunch.config.load_config_default(args, 0)
        config_dict[index] = config
        status_dict[index] = False

    #Return index of 0 if no such such launch file exists
    except roslaunch.core.RLException as e:
        rospy.logerr(str(e))
        return StartLaunchResponse(0)

    except Exception as e:
        rospy.logerr(str(e))
        return StartLaunchResponse(0)

    unlaunched_list.append(index)                                   #add index to unlaunched list for main loop to launch
    loop_count = 0
    while len(unlaunched_list) != 0:                                #block until launched in main loop
        loop_count += 1
        rospy.sleep(0.033)
        pass
    dt = (rospy.Time.now() - t_start).to_sec()
    rospy.loginfo("[Launch Manager] start_launch complete. It took %d loops and %f s", loop_count, dt)
    return StartLaunchResponse(index)

#Callback function for stop launch
def stop_launch(req):
    rospy.loginfo("[Launch Manager] stopping launch %d", req.launch_id)
    exceed_timeout = False

    #Clear any dead launches
    launch_exists(req)

    #Check if launch_id exists before killing

    if req.launch_id in launch_dict:

        lnch = launch_dict.get(req.launch_id)       #Get launch item from dictionary
        lnch.shutdown()                             #Shutdown the launch
        shutdown_start_time = rospy.Time.now().to_sec()

        nodes = roslaunch.node_args.get_node_list(config_dict.get(req.launch_id))
        nodes = [str(node) for node in nodes]

        for node in nodes:
            while rosnode.rosnode_ping(node, 1, False) and not exceed_timeout:
                if (rospy.Time.now().to_sec() - shutdown_start_time) > stop_launch_timeout:
                    exceed_timeout = True
                    rospy.logwarn("[launch manager] Stop launch exceeded timeout")
                    break
        rosnode.cleanup_master_blacklist(rosgraph.Master(nodes[0]), nodes)

        launch_dict.pop(req.launch_id)              #Remove the launch from dictionary
        avail_id_list.append(req.launch_id)         #Add launch id back to avail list
        return StopLaunchResponse(True)

    rospy.logwarn("[Launch Manager] Unable to stop, index does not correspond to a launch")
    return StopLaunchResponse(False)

#Callback function for checking if launch exists
def launch_exists(req):
    rospy.loginfo("[Launch Manager] performing exist check %d", req.launch_id)
    global exists_id, exists_check, exists, launch_dict
    
    #Prevent multiple exists checks
    # rospy.loginfo("[Launch Manager] clearing old exist checks")
    while exists_check:
        pass
    # rospy.loginfo("[Launch Manager] old exist check done")

    response = LaunchExistsResponse()

    #Check if launch_id is in dictionary
    if req.launch_id in launch_dict:
        exists_id = req.launch_id
        exists_check = True

        #Wait for check to be done on main thread
        # rospy.loginfo("[Launch Manager] clearing new exist checks %d", exists_id)
        # print(launch_dict[exists_id].runner)
        if not exists_id in launch_dict.keys() or\
            launch_dict[exists_id].runner is None:
            # rospy.loginfo("[Launch Manager] it's already out")
            exists_check = False
        t_check_start = rospy.Time.now()
        while exists_check:
            # rospy.loginfo("exists_check loop")
            # print(launch_dict[exists_id].runner)
            if launch_dict[exists_id].runner is None:
                exists_check = False
            dt = (rospy.Time.now() - t_check_start).to_sec()
            # print("in this loop for ", dt, "seconds")
            if (dt > 10.):
                # rospy.loginfo("[Launch Manager] exist check loop timeout")
                exists = False
                break
            rospy.sleep(0.066)
            pass
        rospy.loginfo("[Launch Manager] exist check done")

        if exists:
            exists = False                          #Reset flag
            response.exists = True
            return response

        else:
            #Remove launch from list of launch dict
            launch_dict.pop(req.launch_id)
            avail_id_list.append(req.launch_id)
            response.exists = False
            rospy.logwarn("[Launch Manager] Clear launch_id %s from list", req.launch_id)
            return response

    else:
        response.exists = False
        rospy.logerr("[Launch Manager] launch_id %s does not correspond to a launch", req.launch_id)
        return response

def launch_status(req):
    response = LaunchExistsResponse()
    if req.launch_id in launch_dict and status_dict.get(req.launch_id):
        ready = True
        nodes = roslaunch.node_args.get_node_list(config_dict.get(req.launch_id))
        nodes = [str(node) for node in nodes]
        node_names = ""
        for node in nodes:
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
        rospy.logerr("[Launch Manager] launch_id %s does not correspond to a launch", req.launch_id)
        return response


def tf_buffer_clear(req):
    if req.data:
        rospy.logwarn("[Launch Manager] Clearing TF BUffer")
        tf2_buffer_.clear()
    return TriggerResponse(success=True, message="[Launch Manager] Clearing TF BUffer")

def launch_check_routine_cb():
    nodes = roslaunch.node_args.get_node_list(config_dict.get(i))
    for node in nodes:
        t_start_ping = rospy.Time.now()
        while not rosnode.rosnode_ping(node, 1, False):
            rospy.loginfo("[Launch Manager] ping node %s", node)
            dt = (rospy.Time.now() - t_start_ping).to_sec()
            # TODO, read timeout-able nodes from a list of transients
            if dt > 1.0 and "map_saver" in node:
                rospy.loginfo("map saver ping timeout %5.2f", dt)
                break
            rospy.sleep(0.02)
        rospy.loginfo("[launch manager]: %s node ready", node)
    status_dict[i] = True


#Main loop of function
def launch_manager():
    global exists, exists_check, stop_launch_timeout

    #Start launch and advertise services to start and stop launch files
    rospy.init_node('launch_manager')

    stop_launch_timeout = rospy.get_param('~stop_launch_timeout')

    start_launch_service = rospy.Service("launch_manager/start_launch", StartLaunch, start_launch)
    stop_launch_service = rospy.Service("launch_manager/stop_launch", StopLaunch, stop_launch)
    launch_exists_service = rospy.Service("launch_manager/launch_exists", LaunchExists, launch_exists)
    launch_status_service = rospy.Service("launch_manager/launch_status", LaunchExists, launch_status)
    buffer_clear_service = rospy.Service('launch_manager/tf_buffer_clear', Trigger, tf_buffer_clear)

    launch_check_rate = 300000 # 300 ms
    launch_check_routine = rospy.Timer(rospy.Duration(nsecs=launch_check_rate), launch_check_routine_cb)

    #Loop while ros core is running, launch vars must start() in main function
    while not rospy.is_shutdown():
        # rospy.loginfo("[Launch Manager] loop, %d, %d", len(unlaunched_list), len(launch_dict.keys()))
        try:
            #Check if there are any unlaunched launch files, if yes then launch
            if len(unlaunched_list) != 0:
                for i in unlaunched_list:               #Run through all unlaunched items to start()
                    t_start_launch = rospy.Time.now()
                    lnch = launch_dict.get(i)           #Get launch corresponding to index i
                    sys.argv = argv_dict.get(i)         #Set roslaunch args before calling start, which refers to sys.argv
                    rospy.loginfo("[launch_manager] Starting launch")
                    lnch.start()                        #Start launch, must be done in main function
                    while not lnch.runner.spin_once():  #Wait for launch to start
                        rospy.loginfo("[launch manager] waiting for launch")
                        pass
                    unlaunched_list.remove(i)           #Remove index from unlaunched list

                    rospy.loginfo("[launch manager] Launch started")
                    nodes = roslaunch.node_args.get_node_list(config_dict.get(i))
                    nodes = [str(node) for node in nodes]

            #Check if there are any exists check requests
            if exists_check and exists_id:
                #Prevent race condition when check is issued before launch is fully started
                rospy.loginfo("[Launch Manager] check existence of %d, %d", exists_id, exists_id in launch_dict.keys())
                if not exists_id in launch_dict.keys():
                    exists_check = False
                elif launch_dict[exists_id].runner is not None:
                    exists = launch_dict[exists_id].runner.spin_once() #Spin must be called in main function
                    exists_check = False

            rospy.sleep(0.033)

        except AttributeError as e:
            rospy.loginfo("[Launch Manager] exception")
            rospy.loginfo(str(e))
        except roslaunch.core.RLException as e:
            rospy.loginfo("[Launch Manager] exception")
            rospy.loginfo(str(e))
        except Exception as e:
            rospy.loginfo("[Launch Manager] exception")
            rospy.loginfo(str(e))
            pass

    #Shutdown all launch when this node shuts down
    rospy.loginfo("[Launch Manager] Shutting down all launches")
    for index in list(launch_dict.keys()):
        try:
            launch_dict[index].shutdown()
        except:
            pass

        launch_dict.pop(index)

if __name__ == "__main__":
    #Initialize all values for launch_ids
    #8 bit index, 0 reserved for null
    for i in range(1,256):
        avail_id_list.append(i)

    try:
        launch_manager()
    except rospy.ROSInterruptException:
        pass
