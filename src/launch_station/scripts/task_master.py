import rospy, rosnode, roslaunch, rosgraph
import asyncio
from roslaunch import roslaunch_logs, rlutil, pmon
from std_msgs.msg import String
import subprocess
import time

try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy

class TaskROSter:
    """
    TaskROSter provides the ROS interface to execute ROS nodes. Basic
    functionality includes providing a healthcheck, becoming a ROS node.
    Secondary functionality includes parsing custom messages to launch the
    following: ROS launch files, ROS nodes.
    """
    def __init__(self):
        """
        ain't it init
        """
        self.connected = False
        self.running = {}
        print("TM init")

    def pub_connections(self, connections):
        try:
            self.pub.publish(connections)
        except Exception as e:
            print (e)

    async def init_node(self):
        """
        Initializes as a ROS Node with the sole purposes of broadcasting the MC
        services (including itself) that are available. They can then be used
        to determine secondary decision making such as runtime profiles.
        """
        # TODO: Implement respawn option
        # TODO: Run without async support for timeout
        print ("initing node")
        rospy.init_node(self.node_name)
        self.pub = rospy.Publisher(self.ros_topic, String, queue_size=1)
        self.connected = True
        print("tm connected check 1: ",self.connected)


    async def healthCheck(self, hb_interval):
        """
        Updates the connection status of TaskMaster periodically.

        @param hb_interval: Interval per heartbeat loop. Not the same as a timeout.
        Strict attempts to enforce hb_interval by enforcing timeouts in processes
        and reducing sleep time with time taken.
        """
        print ("tm healthcheck")
        try:
            if self.init_timeout > hb_interval:
                raise AssertionError("Connection timeout cannot be longer than heartbeat interval")
        except AssertionError as ae:
            # TODO: do cleanup
            print (ae)
            sys.exit()

        while True:
            stopwatch_start = time.perf_counter()
            try:
                # Checks existance for ROSMaster.
                param_server = rosgraph.Master('/roslaunch')
                self.run_id = param_server.getParam('/run_id')
                print ("run_id: ", self.run_id)
                if not self.connected:
                    #await to_thread(self.init_node)
                    try:
                        print("tm connected check 2: ",self.connected)
                        await asyncio.wait_for(self.init_node(), self.init_timeout)
                    except TimeoutError as te:
                        print (te)
                        pass
                    print ("Node init")
                print ("connected = true")
                stopwatch_stop = time.perf_counter()
            except ConnectionRefusedError as cre:
                self.connected = False
                print ("no roscore")
                print (cre)
                stopwatch_stop = time.perf_counter()
            await asyncio.sleep(hb_interval - (stopwatch_stop - stopwatch_start))

    def kill(self, pid, timeout):
        print ("Killing")
        if pid in self.running_pid:
            print ("kill")
        return success, msg

    def launch(self, parent):
        try:
            parent.start()
        except Exception as e:
            print ("parent start")
            print (e)

        try:
            parent.spin()
        except Exception as e:
            print ("parent spin")
            print (e)

    def start_launchfile(self, launchfile, timeout, pkg, launch_args):
        print ("\nGOT LAUNCHFILE")
        # Find Absolute path to launch file, arguments are of non-zero index.
        if pkg is not None:
            rl_obj = rlutil.resolve_launch_arguments([pkg, launchfile])
        else:
            rl_obj = rlutil.resolve_launch_arguments([launchfile])

        print(rl_obj)
        # Extract out filename from absolute path to use as Name Identifier
        if '/' in rl_obj[0]:
            launch_file = rl_obj[0][rl_obj[0].rfind('/')+1:]
        else:
            launch_file = rl_obj[0]

        name = launch_file
        # TODO: revisit
        p = roslaunch.parent.ROSLaunchParent(self.run_id, rl_obj,
            is_core=False,
            port=launch_args['Port'],
            timeout=timeout,
            #TODO: Support for all roslaunch_args
            master_logger_level=launch_args['Loglevel'],
            sigint_timeout=timeout,
            sigterm_timeout=timeout)

        try:
            print ("start_launch")
            p.start()
            print ("p has started")
            # Not sure if required when inside an async loop
            # Returned None when I tried.
            #launch_success = p.spin_once()


            # TODO: To place at another location.
            self.master = rosgraph.Master('/rosnode', master_uri="http://localhost:11311")
            print ('URI: ', p.runner.server_uri)
            pid_list = []
            for node in p.runner.config.nodes:
                node_name = "/" + node.name
                print (self.master.getSystemState())
                api = rosnode.get_api_uri(self.master, node_name)
                print ("API: ", api)
                node = ServerProxy(api)
                pid = rosnode._succeed(node.getPid('/rosnode'))
                print ("PID: ", pid)
                pid_list.append(pid)
                print ("Appended PID")
                msg = ""
            print ("Adding to dict: ", name)
            self.running[str(name)] = pid_list
            name = launch_file + "_" + str(pid_list[0])
            msg = "No news is good news"
            launch_success = True
            print ("Success: ", launch_success)
        except Exception as e:
            print (e)
            msg = e
            launch_success = False
        #finally:
        #    p._stop_infrastructure()

        return name, launch_success, msg

    def start_ros_node(self, pkg, executable, params, launch_args, timeout):
        print ("start_ros_node")
        # Support for launch_args
        node = roslaunch.core.Node(pkg, executable)
        config = roslaunch.ROSLaunchConfig()
        config.nodes.append(node)

        name = executable

        print (params)
        for key in params:
            key = roslaunch.core.Param(key, params[key])
            config.add_param(key)

        launch_runner = roslaunch.launch.ROSLaunchRunner(self.run_id, config=config)
        #success, failure = launch_runner.launch()
        #if success is not None:
        #    launch_success = True
        #    name = executable + "_" +
        #    msg = success
        #else if failure is not None:
        #    launch_success = False
        #    name = executable
        #    msg = failure

        try:
            p, launch_success = launch_runner.launch_node(node)
            print ("Process: ", p)
            print ("PID: ", p.pid)

            print ("Launch success: ", launch_success)
            if launch_success:
                name = name + "_" + str(p.pid)
            print ("Name: ", name)
            msg = "No news is good news"
            self.running[str(name)] = p.pid
        except Exception as e:
            print (e)
            msg = e

        return name, launch_success, msg

    def start_executable(self, executable, args, timeout):
        # TODO: BREAKUP TO SYSTEM TASKING
        print ("start_exec")

        # Not gonna handle backslashes
        if '/' in executable:
            name = executable[executable.rfind('/')+1:]
        else:
            name = executable

        # shell=True starts a new shell as the process reported, not the actual command.
        try:
            print ("a")
            cmd = []
            cmd.append(executable)
            if isinstance(args, list):
                print ("d")
                cmd.append(str(executable))
                if len(args) > 0:
                    for idx in args:
                        cmd.append(args[idx])
            elif args == "":
                print ("b")
                pass
            else:
                raise Exception("Only List type for executable arguments is supported")
            print ("e: ", type(cmd))
            print (cmd)
            ps = subprocess.Popen(cmd)
            print ("f")
            name = name + "_" + str(ps.pid)
            print ("g")
            launch_success = True
            self.running[str(name)] = ps.pid
            msg = "No news is good news"
        except Exception as e:
            launch_success = False
            print (e)
            msg = e
            print ("the return of the executable")

        return name, launch_success, msg
