import rospy, rosnode, roslaunch, rosgraph
import asyncio
import logging
import errno
from roslaunch import roslaunch_logs, rlutil, pmon
from std_msgs.msg import String
import subprocess, os, signal
import time

try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy

def test(self, signal, frame):
    print ("Hello signal")

#log = logging.getLogger(__name__)
log = logging.getLogger(name="mcLogger")

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
        log.info("Setting up TaskMaster")
        self.connected = False
        self.orbituary = []
        self.running_launches = {}
        self.running_nodes= {}
        self.running_execs = {}
        signals = (signal.SIGINT, test)

    def pub_connections(self, msg):
        try:
            self.pub_conn.publish(msg)
        except Exception as err:
            log.error(err)

    def pub_death(self, msg):
        try:
            self.pub_death.publish(msg)
        except Exception as err:
            log.error(err)

    async def init_node(self):
        """
        Initializes as a ROS Node with the sole purposes of broadcasting the MC
        services (including itself) that are available. They can then be used
        to determine secondary decision making such as runtime profiles.
        """
        # TODO: Implement respawn option
        # TODO: Run without async support for timeout
        try:
            rospy.init_node(self.node_name)
        except Exception as err:
            log.error(err)
        self.pub_conn = rospy.Publisher(self.conn_topic, String, queue_size=1)
        self.pub_death = rospy.Publisher(self.death_topic, String, queue_size=1)
        self.connected = True
        log.info("TaskMaster node has been initialized")


    async def healthCheck(self, hb_interval):
        """
        Updates the connection status of TaskMaster periodically.

        @param hb_interval: Interval per heartbeat loop. Not the same as a timeout.
        Strict attempts to enforce hb_interval by enforcing timeouts in processes
        and reducing sleep time with time taken.
        """
        try:
            if self.init_timeout > hb_interval:
                raise AssertionError("Connection timeout of {self.init_timeout} is longer than heartbeat interval of {hb_interval}")
        except AssertionError as err:
            log.error(err)
            #TODO: Ensure clean return
            return

        while True:
            stopwatch_start = time.perf_counter()
            try:
                # Checks existance for ROSMaster.
                param_server = rosgraph.Master('/roslaunch')
                self.run_id = param_server.getParam('/run_id')
                if not self.connected:
                    #await to_thread(self.init_node)
                    try:
                        await asyncio.wait_for(self.init_node(), self.init_timeout)
                    except TimeoutError as err:
                        log.error(err)
                        pass
                print ("connected = true")
            except ConnectionRefusedError as err:
                log.error(err)
            stopwatch_stop = time.perf_counter()
            log.debug(f"Healthcheck loop for TaskMaster took {stopwatch_stop-stopwatch_start}")
            await asyncio.sleep(hb_interval - (stopwatch_stop - stopwatch_start))


    async def launch_cleaner(self, clean_missing=None, to_clean=None, cleanup=None):
        """
        """
        result = False
        msg = "to_clean: %s" % (str(to_clean))
        # TODO: Ensure that a to_clean command kills something or give error reply
        if len(self.running_launches) > 0:
            for k, v in self.running_launches.items():
                parent, pid_list = v
                if clean_missing:
                    for pid in pid_list:
                        # Check if PID exists
                        success, msg = self.cautious_kill(pid, 0):
                        if success:
                            result = True
                            pass
                        else:
                            to_clean = k
                if cleanup:
                    to_clean = k
                if to_clean == k:
                    self.orbituary.append(k)
                    parent.shutdown()
                    # Just to ensure all childs are dead
                    for pid in pid_list:
                        result, msg = self.cautious_kill(pid, signal.SIGKILL)

        return result, msg



    async def node_cleaner(self, clean_missing=None, to_clean=None, cleanup=None):
        # TODO: Ensure that a to_clean command kills something or give error reply
        result = False
        msg = "to_clean: %s" % (str(to_clean))
        if len(self.running_nodes) > 0:
            for k, v in self.running_nodes.items():
                pid, runner = v
                if clean_missing:
                    success, msg = self.cautious_kill(pid, 0):
                    if success:
                        result = True
                        pass
                    else:
                        to_clean = k
                if cleanup:
                    to_clean= k
                if to_clean == k:
                    self.orbituary.append(k)
                    runner.stop()
                    # Double stabbing
                    result, msg = self.cautious_kill(pid, signal.SIGKILL)
        return result, msg


    async def exec_cleaner(self, clean_missing=None, to_clean=None, cleanup=None):
        # TODO: Ensure that a to_clean command kills something or give error reply
        result = False
        msg = "to_clean: %s" % (str(to_clean))
        if len(self.running_execs) > 0:
            for k, v in self.running_execs.items():
                pid = v
                if clean_missing:
                    success, msg = self.cautious_kill(pid, 0):
                    if success:
                        result = True
                        pass
                    else:
                        to_clean = k
                if cleanup:
                    to_clean = k
                if to_clean == k:
                    self.orbituary.append(k)
                    result, msg = self.cautious_kill(pid, signal.SIGKILL)
        return result, msg



    async def watchdog(self, hb_interval):
        """
        Reports processes/nodes that died unexpectedly and kills associated ones
        """
        while True:
            await asyncio.gather(
                self.launch_cleaner(clean_missing=True),
                self.node_cleaner(clean_missing=True),
                self.exec_cleaner(clean_missing=True),
                return_exceptions=True,
            )
            await asyncio.sleep(hb_interval)


    def cautious_kill(self, pid, sig):
        try:
            os.kill(pid, sig)
            return True, "Success"
        except OSError as err:
            if err.errno == errno.EINTR:
                log.warning(errno.EINTR)
            if err.errno == errno.ESRCH:
                log.debug(errno.ESRCH)
            elif err.errno == errno.EPERM:
                log.error(errno.EPERM)
            elif err.errno == errno.EINVAL:
                log.error(errno.EINVAL)
            return False, err


    async def async_start(self, *args):
        if args[0] == "roslaunch":
            args[1].start()
        if args[0] == "rosnode":
            p, result = args[1].launch_node(args[2])
            print(p)
            print(result)
            return p, result
        if args[0] == "executable":
            ps = subprocess.Popen(args[1])
            return ps


    async def async_stop(self, *args):
        await asyncio.gather(
            self.launch_cleaner(cleanup=True),
            self.node_cleaner(cleanup=True),
            self.exec_cleaner(cleanup=True),
            return_exceptions=True,
        )


    async def start_launchfile(self, launchfile, timeout, launch_args, pkg=None):
        #TODO: Requires proper signal handling
        """
        Starter trigger for ROS Launch files

        @param launchfile: The full name or absolute path of the launchfile
        @param tiomeout: Timeout for it's execution
        @param pkg: Package for which the launch file belongs to. Can be ommited
            if the absolute path to the launchfile is given.
        @param: launch_args: Arguments to the launch command.
        """
        # Find Absolute path to launch file, arguments are of non-zero index.
        name = launchfile
        msg = "No news is good news"
        launch_success = False
        try:
            if pkg is not None:
                rl_obj = rlutil.resolve_launch_arguments([pkg, launchfile])
            else:
                rl_obj = rlutil.resolve_launch_arguments([launchfile])

            # Extract out filename from absolute path to use as Name Identifier
            if '/' in rl_obj[0]:
                launch_file = rl_obj[0][rl_obj[0].rfind('/')+1:]
            else:
                launch_file = rl_obj[0]

            p = roslaunch.parent.ROSLaunchParent(self.run_id, rl_obj,
                is_core=False,
                port=launch_args['Port'],
                timeout=timeout,
                #TODO: Support for all roslaunch_args
                master_logger_level=launch_args['Loglevel'],
                sigint_timeout=timeout,
                sigterm_timeout=timeout)

            await asyncio.wait_for(self.async_start("roslaunch", p), timeout)

            # TODO: To place at another location.
            self.master = rosgraph.Master('/rosnode', master_uri="http://localhost:11311")
            pid_list = []
            for node in p.runner.config.nodes:
                node_name = "/" + node.name
                api = rosnode.get_api_uri(self.master, node_name)
                node = ServerProxy(api)
                pid = rosnode._succeed(node.getPid('/rosnode'))
                pid_list.append(pid)
                msg = ""
            name = launch_file + "_" + str(pid_list[0])
            self.running_launches[str(name)] = (p, pid_list)
            msg = "No news is good news"
            launch_success = True
        except Exception as e:
            print (e)
            msg = e
            launch_success = False

        return name, launch_success, msg

    async def start_ros_node(self, pkg, executable, params, launch_args, timeout):
        # Support for launch_args
        name = executable
        msg = "No news is good news"
        launch_success = False
        try:
            node = roslaunch.core.Node(pkg, executable)
            config = roslaunch.ROSLaunchConfig()
            config.nodes.append(node)


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

            #what = asyncio.create_task(self.async_start(launch_runner.launch_node(node)))
            p, launch_success = await asyncio.wait_for(self.async_start("rosnode", launch_runner, node), timeout)
            if launch_success:
                name = name + "_" + str(p.pid)
            self.running_nodes[str(name)] = (p.pid, launch_runner)
        except Exception as e:
            print (e)
            msg = e

        return name, launch_success, msg

    async def start_executable(self, executable, timeout, args=None):
        # TODO: BREAKUP TO SYSTEM TASKING

        # Not gonna handle backslashes
        if '/' in executable:
            name = executable[executable.rfind('/')+1:]
        else:
            name = executable
            msg = "No news is good news"

        # shell=True starts a new shell as the process reported, not the actual command.
        try:
            cmd = []
            cmd.append(str(executable))
            if isinstance(args, list):
                if len(args) > 0:
                    for idx in args:
                        cmd.append(idx)
            elif args == None:
                pass
            else:
                raise Exception("Only List type for executable arguments is supported")
            #TODO: Resolve errror with wait_for needed and awaitable
            ps = await asyncio.wait_for(self.async_start("executable", cmd), timeout)
            name = name + "_" + str(ps.pid)
            launch_success = True
            self.running_execs[str(name)] = (ps.pid)
        except Exception as err:
            launch_success = False
            msg = err
            log.error(err)

        return name, launch_success, msg
