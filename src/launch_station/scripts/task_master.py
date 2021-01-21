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

log = logging.getLogger(name="mcLogger")

class TaskROSter:
    """
    TaskROSter provides the ROS interface to execute ROS Nodes. Basic
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
        self.immortal = {}
        self.running_launches = {}
        self.running_nodes= {}
        self.running_execs = {}

    def pub_connections(self, msg):
        """
        Publishes services that are connected.

        @param msg [list]: List of services that are connected.
        """
        try:
            self.pub_conn.publish(msg)
        except Exception as err:
            log.error(err)

    def pub_death(self, msg):
        """
        Publishes nodes that have died.

        @param msg [list]: List of nodes that died.
        """
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
            except ConnectionRefusedError as err:
                log.error(err)
            stopwatch_stop = time.perf_counter()
            log.debug(f"Healthcheck loop for TaskMaster took {stopwatch_stop-stopwatch_start}")
            await asyncio.sleep(hb_interval - (stopwatch_stop - stopwatch_start))


    async def clean_launch(self, clean_missing=None, clean=None, clean_all=None):
        """
        Loops throught the running ROS Launchfiles as registered and cleans
        according to the arguments given. Only one option can be populated.

        @param clean_missing [bool]: True: check each registered ROS Launchfile
            ID is alive. Places ID up to be cleaned if not alive as part
            of a de-registration process.
        @param clean [str]: Identifier to be cleaned.
        @param clean_all [bool]: True cleans all the existing registered process

        @return result [bool]: True if something was cleaned.
        @return msg [str]: Provisioned for error messages.
        """
        result = False
        msg = "to clean: %s" % (str(clean))
        # TODO: Ensure that a to_clean command kills something or give error reply
        if len(self.running_launches) > 0:
            for k, v in self.running_launches.items():
                parent, pid_list = v
                if clean_missing:
                    for pid in pid_list:
                        # Check if PID exists
                        success, msg = self.clean_carefully(pid, 0)
                        if not success:
                            log.debug(f"Needs cleaning: {k}")
                            clean = k
                if clean_all:
                    to_clean = k
                if clean == k:
                    log.info(f"Cleaning: {k}")
                    self.orbituary.append(k)
                    parent.shutdown()
                    # TODO: Possible exception from shutdown() not handled.
                    # Just to ensure all childs are dead
                    for pid in pid_list:
                        result, msg = self.clean_carefully(pid, signal.SIGINT)
                        if not result:
                            log.error(msg)
                            self.immortal[k] = v
                            break
                        else:
                            pid_list.remove(pid)
                            log.debug(f"Removing {k} from list of running nodes")
                    del self.running_launches[k]

        # Something was cleaned
        if result:
            master = rosgraph.Master('/rosnode')
            pinged, unpinged ==rosnode.rosnode_ping_all
            cleanup_master_blacklist(master, unpinged)

        log.debug(f"Result from cleaning ROS Launchfiles: {result}, msg: {msg}")
        return result, msg


    async def clean_node(self, clean_missing=None, clean=None, clean_all=None):
        """
        Loops throught the running ROS nodes as registered and cleans
        according to the arguments given. Only one option can be populated.

        @param clean_missing [bool]: True: check each registered ROS node ID
            is alive. Places ID up to be cleaned if not alive as part
            of a de-registration process.
        @param clean [str]: Identifier to be cleaned.
        @param clean_all [bool]: True cleans all the existing registered process

        @return result [bool]: True if something was cleaned.
        @return msg [str]: Provisioned for error messages.
        """
        result = False
        msg = "to clean: %s" % (str(clean))
        if len(self.running_nodes) > 0:
            for k, v in self.running_nodes.items():
                pid, runner = v
                if clean_missing:
                    success, msg = self.clean_carefully(pid, 0)
                    if not success:
                        log.debug(f"Needs cleaning: {k}")
                        clean = k
                if clean_all:
                    clean = k
                if clean == k:
                    log.info(f"Cleaning: {k}")
                    self.orbituary.append(k)
                    runner.stop()
                    # TODO: Possible exception from stop() not handled.
                    result, msg = self.clean_carefully(pid, signal.SIGKILL)
                    if not result:
                        log.error(msg)
                    else:
                        log.debug(f"Removing {k} from list of running nodes")
                        del self.running_nodes[k]

        log.debug(f"Result from cleaning ROS nodes: {result}, msg: {msg}")
        return result, msg


    async def clean_exec(self, clean_missing=None, clean=None, clean_all=None):
        """
        Loops throught the running executables as registered and cleans
        according to the arguments given. Only one option can be populated.

        @param clean_missing [bool]: True: check each registered executable ID
            is alive. Places executable ID up to be cleaned if not alive as part
            of a de-registration process.
        @param clean [str]: Identifier to be cleaned.
        @param clean_all [bool]: True cleans all the existing registered process

        @return result [bool]: True if something was cleaned.
        @return msg [str]: Provisioned for error messages.
        """
        result = False
        msg = "to clean: %s" % (str(clean))
        if len(self.running_execs) > 0:
            for k, v in self.running_execs.items():
                pid = v
                if clean_missing:
                    success, msg = self.clean_carefully(pid, 0)
                    if not success:
                        log.debug(f"Needs cleaning: {k}")
                        clean = k
                if clean_all:
                    clean = k
                if clean == k:
                    log.info(f"Cleaning: {k}")
                    self.orbituary.append(k)
                    result, msg = self.clean_carefully(pid, signal.SIGKILL)
                    if not result:
                        log.error(msg)
                    else:
                        log.debug(f"Removing {k} from list of running execs")
                        del self.running_execs[k]

        log.debug(f"Result from cleaning executables: {result}, msg: {msg}")
        return result, msg


    async def watchdog(self, hb_interval):
        """
        Reports processes/nodes that died unexpectedly and kills associated ones
        Sleeps for an interval after each loop. Does not run at a fixed rate as
        cleaning time is undetermined.

        @param hb_interval: Interval in seconds to clean.
        """
        while True:
            log.debug("Checking for dead processes")
            await asyncio.gather(
                self.clean_launch(clean_missing=True),
                self.clean_node(clean_missing=True),
                self.clean_exec(clean_missing=True),
                return_exceptions=True,
            )
            await asyncio.sleep(hb_interval)


    def clean_carefully(self, pid, sig):
        """
        Runs the given signal against the provided PID.

        While all UNIX signals are accepted, the common ones used are:
        sig=0: Checks if PID exists.
        sig=SIGINT: Interrupts the PID.
        sig=SIGKILL: Kill's the PID, but may cause some errors, more testing
            required.

        @param pid [int]: PID of process in contention.
        @param sig [int|signal]: Unix signal to run against the PID.

        @return errno [int]: Corresponding errors
        """
        try:
            os.kill(pid, sig)
            return True, "Success"
        except OSError as err:
            if err.errno == errno.EINTR:
                log.warning(f"Cleaning error: {errno.EINTR}")
            if err.errno == errno.ESRCH:
                log.warning(f"Cleaning error: {errno.ESRCH}")
            elif err.errno == errno.EPERM:
                log.warning(f"Cleaning error: {errno.EPERM}")
            elif err.errno == errno.EINVAL:
                log.warning(f"Cleaning error: {errno.EINVAL}")
            return False, err


    async def async_start(self, *args):
        """
        Filters based on the supplied args and triggers accordingly.

        @param args [str]: Arguments

        @return p [rosnode process]: Process corresponding to the ROS Node.
        @return result [bool]: True = Success
        @return ps [process]: Process corresponding to the executable
        """
        if args[0] == "roslaunch":
            args[1].start()
        elif args[0] == "rosnode":
            p, result = args[1].launch_node(args[2])
            return p, result
        elif args[0] == "executable":
            ps = subprocess.Popen(args[1])
            return ps
        else:
            log.error("Unrecognized async start type")


    async def async_stop(self):
        """
        Start the cleanup on all the nodes.
        """
        log.info("Cleaning up all the nodes launched")
        await asyncio.gather(
            self.clean_launch(clean_all=True),
            self.clean_node(clean_all=True),
            self.clean_exec(clean_all=True),
            return_exceptions=True,
        )


    async def start_launchfile(self, launchfile, timeout, pkg=None):
        #TODO: Requires proper signal handling
        """
        Starter for ROS Launchfiles.

        @param launchfile [str]: The name or absolute path of the launchfile.
        @param timeout [float]: Timeout in seconds for launchfile to launch.
        @param pkg [str]: Package for which the launch file belongs to. Can be
            ommited if the absolute path to the launchfile is given.

        @return name [str]: Name of the ROS Node. If launch_success is True,
            name will be appended with the PID it corresponds to.
        @return launch_success [bool]: True if no errors occurred along the way.
            False otherwise.
        @return msg [str]: Message of the success if launch_success is True, or
            the error message if launch_success if False.
        """
        name = launchfile
        msg = "No news is good news"
        launch_success = False

        # Find Absolute path to launch file, arguments are of non-zero index.
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

            # Controls the launch parameters. The default follows ROS 1
            # default configuration where master not remote.
            p = roslaunch.parent.ROSLaunchParent(self.run_id, rl_obj,
                is_core=False,
                timeout=timeout,
                sigint_timeout=timeout,
                sigterm_timeout=timeout)

            # TODO: Resolve asynchronous launching of identical launchfiles.
            # If two identical launchfiles are launched asynchronously, the
            # observed asynchronous behavior is that the former launch will
            # trigger a shutdown down. Before the former is down, the latter
            # will observe the existence of the former and hence trigger
            # a shutdown as well. End result is both shuts down.
            # Probably a race condition.
            log.info(f"Starting ROS Launchfile: {name}")
            await asyncio.wait_for(self.async_start("roslaunch", p), timeout)

            # TODO: To place at another location.
            self.master = rosgraph.Master('/rosnode')
            pid_list = []

            print ("1")
            for node in p.runner.config.nodes:
                print ("2")
                node_name = "/" + node.name
                print (node_name)
                api = rosnode.get_api_uri(self.master, node_name)
                print ("api: ", api)
                node = ServerProxy(api)
                print ("5")
                pid = await rosnode._succeed(node.getPid(node_name))
                print ("6")
                pid_list.append(pid)
                print ("7")

            name = launch_file + "_" + str(pid_list[0])
            print ("8")
            self.running_launches[str(name)] = (p, pid_list)
            print ("9")
            log.debug(f"Appending {name}=({p}, {pid_list})")
            print ("0")
            launch_success = True
        except Exception as err:
            log.error(err)
            msg = str(err)
            launch_success = False

        log.debug(f"Name: {name}, Launch Success: {launch_success}, Msg: {msg}")
        return name, launch_success, msg

    async def start_ros_node(self, pkg, executable, timeout, params=None):
        """
        Starter for ROS Nodes.
        Converts the executable (ROS Node) as a launch_runner and parses each
        params as a ConfigParameter to be launch alongside the executable,
        influencing the parameters inside the executable.

        @param pkg [str]; Name of the package which the executable belongs to.
        @param executable [str]: Name of the executable to trigger.
        @param timeout [float]: Timeout in seconds for executable to trigger.
        @param params: [dict]: Fully qualified parameters
            and their corresponding values.

        @return name [str]: Name of the ROS Node. If launch_success is True,
            name will be appended with the PID it corresponds to.
        @return launch_success [bool]: True if no errors occurred along the way.
            False otherwise.
        @return msg [str]: Message of the success if launch_success is True, or
            the error message if launch_success if False.
        """
        name = executable
        msg = "No news is good news"
        launch_success = False
        try:
            node = roslaunch.core.Node(pkg, executable)
            config = roslaunch.ROSLaunchConfig()
            config.nodes.append(node)

            if params is not None and len(params.keys()) > 0:
                for idx in params:
                    key = roslaunch.core.Param(idx, params[idx])
                    log.debug(f"Adding config parameter: {idx}:{params[idx]}")
                    config.add_param(key)

            launch_runner = roslaunch.launch.ROSLaunchRunner(self.run_id,
                config=config)

            log.info(f"Starting ROS Node: {name}")
            p, launch_success = await asyncio.wait_for(self.async_start(
                "rosnode", launch_runner, node), timeout)
            if launch_success:
                name = name + "_" + str(p.pid)
            log.debug(f"Appending {name}=({p.pid}, launch_runner)")
            self.running_nodes[str(name)] = (p.pid, launch_runner)
        except Exception as err:
            log.error(err)
            msg = str(err)
            launch_success = False

        log.debug(f"Name: {name}, Launch Success: {launch_success}, Msg: {msg}")
        return name, launch_success, msg

    async def start_executable(self, executable, timeout, args=None):
        """
        Starter for executable. Preprocessing on the executable variable is done
        to extract out the name.

        @param executable [str]: Executable to be launched. Basically arg[0] in
            a list of executable commands/arguments/options.
            Either absolute or relative paths are supported.
        @param timeout [float]: Timeout in seconds for executable to trigger.
        @param args [list of str]: Supporting arguments and options.
            Currently only a list of arguments is supported. Each argument
            needs to be an element in the list. Each element will then be
            converted into strings.

        @return name [str]: Name of the executable. If launch_success is True,
            name will be appended with the PID it corresponds to.
        @return launch_success [bool]: True if no errors occurred along the way.
            False otherwise.
        @return msg [str]: Message of the success if launch_success is True, or
            the error message if launch_success if False.
        """
        msg = "No news is good news"
        name = "No name"
        launch_success = False

        # Not gonna handle backslashes
        #
        # Extract name from absolute paths
        if '/' in executable:
            name = executable[executable.rfind('/')+1:]
        else:
            name = executable

        try:
            cmd = []
            cmd.append(str(executable))
            # Argument support is very weak alot of possible combinations and
            # proper parsing is required.
            if isinstance(args, list):
                if len(args) > 0:
                    for idx in args:
                        # This parser requires each argument to be an element in
                        # the list. Not bundled together as a single element.
                        #
                        # Only strings will be tolerated.
                        cmd.append(str(idx))
            elif args == None:
                pass
            else:
                msg = "Only List type for executable arguments is supported"
                log.error(msg)
                raise Exception(msg)

            log.info(f"Starting executable: {name}")
            ps = await asyncio.wait_for(self.async_start("executable", cmd), \
                    timeout)
            name = name + "_" + str(ps.pid)
            launch_success = True
            log.debug(f"Appending {name}=({ps.pid})")
            self.running_execs[str(name)] = (ps.pid)
        except Exception as err:
            launch_success = False
            msg = str(err)
            log.error(err)

        log.debug(f"Name: {name}, Launch Success: {launch_success}, Msg: {msg}")
        return name, launch_success, msg
