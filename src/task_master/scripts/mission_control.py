import json
import signal
import time
import sys
import os

import asyncio
from communicator import AMQPHandler
from db import Mongo

import logging
import logging.config

script_dir = os.path.dirname(os.path.realpath(__file__))
log_conf = script_dir+"/logging.conf"

logging.config.fileConfig(log_conf, disable_existing_loggers=False)
log = logging.getLogger(name="mcLogger")



class TaskMaster:
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
        try:
            self.commauth = rospy.get_param("~commauth", "guest:guest")
            self.commadd = rospy.get_param("~commadd", "localhost:5672")
            self.hb_interval = rospy.get_param("~hb_interval", 1)
            self.robot_ex = rospy.get_param("~robot_ex", "robot_ex")
            self.robot_cancel_key = rospy.get_param("~robot_cancel_key", "robot_cancel")
            self.robot_cancel_reply_key = rospy.get_param("~robot_cancel_reply_key", "robot_cancel_reply")
            self.robot_launch_key = rospy.get_param("~robot_launch_key", "robot_launch")
            self.robot_launch_reply_key = rospy.get_param("~robot_launch_reply_key", "robot_launch_reply")
            self.robot_sys_key = rospy.get_param("~robot_sys_key", "robot_sys")
            self.robot_orbituary_key = rospy.get_param("~robot_orbituary_key", "robot_orbituary")
            self.orbituary_timeout = rospy.get_param("~orbituary_timeout", 10)
        except Exception as err:
            log.error(f"Unable to initialise TaskMaster. {err}")

        self.orbituary = {}
        self.immortal = {}
        self.running_launches = {}
        self.running_nodes= {}
        self.running_execs = {}
        self.orbituary_lock = asyncio.Lock()
        self.running_lock = asyncio.Lock()

        self.pub_conn = rospy.Publisher("/taskmaster", String, queue_size=10)
        self.pub_death = rospy.Publisher("/orbituary", String, queue_size=10)
        self.master = rosgraph.Master('/rosnode')
        log.info(f"TaskMaster node has been initialized")
        self.connected = True

        comm_login="amqp://%s@%s" % (args.commauth, args.commadd)

        self.comm = AMQPHandler(comm_login, self.robot_ex, self.hb_interval)
        self.db = Mongo()

        loop = asyncio.get_event_loop()
        signals = (signal.SIGHUP, signal.SIGTERM, signal.SIGINT)
        for s in signals:
            loop.add_signal_handler(
                s, lambda s=s: asyncio.create_task(self.shutdown(loop, signal=s)))
        # comment out the line below to see how unhandled exceptions behave
        loop.set_exception_handler(self.handle_exception)
        log.debug("Setting up Exception Handler")
        queue = asyncio.Queue()

        try:
            loop.create_task(
                self.comm.healthCheck(
                    hb_interval = self.hb_interval
                    )
            )
            loop.create_task(
                self.watchdog(
                    hb_interval = self.hb_interval)
            )
            loop.create_task(
                self.systemCheck(
                    hb_interval = self.hb_interval)
            )
            loop.create_task(
                self.comm.subscribe(
                    routing_key = amqp_cancel_key,
                    async_msg_proc_func = self.process_cancel_msg,
                    reply_routing_key = amqp_cancel_reply_key,
                    reply_prep = self.msg_to_json
                )
            )
            loop.create_task(
                self.comm.subscribe(
                    routing_key = amqp_launch_key,
                    async_msg_proc_func = self.process_task_msg,
                    reply_routing_key = amqp_launch_reply_key,
                    reply_prep = self.msg_to_json
                )
            )
            loop.run_forever()
        finally:
            print ("before loop close")
            #tasks = [t for t in asyncio.all_tasks()]
            #for task in tasks:
            #    task.cancel()
            loop.close()


    async def comm_death_pub(self, orbituary_list):
        """
        Triggers a publishing of dead processes on the communicator.

        @param [list]: List of dead nodes with time since death less than
            orbituary_timeout
        """
        if self.comm.connected:
            result = await self.comm.publish(
                routing_key = self.robot_orbituary_key,
                msg = orbituary_list,
                msg_prep = self.msg_to_json
            )
            return result


    async def comm_hb_pub(self, msg):
        """
        Triggers a publishing of connected services on the communicator

        @param msg [list]: List of connected services.
        """
        if self.comm.connected:
            result = await self.comm.publish(
                routing_key = self.robot_sys_key,
                msg = msg,
                msg_prep = self.msg_to_json
            )
            return result


    async def death_pub(self, orbituary_list):
        """
        Triggers a publishing of dead processes on the TM.

        @param [list]: List of dead nodes with time since death less than
            orbituary_timeout
        """
        result = self.pub_dead(json.dumps(orbituary_list))
        await asyncio.sleep(0.5)
        return result


    async def hb_pub(self, msg):
        """
        Triggers a publishing of connected services on the TM.

        @param msg [list]: List of connected services.
        """
        result = self.pub_conn(json.dumps(msg))
        await asyncio.sleep(0.5)
        return result


    async def process_cancel_msg(self, json_msg):
        """
        Processes the cancel_msg.

        @param json_msg [json]: Message

        @return result [bool]: True if cancelling had no exceptions.
        """
        # TODO: Write JSON messag how-tos in comments.
        cancel_msg = json.loads(json_msg.decode("utf-8"))

        # Triggers shutdown by raising SIGTERM for exception handler to catch
        # and commence shutdown sequence.
        if "Shutdown" in cancel_msg:
            if cancel_msg["Shutdown"] == True:
                # 15 = SIGTERM
                signal.raise_signal(signal.SIGINT)
                return True
        else:
            name = cancel_msg["Name"]
            timeout = cancel_msg["Timeout"]
            # TODO: Check that processes are running so that they can be killed.
            # Prevents a half-assed kill job.
            try:
                if name in self.tm.running_launches:
                    success, msg = await asyncio.wait_for(self.tm.clean_launch(
                        clean=name), timeout)
                elif name in self.tm.running_nodes:
                    success, msg = await asyncio.wait_for(self.tm.clean_node(
                        clean=name), timeout)
                elif name in self.tm.running_execs:
                    success, msg = await asyncio.wait_for(self.tm.clean_exec(
                        clean=name), timeout)
                else:
                    raise Exception
            except Exception as err:
                success = False
                msg = "Task not found"
                log.error(f"Cancelling message: {err}")

            result = {'Success': success, 'Msg': msg}
            log.debug(f"Reply message: {result}")
            return result


    async def process_task_msg(self, json_msg):
        """
        Filters the message to know which starters to trigger.
        Filtering is based on the combination of fields that is not None.

        @param json_msg [str]: Message to parse.

        @return result [dict]: Result of the processing for tasks
            name [str]: identifier for task, PID is attached if success is True.
            success [bool]: True if task was processed/executed without errors.
            statement [str]: Provision for error codes if success is False.
        """
        msg = json.loads(json_msg.decode("utf-8"))

        # Catch all for unacceptable msg formats
        name = "Error"
        success = False
        statement = "Unrecognized message format"

        try:
            # Strict compliance with accepted formats reduces assumed
            # functionality

            # ROSPY API requires nodes to be started and spun in the main thread
            if msg['Timeout'] is not None and \
                msg['Pkg'] is not None and \
                msg['Executable'] is None and \
                msg['Params'] is None and \
                msg['Args'] is None and \
                msg['Launchfile'] is not None:
                    log.info("Starting a launchfile")
                    name, success, statement = await self.tm.start_launchfile(
                        pkg = msg['Pkg'],
                        launchfile = msg['Launchfile'],
                        timeout = msg['Timeout']
                    )
            elif msg['Timeout'] is not None and \
                msg['Pkg'] is None and \
                msg['Executable'] is None and \
                msg['Params'] is None and \
                msg['Args'] is None and \
                msg['Launchfile'] is not None:
                    log.info("Starting a launchfile without package")
                    name, success, statement = await self.tm.start_launchfile(
                        launchfile = msg['Launchfile'],
                        timeout = msg['Timeout']
                    )
            elif msg['Timeout'] is not None and \
                msg['Pkg'] is not None and \
                msg['Executable'] is not None and \
                msg['Params'] is not None and \
                msg['Args'] is None and \
                msg['Launchfile'] is None:
                    log.info("Starting a ROS node with params")
                    name, success, statement = await self.tm.start_ros_node(
                        pkg = msg['Pkg'],
                        executable = msg['Executable'],
                        params = msg['Params'],
                        timeout = msg['Timeout']
                    )
            elif msg['Timeout'] is not None and \
                msg['Pkg'] is not None and \
                msg['Executable'] is not None and \
                msg['Params'] is None and \
                msg['Args'] is None and \
                msg['Launchfile'] is None:
                    log.info("Starting a ROS node without params")
                    name, success, statement = await self.tm.start_ros_node(
                        pkg = msg['Pkg'],
                        executable = msg['Executable'],
                        timeout = msg['Timeout']
                    )
            elif msg['Timeout'] is not None and \
                msg['Pkg'] is None and \
                msg['Executable'] is not None and \
                msg['Params'] is None and \
                msg['Args'] is not None and \
                msg['Launchfile'] is None:
                    log.info("Starting an executable")
                    name, success, statement = await self.tm.start_executable(
                        executable = msg['Executable'],
                        args = msg['Args'],
                        timeout = msg['Timeout']
                    )
            elif msg['Timeout'] is not None and \
                msg['Pkg'] is None and \
                msg['Executable'] is not None and \
                msg['Params'] is None and \
                msg['Args'] is None and \
                msg['Launchfile'] is None:
                    log.info("Starting an exeuctable without args")
                    name, success, statement = await self.tm.start_executable(
                        executable = msg['Executable'],
                        timeout = msg['Timeout']
                    )
        except Exception as err:
            log.error(f"Exception filtering messages. {err}")
        result = {'Name': name, 'Success': success, 'Msg': msg}
        log.debug(f"Returning {result}")
        return result


    def msg_to_json(self, msg):
        """
        To convert messages to json

        @param msg [str]: Message

        @return json_msg [json]: json-fied msg
        """
        json_msg = json.dumps(msg).encode()
        return json_msg


    def handle_exception(self, loop, context=None):
        """
        To handle all uncaught exceptions throughout MissionControl.
        Prints the context of the exceptions before triggering a shutdown.

        @param loop [loop]: current asyncio loop
        @param context [Exception context]: Context of Exception to print, log.
        """
        print ("there is an exception")
        msg = context.get("exception", context["message"])
        log.error(f"Caught exception: {msg}")
        log.info(f"Shutting down...")
        asyncio.create_task(self.shutdown(loop))


    async def shutdown(self, loop, signal=None):
        """
        Cleanup tasks tied to the service's shutdown. Cleans TM's tasks first,
        publishes one the orbituary list one last time. Before killing all
        tasks and events in the event loop.

        @param loop [loop]: Loop whose events and tasks are to be stopped.
        """
        log.info("Shutdown sequence triggered")
        if signal:
            logging.info(f"Received exit signal {signal.name}...")

        stop_result = await self.tm.async_stop()
        pub_result = await self.systemCheck(self.hb_interval, final=True)
        tasks = [t for t in asyncio.all_tasks()]

        log.info(f"Cancelling {len(tasks)} outstanding tasks")
        #[task.cancel() for task in tasks]
        #await asyncio.gather(*tasks, return_exceptions=True)

        for task in tasks:
            try:
                log.debug(f"Cancelling {task}")
                task.cancel()
            except asyncio.CancelledError as ce:
                log.error(f"Cancellation Error with {task}")
                continue
        print("before loop stop")
        loop.stop()
        tasks = [t for t in asyncio.all_tasks()]
        for task in tasks:
            try:
                log.debug(f"Cancelling {task}")
                task.cancel()
            except asyncio.CancelledError as ce:
                log.error(f"Cancellation Error with {task}")
                continue
        self.tm.shutdown()


    def pub_connections(self, msg):
        """
        Publishes services that are connected.

        @param msg [list]: List of services that are connected.
        """
        try:
            self.pub_conn.publish(msg)
            return True
        except Exception as err:
            log.error(f"Unable to establish connection. {err}")
            return False


    def pub_dead(self, msg):
        """
        Publishes nodes that have died.

        @param msg [list]: List of nodes that died.
        """
        try:
            pub_msg = msg
            self.pub_death.publish(pub_msg)
            return True
        except Exception as err:
            log.error(f"Unable to publish orbituary. {err}")
            return False


    async def systemCheck(self, hb_interval, final=None):
        """
        Publishes a Heartbeat periodically. Heartbeat is loaded with the
        connection status of the communication, task and database layer to
        provide secondary decision making.

        @param hb_interval [float]: Interval per heartbeat loop. Not the same as a
        timeout. Strict attempts to enforce hb_interval by enforcing timeouts
        in processes and reducing sleep time with time taken.
        """
        while True:
            stopwatch_start = time.perf_counter()
            # Update the dictionary periodically because primitive Types are
            # passed by value and not reference.
            self.connections = {
                "comm_connected": self.comm.connected,
                "db_connected": self.db.connected
            }
            # Trigger a cleanup on the orbituary list and save a local copy to
            # prevent changes.
            orbituary_list = await self.edit_orbituary(mode="update")
            try:
                # Time sensitive, can't add to queue and hope it gets pub soon.
                hb_code = await asyncio.gather(
                    self.comm_hb_pub(msg = self.connections),
                    self.hb_pub(msg = self.connections),
                    return_exceptions=True
                )
                death_code = await asyncio.gather(
                    self.comm_death_pub(orbituary_list),
                    self.death_pub(orbituary_list),
                    return_exceptions=True
                )
            except TimeoutError as err:
                log.error(f"System Healthcheck error: {err}")
            except ConnectionError as err:
                log.error(f"System Healthcheck error: {err}")
            except Exception as exc:
                log.error(f"System Healthcheck error: {exc}")
            stopwatch_stop = time.perf_counter()
            log.debug(f"Healthcheck loop for MissionControl took \
                {stopwatch_stop-stopwatch_start}")
            if final == True:
                return final
            await asyncio.sleep(hb_interval-(stopwatch_stop-stopwatch_start))


    async def edit_orbituary(self, mode, idx=None):
        """
        Consolidate all operations that require locking self.orbituary.
        Operations include updating the time since death, appending and removing

        @param mode [str]: Operation which caller wants to execute
        @param k [key]: key in self.orbituary specified by caller, corresponding
            to the operation required.

        @returns [dict]: a copy of self.orbituary as a snapshot
        """
        delete = []
        if mode == "update":
            await self.orbituary_lock.acquire()
            try:
                for k, v in self.orbituary.items():
                    if (time.time_ns() - v) > (self.orbituary_timeout * 10**9):
                        delete.append(k)
            except Exception as err:
                log.error(f"Updating time of death: {err}")
            finally:
                self.orbituary_lock.release()
        elif mode == "append":
            await self.orbituary_lock.acquire()
            try:
                if idx is not None:
                    self.orbituary[idx]=time.time_ns()
            except Exception as err:
                log.error(f"Appending orbituary: {err}")
            finally:
                self.orbituary_lock.release()
        else:
            log.error(f"Unknown error editing orbituary")

        if len(delete) > 0 :
            await self.orbituary_lock.acquire()
            for idx in delete:
                try:
                    del self.orbituary[idx]
                except Exception as err:
                    log.error(f"Error deleting orbituary item: {err}")
                    break
            self.orbituary_lock.release()

        if mode == "update":
            return self.orbituary


    async def edit_running(self, mode, running, *args):
        """
        Consolidate all operations that require locking lists of running
        processes. Operations include appending and deleting.

        @param mode [str]: Operation which caller wants to execute
        @param running [dict]: dict which the caller wants to perform operation
            on.
        @param args [*args]: arguments pertaining to the dictionary.

        @returns [dict]: a copy of self.orbituary as a snapshot
        """
        if mode == "append":
            await self.running_lock.acquire()
            try:
                running[args[0]] = args[1]
            except Exception as err:
                log.error(f"{err} while appending processes")
            finally:
                self.running_lock.release()

        elif mode == "delete":
            await self.running_lock.acquire()
            try:
                del running[args[0]]
            except Exception as err:
                log.error(f"{err} while deleting processes")
            finally:
                self.running_lock.release()
        else:
            log.error(f"Unknown error editing running_processes")


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
        cleaned = []
        if len(self.running_launches) > 0:
            for k, v in self.running_launches.items():
                parent, pid_list = v
                if clean_missing:
                    for pid in pid_list:
                        # Check if PID exists
                        success, msg = self.clean_carefully(pid, 0)
                        if not success:
                            log.debug(f"Missing Launchfile: {k}")
                            pid_list.remove(pid)
                            clean = k
                            break
                if clean_all:
                    to_clean = k
                if clean == k:
                    log.info(f"Cleaning: {k}")
                    try:
                        # Renders all PIDs in pid_list to be abandoned childs.
                        log.debug(f"Shutting down {parent}")
                        parent.shutdown()
                        result = True
                        msg = "Cleaned"
                        log.info(f"Deleting {k} from running_launchfiles")
                        cleaned.append(k)
                    except Exception as err:
                        log.error(f"When shutting down {parent}: {err}")
                        # Because cleaning the parents failed, manual cleaning
                        # is the next best thing.
                        # Not sure if this works or even required.
                        for pid in pid_list:
                            # PIDs in list may or may not be alive.
                            # Might not be able to catch any exception here.
                            log.warning(f"Manually cleaning PID: {pid}")
                            result, msg = self.clean_carefully(pid,
                                signal.SIGTERM)
        else:
            result = True
            msg = "Nothing to clean"

        # Something was cleaned
        log.debug(f"Triggering rosnode cleanup")
        if self.connected:
            master = rosgraph.Master('/rosnode')
            pinged, unpinged = rosnode.rosnode_ping_all()
            rosnode.cleanup_master_blacklist(master, unpinged)

        for idx in cleaned:
            await self.edit_running("delete", self.running_launches, idx)
            await self.edit_orbituary(mode="append", idx=idx)

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
        cleaned = []
        if len(self.running_nodes) > 0:
            for k, v in self.running_nodes.items():
                pid, runner = v
                if clean_missing:
                    success, msg = self.clean_carefully(pid, 0)
                    if not success:
                        log.debug(f"Missing node: {k}")
                        clean = k
                        continue
                if clean_all:
                    clean = k
                if clean == k:
                    log.info(f"Cleaning: {k}")
                    try:
                        runner.stop()
                        result = True
                        msg = "Cleaned"
                        log.info(f"Deleting {k} from running_nodes")
                        cleaned.append(k)
                    except Exceptions as err:
                        result, msg = self.clean_carefully(pid, signal.SIGTERM)
                        log.error(f"Error {err} cleaning node")
        else:
            result = True
            msg = "Nothing to clean"


        for idx in cleaned:
            await self.edit_running("delete", self.running_nodes, idx)
            await self.edit_orbituary(mode="append", idx=idx)

        log.debug(f"Result from cleaning ROS nodes: {result}, msg: {msg}")
        return result, msg


    async def clean_exec(self, clean_missing=None, clean=None, clean_all=None):
        """
        Loops throught the running executables as registered and cleans
        according to the arguments given. Only one option can be populated.

        @param clean_missing [bool]: True: check each registered executable ID
            is alive.
        @param clean [str]: Identifier to be cleaned.
        @param clean_all [bool]: True cleans all the existing registered process

        @return result [bool]: True if something was cleaned.
        @return msg [str]: Provisioned for error messages.
        """
        result = False
        msg = "to clean: %s" % (str(clean))
        cleaned = []
        if len(self.running_execs) > 0:
            for k, v in self.running_execs.items():
                pid = v
                if clean_missing:
                    success, msg = self.clean_carefully(pid, 0)
                    if not success:
                        log.debug(f"Executable: {k}")
                        cleaned.append[k]
                        continue
                if clean_all:
                    clean = k
                if clean == k:
                    log.info(f"Cleaning: {k}")
                    result, msg = self.clean_carefully(pid, signal.SIGTERM)
                    if result:
                        log.debug(f"Removing {k} from list of running execs")
                        result = True
                        msg = "Cleaned"
                        cleaned.append(k)
                    else:
                        log.error(f"Error {err} cleaning exec")
        else:
            result = True
            msg = "Nothing to clean"


        for idx in cleaned:
            await self.edit_running("delete", self.running_execs, idx)
            await self.edit_orbituary(mode="append", idx=idx)

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
            log.debug(f"Checking for dead processes")
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
        sig=SIGTERM: Terminate processes.
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
                log.warning(f"Cleaning error: errno.EINTR")
            if err.errno == errno.ESRCH:
                log.warning(f"Cleaning error: errno.ESRCH")
            elif err.errno == errno.EPERM:
                log.warning(f"Cleaning error: errno.EPERM")
            elif err.errno == errno.EINVAL:
                log.warning(f"Cleaning error: errno.EINVAL")
            return False, str(err)


    async def async_start(self, *args):
        """
        Filters based on the supplied args and triggers accordingly.

        @param args [str]: Arguments

        @return p [rosnode process]: Process corresponding to the ROS Node.
        @return result [bool]: True = Success
        @return ps [process]: Process corresponding to the executable
        """
        try:
            if args[0] == "roslaunch":
                args[1].start()
                return True
            elif args[0] == "rosnode":
                p, result = args[1].launch_node(args[2])
                return p, result
            elif args[0] == "executable":
                ps = subprocess.Popen(args[1], stdout=subprocess.DEVNULL)
                return ps
            else:
                log.error(f"Unrecognized async start type")
                raise Exception
        except Exception as err:
            log.error(f"Unable to start: {err}")
            return False


    async def async_stop(self):
        """
        Start the cleanup on all the nodes.
        """
        log.info(f"Cleaning up all the nodes launched")
        result = await asyncio.gather(
            self.clean_launch(clean_all=True),
            self.clean_node(clean_all=True),
            self.clean_exec(clean_all=True),
            return_exceptions=True,
        )
        for idx in range(len(result)):
            if not result[idx][0]:
                return False
        return True


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
        existing_launch = None

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

            await self.running_lock.acquire()
            try:
                # Workaround for weird async bug regarding behavior of duplicate
                # launchfiles. Kills existing launchfiles before triggering new
                # ones. Not true ROS behavior replicated but the best decent
                # workaround available.
                for key in self.running_launches.keys():
                    mo = re.search(str(launch_file)+"_[0-9]+$", key)
                    if mo is not None:
                        existing_launch = key
            except Exception as err:
                log.error(f"{err} during prelaunch check")
            finally:
                self.running_lock.release()
                if existing_launch is not None:
                    result, message = await self.clean_launch(clean=key)

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
            pid_list = []
            # ROSLaunch Parent throws only an exception and returns nothing.
            await asyncio.wait_for(self.async_start("roslaunch", p), timeout)

            for node in p.runner.config.nodes:
                node_name = "/" + node.name
                pid_not_retrieved = True
                while pid_not_retrieved:
                    try:
                        # As a launch may take an indefinite amount of time to
                        # launch and register itself, the subsequent checks may
                        # fail if executed immediately.
                        #
                        # Getting all the node names via
                        # rosnode.get_node_names() purely for
                        # the purpose of adding a slight undefined delay to
                        # allow for proper
                        # registration. There is a suspicion that it registers
                        # all pending
                        # registration before proceeding on. I thought having
                        # it in a
                        # refreshing loop would suffice, but no.
                        #
                        # Update: I added another sleep as a necessary time
                        # buffer.
                        running_nodes = rosnode.get_node_names()
                        log.debug(f"Current running nodes: {running_nodes}")
                        await asyncio.sleep(timeout)
                        api = rosnode.get_api_uri(self.master, node_name)
                        node = ServerProxy(api)
                        pid = rosnode._succeed(node.getPid('/rosnode'))
                        pid_not_retrieved = False
                    except Exception as err:
                        log.error(f"Error getting ROSLaunch PID: {err}")
                        continue
                pid_list.append(pid)

            name = launch_file + "_" + str(pid_list[0])
            await self.edit_running("append", self.running_launches, str(name),\
                (p, pid_list))
            log.debug(f"Appending {name}=({p}, {pid_list})")
            launch_success = True

        except Exception as err:
            log.error(f"Exception: {err} launching files")
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
            await self.edit_running("append", self.running_nodes, str(name), \
                (p.pid, launch_runner))
        except Exception as err:
            log.error("Exception: {err} launching node")
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
            await self.edit_running("append", self.running_execs, str(name), \
                (ps.pid))
        except Exception as err:
            launch_success = False
            log.error(f"Exception {err} launching exec")
            msg = str(err)

        log.debug(f"Name: {name}, Launch Success: {launch_success}, Msg: {msg}")
        return name, launch_success, msg


def main() -> None:
    rospy.init_node("task_master", disable_signals=True)
    tm = TaskMaster()
    rospy.spin()


if __name__ == "__main__":
    main()
