import logging
import logging.config
import json
import signal
import time
import sys

import argparse
import asyncio
from communicator import AMQPHandler
from task_master import TaskROSter
from db import Mongo

logging.config.fileConfig('logging.conf', disable_existing_loggers=False)
log = logging.getLogger(name="mcLogger")

class MissionControl():
    """
    Control for dispatching tasks. Has a communication layer through self.comm.
    A task dispatcher through self.tm. Has a DB to help keep track of task states and configs through self.db
    """
    def __init__(self):
        """
        The relevant components are initialized.
        """
        self.comm = AMQPHandler()
        self.tm = TaskROSter()
        self.db = Mongo()


    def tm_death_pub(self, orbituary_list):
        """
        Triggers a publishing of dead processes on the TM.

        @param [list]: List of dead nodes with time since death less than
            orbituary_timeout
        """
        if self.tm.connected:
            result = self.tm.pub_dead(json.dumps(orbituary_list))
            print ('tm death: ', result)
            return result



    async def comm_death_pub(self, orbituary_list):
        """
        Triggers a publishing of dead processes on the communicator.

        @param [list]: List of dead nodes with time since death less than
            orbituary_timeout
        """
        if self.comm.connected:
            result = await self.comm.publish(
                routing_key = self.comm.sysorbituary_key,
                msg = orbituary_list,
                msg_prep = self.msg_to_json
            )
            print ("comm death: ", result)
            return result


    def tm_hb_pub(self, msg):
        """
        Triggers a publishing of connected services on the TM.

        @param msg [list]: List of connected services.
        """
        if self.tm.connected:
            result = self.tm.pub_connections(json.dumps(msg))
            return result


    async def comm_hb_pub(self, msg):
        """
        Triggers a publishing of connected services on the communicator

        @param msg [list]: List of connected services.
        """
        if self.comm.connected:
            result = await self.comm.publish(
                routing_key = self.comm.syscheck_key,
                msg = msg,
                msg_prep = self.msg_to_json
            )
            return result


    async def systemCheck(self, hb_interval, final=None):
        """
        Publishes a Heartbeat periodically. Heartbeat is loaded with the
        connection status of the communication, task and database layer to
        provide secondary decision making.

        @param hb_interval [float]: Interval per heartbeat loop. Not the same as a
        timeout. Strict attempts to enforce hb_interval by enforcing timeouts
        in processes and reducing sleep time with time taken.
        """
        code = None
        try:
            # Unsure how to properly measure the timeout as there are multiple
            # asynchronous publishing
            if self.pub_timeout > hb_interval:
                raise AssertionError("Connection timeout of {self.pub_timeout} \
                    is longer than heartbeat interval of {hb_interval}")
        except AssertionError as err:
            log.error(err)
            return

        while True:
            stopwatch_start = time.perf_counter()
            # Update the dictionary periodically because primitive Types are
            # passed by value and not reference.
            self.connections = {
                "comm_connected": self.comm.connected,
                "tm_connected": self.tm.connected,
                "db_connected": self.db.connected
            }
            # Trigger a cleanup on the orbituary list and save a local copy to
            # prevent changes.
            orbituary_list = await self.tm.edit_orbituary(mode="update")
            try:
                # Time sensitive, can't add to queue and hope it gets pub soon.
                await asyncio.gather(
                    self.comm_hb_pub(msg = self.connections),
                    asyncio.wait_for(self.tm_hb_pub(msg = self.connections),
                        self.pub_timeout),
                    return_exceptions=True,
                )
                if len(orbituary_list.keys()) > 0:
                    print ("walaueh")
                    code = await asyncio.gather(
                        self.comm_death_pub(orbituary_list),
                        asyncio.wait_for(self.tm_death_pub(orbituary_list),
                            self.pub_timeout),
                        return_exceptions=True,
                    )
                    for i in range(len(code)):
                        print ("code: ", type(code[i]))
            except TimeoutError as err:
                log.error(f"System Healthcheck error: {err}")
            except ConnectionError as err:
                log.error(f"System Healthcheck error: {err}")
            except Exception as exc:
                print ("Weird: ", exc)
            stopwatch_stop = time.perf_counter()
            log.debug(f"Healthcheck loop for MissionControl took \
                {stopwatch_stop-stopwatch_start}")
            if final == True:
                print ("final: ", code[0])
                return code[0]
            await asyncio.sleep(hb_interval-(stopwatch_stop-stopwatch_start))


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
                signal.raise_signal(15)
        else:
            name = cancel_msg["Name"]
            timeout = cancel_msg["Timeout"]
            if name in self.tm.running_launches.keys():
                success, msg = await asyncio.wait_for(self.tm.clean_launch(
                    clean=name), timeout)
            elif name in self.tm.running_nodes.keys():
                success, msg = await asyncio.wait_for(self.tm.clean_node(
                    clean=name), timeout)
            elif name in self.tm.running_execs.keys():
                success, msg = await asyncio.wait_for(self.tm.clean_exec(
                    clean=name), timeout)
            else:
                success = False
                msg = "Task not found"
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


    def handle_exception(self, loop, context):
        """
        To handle all uncaught exceptions throughout MissionControl.
        Prints the context of the exceptions before triggering a shutdown.

        @param loop [loop]: current asyncio loop
        @param context [Exception context]: Context of Exception to print, log.
        """
        msg = context.get("exception", context["message"])
        log.error(f"Caught exception: {msg}")
        log.info(f"Shutting down...")
        asyncio.create_task(shutdown(loop))


    async def shutdown(self, loop, signal=None):
        """
        Cleanup tasks tied to the service's shutdown. Cleans TM's tasks first,
        publishes one the orbituary list one last time. Before killing all
        tasks and events in the event loop.

        @param loop [loop]: Loop whose events and tasks are to be stopped.
        """
        log.info("Shutdown sequence triggered")
        if signal:
            log.debug(f"Received shutdown signal {signal.name}...")

        stop_result = await self.tm.async_stop()
        pub_result = await self.systemCheck(self.sys_hb_interval, final = True)
        print ("Shutdown: ", stop_result, pub_result)
        tasks = [t for t in asyncio.all_tasks() if t is not
                 asyncio.current_task()]

        log.info(f"Cancelling {len(tasks)} outstanding tasks")
        for task in tasks:
            try:
                log.debug(f"Cancelling {task}")
                task.cancel()
            except asyncio.CancelledError as ce:
                log.error(f"Cancellation Error with {task}")
                continue
        loop.stop()


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Mission Control for task management.')
    parser.add_argument('-V', '--version', action='version', version='0.1')
    parser.add_argument(
        '--commauth',
        default='guest:guest',
        metavar='guest:guest',
        type=str,
        help="Authentication credentials for communication server."
    )
    parser.add_argument(
        '--commadd',
        default='localhost:5672',
        metavar='localhost:5672',
        type=str,
        help="Address for communication server."
    )
    parser.add_argument(
        '--commproto',
        default='amqp',
        choices=['amqp'],
        type=str,
        help="Communication protocol."
    )
    parser.add_argument(
        '--sys_hb_interval',
        default=1,
        type=float,
        help="Interval for system heartbeat in secs."
    )
    parser.add_argument(
        '--amqp_ex',
        default="robot_ex",
        type=str,
        help="Exchange Name for AMQP protocol."
    )
    parser.add_argument(
        '--amqp_cancel_key',
        default="robot_cancel",
        type=str,
        help="Set routing key of queue for cancelling tasks on the AMQP protocol."
    )
    parser.add_argument(
        '--amqp_cancel_reply_key',
        default="robot_cancel_reply",
        type=str,
        help="Route Name for cancelling tasks on the AMQP protocol."
    )
    parser.add_argument(
        '--amqp_launch_key',
        default="robot_launch",
        type=str,
        help="Route Name for launching tasks on the AMQP protocol.")
    parser.add_argument(
        '--amqp_launch_reply_key',
        default="robot_launch_reply",
        type=str,
        help="Route Name for launching tasks on the AMQP protocol.")
    parser.add_argument(
        '--amqp_sys_key',
        default="robot_sys",
        type=str,
        help="Route Name for publishing system check on the AMQP protocol."
    )
    args = parser.parse_args()
    log.info("Arguments parsed")

    comm_login="%s://%s@%s" % (args.commproto, args.commauth, args.commadd)
    sys_hb_interval = args.sys_hb_interval

    mc = MissionControl()
    loop = asyncio.get_event_loop()
    log.debug("Setting up Signal Handler")
    # May want to catch other signals too
    signals = (signal.SIGHUP, signal.SIGTERM, signal.SIGINT)
    for s in signals:
        loop.add_signal_handler(
            s, lambda s=s: asyncio.create_task(mc.shutdown(loop, signal=s)))
    # comment out the line below to see how unhandled exceptions behave
    log.debug("Setting up Exception Handler")
    loop.set_exception_handler(mc.handle_exception)
    queue = asyncio.Queue()

    mc.tm.init_timeout = 0.17 # 0.1503 was the longest time in 10 runs
    mc.tm.node_name = "TaskMaster"
    mc.tm.conn_topic = "/taskmaster"
    mc.tm.orbituary_topic = "/orbituary"
    mc.pub_timeout = 0.018 # 0.0143 was the longest time in many runs

    if args.commproto == "amqp":
        log.debug("AMQP Protocol selected for communication layer")
        mc.comm.amqp_ex = args.amqp_ex
        mc.comm.login = comm_login
        # Longest time is 0.159 in 20 runs.
        mc.comm.connection_robust_timeout = 0.18
        mc.comm.connection_timeout = 0.21
        amqp_ex = args.amqp_ex
        amqp_cancel_key = args.amqp_cancel_key
        amqp_cancel_reply_key = args.amqp_cancel_reply_key
        amqp_launch_key = args.amqp_launch_key
        amqp_launch_reply_key = args.amqp_launch_reply_key
        mc.comm.syscheck_key = args.amqp_sys_key
        mc.comm.sysorbituary_key = "robot_orbituary"
        mc.sys_hb_interval = args.sys_hb_interval
        mc.comm.sys_hb_interval = args.sys_hb_interval
        mc.tm.orbituary_timeout = 10 # in secs

        log.debug("Finished loading default parameters")
        try:
            loop.create_task(
                mc.comm.healthCheck(
                    hb_interval = sys_hb_interval
                    )
            )
            loop.create_task(
                mc.tm.healthCheck(
                    hb_interval = sys_hb_interval)
            )
            loop.create_task(
                mc.tm.watchdog(
                    hb_interval = sys_hb_interval)
            )
            loop.create_task(
                mc.systemCheck(
                    hb_interval = sys_hb_interval)
            )
            loop.create_task(
                mc.comm.subscribe(
                    routing_key = amqp_cancel_key,
                    async_msg_proc_func = mc.process_cancel_msg,
                    reply_routing_key = amqp_cancel_reply_key,
                    reply_prep = mc.msg_to_json
                )
            )
            loop.create_task(
                mc.comm.subscribe(
                    routing_key = amqp_launch_key,
                    async_msg_proc_func = mc.process_task_msg,
                    reply_routing_key = amqp_launch_reply_key,
                    reply_prep = mc.msg_to_json
                )
            )
            loop.run_forever()
        finally:
            loop.close()

if __name__ == "__main__":
    main()
