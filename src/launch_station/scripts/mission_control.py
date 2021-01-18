import logging
import sys
import json
import signal

import time
import argparse
import asyncio
from communicator import AMQPHandler
from task_master import TaskROSter
from db import Mongo

class MissionControl():
    """
    Control for dispatching tasks. Has a communication layer through self.comm.
    A task dispatcher through self.tm. Has a DB to help keep track of task states and configs through self.db
    """
    def __init__(self):
        """
        ain't it init
        """
        self.comm = AMQPHandler()
        self.tm = TaskROSter()
        self.db = Mongo()

    async def tm_hb_pub(self, msg):
        if self.tm.connected:
            print ("-c-")
            print ("tm check pass")
            self.tm.pub_connections(json.dumps(msg))
            print ("-d-")

    async def comm_hb_pub(self, msg):
        if self.comm.connected:
            print ("-a-")
            await self.comm.publish(
                routing_key = self.comm.syscheck_key,
                msg = msg,
                msg_prep = self.msg_to_json
            )
            print ("-b-")

    async def systemCheck(self, hb_interval):
        """
        Publishes a Heartbeat periodically. Heartbeat is loaded with the connection
        status of the communication, task and database layer to provide
        secondary decision making.

        @param hb_interval: Interval per heartbeat loop. Not the same as a timeout.
        Strict attempts to enforce hb_interval by enforcing timeouts in processes
        and reducing sleep time with time taken.
        @param topic: Topic for the communication layer to publish to.
        """
        # TODO: Handle Shutdown
        self.shutdown = False
        try:
            # Unsure how to properly measure the timeout as there are multiple
            # asynchronous publishing
            if self.pub_timeout > hb_interval:
                raise AssertionError("Timeout for publishing cannot be longer than heartbeat interval")
        except AssertionError as ae:
            # TODO: do cleanup
            print (ae)

        while True:
            stopwatch_start = time.perf_counter()
            print ("whiling")
            # Update the dictionary periodically because primitive Types are
            # passed by value and not reference.
            self.connections = {
                "comm_connected": self.comm.connected,
                "tm_connected": self.tm.connected,
                "db_connected": self.db.connected
            }
            try:
                await asyncio.gather(
                        asyncio.wait_for(self.comm_hb_pub(msg = self.connections), self.pub_timeout),
                        asyncio.wait_for(self.tm_hb_pub(msg = self.connections), self.pub_timeout)
                    )
                stopwatch_stop = time.perf_counter()
                print ("syscheck no errors")
            except TimeoutError as te:
                print ("dump")
                print (te)
                stopwatch_stop = time.perf_counter()
                pass
            except ConnectionError as ce:
                print (ce)
            print ("sleep")
            await asyncio.sleep(hb_interval - (stopwatch_stop - stopwatch_start))

    def process_cancel_msg(self, json):
        msg = json.load(json)
        success, msg = tm.kill(pid = msg["PID"], timeout = msg["timeout"])
        result = {'Success': success, 'Msg': msg}
        return result

    def process_task_msg(self, json_msg):
        """
        To filter the message to know which starters to trigger.
        Filtering is based on the combination of fields that is not None.

        @param json_msg: Message to parse.
        """
        print ("Processing: ", json_msg)
        msg = json.loads(json_msg.decode("utf-8"))
        print ("loads: ", msg)
        if msg['Args'] == '':
            print ("None")
        try:
            # ROSPY API requires nodes to be started and spun in the main thread.
            if msg['Timeout'] is not None and \
                msg['Pkg'] is not None and \
                msg['Executable'] is None and \
                msg['Params'] is None and \
                msg['Args'] is not None and \
                msg['Launchfile'] is not None:
                    print ("Launch file with pkg")
                    name, success, msg = self.tm.start_launchfile(
                        pkg = msg['Pkg'],
                        launchfile = msg['Launchfile'],
                        launch_args = msg['Args'],
                        timeout = msg['Timeout']
                    )
            elif msg['Timeout'] is not None and \
                msg['Pkg'] is None and \
                msg['Executable'] is None and \
                msg['Params'] is None and \
                msg['Args'] is not None and \
                msg['Launchfile'] is not None:
                    print ("Launch file without pkg")
                    name, success, msg = self.tm.start_launchfile(
                        launchfile = msg['Launchfile'],
                        launch_args = msg['Args'],
                        timeout = msg['Timeout']
                    )
            elif msg['Timeout'] is not None and \
                msg['Pkg'] is not None and \
                msg['Executable'] is not None and \
                msg['Params'] is not None and \
                msg['Args'] is not None and \
                msg['Launchfile'] is None:
                    print ("Launch ROS Node")
                    name, success, msg = self.tm.start_ros_node(
                        pkg = msg['Pkg'],
                        executable = msg['Executable'],
                        params = msg['Params'],
                        launch_args = msg['Args'],
                        timeout = msg['Timeout']
                    )
            elif msg['Timeout'] is not None and \
                msg['Pkg'] is None and \
                msg['Executable'] is not None and \
                msg['Params'] is None and \
                msg['Args'] is not None and \
                msg['Launchfile'] is None:
                    print ("LaunchExec")
                    name, success, msg = self.tm.start_executable(
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
                    print ("LaunchExec without args")
                    name, success, msg = self.tm.start_executable(
                        executable = msg['Executable'],
                        timeout = msg['Timeout']
                    )
        except Exception as e:
            print (e)
        result = {'Name': name, 'Success': success, 'Msg': msg}
        return result


    def msg_to_json(self, msg):
        json_msg = json.dumps(msg).encode()
        print ("Message: ", json_msg)
        return json_msg

    def terminate(signal, frame):
        #TODO: Do proper cleanup
        print ("Hello World")
        sys.exit()


async def main() -> None:
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

    comm_login="%s://%s@%s" % (args.commproto, args.commauth, args.commadd)
    sys_hb_interval = args.sys_hb_interval
    mc = MissionControl()
    mc.tm.init_timeout = 0.17 # 0.1503 was the longest time in 10 runs
    mc.tm.node_name = "TaskMaster"
    mc.tm.ros_topic = "/taskmaster"
    mc.pub_timeout = 0.017 # 0.0143 was the longest time in many runs

    signal.signal(signal.SIGINT, mc.terminate)

    if args.commproto == "amqp":
        mc.comm.amqp_ex = args.amqp_ex
        mc.comm.login = comm_login
        # Longest time is 0.159 in 20 runs.
        mc.comm.connection_robust_timeout = 0.18
        mc.comm.connection_timeout = 0.2
        amqp_ex = args.amqp_ex
        amqp_cancel_key = args.amqp_cancel_key
        amqp_cancel_reply_key = args.amqp_cancel_reply_key
        amqp_launch_key = args.amqp_launch_key
        amqp_launch_reply_key = args.amqp_launch_reply_key
        mc.comm.syscheck_key = args.amqp_sys_key

        await asyncio.gather(
            # TODO: implement timeout with asyncio.wait_for
            mc.comm.healthCheck(
                hb_interval = sys_hb_interval
            ),
            mc.tm.healthCheck(
                hb_interval = sys_hb_interval
            ),
            mc.systemCheck(
                hb_interval = sys_hb_interval
            ),
            mc.comm.subscribe(
                routing_key = amqp_cancel_key,
                msg_proc_func = mc.process_cancel_msg,
                reply_routing_key = amqp_cancel_reply_key,
                reply_prep = mc.msg_to_json
            ),
            mc.comm.subscribe(
                routing_key = amqp_launch_key,
                msg_proc_func = mc.process_task_msg,
                reply_routing_key = amqp_launch_reply_key,
                reply_prep = mc.msg_to_json
            )
        )

if __name__ == "__main__":
    asyncio.run(main())
    #LaunchStation()
