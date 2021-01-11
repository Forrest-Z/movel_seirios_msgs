import logging
import sys
import json

from time import sleep
import argparse
import asyncio
from communicator import AMQPHandler
from task_master import RosTask
from db import Mongo

class MissionControl():
    def __init__(self):
        self.comm = AMQPHandler()
        self.tm = RosTask()
        self.db = Mongo()

    async def systemCheck(self, hb_interval, topic, login):
        self.shutdown = False
        while True:
            print ("whiling")
            self.connections = {
                "comm_connected": self.comm.connected,
                "tm_connected": self.tm.connected,
                "db_connected": self.db.connected
            }

            try:
                if self.comm.connected:
                    await self.comm.publish(
                        topic,
                        msg = self.connections,
                        encoder = self.msg_to_json
                    )
                if self.tm.connected:
                    print ("tm check pass")
                    self.tm.pub_connections(self.connections)
                print ("sleep")
            except Exception as e:
                print ("dump")
                print (e)
            await asyncio.sleep(hb_interval)

    def process_cancel_msg(self, json):
        msg = json.load(json)
        success, msg = tm.kill(pid = msg["PID"], timeout = msg["timeout"])
        result = {'Success': success, 'Msg': msg}
        return result

    def process_launch_msg(self, json):
        msg = json.load(json)
        if msg['Pkg'] is not None and \
            msg['Lauchfile'] is not None and \
            msg['Timeout'] is not None and \
            msg['Type'] is None and \
            msg['Params'] is None and \
            msg['Args'] is None and \
            msg['Launchfile'] is None:
            name, pid, success, msg = tm.start_launchfile(
                pkg = msg['Pkg'],
                launchfile = msg['Launchfile'],
                timeout = msg['Timeout']
            )
        elif msg['Pkg'] is not None and \
            msg['Timeout'] is not None and \
            msg['Type'] is not None and \
            msg['Params'] is not None and \
            msg['Lauchfile'] is None and \
            msg['Args'] is None and \
            msg['Launchfile'] is None:
            name, pid, success, msg = tm.start_ros_obj(
                pkg = msg['Pkg'],
                executable = msg['Executable'],
                param = msg['Param'],
                timeout = msg['Timeout']
            )
        elif msg['Pkg'] is None and \
            msg['Timeout'] is not None and \
            msg['Type'] is not None and \
            msg['Params'] is None and \
            msg['Lauchfile'] is None and \
            msg['Args'] is not None and \
            msg['Launchfile'] is None:
            name, pid, success, msg = tm.start_executable(
                executable = msg['Executable'],
                args = msg['Args'],
                timeout = msg['Timeout'],
            )
        #else:
        #    return name=None, pid=-1, Success=False, msg="Unrecognized msg"
        result = {'Name': name, 'PID': pid, 'Success': success, 'Msg': msg}
        return result

    def msg_to_json(self, msg):
        json_msg = json.dumps(msg).encode()
        print ("Message: ", json_msg)
        return json_msg

async def amain() -> None:
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
        '--reconnect_delay',
        default=3,
        type=float,
        help="Attempt reconnect after delay in secs."
    )
    parser.add_argument(
        '--sys_hb_interval',
        default=3,
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
    reconnect_delay = args.reconnect_delay
    sys_hb_interval = args.sys_hb_interval
    mc = MissionControl()

    if args.commproto == "amqp":
        mc.comm.amqp_ex = args.amqp_ex
        amqp_ex = args.amqp_ex
        amqp_cancel_key = args.amqp_cancel_key
        amqp_cancel_reply_key = args.amqp_cancel_reply_key
        amqp_launch_key = args.amqp_launch_key
        amqp_launch_reply_key = args.amqp_launch_reply_key
        amqp_sys_key = args.amqp_sys_key

        await asyncio.gather(
                # TODO: implement timeout with asyncio.wait_for
            mc.comm.healthCheck(
                hb_interval = sys_hb_interval/5,
                reconnect_delay = reconnect_delay,
                comm_login = comm_login
            ),
            mc.tm.healthCheck(
                hb_interval = sys_hb_interval/5,
                reconnect_delay = reconnect_delay
            ),
            mc.systemCheck(
                hb_interval = sys_hb_interval,
                topic = amqp_sys_key,
                login = comm_login
            ),
            mc.comm.subscribe(
                routing_key = amqp_cancel_key,
                msg_proc_func = mc.process_cancel_msg,
                reply_routing_key = amqp_cancel_reply_key,
                reply_encoder = mc.msg_to_json
            ),
            mc.comm.subscribe(
                routing_key = amqp_launch_key,
                msg_proc_func = mc.process_launch_msg,
                reply_routing_key = amqp_launch_reply_key,
                reply_encoder = mc.msg_to_json
            )
        )

def main():
    return asyncio.run(amain())

if __name__ == "__main__":
    sys.exit(main())
    #LaunchStation()
