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
        self.connections = {
            "comm_connected": self.comm.connected,
            "tm_connected": self.tm.connected,
            "db_connected": self.db.connected
        }

    async def systemCheck(self, hb_interval, amqp_exchange,
            amqp_pub_syscheck_route):
        self.shutdown = False
        while True:
            print ("whiling")
            try:
                if self.comm.connected:
                    print("hi")
                    json_msg = self.msg_to_json(self.connections)
                    await self.comm.send(amqp_exchange,
                            amqp_pub_syscheck_route, json_msg)
                if self.tm.connected:
                    print ("hi2")
                    #self.tm.pub()
                print ("sleep")
            except Exception as e:
                print ("dump")
                print (e)
            await asyncio.sleep(hb_interval)

    async def listen(self, amqp_exchange, amqp_queue, msg_proc_func):
        while True:
            try:
                await self.comm.receive(amqp_exchange, amqp_queue, msg_proc_func)
                print ("morning")
            except:
                print ("night")
            await asyncio.sleep(0.001)

    def process_msg(self, json):
        msg = json.load(json)
        return msg

    def msg_to_json(self, msg):
        json_msg = json.dumps(msg)
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
        '--amqp_consume_route',
        default="robot_q",
        type=str,
        help="Route Name for AMQP protocol. Only one route is supported at the \
        moment. Additional route means another msg processing is required."
    )
    parser.add_argument(
        '--amqp_pub_syscheck_route',
        default="robot_ex",
        type=str,
        help="Route Name for publishing system check on the AMQP protocol."
    )
    args = parser.parse_args()

    commlogin="%s://%s@%s" % (args.commproto, args.commauth, args.commadd)
    reconnect_delay = args.reconnect_delay
    sys_hb_interval = args.sys_hb_interval
    amqp_ex = args.amqp_ex
    amqp_consume_route = args.amqp_consume_route
    amqp_pub_syscheck_route = args.amqp_pub_syscheck_route

    mc = MissionControl()

    await asyncio.gather(
            # TODO: implement timeout with asyncio.wait_for
        mc.comm.connect(reconnect_delay, commlogin),
        mc.systemCheck(sys_hb_interval, amqp_ex, amqp_pub_syscheck_route),
        mc.listen(amqp_ex, amqp_consume_route, mc.process_msg)
    )

def main():
    return asyncio.run(amain())

if __name__ == "__main__":
    sys.exit(main())
    #LaunchStation()
