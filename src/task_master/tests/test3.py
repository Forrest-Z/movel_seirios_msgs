import rospy, rosnode, roslaunch, time
from roslaunch import roslaunch_logs
import logging
import sys

from time import sleep
import argparse
import asyncio
from aio_pika import connect_robust, Message
from aio_pika.exceptions import AMQPConnectionError, ConnectionClosed, ChannelClosed

import functools
import contextvars


def test_msg_processor(msg):
    print("{}!!!!".format(msg))
    return True, msg.decode("utf-8")


class AMQPHandler:
    def __init__(self):
        print("amqp init")
        self.connected = False

    async def connect(
        self, reconnect_delay=3, amqp_connect_string="amqp://guest:guest@localhost:5672"
    ):
        print("amqp attempt")
        print(amqp_connect_string)
        # While look because the alternative re-connect attempt is recursive,
        # which sort of has a limit of 997 attempts. Wanted indefinite retries.
        # This blocks the thread indefinitely.
        while not self.connected:
            print("rabbit")
            try:
                self.amqp_connect_string = amqp_connect_string
                # TODO: Add timeout for connect_robust and channel creation
                self.connection = await connect_robust(self.amqp_connect_string)
                self.channel = await self.connection.channel()
                self.connected = True
                print("rabbit connected")
                return self.connected
            except Exception as exc:
                await asyncio.sleep(reconnect_delay)

    async def close(self):
        print("hi")
        await self.connection.close()

    async def send(self, amqp_exchange, amqp_queue, msg):
        print("send")
        exchange = await self.channel.declare_exchange(amqp_exchange, auto_delete=False)
        queue = await self.channel.declare_queue(amqp_queue, auto_delete=False)
        routing_key = amqp_queue
        await queue.bind(exchange, routing_key)
        await exchange.publish(
            Message(
                # body=json.dumps(msg).encode(),
                bytes(msg, "utf-8")
            ),
            routing_key,
        )

    async def receive(
        self,
        amqp_exchange,
        amqp_queue,
        msg_proc_func=None,
        awaitable_msg_proc_func=None,
        redirect_to_exchange=None,
        redirect_to_queue=None,
    ):
        print("receive")
        routing_key = amqp_queue
        exchange = await self.channel.declare_exchange(amqp_exchange, auto_delete=False)
        queue = await self.channel.declare_queue(amqp_queue, auto_delete=False)
        await queue.bind(exchange, routing_key)

        async for message in queue:
            mpf = msg_proc_func

            if awaitable_msg_proc_func is not None:
                mpf = awaitable_msg_proc_func
                proc_status, proc_result = await mpf(message.body)
            else:
                proc_status, proc_result = mpf(message.body)

            if (redirect_to_exchange != None) and (redirect_to_queue != None):
                await self.send(redirect_to_exchange, redirect_to_queue, proc_result)

            if proc_status == True:
                print("ack")
                message.ack()


class TaskMaster:
    def __init__(self):
        self.connected = False
        print("TM init")

    def pub_connections(self):
        print("badum")


class DB:
    def __init__(self):
        self.connected = True


class MissionControl:
    def __init__(self):
        self.comm = AMQPHandler()
        self.tm = TaskMaster()
        self.db = DB()
        self.connections = {
            "comm_connected": self.comm.connected,
            "tm_connected": self.tm.connected,
            "db_connected": self.db.connected,
        }

    # To be launch approx at the same time any attempts of service connections
    # are made. Purpose to periodically send a status on all the sevices to all
    # the services.
    async def systemCheck(self, hb_interval=1):
        self.shutdown = False
        print("whiling")
        while True:
            try:
                if self.comm.connected:
                    print("hi")
                    # HERE
                    await self.comm.send("test_ex", "test_queue", "self.connections")
                if self.tm.connected:
                    print("hi2")
                    # self.tm.pub()
                print("sleep")
            except Exception as e:
                print("dump")
                print(e)
            await asyncio.sleep(hb_interval)

    async def listen(self, amqp_exchange, amqp_queue, msg_proc_func):
        while True:
            try:
                print ("morning")
                await self.comm.receive(amqp_exchange, amqp_queue, msg_proc_func)
            except:
                print ("night")
            await asyncio.sleep(0.001)

    # sync def

    # def bringupConnections(self, reconnect_delay, commlogin):
    # TODO: Add both general and individual timeout (i.e, timeout per
    # service connection attempt or all the attempts as a whole.


def test_msg_processor(msg):
    print("{}!!!!".format(msg))
    return True, msg.decode("utf-8")


async def to_thread(func, /, *args, **kwargs):
    """Asynchronously run function *func* in a separate thread.
    Any *args and **kwargs supplied for this function are directly passed
    to *func*. Also, the current :class:`contextvars.Context` is propogated,
    allowing context variables from the main thread to be accessed in the
    separate thread.
    Return a coroutine that can be awaited to get the eventual result of *func*.
    """
    loop = asyncio.get_running_loop()
    ctx = contextvars.copy_context()
    func_call = functools.partial(ctx.run, func, *args, **kwargs)
    return await loop.run_in_executor(None, func_call)


async def amain() -> None:
    parser = argparse.ArgumentParser(description="Mission Control for task management.")
    parser.add_argument("-V", "--version", action="version", version="0.1")
    parser.add_argument(
        "--commauth",
        default="guest:guest",
        type=str,
        metavar="guest:guest",
        help="Authentication credentials for communication server.",
    )
    parser.add_argument(
        "--commadd",
        default="localhost:5672",
        metavar="localhost:5672",
        type=str,
        help="Address for communication server.",
    )
    parser.add_argument(
        "--commproto",
        default="amqp",
        choices=["amqp"],
        type=str,
        help="Communication protocol.",
    )
    parser.add_argument(
        "--reconnect_delay",
        default=3,
        type=float,
        help="Attempt reconnect after delay in secs.",
    )
    parser.add_argument(
        "--sys_hb_interval",
        default=3,
        type=float,
        help="Interval for system heartbeat in secs.",
    )
    args = parser.parse_args()

    commlogin = "%s://%s@%s" % (args.commproto, args.commauth, args.commadd)
    reconnect_delay = args.reconnect_delay
    sys_hb_interval = args.sys_hb_interval

    mc = MissionControl()

    await asyncio.gather(
        mc.comm.connect(reconnect_delay, commlogin),
        mc.systemCheck(sys_hb_interval),
        mc.listen("test_ex", "test_queue", test_msg_processor)
#        mc.comm.receive("test_ex", "test_queue", test_msg_processor)
    )


def main():
    return asyncio.run(amain())


if __name__ == "__main__":
    sys.exit(main())
