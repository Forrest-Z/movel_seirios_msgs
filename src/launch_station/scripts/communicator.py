import asyncio
import time
import sys

from aio_pika import connect_robust, Message
from aio_pika.exceptions import AMQPConnectionError, ConnectionClosed, \
                                ChannelClosed

class AMQPHandler():
    """
    AMQPHandler is to handle the common functions required of RabbitMQ

    The basic functionality is subscribing to messages, send messages
    and a simple healthcheck
    """
    def __init__(self):
        """
        ain't it init
        """
        self.connected = False
        self.connnection = None
        self.prev_hb_time = 0.0
        self.comm_login = None
        self.amqp_ex = None

    async def healthCheck(self, hb_interval):
        """
        Updates the connection status of AMQPHandler.
        Attempts to connect if not connected. Used as an initial connect attempt
        as well as a secondary reconnection mechanism. Never to be returned.

        Timers are added to keep the heartbeat interval strict.

        @param hb_interval: Interval per heartbeat loop. Not the same as a timeout.
        Strict attempts to enforce hb_interval by enforcing timeouts in processes
        and reducing sleep time with time taken.
        """
        print ("Comm healthcheck")
        try:
            if self.connection_timeout > hb_interval:
                raise AssertionError("Connection timeout cannot be longer than heartbeat interval")
        except AssertionError as ae:
            # TODO: do cleanup
            print (ae)
            print (self.connection_timeout)
            print (hb_interval)
            sys.exit()

        while True:
            print ("Comm healthcheck1")
            stopwatch_start = time.perf_counter()
            try:
                print ("Comm healthcheck2")
                self.hb_time = self.connection.heartbeat_last
                print ("HB: ", self.hb_time)
                self.connected = True
                stopwatch_stop = time.perf_counter()
            except AttributeError as ae:
                print ("triggering Attempt comm connection")
                print (ae)
                self.connected = False
                # Backup mechanism to reconnect for when the connect_robust() fails
                # in ensuring reconnects.
                try:
                    await asyncio.wait_for(self.connect(self.login), self.connection_timeout)
                except TimeoutError as te:
                    print (te)
                    pass
                stopwatch_stop = time.perf_counter()
            print ("comm healthcheck sleep")
            print (stopwatch_stop-stopwatch_start)
            await asyncio.sleep(hb_interval - (stopwatch_stop - stopwatch_start))

    async def connect(self,
            amqp_connect_string="amqp://guest:guest@localhost:5672"):
        """
        Connects to a running RabbitMQ instance. By default, connection_robust()
        is used to enable reconnection by default

        @param amqp_connect_string: String that contains the login credentials,
        address and port of the RabbitMQ server
        """
        print ("amqp attempt")
        if not self.connected:
            try:
                self.amqp_connect_string = amqp_connect_string
                self.connection = await connect_robust(self.amqp_connect_string,
                        timeout=self.connection_robust_timeout)
                self.channel = await self.connection.channel()
                self.connected = True
                print("rabbit connected")
            except ConnectionError as ce:
                self.connected = False
                print("rabbit not connected")
                print (ce)
            except TimeoutError as te:
                self.connected = False
                print ("rabbit lost the race")
                print (ce)
            except AMQPConnectionError as ace:
                self.connected = False
                print ("Wrong Bunny")
                print (ace)

    async def close(self):
        print("bye")
        await self.connection.close()

    async def publish(self, routing_key, msg, encoder):
        print ("send")
        exchange = await self.channel.declare_exchange(self.amqp_ex, auto_delete=False)
        queue = await self.channel.declare_queue(routing_key, auto_delete=False)
        await queue.bind(exchange, routing_key)
        await exchange.publish(
            Message(
                body=encoder(msg)
            ),
            routing_key
        )
        print ("Send successfully")

    async def subscribe(self, routing_key, msg_proc_func=None, awaitable_msg_proc_func=None, reply_routing_key=None, reply_encoder=None):
        print ("while loop")
        while True:
            try:
                print ("receive")
                exchange = await self.channel.declare_exchange(self.amqp_ex, auto_delete=False)
                print ("2")
                queue = await self.channel.declare_queue(routing_key, auto_delete=False)
                print ("3")
                await queue.bind(exchange, routing_key)
                print ("4")

                async for message in queue:
                    print ("\n#######\nRECEIVE\n########")

                    if awaitable_msg_proc_func is not None:
                        print ("5")
                        process_msg = awaitable_msg_proc_func
                        print ("6")
                        result = await process_msg(message.body)
                        print ("7")
                    else:
                        print ("8")
                        process_msg = msg_proc_func
                        print ("9")
                        result = process_msg(message.body)
                        print (result)
                        print ("10")

                    print ("proc pass")
                    print (result)
                    print ("ack")
                    message.ack()
                    if reply_routing_key is not None and \
                    reply_encoder is not None:
                        await self.publish(
                            routing_key = reply_routing_key,
                            msg = result,
                            encoder = reply_encoder
                        )
            except Exception as e:
                print ("cant hear")
            await asyncio.sleep(1.0)
        print ("Outside while")
