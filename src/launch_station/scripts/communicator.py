import asyncio
import time
import sys

from aio_pika import connect_robust, Message
from aio_pika.exceptions import AMQPConnectionError, ConnectionClosed, \
                                ChannelClosed, ChannelInvalidStateError, \
                                ChannelNotFoundEntity


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
            # Cause reset is needed.
            # TODO: Implement kill all

        while True:
            stopwatch_start = time.perf_counter()
            try:
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
                    print ("TimeoutError in healthcheck")
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
                # TODO: add_close_callback()
                self.connection = await connect_robust(self.amqp_connect_string,
                        timeout=self.connection_robust_timeout)
                self.channel = await self.connection.channel(
                        publisher_confirms = True)
                # Weird bug when checking for an exchange before declaring one.
                # Unable to solve, but in a one exchange design, it's okay to
                # make the exchange a class member. If design expands to include
                # multiple exchanges or connection pools, a redesign is needed.
                self.exchange = await self.channel.declare_exchange(self.amqp_ex, auto_delete=False)
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


    async def publish(self, routing_key, msg, msg_prep):
        """
        msg_prep processes and formats the msg before sending it to routing_key.

        @param routing_key: Queue address for msg.
        @param msg: Message.
        @param msg_prep: Preperation function to ensure correct format.
        """
        print ("send")
        # Same error behavior with get_queue before declare_queue.
        queue = await self.channel.declare_queue(routing_key, auto_delete=False)
        print ("1")
        await queue.bind(self.exchange, routing_key)
        print ("2")
        await self.exchange.publish(
            Message(
                body=msg_prep(msg)
            ),
            routing_key
        )
        print ("Send successfully")


    async def subscribe(self, routing_key, msg_proc_func=None, async_msg_proc_func=None, reply_routing_key=None, reply_prep=None):
        """
        Provisions a queue to connect to an exchange(set by publishers) and
        awaits for messages.
        Should multiple queues be required, more asynchronous instances are
        required. Upon receipt of a message, message will be processed by the
        custom appointed function.

        The result of the message processing can then be published if the
        required reply addresses and mechanisms are provided.


        @param routing_key: Routing key identifier for the queue.

        @param msg_proc_func: Custom synchronnous processing for messages.

        @param async_msg_proc_func: Custom asynchronous processing.

        @param reply_routing_key: Routing Key identifier for replies.

        @param reply_prep: Prepare the reply in a format agreed upon.
        """

        print ("while loop")
        while True:
            try:
                print ("*b*")
                # TODO: Properly set queues
                print ("*c*")
                queue = await self.channel.declare_queue(routing_key, auto_delete=False)
                print ("*d*")
                await queue.bind(self.exchange, routing_key)
                print ("*e*")
            except ChannelClosed as cc:
                print ("Channel closed")
                print (cc)
            except AttributeError as ae:
                print ("no Channel")
                print(ae)

            print ("subsc")
            try:
                # TODO: Implement workers to dispatch tasks faster.
                async for message in queue:
                    if async_msg_proc_func is not None:
                        process_msg = async_msg_proc_func
                        # TIMEOUT AND EXCEPTION CATCHER
                        result = await process_msg(message.body)
                    else:
                        process_msg = msg_proc_func
                        # TIMEOUT AND EXCEPTION CATCHER
                        print ("PROCESS")
                        result = process_msg(message.body)
                        print (result)

                    # Message acknowledgement is only a receipt, not validation
                    print ("ack")
                    message.ack()
                    if reply_routing_key is not None and \
                    reply_prep is not None:
                        # TIMEOUT AND EXCEPTION CATCHER
                        await self.publish(
                            routing_key = reply_routing_key,
                            msg = result,
                            msg_prep = reply_prep
                        )
            except Exception as e:
                print ("Where error:", e)
                pass
            print ("SUB sleeper 2")
            await asyncio.sleep(1.0)
