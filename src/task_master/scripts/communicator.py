import asyncio
import time
import sys
import logging

from aio_pika import connect_robust, Message
from aio_pika.exceptions import AMQPConnectionError, ConnectionClosed, \
                                ChannelClosed, ChannelInvalidStateError, \
                                ChannelNotFoundEntity

log = logging.getLogger(name="mcLogger")

class AMQPHandler(object):
    """
    AMQPHandler is to handle the common functions required of RabbitMQ

    The basic functionality is subscribing to messages, send messages
    and a simple healthcheck
    """
    def __init__(self, login, robot_ex, timeout):
        """
        ain't it init
        """
        log.info(f"Setting up Communicator")
        self.connected = False
        self.connnection = None
        self.login = login
        self.robot_ex = robot_ex
        self.timeout = timeout


    async def healthCheck(self, hb_interval):
        """
        Updates the connection status of AMQPHandler.
        Attempts to connect if not connected. Used as an initial connect attempt
        as well as a secondary reconnection mechanism. Never to be returned.

        Timers are added to keep the heartbeat interval strict.

        @param hb_interval: Interval per heartbeat loop. Not the same as a
        timeout. Strict attempts to enforce hb_interval by enforcing timeouts
        in processes and reducing sleep time with time taken.
        """
        try:
            if self.timeout > hb_interval:
                raise AssertionError("Communicator connection timeout of \
                    {self.timeout} is longer than heartbeat interval\
                    of {hb_interval}")
        except AssertionError as err:
            log.error(err)
            return

        while True:
            stopwatch_start = time.perf_counter()

            try:
                self.hb_time = self.connection.heartbeat_last
                self.connected = True
            except AttributeError as err:
                log.debug(f"Waiting for communicator heartbeat: {err}")
                self.connected = False
                try:
                    # Backup mechanism to reconnect for when the
                    # connect_robust() fails in ensuring reconnects.
                    await self.connect(self.login)
                except TimeoutError as err:
                    log.error(f"Trying to get communicator heartbeat: {err}")
                except Exception as exc:
                    # Likely a bug in aio-pika. A general/empty Exception will
                    # get triggered occasionally. No reactive action can be
                    # be taken as exception is empty without information.
                    log.error(f"Trying to get communicator heartbeat: {exc}")

            stopwatch_stop = time.perf_counter()
            log.debug(f"Healthcheck loop for Communicator took \
                    {stopwatch_stop-stopwatch_start}")
            #await asyncio.sleep(hb_interval-(stopwatch_stop-stopwatch_start))
            await asyncio.sleep(hb_interval)


    async def connect(self,
        amqp_connect_string="amqp://guest:guest@localhost:5672"):
        """
        Connects to a running RabbitMQ instance. By default, connection_robust()
        is used to enable reconnection by default

        @param amqp_connect_string: String that contains the login credentials,
        address and port of the RabbitMQ server
        """
        if not self.connected:
            try:
                log.debug(f"Communicator connection attempt")
                self.amqp_connect_string = amqp_connect_string
                self.connection = await connect_robust(self.amqp_connect_string,
                        timeout=self.timeout)
                self.connection.add_close_callback(self.close)
                self.channel = await self.connection.channel(
                        publisher_confirms = True)
                # Weird bug when checking for an exchange before declaring one.
                # Unable to solve, but in a one exchange design, it's okay to
                # make the exchange a class member. If design expands to include
                # multiple exchanges or connection pools, a redesign is needed.
                self.exchange = await self.channel.declare_exchange(
                    self.robot_ex)
                self.connected = True
                log.debug(f"Communicator connected")
            except ConnectionError as err:
                self.connected = False
                log.error(f"Communicator connect attempt: {err}")
            except TimeoutError as err:
                self.connected = False
                log.error(f"Communicator connect attempt: {err}")
            except AMQPConnectionError as err:
                self.connected = False
                log.error(f"Communicator connect  attempt: {err}")


    async def close(self):
        await self.connection.close()


    # Having an improper timeout for publishing leads to mis-reported exceptions
    # It is okay not to have a timeout.
    async def publish(self, routing_key, msg, msg_prep):
        """
        msg_prep processes and formats the msg before sending it to routing_key.

        @param routing_key[str]: Queue address for msg.
        @param msg[str]: Message.
        @param msg_prep[str]: Preperation function to ensure correct format.
        """
        # Same error behavior with get_queue before declare_queue.
        try:
            queue = await self.channel.declare_queue(routing_key)
            await queue.bind(self.exchange, routing_key)
            await self.exchange.publish(
                Message(
                    body=msg_prep(msg)
                ),
                routing_key
            )
            return True
        except Exception as err:
            log.error(f"While publishing: {err}")
            return False


    async def subscribe(self, routing_key, msg_proc_func=None,
        async_msg_proc_func=None, reply_routing_key=None, reply_prep=None):
        """
        Provisions a queue to connect to an exchange(set by publishers) and
        awaits for messages.
        Should multiple queues be required, more asynchronous instances are
        required. Upon receipt of a message, message will be processed by the
        custom appointed function.

        The result of the message processing can then be published if the
        required reply addresses and mechanisms are provided.

        @param routing_key[str]: Routing key identifier for the queue.
        @param msg_proc_func[str]: Custom synchronnous processing for messages.
        @param async_msg_proc_func[str]: Custom asynchronous processing.
        @param reply_routing_key[str]: Routing Key identifier for replies.
        @param reply_prep[str]: Prepare the reply in a format agreed upon.
        """
        while True:
            try:
                queue = await self.channel.declare_queue(routing_key)
                await queue.bind(self.exchange, routing_key)
                try:
                    # TODO: Implement workers to dispatch tasks faster.
                    #       Becareful about data race.
                    async for message in queue:
                        log.debug(f"Received: {message}")
                        if async_msg_proc_func is not None:
                            process_msg = async_msg_proc_func
                            # TIMEOUT AND EXCEPTION CATCHER
                            result = await process_msg(message.body)
                        else:
                            process_msg = msg_proc_func
                            # TIMEOUT AND EXCEPTION CATCHER
                            result = process_msg(message.body)

                        log.debug(f"Result from process {result}")
                        # Message acknowledgement is only a receipt,
                        # not validation
                        message.ack()

                        log.info(f"Message acknowledged")
                        if reply_routing_key is not None and \
                        reply_prep is not None:
                            await self.publish(
                                routing_key = reply_routing_key,
                                msg = result,
                                msg_prep = reply_prep
                            )
                except Exception as err:
                    log.error(f"Error {err} consuming message")
            except ChannelClosed as err:
                log.error(f"Error {err} consuming message")
            except AttributeError as err:
                log.error(f"Error {err} consuming message")

            await asyncio.sleep(self.timeout)
