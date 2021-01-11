import asyncio
from aio_pika import connect_robust, Message
from aio_pika.exceptions import AMQPConnectionError, ConnectionClosed, \
                                ChannelClosed

def test_msg_processor(msg):
    print('{}!!!!'.format(msg) )
    return True, msg.decode('utf-8')

class AMQPHandler():
    def __init__(self):
        self.connected = False
        self.connnection = None
        self.prev_hb_time = 0.0

    async def healthCheck(self, hb_interval, reconnect_delay, comm_login):
        print ("Comm healthcheck")
        while True:
            try:
                self.hb_time = self.connection.heartbeat_last
                self.connected = True
            except Exception as e:
                print ("triggering Attempt comm connection")
                print (e)
                self.connected = False
                await self.connect(reconnect_delay, comm_login)
            print ("comm healthcheck sleep")
            await asyncio.sleep(hb_interval)

    async def connect(self, reconnect_delay=3,
            amqp_connect_string="amqp://guest:guest@localhost:5672"):
        print ("amqp attempt")
        if not self.connected:
            try:
                self.amqp_connect_string = amqp_connect_string
                #TODO: Add timeout for connect_robust and channel creation
                self.connection = await connect_robust(self.amqp_connect_string)
                self.channel = await self.connection.channel()
                self.connected = True
                # Because of the robust connection, reconnection mechanics is
                # handled by library
                # There may be a race on connection. Require further testing.
                print("rabbit connected")
                return self.connected
            except Exception as exc:
                self.connected = False
                await asyncio.sleep(reconnect_delay)

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

    async def subscribe(self, routing_key, msg_proc_func, reply_routing_key=None, reply_encoder=None):
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
                    print ("5")

                    process_msg = msg_proc_func
                    result = process_msg(message.body)

                    if result['Success'] == True:
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
