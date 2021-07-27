import asyncio
from aio_pika import connect_robust, Message
from aio_pika.exceptions import AMQPConnectionError, ConnectionClosed, ChannelClosed

def test_msg_processor(msg):
    print('{}!!!!'.format(msg) )
    return True, msg.decode('utf-8')

class AMQPHandler():
    def __init__(self, asyncio_loop):
        print("amqp init")
        self.connected = False
        self.loop = asyncio_loop

    async def connect(self, reconnect_delay=3,
            amqp_connect_string="amqp://guest:guest@localhost:5672"):
        print ("amqp attempt")
        print (amqp_connect_string)
        while not self.connected:
            print ("rabbit")
            try:
                self.amqp_connect_string = amqp_connect_string
                #TODO: Add timeout for connect_robust and channel creation
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
        print ("send")
        routing_key = amqp_queue
        exchange = await self.channel.declare_exchange(amqp_exchange, auto_delete=False)
        queue = await self.channel.declare_queue(amqp_queue, auto_delete=False)
        await queue.bind(exchange, routing_key)
        await exchange.publish(
            Message(
                    #body=json.dumps(msg).encode(),
                    bytes(msg, 'utf-8')
                ),
                routing_key
        )

    async def receive(self, amqp_exchange, amqp_queue, msg_proc_func=None, awaitable_msg_proc_func=None, redirect_to_exchange=None, redirect_to_queue=None):
        print ("receive")
        routing_key = amqp_queue
        exchange = await self.channel.declare_exchange(amqp_exchange, auto_delete=False)
        queue = await self.channel.declare_queue(amqp_queue, auto_delete=False)
        await queue.bind(exchange, routing_key)
        
        async for message in queue.iterator():
        # async for message in queue:
            mpf = msg_proc_func
            if awaitable_msg_proc_func is not None:
                mpf = awaitable_msg_proc_func
                # proc_status, proc_result = await mpf(message.body)
                proc_status, proc_result = mpf(message.body)
            else:
                proc_status, proc_result = mpf(message.body)

            if((redirect_to_exchange != None) and (redirect_to_queue != None)):
                await self.send(redirect_to_exchange, redirect_to_queue, proc_result)

            if proc_status == True:
                print ("ack")
                message.ack()
