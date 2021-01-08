import rospy, rosnode, roslaunch, time
from roslaunch import roslaunch_logs
import logging

from time import sleep
import argparse
import asyncio
import threading
import concurrent.futures
from communicator import AMQPHandler

#class LaunchStation(object):
#    """
#    client = pymongo.MongoClient("mongodb://localhost:27017/")
#    db = client["ros"]
#    collection = db["gmapping"]
#
#    print "parseing"
#    if collection.find_one({"_id": "metadata"}):
#        print collection.find({"_id": "metadata"})
#        print False
#        print True
#        print "hi"
#        pass
#    else:
#        print "Writing metadata"
#        doc_metadata = {}
#        doc_metadata["_id"] = "metadata"
#        doc_metadata["name"] = "gmapping"
#        post_id = collection.insert_one(doc_metadata)
#
#    if collection.find_one({"_id": "default_configs"}):
#        pass
#    else:
#        print "Writing configs"
#        with open('/home/ryan/gmapping_params.yaml') as f:
#            doc_config = yaml.safe_load(f)
#        doc_config["_id"] = "default_configs"
#        post_id = collection.insert_one(doc_config)
#
#    if collection.find_one({"_id": "need"}):
#        pass
#    else:
#        print "Writing needs"
#        doc_need = {}
#        doc_need["_id"] = "need"
#        doc_need["topics"] = ["/tf", "/scan"]
#        doc_need["services"] = []
#        doc_need["actions"] = []
#        post_id = collection.insert(doc_need)
#
#    if collection.find_one({"_id": "want"}):
#        pass
#    else:
#        print "Writing wants"
#        doc_want = {}
#        doc_want["_id"] = "want"
#        doc_want["topics"] = []
#        doc_want["services"] = []
#        doc_want["actions"] = []
#        post_id = collection.insert(doc_want)
#
#    if collection.find_one({"_id": "provide"}):
#        pass
#    else:
#        print "Writing provide"
#        doc_provide = {}
#        doc_provide["_id"] = "provide"
#        doc_provide["topics"] = ["/map", "/map_metadata", "gmapping/entropy"]
#        doc_provide["services"] = []
#        doc_provide["actions"] = []
#        post_id = collection.insert(doc_provide)
#
#
#async def main(loop):
#    connection = await aio_pika.connect_robust(
#            "amqp://guest:guest@127.0.0.1:5672", loop=loop
#    )
#
#    async with connection:
#        routing_key = "routing-key"
#
#        channel = await connection.channel()
#        myexchange = await channel.declare_exchange("my-exchange", type='fanout')
#
#        await myexchange.publish(
#            aio_pika.Message(body="Hello {}".format(routing_key).encode()),
#            routing_key=routing_key,
#            )
#async def main():
#    loop = asyncio.get_event_loop()
#
#    async def get_connection():
#        return await aio_pika.connect_robust("amqp://guest:guest@localhost:5672")
#
#    connection_pool = Pool(get_connection, max_size=2, loop=loop)
#
#    async def get_channel() -> aio_pika.Channel:
#        async with connection_pool.acquire() as connection:
#            return await connection.channel()
#
#    channel_pool = Pool(get_channel, max_size=10, loop=loop)
#    queue_name = "my-queue"
#
#    async def consume():
#        async with channel_pool.acquire() as channel:  # type: aio_pika.Channel
#            await channel.set_qos(10)
#
#            queue = await channel.declare_queue(
#                queue_name, durable=False, auto_delete=False
#            )
#
#            async with queue.iterator() as queue_iter:
#                async for message in queue_iter:
#                    print(message)
#                    await message.ack()
#
#    async def publish():
#        async with channel_pool.acquire() as channel:  # type: aio_pika.Channel
#            await channel.default_exchange.publish(
#                aio_pika.Message(("Channel: %r" % channel).encode()),
#                queue_name,
#            )

class TaskManager():
    def __init__(self):
        self.connected = False
        print ("TM init")

    def pub_connections(self):
        print("badum")
#    async def connect(self, reconnect_delay, connections):
#        connections.append("tm")
#        print ("tm connect")
#        await asyncio.sleep(reconnect_delay)

    def connect(self, reconnect_delay):
        print ("tm attempt")
        while not self.connected:
            self.connected = True
            print("tm connected")
            return self.connected
        #return False
        #return False
#        node = roslaunch.core.Node('gmapping', 'slam_gmapping', output="screen", name="gmapping")
#        config = roslaunch.ROSLaunchConfig()
#        config.nodes.append(node)
#
##        for doc in db['gmapping'].find():
##            for key, value in doc.items():
##                param = roslaunch.core.Param("/gmapping/"+str(key), value)
##                config.add_param(param)
#
#        delta = roslaunch.core.Param("/gmapping/delta", 0.03)
#        particles = roslaunch.core.Param("/gmapping/particles", 20)
#
#        run_id = roslaunch_logs.get_run_id()
#        launch_runner = roslaunch.launch.ROSLaunchRunner(run_id, config=config)
#        one, two = launch_runner.launch()
#
#        launch_runner.spin()



    #def test_msg_processor(msg):
    #print('{}!!!!'.format(msg) )
    #    return True, msg.decode('utf-8')

class DB():
    def __init__(self):
        self.connected = True

class MissionControl():
    def __init__(self):
        self.loop = asyncio.get_event_loop()
        self.comm = AMQPHandler(self.loop)
        self.tm = TaskManager()
        self.db = DB()
        self.connections = {"comm_connected": self.comm.connected,
               "tm_connected": self.tm.connected,
               "db_connected": self.db.connected}

    def systemCheck(self, hb_interval=1):
        self.shutdown = False
        while True:
            print ("whiling")
            try:
                if self.comm.connected:
                    print("hi")
                    #self.loop.call_soon(self.comm.send, 'test_ex', 'test_queue', 'Test Message2')
                    self.comm.send('test_ex', 'test_queue', 'Test Message2')
                if self.tm.connected:
                    print ("hi2")
                    #self.tm.pub()
                print ("sleep")
            except Exception as e:
                print ("dump")
                print (e)
            sleep (hb_interval)


    #sync def

    #def bringupConnections(self, reconnect_delay, commlogin):
            #TODO: Add both general and individual timeout (i.e, timeout per
            # service connection attempt or all the attempts as a whole.
def test_msg_processor(msg):
    print('{}!!!!'.format(msg) )
    return True, msg.decode('utf-8')

def main() -> None:
    parser = argparse.ArgumentParser(
        description='Mission Control for task management.')
    parser.add_argument('-V', '--version', action='version',
            version='0.1')
    parser.add_argument('--commauth', default='guest:guest',
            type=str, metavar='guest:guest',
            help="Authentication credentials for communication server.")
    parser.add_argument('--commadd', default='localhost:5672',
            metavar='localhost:5672',
            type=str, help="Address for communication server.")
    parser.add_argument('--commproto', default='amqp', choices=['amqp'],
            type=str, help="Communication protocol.")
    parser.add_argument('--reconnect_delay', default=3,
            type=float, help="Attempt reconnect after delay in secs.")
    parser.add_argument('--sys_hb_interval', default=3,
            type=float, help="Interval for system heartbeat in secs.")
    args = parser.parse_args()

    commlogin="%s://%s@%s" % (args.commproto, args.commauth, args.commadd)
    reconnect_delay = args.reconnect_delay
    sys_hb_interval = args.sys_hb_interval

    mc = MissionControl()

    with concurrent.futures.ThreadPoolExecutor(max_workers=5) as executor:
        tm_connection = executor.submit(mc.tm.connect, reconnect_delay)
        comm_connection = executor.submit(mc.loop.run_until_complete(
            mc.comm.connect(reconnect_delay, commlogin)))
        sys_check = executor.submit(mc.systemCheck, sys_hb_interval)
#        mc.loop.create_task(mc.comm.send('test_ex', 'test_queue', 'Test Message2'))
#        mc.loop.create_task(mc.comm.receive('test_ex', 'test_queue', test_msg_processor))
#        mc.loop.run_forever()

        comm_consume= executor.submit(mc.loop.run_until_complete(
            mc.comm.receive('test_ex', 'test_queue', test_msg_processor)))


if __name__ == "__main__":
    main()
    #LaunchStation()
    #TaskManager()
