import rospy, rosnode, roslaunch, time
from roslaunch import roslaunch_logs

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
        print ("TM init")

#    async def connect(self, reconnect_delay, connections):
#        connections.append("tm")
#        print ("tm connect")
#        await asyncio.sleep(reconnect_delay)
    def connect(self, reconnect_delay, connections):
        print ("tm attempt")
        connected = False
        while not connected:
            print("taskmaster")
            print ("tm sleep")
            connected = False
            sleep(reconnect_delay*3)
            print ("tm wakeup")
            connections.append("tm")
            return connections
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


class MissionControl():
    def __init__(self):
        self.loop = asyncio.get_event_loop()
        self.comm = AMQPHandler(self.loop)
        self.tm = TaskManager()
        self.connections = [2]

    #async def systemCheck(self, reconnect_delay, commlogin):
    #    comm_connection = asyncio.create_task(self.comm.connect(reconnect_delay,
    #        self.connections, commlogin))
    #    tm_connection = asyncio.create_task(self.tm.connect(reconnect_delay,
    #        self.connections))
    #    connected_tasks = await asyncio.gather(comm_connection, tm_connection)

    def systemCheck(self, reconnect_delay, commlogin):
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
            tm_connection = executor.submit(self.tm.connect, reconnect_delay,
                self.connections)
            comm_connection = executor.submit(self.loop.run_until_complete(
                self.comm.connect(reconnect_delay, connections, commlogin)))
            print (self.connections)
        print (connections)

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
            type=float, help="Attempt reconnect after delay (secs).")
    args = parser.parse_args()

    commlogin="%s://%s@%s" % (args.commproto, args.commauth, args.commadd)

    mc = MissionControl()

    #mc.loop.run_until_complete(mc.systemCheck(args.reconnect_delay, commlogin))
    mc.systemCheck(args.reconnect_delay, commlogin)

if __name__ == "__main__":
    main()
    #LaunchStation()
    #TaskManager()
