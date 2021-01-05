#!/usr/bin/env python

import rospy, rosnode, roslaunch
from roslaunch import roslaunch_logs
#import pymongo, datetime, json
#import pika
import time

# Apologies, no time to learning the alternative reality of asynchronous style
# programming with asyncio
import aio_pika, asyncio
from aio_pika.pool import Pool

class LaunchCommunicator(object):
    """
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main(loop))
    loop.close()

    async def main(loop):
        connection = await aio_pika.connect_robust(
        "amqp://guest:guest@localhost:5672", loop=loop
)

    async with connection:
        routing_key = "routing-key"

        channel = await connection.channel("my-channel")

    await channel.default_exchange.publish(
        aio_pika.Message(body="Hello {}".format(routing_key).encode()),
        routing_key=routing_key,
    )
    """




class LaunchStation(object):
    """
    client = pymongo.MongoClient("mongodb://localhost:27017/")
    db = client["ros"]
    collection = db["gmapping"]

    print "parseing"
    if collection.find_one({"_id": "metadata"}):
        print collection.find({"_id": "metadata"})
        print False
        print True
        print "hi"
        pass
    else:
        print "Writing metadata"
        doc_metadata = {}
        doc_metadata["_id"] = "metadata"
        doc_metadata["name"] = "gmapping"
        post_id = collection.insert_one(doc_metadata)

    if collection.find_one({"_id": "default_configs"}):
        pass
    else:
        print "Writing configs"
        with open('/home/ryan/gmapping_params.yaml') as f:
            doc_config = yaml.safe_load(f)
        doc_config["_id"] = "default_configs"
        post_id = collection.insert_one(doc_config)

    if collection.find_one({"_id": "need"}):
        pass
    else:
        print "Writing needs"
        doc_need = {}
        doc_need["_id"] = "need"
        doc_need["topics"] = ["/tf", "/scan"]
        doc_need["services"] = []
        doc_need["actions"] = []
        post_id = collection.insert(doc_need)

    if collection.find_one({"_id": "want"}):
        pass
    else:
        print "Writing wants"
        doc_want = {}
        doc_want["_id"] = "want"
        doc_want["topics"] = []
        doc_want["services"] = []
        doc_want["actions"] = []
        post_id = collection.insert(doc_want)

    if collection.find_one({"_id": "provide"}):
        pass
    else:
        print "Writing provide"
        doc_provide = {}
        doc_provide["_id"] = "provide"
        doc_provide["topics"] = ["/map", "/map_metadata", "gmapping/entropy"]
        doc_provide["services"] = []
        doc_provide["actions"] = []
        post_id = collection.insert(doc_provide)


    node = roslaunch.core.Node('gmapping', 'slam_gmapping', output="screen", name="gmapping")
    config = roslaunch.ROSLaunchConfig()
    config.nodes.append(node)

#    for doc in db['gmapping'].find():
#        for key, value in doc.items():
#            param = roslaunch.core.Param("/gmapping/"+str(key), value)
#            config.add_param(param)




    delta = roslaunch.core.Param("/gmapping/delta", 0.03)
    particles = roslaunch.core.Param("/gmapping/particles", 20)

    run_id = roslaunch_logs.get_run_id()
    launch_runner = roslaunch.launch.ROSLaunchRunner(run_id, config=config)
    p, code = launch_runner.launch()
    """

async def main(loop):
    connection = await aio_pika.connect_robust(
            "amqp://guest:guest@127.0.0.1:5672", loop=loop
    )

    async with connection:
        routing_key = "routing-key"

        channel = await connection.channel()
        myexchange = await channel.declare_exchange("my-exchange", type='fanout')

        await myexchange.publish(
            aio_pika.Message(body="Hello {}".format(routing_key).encode()),
            routing_key=routing_key,
            )
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

if __name__ == "__main__":
    try:
        #LaunchStation()
        loop = asyncio.get_event_loop()
        loop.run_until_complete(main(loop))
        loop.close()
    except rospy.ROSInterruptException:
        pass
