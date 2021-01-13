from test_communicator import AMQPHandler
import asyncio
import json

def main():
    loop = asyncio.get_event_loop()

    AMQPH = AMQPHandler(loop)

    loop.run_until_complete(AMQPH.connect())
    print('connected')
    loop.run_until_complete(AMQPH.send('test_ex', 'test_queue', 'Test Message!'))
    print('sended')
    loop.run_until_complete(AMQPH.send('test_ex', 'test_queue', 'Test Message1'))
    print("1")
    loop.run_until_complete(AMQPH.send('test_ex', 'test_queue', 'Test Message2'))
    print("2")
    msg = {'Timeout': 0.1, 'Pkg': 'broadcast_pose', 'Executable': None, 'Params': None, 'Args': {"Local_only": True, "Port": 11311, 'Loglevel': 'master_logger_level'}, 'Launchfile':'broadcast_pose.launch'}
    json_msg = json.dumps(msg)
    print (json_msg)
    print (type(json_msg))
    loop.run_until_complete(AMQPH.send('robot_ex', 'robot_launch', json.dumps(msg)))
    print("3")

    loop.run_until_complete(AMQPH.receive('test_ex', 'test_queue', test_msg_processor))
    loop.close()


def test_msg_processor(msg):
    print('{}!!!!'.format(msg) )
    return True, msg.decode('utf-8')

if __name__ == '__main__':
    main()
