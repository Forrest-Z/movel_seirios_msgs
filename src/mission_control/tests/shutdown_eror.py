from test_communicator import AMQPHandler
import asyncio
import json

def main():
    loop = asyncio.get_event_loop()

    AMQPH = AMQPHandler(loop)

    loop.run_until_complete(AMQPH.connect())
    print('connected')
    error_prone_msg = {'Shutdown': False, 'Timeout': 0.1, 'Name': 'broadcast_pose.launch_229723'}
    loop.run_until_complete(AMQPH.send('robot_ex', 'robot_cancel', json.dumps(error_prone_msg)))
    print("1")

    loop.run_until_complete(AMQPH.receive('robot_ex', 'robot_cancel_reply', test_msg_processor))
    loop.close()


def test_msg_processor(msg):
    print('{}!!!!'.format(msg) )
    return True, msg.decode('utf-8')

if __name__ == '__main__':
    main()
