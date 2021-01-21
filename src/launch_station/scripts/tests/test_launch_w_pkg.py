from test_communicator import AMQPHandler
import asyncio
import json

def main():
    loop = asyncio.get_event_loop()

    AMQPH = AMQPHandler(loop)
    loop.run_until_complete(AMQPH.connect())

    error_prone_msg = {'imeout': 0.1, 'Pkg': 'broadcast_pose', 'Executable': None, 'Params': None, 'Args': None, 'Launchfile':'broadcast_pose.launch'}
    error_free_msg = {'Timeout': 0.1, 'Pkg': 'broadcast_pose', 'Executable': None, 'Params': None, 'Args': None, 'Launchfile':'broadcast_pose.launch'}

    loop.run_until_complete(AMQPH.send('robot_ex', 'robot_launch', json.dumps(error_prone_msg)))
    loop.run_until_complete(AMQPH.send('robot_ex', 'robot_launch', json.dumps(error_free_msg)))
    loop.run_until_complete(AMQPH.receive('robot_ex', 'robot_launch_reply', test_msg_processor))
    loop.close()


def test_msg_processor(msg):
    print('{}!!!!'.format(msg) )
    return True, msg.decode('utf-8')

if __name__ == '__main__':
    main()
