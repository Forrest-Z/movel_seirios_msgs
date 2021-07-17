from test_communicator import AMQPHandler
import asyncio
import json

def main():
    # loop = asyncio.get_event_loop()
    loop = asyncio.new_event_loop()
    
    try:
        AMQPH = AMQPHandler(loop)
        loop.run_until_complete(AMQPH.connect())

        # error_prone_msg = {'Timeout': 0.1, 'Pk': 'broadcast_pose', 'Executable': None, 'Params': None, 'Args': None, 'Launchfile':'broadcast_pose.launch'}
        # error_free_msg = {'Timeout': 0.1, 'Pkg': 'broadcast_pose', 'Executable': None, 'Params': None, 'Args': None, 'Launchfile':'broadcast_pose.launch'}

        # with roslaunch args
        error_free_msg = {
            'Timeout': 5, 
            'Pkg': 'movel_rosbag_recorder', 
            'Launchfile':'wss_recorder.launch', 
            'Args': ["topics:=/tf;/pose;/map;/scan;/tf_static"], 
            'Executable': None, 'Params': None,   
        }
        
        # # no roslaunch args
        # error_free_msg = {
        #     'Timeout': 5, 
        #     'Pkg': 'yocs_velocity_smoother', 
        #     'Launchfile': 'standalone.launch', 
        #     'Args': None, 
        #     'Executable': None, 'Params': None,
        # }

    #    loop.run_until_complete(AMQPH.send('robot_ex', 'robot_launch', json.dumps(error_prone_msg)))
        loop.run_until_complete(AMQPH.send('robot_ex', 'robot_launch', json.dumps(error_free_msg)))
        loop.run_until_complete(AMQPH.receive('robot_ex', 'robot_launch_reply', test_msg_processor))    
    finally:
        loop.close()


def test_msg_processor(msg):
    #print('{}!!!!'.format(msg) )
    print (msg)
    return True, msg.decode('utf-8')

if __name__ == '__main__':
    main()
