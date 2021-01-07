from communicator import AMQPHandler
import asyncio

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
    loop.run_until_complete(AMQPH.send('test_ex', 'test_queue', 'Test Message2'))
    print("3")

    loop.run_until_complete(AMQPH.receive('test_ex', 'test_queue', test_msg_processor))
    loop.close()


def test_msg_processor(msg):
    print('{}!!!!'.format(msg) )
    return True, msg.decode('utf-8')

if __name__ == '__main__':
    main()
