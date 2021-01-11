import rospy, rosnode, roslaunch, rosgraph
import asyncio
import functools
import contextvars
from roslaunch import roslaunch_logs
from std_msgs.msg import String

async def to_thread(func, /, *args, **kwargs):
    """Asynchronously run function *func* in a separate thread.
    Any *args and **kwargs supplied for this function are directly passed
    to *func*. Also, the current :class:`contextvars.Context` is propogated,
    allowing context variables from the main thread to be accessed in the
    separate thread.
    Return a coroutine that can be awaited to get the eventual result of *func*.
    """
    loop = asyncio.get_running_loop()
    ctx = contextvars.copy_context()
    func_call = functools.partial(ctx.run, func, *args, **kwargs)
    return await loop.run_in_executor(None, func_call)

class RosTask:
    def __init__(self):
        self.connected = False
        print("TM init")

    def pub_connections(self, connections):
        self.pub.publish(connections)

    def init_node(self):
        print ("initing node")
        rospy.init_node('task_master')
        self.pub = rospy.Publisher('robot_sys', String, queue_size=1)

    async def healthCheck(self, hb_interval, reconnect_delay):
        print ("tm healthcheck")
        while True:
            try:
                param_server = rosgraph.Master('/roslaunch')
                self.run_id = param_server.getParam('/run_id')
                print ("run_id: ", self.run_id)
                if not self.connected:
                    await to_thread(self.init_node)
                    print ("Node init")
                    #TODO Capture SIGINT
                print ("connected = true")
                self.connected = True
            except Exception as exc:
                self.connected = False
                print ("no roscore")
                print (exc)
                await asyncio.sleep(reconnect_delay)
            await asyncio.sleep(hb_interval)

    def kill(self, pid, timeout):
        if pid in self.running_pid:
            print ("kill")
        return success, msg

    def start_launchfile(self, pkg, launchfile, timeout):
        print ("start_launch")
        return name, pid, success, msg

    def start_ros_obj(self, pkg, executable, param, timeout):
        print ("start_ros_obj")
        return name, pid, success, msg

    def start_executable(self, executable, args, timeout):
        print ("start_exec")
        return name, pid, success, msg
