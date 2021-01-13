import rospy, rosnode, roslaunch, rosgraph
import asyncio
import functools
import contextvars
import signal
import threading
from roslaunch import roslaunch_logs, rlutil, pmon
from std_msgs.msg import String
try:
        from xmlrpc.client import ServerProxy
except ImportError:
        from xmlrpclib import ServerProxy

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
        try:
            self.pub.publish(connections)
        except Exception as e:
            print (e)

    def init_node(self):
        #TODO: Implement respawn option
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
                    #await to_thread(self.init_node)
                    self.init_node()
                    print ("Node init")
                print ("connected = true")
                self.connected = True
            except Exception as exc:
                self.connected = False
                print ("no roscore")
                print (exc)
                await asyncio.sleep(reconnect_delay)
            await asyncio.sleep(hb_interval)

    def kill(self, pid, timeout):
        print ("Killing")
        if pid in self.running_pid:
            print ("kill")
        return success, msg

    def launch(self, parent):
        try:
            parent.start()
        except Exception as e:
            print ("parent start")
            print (e)

        try:
            parent.spin()
        except Exception as e:
            print ("parent spin")
            print (e)

    def start_launchfile(self, launchfile, timeout, pkg, launch_args):
        print ("\nGOT LAUNCHFILE")
        if pkg is not None:
            rl_obj = rlutil.resolve_launch_arguments([pkg, launchfile])
        else:
            rl_obj = rlutil.resolve_launch_arguments([launchfile])

        print(rl_obj)

        # TODO: revisit
        p = roslaunch.parent.ROSLaunchParent(self.run_id, rl_obj,
                is_core=False,
                port=launch_args['Port'],
                timeout=timeout,
                master_logger_level=launch_args['Loglevel'])
                #sigint_timeout=timeout,
                #sigterm_timeout=timeout)
        print ("p has started")
        try:
            #launch_success = await to_thread(p.runner.spin_once)
            #launch_success = await asyncio.run_coroutine_threadsafe(p.runner.spin_once, asyncio.get_running_loop)

            p.start()
            launch_success = p.spin_once()
            #launch_thread = threading.Thread(target=self.launch, args=(p,))
            #print ("launch_thread created")
            #launch_thread.start()
            #print ("launch_thread started")

        except Exception as e:
            print ("To thread exception")
            print (e)
            print ("thread exception finished printing")

        #finally:
        #    p._stop_infrastructure()

        #launch_thread.join()
        print ("start_launch")
        master = rosgraph.Master('/rosnode', master_uri="http://localhost:11311")
        print ('URI: ', p.runner.server_uri)
        for node in p.runner.config.nodes:
            name = node.name
            node= node
            print ("Name: ", name)


            node = "/" + name
            print ("New name: ", name)
            print (master.getSystemState())
            api = rosnode.get_api_uri(master, node)
            print ("API: ", api)
            node = ServerProxy(api)
            pid = rosnode._succeed(node.getPid('/rosnode'))
            print ("PID: ", pid)
            msg = ""


        return name, pid, launch_success, msg

    def start_ros_obj(self, pkg, executable, param, timeout):
        print ("start_ros_obj")
        return name, pid, success, msg

    def start_executable(self, executable, args, timeout):
        print ("start_exec")
        return name, pid, success, msg
