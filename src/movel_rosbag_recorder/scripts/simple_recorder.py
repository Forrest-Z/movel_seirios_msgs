#!/usr/bin/env python3

import io
import os
import subprocess
import signal
import rospy


class SimpleRecorder:
    
    def __init__(self, bag_name, topics=None, split_duration_secs=0, use_compression=False):
        self.print_name = f"[{self.__class__.__name__}]"
        self.cmd = self.__create_rosbag_cmd(bag_name, topics, split_duration_secs, use_compression)
        self.pid = -1
        self.pgid = -1

    def __create_rosbag_cmd(self, bag_name, topics, split_duration_secs, use_compression):
        # write rosbag command line statement 
        output = io.StringIO()
        output.write("rosbag record ")
        # output bag name
        output.write(f"--output-name={bag_name} ")
        # compression
        if use_compression:
            output.write(f"--bz2 ")
        # split
        if split_duration_secs > 0:
            output.write(f"--split --duration={split_duration_secs}s ")
        # topics
        if topics is None:
            output.write("-a ")   # all topics
        else:
            for topic in topics:
                # clean string
                if topic[0] != "/": 
                    topic = "/" + topic
                output.write(topic + " ")
        cmd = output.getvalue()
        output.close()
        rospy.logwarn(f"{self.print_name} cmd: {cmd}")
        return cmd
        
    def start(self):
        # run as subprocess
        ps = subprocess.Popen(self.cmd, shell=True, start_new_session=True, stdout=subprocess.DEVNULL)
        self.pid = ps.pid
        self.pgid = os.getpgid(self.pid)
        rospy.logwarn(f"{self.print_name} start record: cmd: {self.cmd}, pid: {self.pid}, pgid: {self.pgid}")
    
    def stop(self):
        if self.pid == -1: return   # not started
        rospy.logwarn(f"{self.print_name} stop record: cmd: {self.cmd}, pid: {self.pid}, pgid: {self.pgid}")
        os.killpg(self.pgid, signal.SIGTERM)   # kill process group

    def __del__(self):
        self.stop()