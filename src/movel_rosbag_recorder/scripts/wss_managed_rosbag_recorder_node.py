#!/usr/bin/env python3

import rospy
import os
import glob
from datetime import date, datetime, timedelta

from simple_recorder import SimpleRecorder
from std_msgs.msg import String


class RecorderNode:
    
    def __init__(self, name):
        # configs
        self.print_name = f"[{name}]"
        self.rosbag_docker_volume = os.getenv("WSS_PLAYER_ROSBAG_PATH", "/home/movel/.config/movel/rosbags")
        self.current_active_bag_pub = rospy.Publisher("/current_active_bag", String, latch=True, queue_size=1)
        # internal variables
        self.recorder = None
        self.topics = None
        self.rosbag_folder = None
        self.rosbag_name = None
        self.next_bag_unix_time = None
        # start
        self.start_recording()

    
    def start_recording(self):
        # get topics from cmd line args 
        topics = rospy.get_param("~topics", "None")
        if topics == "None":
            rospy.logerr(f"{self.print_name} no topics specified! Exiting")
            exit(0)
        else:
            self.topics = topics.split(";")
            rospy.logwarn(f"{self.print_name} record_topics: {self.topics}")
        # start recording
        rospy.logwarn(f"{self.print_name} start recording ...")
        self.__start_recording_loop()


    def __start_recording_loop(self):
        d_2 = rospy.Duration(2)
        r_10 = rospy.Rate(10)
        self.next_bag_unix_time = -1.0   # start new bag
        while not rospy.is_shutdown():
            time_remaining = self.next_bag_unix_time - rospy.Time.now().to_sec()
            # sleep longer if lots of time remaining
            if time_remaining > 3.0:
                rospy.sleep(d_2)
            elif time_remaining > 0.0:
                r_10.sleep()
            else:   # new bag
                # prep folder and bag name
                self.__prep_rosbag_for_recording()
                # start recording for the hour
                bag_name = os.path.join(self.rosbag_folder, self.rosbag_name)
                self.recorder = SimpleRecorder(bag_name, topics=self.topics, use_compression=True)
                self.recorder.start()
            # publish active bag
            current_active_bag = os.path.join(self.rosbag_folder, self.rosbag_name)
            self.current_active_bag_pub.publish(String(current_active_bag))
        # stop and exit node
        rospy.logwarn(f"{self.print_name} stopped recording")
        self.__stop_recording_loop()


    def __prep_rosbag_for_recording(self):
        # recording time
        time_ros = rospy.Time.now()
        time_dt = datetime.fromtimestamp(time_ros.to_sec())
        # folder for day
        day_unix_time = datetime(time_dt.year, time_dt.month, time_dt.day).timestamp()
        self.rosbag_folder = os.path.join(self.rosbag_docker_volume, f"{day_unix_time:.0f}")
        if not os.path.exists(self.rosbag_folder):
            os.makedirs(self.rosbag_folder)
        # rosbag hour name
        hour_dt = datetime(time_dt.year, time_dt.month, time_dt.day, time_dt.hour)
        # hour_dt = datetime(time_dt.year, time_dt.month, time_dt.day, time_dt.hour, (time_dt.minute // 5) * 5)   # 5 mins   # testing
        hour_unix_time = hour_dt.timestamp()
        rosbag_name = f"{hour_unix_time:.0f}"
        # create new bag if there already exists bags for the hour
        rosbag_path = os.path.join(self.rosbag_folder, rosbag_name)
        hour_bag_count = len(glob.glob(f"{rosbag_path}*"))
        if hour_bag_count > 0:   # bag already exists
            rosbag_name = f"{rosbag_name}-{hour_bag_count}"
        self.rosbag_name = f"{rosbag_name}.bag"
        # stop time for bag
        self.next_bag_unix_time = (hour_dt + timedelta(hours=1)).timestamp()
        # self.next_bag_unix_time = (hour_dt + timedelta(minutes=5)).timestamp()   # testing


    def __stop_recording_loop(self):
        # reset all variables
        self.recorder = None   # recorder destructor used 
        self.topics = None
        self.rosbag_folder = None
        self.rosbag_name = None
        self.next_bag_unix_time = None


    def __del__(self):
        self.__stop_recording_loop()



if __name__ == "__main__":
    rospy.init_node('wss_managed_rosbag_recorder_node')
    node = RecorderNode(rospy.get_name())
    rospy.spin()