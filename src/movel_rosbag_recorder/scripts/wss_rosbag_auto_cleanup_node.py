#!/usr/bin/env python3

import rospy
import os
import glob
import json
from datetime import date, datetime, timedelta

from simple_recorder import SimpleRecorder
from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger
from movel_seirios_msgs.srv import StringArrayTrigger


class RosbagAutoCleanupNode:
    
    def __init__(self, name):
        self.print_name = f"[{name}]"
        self.rosbag_docker_volume = os.getenv("WSS_PLAYER_ROSBAG_PATH", "/home/movel/.config/movel/rosbags")
        self.current_active_bag_sub = rospy.Subscriber("/current_active_bag", String, self.current_active_bag_CB)
        # clean up with rosbag manager node  
        self.reindex_unclosed_bags_client = rospy.ServiceProxy("/reindex_unclosed_bags", StringArrayTrigger)
        self.merge_bags_client = rospy.ServiceProxy("/merge_bags", StringArrayTrigger)
        self.list_unclosed_bags_client = rospy.ServiceProxy("/list_unclosed_bags", Trigger)
        self.list_unmerged_bags_client = rospy.ServiceProxy("/list_unmerged_bags", Trigger)
        # internal variables
        self.current_active_bag = "None"   # string to simplify checks in cleanup loop
        # start
        self.__start_auto_cleanup_loop()
        

    def current_active_bag_CB(self, msg):
        self.current_active_bag = msg.data

        
    def __start_auto_cleanup_loop(self):
        d_10m = rospy.Duration(600)   # 10 min
        # d_10m = rospy.Duration(5)   # testing 5 sec
        while not rospy.is_shutdown():
            rospy.sleep(d_10m)   # wait for all nodes to launch fully on 1st loop
            rospy.logwarn(f"{self.print_name} running auto clean up ...")
            # get folders and bags for cleanup
            res = self.list_unclosed_bags_client()
            unclosed_bags_in_folders = json.loads(res.message)
            res = self.list_unmerged_bags_client()
            unmerged_bags_in_folders = json.loads(res.message)
            # reindex unclosed bags (not closed properly)
            rospy.loginfo(f"{self.print_name} reindexing unclosed bags ...")
            current_active_folder = os.path.dirname(self.current_active_bag)
            for folder, bags in unclosed_bags_in_folders.items():
                srv_input = []
                # reindex whole folder
                if folder != current_active_folder:
                    cleanup_folder = folder
                    srv_input = [cleanup_folder]
                # ignore current active bags
                else:
                    cleanup_folder = folder
                    ignore_bags = [f"{self.current_active_bag}.active"]
                    srv_input = [cleanup_folder, *ignore_bags]
                self.reindex_unclosed_bags_client(srv_input)
            # merge bags split bags within each hour
            rospy.loginfo(f"{self.print_name} merging split bags ...")
            bag = os.path.basename(self.current_active_bag)
            current_active_group = bag.split("-")[0] if "-" in bag else bag.split(".")[0]
            for folder, merge_groups in unmerged_bags_in_folders.items():
                for group, bags in merge_groups.items():
                    # merge bags in group
                    if group != current_active_group: 
                        i = 1
                        while len(bags) > 1:
                            srv_input = []
                            bag1 = os.path.join(folder, bags.pop(0))
                            bag2 = os.path.join(folder, bags.pop(0))
                            outbag = os.path.join(folder, f"{group}-merge-{i}.bag")
                            srv_input = [bag1, bag2, outbag]
                            self.merge_bags_client(srv_input)
                            # add merged outbag back into list
                            bags.insert(0, outbag)
                            i += 1
                        # final merged bag
                        final_merged_bag = os.path.join(folder, bags[0])
                        new_name = os.path.join(folder, f"{group}.bag")
                        os.rename(final_merged_bag, new_name)
                    # ignore current active bags    
                    else:
                        continue
            rospy.logwarn(f"{self.print_name} running auto clean up done")



if __name__ == "__main__":
    rospy.init_node('wss_rosbag_auto_cleanup_node')
    node = RosbagAutoCleanupNode(rospy.get_name())
    rospy.spin()