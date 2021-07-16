#!/usr/bin/env python3

import rospy
import io
import os
import glob
import fnmatch
import json

from rosbag_merger import merge_bags, reindex_bag, compress_bag
from std_srvs.srv import Trigger
from movel_seirios_msgs.srv import StringArrayTrigger


class RosbagManagerNode:
    
    def __init__(self, name):
        self.print_name = f"[{name}]"
        self.rosbag_docker_volume = os.getenv("WSS_PLAYER_ROSBAG_PATH", "/home/movel/.config/movel/rosbags")
        self.reindex_unclosed_bags_srv = rospy.Service("/reindex_unclosed_bags", StringArrayTrigger, self.reindex_unclosed_bags_CB)
        self.merge_bags_srv = rospy.Service("/merge_bags", StringArrayTrigger, self.merge_bags_CB)
        self.list_unclosed_bags_srv = rospy.Service("/list_unclosed_bags", Trigger, self.list_unclosed_bags_CB)
        self.list_unmerged_bags_srv = rospy.Service("/list_unmerged_bags", Trigger, self.list_unmerged_bags_CB)


    def reindex_unclosed_bags_CB(self, req):
        cleanup_folder = req.input[0]
        ignore_bags = req.input[1:] if len(req.input) > 1 else None
        if not os.path.isdir(cleanup_folder):
            rospy.logerr(f"{self.print_name} /reindex_unclosed_bags: '{cleanup_folder}' is not a folder")
        elif not os.path.exists(cleanup_folder):
            rospy.logerr(f"{self.print_name} /reindex_unclosed_bags: '{cleanup_folder}' does not exist")
        else:
            rospy.logwarn(f"{self.print_name} /reindex_unclosed_bags: reindexing bags in folder '{cleanup_folder}', ignoring bags '{ignore_bags}'")
            self.__reindex_unclosed_bags(cleanup_folder, ignore_bags=ignore_bags)
        # srv response
        return {"success": True, "message": ""}


    def merge_bags_CB(self, req):
        bag1 = req.input[0]
        bag2 = req.input[1]
        outbag = req.input[2]
        if not os.path.exists(bag1):
            rospy.logerr(f"{self.print_name} /merge_bags: '{bag1}' does not exist")
        elif not os.path.exists(bag2):
            rospy.logerr(f"{self.print_name} /merge_bags: '{bag2}' does not exist")
        elif os.path.exists(outbag):
            rospy.logerr(f"{self.print_name} /merge_bags: '{outbag}' already exists, please remove it manually")
        else:
            rospy.logwarn(f"{self.print_name} /merge_bags: merging bags '{bag1}' and '{bag2}' into '{outbag}'")
            merge_bags(bag1, bag2, outbag)
            rospy.logwarn(f"{self.print_name} /merge_bags: merge bag into '{outbag}' successful, removing '{bag1}' and '{bag2}'")
            os.remove(bag1)
            os.remove(bag2)
        # srv response
        return {"success": True, "message": ""}
        
    
    def list_unclosed_bags_CB(self, req):
        # look thorugh all day folders in rosbag docker volume mount
        path = self.rosbag_docker_volume
        day_folders = [os.path.join(path, d) for d in os.listdir(path)]
        day_folders = [d for d in day_folders if os.path.isdir(d)]
        unclosed_bags_in_folders = {}
        for folder in day_folders:
            unclosed_bags = [f for f in os.listdir(folder) if f.endswith(".active")]   # bags not closed properly
            if len(unclosed_bags) > 0:
                unclosed_bags_in_folders[f"{folder}"] = unclosed_bags
        message = json.dumps(unclosed_bags_in_folders, sort_keys=True)   # json
        # srv response
        return {"success": True, "message": message}


    def list_unmerged_bags_CB(self, req):
        # look thorugh all day folders in rosbag docker volume mount
        path = self.rosbag_docker_volume
        day_folders = [os.path.join(path, d) for d in os.listdir(path)]
        day_folders = [d for d in day_folders if os.path.isdir(d)]
        unmerged_bags_in_folders = {}
        for folder in day_folders:
            # group all bags by hour (in unix time)
            merge_groups = {}
            bag_files = [f for f in os.listdir(folder) if f.endswith(".bag")]
            for bag in bag_files:
                # group is hour in unix time
                group = bag.split("-")[0] if "-" in bag else bag.split(".")[0]
                if group in merge_groups:
                    merge_groups[group].append(bag)
                else:
                    merge_groups[group] = [bag]
            # remove singleton groups
            merge_groups = {group: bags for group, bags in merge_groups.items() if len(bags) > 1}
            # add to main dict
            unmerged_bags_in_folders[f"{folder}"] = merge_groups
        message = json.dumps(unmerged_bags_in_folders, sort_keys=True)   # json
        # srv response
        return {"success": True, "message": message}


    def __reindex_unclosed_bags(self, cleanup_folder, ignore_bags=None):
        # get active (unclosed) bags
        unclosed_bags = [f for f in os.listdir(cleanup_folder) if f.endswith(".active")]
        unclosed_bags = [os.path.join(cleanup_folder, f) for f in unclosed_bags]
        # ignore bags that are truly active (currently used for recording)
        ignore_bags = [] if ignore_bags is None else ignore_bags
        unclosed_bags_filtered = []
        for bag in unclosed_bags:
            if bag in ignore_bags:
                continue
            else:
                unclosed_bags_filtered.append(bag)
        # reindex
        for bag in unclosed_bags_filtered:
            rename_bag_to = bag[:-len(".active")]
            reindex_bag(bag, rename_bag_to=rename_bag_to, remove_backup=True)
            compress_bag(rename_bag_to, remove_backup=True)
            rospy.logwarn(f"{self.print_name} reindexed bag '{bag}', renamed to '{rename_bag_to}'")
        


if __name__ == "__main__":
    rospy.init_node('wss_rosbag_manager_node')
    node = RosbagManagerNode(rospy.get_name())
    rospy.spin()