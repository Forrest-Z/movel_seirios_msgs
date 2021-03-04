#!/usr/bin/env python
import requests
import rospy
from std_msgs.msg import Empty, String

class FileTransfer:
    def __init__(self):
        self.map_dir = rospy.get_param("~map_dir")
        self.map_name = ""
        self.transferred_pub = rospy.Publisher("/file_transfer/transferred", Empty, queue_size = 1)
        rospy.Subscriber("/map_active", String, self.mapname_cb, queue_size = 1)
        rospy.Subscriber("/wallinspection_stopped", Empty, self.wallinspection_cb, queue_size = 1)

    def mapname_cb(self,msg):
        self.map_name = msg.data;

    def wallinspection_cb(self,msg):
        rospy.loginfo("[file_transfer] Transferring map image: %s", self.map_dir + self.map_name + '.pgm')
        rospy.loginfo("[file_transfer] Transferring map yaml: %s", self.map_dir + self.map_name + '.yaml')

        files = {self.map_name + '.pgm': open(self.map_dir + self.map_name + '.pgm', 'rb'), self.map_name + '.yaml': open(self.map_dir + self.map_name + '.yaml', 'rb')}
        try:
            rospy.logdebug("Sending map")
            r = requests.post("http://192.168.1.4:5000/images", files=files)
        except Exception as e:
            print("Post call failed " + str(e))
        self.transferred_pub.publish(Empty())

def main():
    rospy.init_node('file_transfer')
    m = FileTransfer()
    rospy.spin()

if __name__=="__main__":
    main()
