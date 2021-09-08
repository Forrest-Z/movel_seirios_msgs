#!/usr/bin/env python3

import rospy
import json
import os

from std_msgs.msg import String


def print_dict(d, indent=4):
    for k, v in d.items():
        space = " " * indent
        if type(v) is dict:
            print(f"{space}{k} :")
            print_dict(v, indent + indent)
        else:
            print(f"{space}{k} : {v}")


def write_map_to_html(map_uri):
    html = f"""
        <!DOCTYPE html>
        <html lang="en">
        <head>
            <title>A simple HTML document</title>
        </head>
        <body>
            <p>Hello World!<p>
            <img src="{map_uri}"/>
        </body>
        </html>
    """
    user = os.environ["USER"]
    outfile = f"/home/{user}/sub_uri_json_to_html.html"
    with open(outfile, "w") as f:
        f.write(html)


def map_uri_json_CB(msg):
    map_attributes = json.loads(msg.data)
    print_dict(map_attributes)
    write_map_to_html(map_attributes["uri"])
    


if __name__ == "__main__":
    rospy.init_node('sub_uri_json_to_html')

    map_uri_json_sub = rospy.Subscriber("/map/uri/json", String, map_uri_json_CB)

    rospy.spin()