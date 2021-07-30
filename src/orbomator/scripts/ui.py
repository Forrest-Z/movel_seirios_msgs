#!/usr/bin/env python3

from tkinter import *
import tkinter.simpledialog as simpledialog 
import tkinter.messagebox
import rospy
import tf
import csv
import math
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import rospkg

class MyWindow:
    def __init__(self, win):
        self.obj=win
        self.node = rospy.init_node('ui_orbmap')
        self.list_point=[]
        self.lbl_topic1=Label(win, text='Translation')
        self.lbl_topic1.place(x=1,y=1)
        self.lbl_topic2=Label(win, text='Rotation')
        self.lbl_topic2.place(x=1,y=55)
        self.lbl1=Label(win, text='X')
        self.lbl2=Label(win, text='Y')
        self.lbl3=Label(win, text='Z')
        self.line_set=False
        self.lbl4=Label(win, text='X')
        self.lbl5=Label(win, text='Y')
        self.lbl6=Label(win, text='Z')
        self.lbl7=Label(win, text='w')
        self.pub_line = rospy.Publisher('tranform_line', Marker, queue_size=1)
        self.pub_orb_done_ = rospy.Publisher('/orb_ui/status', Bool, queue_size=1)
        self.path_value=rospy.get_param('/GUI_Orbomator/path')
        self.t1=Entry(win, width=6)
        self.t2=Entry(win, width=6)
        self.t3=Entry(win, width=6)
        self.t4=Entry(win, width=6)
        self.t5=Entry(win, width=6)
        self.t6=Entry(win, width=6)
        self.t7=Entry(win, width=6)
        self.lbl1.place(x=20, y=20)
        self.t1.place(x=40, y=20)
        self.lbl2.place(x=100, y=20)
        self.t2.place(x=120, y=20)
        self.lbl3.place(x=180, y=20)
        self.t3.place(x=200, y=20)
        self.lbl4.place(x=20, y=80)
        self.t4.place(x=40, y=80)
        self.lbl5.place(x=100, y=80)
        self.t5.place(x=120, y=80)
        self.lbl6.place(x=180, y=80)
        self.t6.place(x=200, y=80)
        self.lbl7.place(x=260, y=80)
        self.t7.place(x=280, y=80)
        self.b1=Button(win, text='Test', command=self.test)
        self.b2=Button(win, text='Line Test', command=self.vis_line)
        self.b3=Button(win, text='Add',command=self.add)
        self.b4=Button(win, text='Finish', command=self.finish)
        self.b5=Button(win, text='Cancel',command=self.cancel)
        self.test_first = False
        self.b1.place(x=20, y=150)
        self.b2.place(x=80, y=150)
        self.b3.place(x=180, y=150)
        self.b4.place(x=240, y=150)
        self.b5.place(x=320, y=150)
        self.theta_angle=0.0
        self.x1=0.0
        self.x2=0.0
        self.y1=0.0
        self.y2=0.0

    def test(self):
        broadcaster = tf.TransformBroadcaster() 
        num1=float(self.t1.get())
        num2=float(self.t2.get())
        num3=float(self.t3.get())
        num4=float(self.t4.get())
        num5=float(self.t5.get())
        num6=float(self.t6.get())
        num7=float(self.t7.get())
        broadcaster.sendTransform((num1,num2,num3),(num4,num5,num6,num7),rospy.Time.now(),"orb_map", "map")

    def vis_line(self):
        xy1=(simpledialog.askstring("Input for setting line", "X1,Y1?")).split(",")
        xy2=(simpledialog.askstring("Input for setting line", "X2,Y2?")).split(",")
        self.x1=float(xy1[0])
        self.y1=float(xy1[1])
        self.x2=float(xy2[0])
        self.y2=float(xy2[1])
        self.visualize_line()

    def add(self):
        num1=float(self.t1.get())
        num2=float(self.t2.get())
        num3=float(self.t3.get())
        num4=float(self.t4.get())
        num5=float(self.t5.get())
        num6=float(self.t6.get())
        num7=float(self.t7.get())
        if not self.line_set:
            xy1=(simpledialog.askstring("Input for setting line", "X1,Y1?")).split(",")
            xy2=(simpledialog.askstring("Input for setting line", "X2,Y2?")).split(",")
            self.x1=float(xy1[0])
            self.y1=float(xy1[1])
            self.x2=float(xy2[0])
            self.y2=float(xy2[1])
        else:
            dist=float(simpledialog.askstring("Input", "Input distance with +/- sign (+ means upper/right region and - means lower/left region)"))

        #print (answer)
        if not self.line_set:
            self.distance = math.sqrt((self.x2-self.x1)**2+(self.y2-self.y1)**2)
            points= [num1,num2,num3,num4,num5,num6,num7,self.x1,self.y1,self.x2,self.y2]
            self.line_set=True
        else:
            del_x= (dist/self.distance)*(self.y1-self.y2)
            del_y= (dist/self.distance)*(self.x2-self.x1)
            xx1=self.x1+del_x
            yy1=self.y1+del_y
            xx2=self.x2+del_x
            yy2=self.y2+del_y
            points= [num1,num2,num3,num4,num5,num6,num7,xx1,yy1,xx2,yy2]
            self.x1=xx1
            self.y1=yy1
            self.x2=xx2
            self.y2=yy2
        self.list_point.append(points)
        self.t1.delete(0, 'end')
        self.t2.delete(0, 'end')
        self.t3.delete(0, 'end')
        self.t4.delete(0, 'end')
        self.t5.delete(0, 'end')
        self.t6.delete(0, 'end')
        self.t7.delete(0, 'end')

    def visualize_line(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03

        # marker color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        # marker line points
        marker.points = []
        # first point
        first_line_point = Point()
        first_line_point.x = self.x1
        first_line_point.y = self.y1
        first_line_point.z = 0.0
        marker.points.append(first_line_point)
        # second point
        second_line_point = Point()
        second_line_point.x = self.x2
        second_line_point.y = self.y2
        second_line_point.z = 0.0
        marker.points.append(second_line_point)

        # Publish the Marker
        self.pub_line.publish(marker)

    def finish(self):

        with open(self.path_value, mode='a') as trans_points:
            csvwriter = csv.writer(trans_points) 
            csvwriter.writerows(self.list_point)
        print(self.list_point)
        self.transform_done(True)
    
    def cancel(self):
        self.transform_done(False)

    def transform_done(self,ui_done):
        ui_done_ = Bool()
        ui_done_.data = ui_done
        self.pub_orb_done_.publish(ui_done_)

if __name__ == "__main__":
    window=Tk()
    mywin=MyWindow(window)
    window.title('Orb_slam_ui')
    window.geometry("500x200+10+10")
    window.mainloop()
