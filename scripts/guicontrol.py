#!/usr/bin/env python
from __future__ import print_function

from Tkinter import *
from PIL import Image
from PIL import ImageTk

import matplotlib, numpy, sys
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import time

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, Range

import cv2
from cv_bridge import CvBridge, CvBridgeError

velocity_abs_limit = 50
steering_abs_limit = 110

rangefinders_data = [0, 0, 0, 0]

root = Tk()

def velocity_slider_callback(val):
    pass

def steering_slider_callback(val):
    steering_pub.publish(float(val))

def range_callback(ros_data):

    rospy.loginfo('Message caught: {} / {}'.format(ros_data.header.frame_id, ros_data.range))


def init_gui():
    global rangefinder1_text, rangefinder2_text, rangefinder3_text, rangefinder4_text

    sliders_widget_frame = Frame(root)
    sliders_widget_frame.pack(side=BOTTOM, fill=BOTH, expand=True)

    left_slider_widget_frame = Frame(sliders_widget_frame)
    left_slider_widget_frame.pack(side=LEFT, fill=BOTH, expand=True)

    velocity_slider_wgt = Scale(left_slider_widget_frame, from_=velocity_abs_limit, to=-velocity_abs_limit, orient=VERTICAL, command=velocity_slider_callback)
    velocity_slider_wgt.pack(side=TOP, fill=Y, expand=True, padx=5, pady=5)

    right_slider_widget_frame = Frame(sliders_widget_frame)
    right_slider_widget_frame.pack(side=RIGHT, fill=BOTH, expand=True)

    steering_slider_wgt = Scale(right_slider_widget_frame, from_=-steering_abs_limit, to=steering_abs_limit, orient=HORIZONTAL, command=steering_slider_callback)
    steering_slider_wgt.pack(side=RIGHT, fill=X, expand=True, padx=5, pady=5)

    texts_widget_frame = Frame(root)
    texts_widget_frame.pack(side=TOP, fill=X)

    rangefinder1_text = Label(texts_widget_frame, text="0.0")
    rangefinder1_text.pack(side=LEFT, fill=BOTH, expand="yes", padx=5, pady=5)
    rangefinder2_text = Label(texts_widget_frame, text="0.0")
    rangefinder2_text.pack(side=LEFT, fill=BOTH, expand="yes", padx=5, pady=5)
    rangefinder3_text = Label(texts_widget_frame, text="0.0")
    rangefinder3_text.pack(side=LEFT, fill=BOTH, expand="yes", padx=5, pady=5)
    rangefinder4_text = Label(texts_widget_frame, text="0.0")
    rangefinder4_text.pack(side=LEFT, fill=BOTH, expand="yes", padx=5, pady=5)



def init_connection():
    global velocity_pub, steering_pub
    global rangefinder1_subscr, rangefinder2_subscr, rangefinder3_subscr, rangefinder4_subscr

    rospy.init_node('gui_control')
    velocity_pub = rospy.Publisher('ur_hardware_driver/velocity_controller/command', Float64, queue_size=10)
    steering_pub = rospy.Publisher('ur_hardware_driver/steering_controller/command', Float64, queue_size=10)

    rangefinder1_subscr = rospy.Subscriber('rangefinder1', Range, range_callback, queue_size=100)
    rangefinder2_subscr = rospy.Subscriber('rangefinder2', Range, range_callback, queue_size=100)
    rangefinder3_subscr = rospy.Subscriber('rangefinder3', Range, range_callback, queue_size=100)
    rangefinder4_subscr = rospy.Subscriber('rangefinder4', Range, range_callback, queue_size=100)
    rospy.loginfo('Topics connected')

if __name__ == '__main__':
    init_gui()
    init_connection()
    root.mainloop()

