#!/usr/bin/env python
from __future__ import print_function

from Tkinter import *
from PIL import Image
from PIL import ImageTk

import numpy as np
import matplotlib, sys
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import time

import rospy
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import Image, Range, JointState

import cv2
from cv_bridge import CvBridge, CvBridgeError

root = Tk()

control_canvas_width = 600
control_canvas_height = 400

manual_control_steering_value = 0
manual_control_velocity_value = 0

manual_control_enable_publish = False
manual_control_steering_set_only = False
manual_control_velocity_set_only = False

manual_control_publish_period_ms = 200

manual_control_enabled = True

# ---------------------------------------------------------------------------

def manual_publish_control():
    ros_controller_set_velocity(manual_control_velocity_value)
    ros_controller_set_steering(manual_control_steering_value)

    if manual_control_enable_publish:
        root.after(manual_control_publish_period_ms, manual_publish_control)

# ---------------------------------------------------------------------------

def manual_control_callback_mouse_released(event):
    global manual_control_velocity_value, manual_control_steering_value, \
            manual_control_enable_publish, \
            manual_control_steering_set_only, manual_control_velocity_set_only

    manual_control_velocity_value = 0
    manual_control_steering_value = 0
    manual_control_enable_publish = False
    manual_control_velocity_set_only = False
    manual_control_steering_set_only = False
    # print ('Released')

def manual_conrtol_callback_mouse_motion(event):
    global manual_control_steering_value, manual_control_velocity_value, \
            manual_control_enable_publish
    
    if not manual_control_enabled:
        return

    if not manual_control_velocity_set_only:
        x = np.clip(event.x, 0, control_canvas_width)
        manual_control_steering_value = (float(x) / control_canvas_width - 0.5) * 2 * steering_abs_limit

    if not manual_control_steering_set_only:
        y = np.clip(event.y, 0, control_canvas_height)
        manual_control_velocity_value = (float(y) / control_canvas_height - 0.5) * -2 * velocity_abs_limit

    if not manual_control_enable_publish:
        root.after(manual_control_publish_period_ms, manual_publish_control)

    manual_control_enable_publish = True
    # print ('Motion', x, y)

def manual_control_callback_ctrl_mouse_click(event):
    global manual_control_steering_set_only

    manual_control_steering_set_only = True

def manual_control_callback_shift_mouse_click(event):
    global manual_control_velocity_set_only

    manual_control_velocity_set_only = True

# ---------------------------------------------------------------------------

def toggle_control_state():
    global manual_control_enabled
    global manual_control_velocity_value, manual_control_steering_value

    if manual_control_enabled:
        toggle_control_state_btn.config(text='Auto')
        manual_control_enabled = False
        autopilot_function()
    else:
        toggle_control_state_btn.config(text='Manual')
        manual_control_enabled = True
        manual_publish_control()

    manual_control_velocity_value = 0
    manual_control_steering_value = 0

# ---------------------------------------------------------------------------

def init_gui():
    global rangefinder1_text, rangefinder2_text, \
            rangefinder3_text, rangefinder4_text, \
            manual_control_canvas_wgt, toggle_control_state_btn, \
            lpf_rangefinder_coefficient

    manual_control_canvas_wgt = Canvas(root, width=control_canvas_width, height=control_canvas_height, \
                                        highlightbackground='red', highlightthickness=3)
    manual_control_canvas_wgt.grid(row=1, column=0, columnspan=4)
    manual_control_canvas_wgt.create_line(control_canvas_width/2, 0, control_canvas_width/2, control_canvas_height, fill='blue')
    manual_control_canvas_wgt.create_line(0, control_canvas_height/2, control_canvas_width, control_canvas_height/2, fill='blue')

    manual_control_canvas_wgt.bind('<ButtonPress-1>', manual_conrtol_callback_mouse_motion)
    manual_control_canvas_wgt.bind('<B1-Motion>', manual_conrtol_callback_mouse_motion)
    manual_control_canvas_wgt.bind('<ButtonRelease-1>', manual_control_callback_mouse_released)
    manual_control_canvas_wgt.bind('<Control-1>', manual_control_callback_ctrl_mouse_click)
    manual_control_canvas_wgt.bind('<Shift-1>', manual_control_callback_shift_mouse_click)

    rangefinder1_text = StringVar(); rangefinder1_text.set(0)
    rangefinder2_text = StringVar(); rangefinder2_text.set(0)
    rangefinder3_text = StringVar(); rangefinder3_text.set(0)
    rangefinder4_text = StringVar(); rangefinder4_text.set(0)

    Label(root, textvariable=rangefinder1_text).grid(row=0, column=0, padx=5, pady=5)
    Label(root, textvariable=rangefinder2_text).grid(row=0, column=1, padx=5, pady=5)
    Label(root, textvariable=rangefinder3_text).grid(row=0, column=2, padx=5, pady=5)
    Label(root, textvariable=rangefinder4_text).grid(row=0, column=3, padx=5, pady=5)

    lpf_rangefinder_coefficient = DoubleVar()
    lpf_rangefinder_coefficient.set(1)
    Scale(root, from_=0, to=1, label='LPF rate', resolution=.1, orient=HORIZONTAL, variable=lpf_rangefinder_coefficient)\
            .grid(row=2, column=2, columnspan=2, padx=5, pady=5)

    toggle_control_state_btn = Button(text='Manual', width=10, command=toggle_control_state)
    toggle_control_state_btn.grid(row=2, column=0, columnspan=2, padx=5, pady=5)

# ---------------------------------------------------------------------------

autopilot_period_ms = 100
autopilot_const_velocity = 100

def autopilot_function():
    print('Executing autopilot')

    print('Forward rangefinders: %.1f / %.1f' % (rangefinders_data[1], rangefinders_data[2]))

    reference_lane_control = 530
    prop_rate = 1.2

    if rangefinders_data[1] >= 0 and rangefinders_data[2] >= 0:
        error = reference_lane_control - control_value

        ros_controller_set_velocity(autopilot_const_velocity)
        ros_controller_set_steering(-error * prop_rate)
    else:
        ros_controller_set_steering(0)
        ros_controller_set_velocity(0)

        # if rangefinders_data[1] < rangefinders_data[2]:
        #     ros_controller_set_steering(steering_abs_limit)
        #     ros_controller_set_velocity(autopilot_const_velocity)
        # else:
        #     ros_controller_set_steering(-steering_abs_limit)
        #     ros_controller_set_velocity(autopilot_const_velocity)  

    if not manual_control_enabled:
        root.after(autopilot_period_ms, autopilot_function)
    else:
        ros_controller_set_velocity(0)
        ros_controller_set_steering(0)

# ---------------------------------------------------------------------------

velocity_abs_limit = 100
steering_abs_limit = 100

def ros_controller_set_velocity(velocity):
    velocity = np.clip(velocity, -velocity_abs_limit, velocity_abs_limit);
    velocity_pub.publish(float(velocity))
    print ('Publishing velocity', velocity)

def ros_controller_set_steering(steering):
    steering = np.clip(steering, -steering_abs_limit, steering_abs_limit);
    steering_pub.publish(float(steering))
    print ('Publishing steering', steering)

rangefinders_data = [0, 0, 0, 0]

def rangefinder1_callback(ros_data):
    rangefinders_data[0] = \
        ros_data.range * lpf_rangefinder_coefficient.get() + \
        (1. - lpf_rangefinder_coefficient.get()) * rangefinders_data[0]
    rangefinder1_text.set("%.1f" % rangefinders_data[0])

def rangefinder2_callback(ros_data):
    rangefinders_data[1] = \
        ros_data.range * lpf_rangefinder_coefficient.get() + \
        (1. - lpf_rangefinder_coefficient.get()) * rangefinders_data[1]
    rangefinder2_text.set("%.1f" % rangefinders_data[1])

def rangefinder3_callback(ros_data):
    rangefinders_data[2] = \
        ros_data.range * lpf_rangefinder_coefficient.get() + \
        (1. - lpf_rangefinder_coefficient.get()) * rangefinders_data[2]
    rangefinder3_text.set("%.1f" % rangefinders_data[2])

def rangefinder4_callback(ros_data):
    rangefinders_data[3] = \
        ros_data.range * lpf_rangefinder_coefficient.get() + \
        (1. - lpf_rangefinder_coefficient.get()) * rangefinders_data[3]
    rangefinder4_text.set("%.1f" % rangefinders_data[3])

def lane_control_cb(ros_data):
    global control_value
    control_value = ros_data.data
    # print(control_value)

prev_time_sec = -1
passed_way_enc = 0

def joint_state_cb(ros_data):
    global prev_time_sec, passed_way_enc
    dt = 0
    current_time_sec = ros_data.header.stamp.to_sec()
    if prev_time_sec > 0:
        dt = current_time_sec - prev_time_sec

    prev_time_sec = current_time_sec
    vel = ros_data.velocity[1]
    passed_way_enc += vel * dt

    print(dt, vel, passed_way_enc)




def ros_controller_init_connection():
    global velocity_pub, steering_pub
    
    velocity_pub = rospy.Publisher('ur_hardware_driver/velocity_controller/command', Float64, queue_size=10)
    steering_pub = rospy.Publisher('ur_hardware_driver/steering_controller/command', Float64, queue_size=10)

    rospy.Subscriber('lane_control', Int32, lane_control_cb, queue_size=10)
    rospy.Subscriber('rangefinder1', Range, rangefinder1_callback, queue_size=10)
    rospy.Subscriber('rangefinder2', Range, rangefinder2_callback, queue_size=10)
    rospy.Subscriber('rangefinder3', Range, rangefinder3_callback, queue_size=10)
    rospy.Subscriber('rangefinder4', Range, rangefinder4_callback, queue_size=10)

    rospy.Subscriber('joint_states', JointState, joint_state_cb, queue_size=100)

# ---------------------------------------------------------------------------

if __name__ == '__main__':
    init_gui()
    rospy.init_node('gui_control')
    ros_controller_init_connection()
    root.mainloop()

