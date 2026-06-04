#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
import tkinter as tk

# ===== INIT ROS =====
rospy.init_node('pid_gui_node')
pub = rospy.Publisher('/pid_values', Float32MultiArray, queue_size=10)

# ===== WINDOW =====
root = tk.Tk()
root.title("PID Tuning GUI")

# ===== VARIABLES =====
setpoint = tk.DoubleVar(value=70)
kp = tk.DoubleVar(value=2.0)
ki = tk.DoubleVar(value=8.0)
kd = tk.DoubleVar(value=0.03)

# ===== PUBLISH FUNCTION =====
def publish_data(event=None):
    msg = Float32MultiArray()
    msg.data = [setpoint.get(), kp.get(), ki.get(), kd.get()]
    pub.publish(msg)

# ===== UI =====
def create_slider(label, var, from_, to, resolution):
    frame = tk.Frame(root)
    frame.pack()

    tk.Label(frame, text=label, width=10).pack(side=tk.LEFT)

    slider = tk.Scale(frame, variable=var, from_=from_, to=to,
                      resolution=resolution, orient=tk.HORIZONTAL,
                      length=300, command=lambda x: publish_data())
    slider.pack(side=tk.LEFT)

# sliders
create_slider("Setpoint", setpoint, 0, 150, 1)
create_slider("Kp", kp, 0, 10, 0.1)
create_slider("Ki", ki, 0, 20, 0.1)
create_slider("Kd", kd, 0, 1, 0.01)

# ===== LOOP =====
def loop():
    publish_data()
    root.after(200, loop)  # kirim tiap 200 ms

loop()

root.mainloop()