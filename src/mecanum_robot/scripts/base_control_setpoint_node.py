#!/usr/bin/env python3

import rospy
import serial #type: ignore
import threading

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray


class BaseController:

    def __init__(self):
        rospy.init_node('base_controller_node')

        # --- Variabel Status Capit ---
        self.capit_state = 0.0 

        # Koneksi Serial
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)
        except:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)

        # Publisher
        self.yaw_pub = rospy.Publisher('/robot_yaw', Float32, queue_size=10)
        self.feedback_pub = rospy.Publisher('/motor_feedback', Float32MultiArray, queue_size=10)

        # Subscriber
        rospy.Subscriber("cmd_vel", Twist, self.cmd_cb)
        rospy.Subscriber("/capit_cmd", Float32, self.capit_cb)
        self.last_cmd_time = rospy.get_time()

        rospy.Timer(rospy.Duration(0.1), self.update_system)

        self.read_thread = threading.Thread(target=self.read_from_serial)
        self.read_thread.daemon = True
        self.read_thread.start()

    def capit_cb(self, msg):
        self.capit_state = msg.data
        rospy.loginfo(f"Capit State: {self.capit_state}")

    def read_from_serial(self):
        while not rospy.is_shutdown():
            if self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line.startswith("FEEDBACK:"):
                        clean_data = line.replace("FEEDBACK:", "").split(',')
                        data_floats = [float(x) for x in clean_data]
                        
                        msg = Float32MultiArray()
                        msg.data = data_floats
                        self.feedback_pub.publish(msg)
                except Exception as e:
                    rospy.logwarn(f"Error parsing serial data: {e}")

    # ==================================================
    # CALLBACK CMD_VEL (PERBAIKAN KINEMATIKA MECANUM)
    # ==================================================
    def cmd_cb(self, msg):
        self.last_cmd_time = rospy.get_time()

        x = msg.linear.x    # Maju (+) / Mundur (-)
        y = msg.linear.y    # Geser Kiri (+) / Geser Kanan (-)
        z = msg.angular.z   # Putar Kiri (+) / Putar Kanan (-)

        fl = x - y - z  # Front Left
        fr = x + y + z  # Front Right
        rl = x + y - z  # Rear Left
        rr = x - y + z  # Rear Right
        
        tl = x
        tr = x
        
        max_rpm = 120.0
        fl_sp = fl * max_rpm
        fr_sp = fr * max_rpm
        rl_sp = rl * max_rpm
        rr_sp = rr * max_rpm

        max_pwm = 255.0
        tl_sp = tl * max_pwm
        tr_sp = tr * max_pwm
        
        proxyCapit = self.capit_state 
        data2 = 255.0
        data3 = 255.0
        data4 = 255.0
        
        # self.send_setpoint(fl_sp, fr_sp, rl_sp, rr_sp, -tl_sp, -tr_sp, proxyCapit, data2, data3, data4)
        # self.send_setpoint(fl_sp, fr_sp, rl_sp, rr_sp, -255, -255, proxyCapit, data2, data3, data4)
        self.send_setpoint(fl_sp, fr_sp, rl_sp, rr_sp, 0, 0, proxyCapit, data2, data3, data4)
        # self.send_setpoint(fl_sp, fr_sp, rl_sp, rr_sp, 0, 0, 1, data2, data3, data4)
        # self.send_setpoint(fl_sp, 0, 0, 0, 0, 0, proxyCapit, data2, data3, data4)

    def send_setpoint(self, fl, fr, rl, rr, tl, tr, data1, data2, data3, data4):
        data = f"{fl:.1f},{fr:.1f},{rl:.1f},{rr:.1f},{tl:.1f},{tr:.1f},{data1:.1f},{data2:.1f},{data3:.1f},{data4:.1f}\n"
        self.ser.write(data.encode())

    def update_system(self, event):
        if rospy.get_time() - self.last_cmd_time > 0.5:
            self.send_setpoint(0, 0, 0, 0, 0, 0, self.capit_state, 0, 0, 0)


if __name__ == '__main__':
    try:
        controller = BaseController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass