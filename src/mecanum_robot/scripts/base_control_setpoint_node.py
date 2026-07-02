#!/usr/bin/env python3

import rospy
import serial #type: ignore
import threading

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray

class BaseController:

    def __init__(self):
        rospy.init_node('base_controller_node')

        # --- Variabel Status ---
        self.capit_state = 0.0 
        self.climb_state = 0.0
        self.motor_climb_state = 0.0
        self.sensor_val = 0.0 # Menyimpan data sensor index ke-2
        self.yaw_pub = 0.0

        # Koneksi Serial
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)
        except:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)

        # Publisher
        self.yaw_pub = rospy.Publisher('robot_yaw', Float32, queue_size=10)
        self.feedback_pub = rospy.Publisher('motor_feedback', Float32MultiArray, queue_size=10)

        # Subscriber
        rospy.Subscriber("cmd_vel", Twist, self.cmd_cb)
        rospy.Subscriber("capit_cmd", Float32, self.capit_cb) 
        rospy.Subscriber("climb", Float32, self.climb_cb)
        rospy.Subscriber("motor_climb", Float32, self.motor_climb_cb) 
        
        # Subscriber baru untuk topik sensor
        rospy.Subscriber("sensor", Float32MultiArray, self.sensor_cb)
        
        self.last_cmd_time = rospy.get_time()

        rospy.Timer(rospy.Duration(0.1), self.update_system)

        self.read_thread = threading.Thread(target=self.read_from_serial)
        self.read_thread.daemon = True
        self.read_thread.start()

    def capit_cb(self, msg):
        self.capit_state = msg.data
        rospy.loginfo(f"Capit State: {self.capit_state}")

    def climb_cb(self, msg):
        self.climb_state = msg.data

    def motor_climb_cb(self, msg):
        self.motor_climb_state = msg.data

    # Callback untuk topik sensor
    def sensor_cb(self, msg):
        # Memastikan array memiliki setidaknya 3 elemen (index 0, 1, 2)
        self.yaw_pub.publish(msg.data[3]) 
        if len(msg.data) > 2:
            self.sensor_val = msg.data[2]
        else:
            rospy.logwarn_throttle(1, "Data array sensor terlalu pendek. Index 2 tidak ditemukan.")

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
        
        max_rpm = 120.0
        fl_sp = fl * max_rpm
        fr_sp = fr * max_rpm
        rl_sp = rl * max_rpm
        rr_sp = rr * max_rpm

        proxyCapit = self.capit_state 
        data2 = self.climb_state  
        data7 = self.motor_climb_state
        data4 = self.sensor_val # Menggunakan data sensor index ke-2
        
        data8_dummy = 88.8
        data9_dummy = 99.0
        
        # Format pengiriman 8 parameter (4 motor roda, climb, motor_climb, capit, data4)
        self.send_setpoint(fl_sp, fr_sp, rl_sp, rr_sp, data2, data4, proxyCapit, data7, data8_dummy, data9_dummy)

    def send_setpoint(self, fl, fr, rl, rr, climb, cahaya, capit, motor_climb, d8, d9):
        # Format pengiriman diperbarui untuk 10 parameter
        data = f"{fl:.1f},{fr:.1f},{rl:.1f},{rr:.1f},{climb:.1f},{cahaya:.1f},{capit:.1f},{motor_climb:.1f},{d8:.1f},{d9:.1f}\n"
        self.ser.write(data.encode())

    def update_system(self, event):
        if rospy.get_time() - self.last_cmd_time > 0.5:
            # Kirim dummy juga saat idle
            self.send_setpoint(0, 0, 0, 0, self.climb_state, self.sensor_val, self.capit_state, self.motor_climb_state, 0.0, 0.0)

if __name__ == '__main__':
    try:
        controller = BaseController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass