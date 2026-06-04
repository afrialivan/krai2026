#!/usr/bin/env python3

import rospy
import serial #type: ignore
import threading

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray


class BaseController:

    def __init__(self):
        rospy.init_node('base_controller_node')

        # Koneksi Serial
        try:
            # Sesuaikan port dengan yang terdeteksi di sistem (USB0 atau ACM0)
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)
        except:
            try:
                self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)
            except Exception as e:
                rospy.logerr(f"Gagal koneksi Serial: {e}")

        # Publisher
        # Sekarang yaw_pub akan berisi data dari MPU9250
        self.yaw_pub = rospy.Publisher('/robot_yaw', Float32, queue_size=10)
        self.feedback_pub = rospy.Publisher('/motor_feedback', Float32MultiArray, queue_size=10)

        # Subscriber
        rospy.Subscriber("cmd_vel", Twist, self.cmd_cb)

        self.last_cmd_time = rospy.get_time()

        # Timer untuk Watchdog (Stop motor jika kehilangan sinyal)
        rospy.Timer(rospy.Duration(0.1), self.update_system)

        # Thread untuk membaca feedback dari serial secara asinkron
        self.read_thread = threading.Thread(target=self.read_from_serial)
        self.read_thread.daemon = True
        self.read_thread.start()

    # ==================================================
    # BACA DATA DARI ESP32 (FEEDBACK + IMU YAW)
    # ==================================================
    def read_from_serial(self):
        while not rospy.is_shutdown():
            if self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    
                    if line.startswith("FEEDBACK:"):
                        clean_data = line.replace("FEEDBACK:", "").split(',')
                        
                        # Konversi string ke float
                        data_floats = [float(x) for x in clean_data]
                        
                        # --- PUBLISH MOTOR FEEDBACK ---
                        # Kita ambil 12 data pertama (RPM & Encoder)
                        msg_motor = Float32MultiArray()
                        msg_motor.data = data_floats[:12]
                        self.feedback_pub.publish(msg_motor)
                        
                        # --- PUBLISH YAW (IMU) ---
                        # Data ke-13 (index 12) adalah nilai Yaw dari MPU9250
                        if len(data_floats) >= 13:
                            yaw_val = data_floats[12]
                            self.yaw_pub.publish(Float32(yaw_val))
                        
                except Exception as e:
                    rospy.logwarn(f"Error parsing serial data: {e}")

    # ==================================================
    # CALLBACK CMD_VEL
    # ==================================================
    def cmd_cb(self, msg):
        self.last_cmd_time = rospy.get_time()

        x = msg.linear.x
        y = msg.linear.y
        z = msg.angular.z

        # KINEMATIKA (Sesuai dengan konfigurasi 6 roda kamu)
        # Urutan pengiriman ke ESP32: fl, fr, rl, rr, tl, tr
        fl = x - y - z
        fr = x + y + z
        rl = x - y - z
        rr = x + y + z
        tl = x - y - z
        tr = x + y + z

        # KONVERSI KE SETPOINT RPM
        max_rpm = 100.0
        self.send_setpoint(fl * max_rpm, fr * max_rpm, 
                           rl * max_rpm, rr * max_rpm, 
                           tl * max_rpm, tr * max_rpm)

    # ==================================================
    # KIRIM DATA KE ESP32
    # ==================================================
    def send_setpoint(self, fl, fr, rl, rr, tl, tr):
        data = f"{fl:.1f},{fr:.1f},{rl:.1f},{rr:.1f},{tl:.1f},{tr:.1f}\n"
        self.ser.write(data.encode())

    # ==================================================
    # WATCHDOG
    # ==================================================
    def update_system(self, event):
        if rospy.get_time() - self.last_cmd_time > 0.5:
            self.send_setpoint(0, 0, 0, 0, 0, 0)


if __name__ == '__main__':
    try:
        controller = BaseController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass