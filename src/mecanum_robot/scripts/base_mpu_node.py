#!/usr/bin/env python3

import rospy
import serial #type: ignore
import threading

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray

class BaseController:

    def __init__(self):
        rospy.init_node('base_controller_node')

        # --- Variabel Status Sistem ---
        self.capit_state = 0.0 
        self.climb_state = 0.0
        self.motor_climb_state = 0.0
        self.sensor_val = 0.0 
        
        # --- Variabel Navigasi MPU6050 (PID Controller) ---
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.is_moving_straight = False
        
        # Variabel PID
        self.Kp = 0.05
        self.Ki = 0.003
        self.Kd = 0.025
        self.integral = 0.0
        self.prev_error = 0.0

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
        rospy.Subscriber("sensor", Float32MultiArray, self.sensor_cb)
        
        self.last_cmd_time = rospy.get_time()

        rospy.Timer(rospy.Duration(0.1), self.update_system)

        self.read_thread = threading.Thread(target=self.read_from_serial)
        self.read_thread.daemon = True
        self.read_thread.start()

    def capit_cb(self, msg):
        self.capit_state = msg.data

    def climb_cb(self, msg):
        self.climb_state = msg.data

    def motor_climb_cb(self, msg):
        self.motor_climb_state = msg.data

    def sensor_cb(self, msg):
        if len(msg.data) > 3:
            self.current_yaw = msg.data[3]
            self.yaw_pub.publish(self.current_yaw) 
        else:
            rospy.logwarn_throttle(1, "Data array sensor terlalu pendek. Index 3 tidak ditemukan.")

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
    # CALLBACK CMD_VEL (MECANUM + KOMPENSASI STRAFING)
    # ==================================================
    def cmd_cb(self, msg):
        self.last_cmd_time = rospy.get_time()

        x = msg.linear.x    
        # Tambahkan multiplier (1.2 hingga 1.5) pada sumbu Y untuk mengkompensasi loss tenaga gesekan roller mecanum
        y = msg.linear.y * 1.5    
        z = msg.angular.z   

        z_final = z 

        is_translating = (abs(x) >= 0.01 or abs(y) >= 0.01)

        # --- LOGIKA HEADING HOLD PID ---
        if abs(z) > 0.05 or not is_translating:
            self.is_moving_straight = False
            self.target_yaw = self.current_yaw
            self.integral = 0.0 
            
        elif is_translating and abs(z) <= 0.05:
            if not self.is_moving_straight:
                self.target_yaw = self.current_yaw
                self.is_moving_straight = True

            error = self.target_yaw - self.current_yaw
            error = (error + 180.0) % 360.0 - 180.0

            self.integral += error
            self.integral = max(min(self.integral, 20.0), -20.0) 

            derivative = error - self.prev_error
            
            z_correction = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
            self.prev_error = error

            # NAIKKAN CLAMPING PID: Beri tenaga ekstra hingga 0.8 (dari sebelumnya 0.5) 
            # agar sistem bisa mengoreksi kemiringan saat bergerak serong/menyamping dengan lebih kuat
            z_correction = max(min(z_correction, 0.8), -0.8)
            
            z_final = -z_correction 

        # --- KINEMATIKA MECANUM MURNI ---
        fl = x - y - z_final
        fr = x + y + z_final
        rl = x + y - z_final
        rr = x - y + z_final
        
        # --- PERLINDUNGAN KECEPATAN (ANTI-SURGING) ---
        max_wheel_speed = max(abs(fl), abs(fr), abs(rl), abs(rr))
        
        if max_wheel_speed > 1.0:
            fl = fl / max_wheel_speed
            rl = rl / max_wheel_speed
            fr = fr / max_wheel_speed
            rr = rr / max_wheel_speed

        # KONVERSI KE SETPOINT RPM
        max_rpm = 120.0
        fl_sp = fl * max_rpm
        fr_sp = fr * max_rpm
        rl_sp = rl * max_rpm
        rr_sp = rr * max_rpm

        self.send_setpoint(fl_sp, fr_sp, rl_sp, rr_sp, 
                           self.climb_state, self.sensor_val, self.capit_state, 
                           self.motor_climb_state, 88.8, 99.0)
    def send_setpoint(self, fl, fr, rl, rr, climb, cahaya, capit, motor_climb, d8, d9):
        data = f"{fl:.1f},{fr:.1f},{rl:.1f},{rr:.1f},{climb:.1f},{cahaya:.1f},{capit:.1f},{motor_climb:.1f},{d8:.1f},{d9:.1f}\n"
        self.ser.write(data.encode())

    def update_system(self, event):
        if rospy.get_time() - self.last_cmd_time > 0.5:
            self.is_moving_straight = False
            self.target_yaw = self.current_yaw
            self.send_setpoint(0, 0, 0, 0, self.climb_state, self.sensor_val, self.capit_state, self.motor_climb_state, 0.0, 0.0)

if __name__ == '__main__':
    try:
        controller = BaseController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass