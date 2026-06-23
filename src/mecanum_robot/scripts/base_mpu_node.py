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
        self.last_cmd_time = rospy.get_time()

        # --- Variabel IMU & PID Heading Hold ---
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.is_moving_straight = False
        
        # Parameter PID (Silakan disesuaikan saat tuning di lantai)
        self.Kp = 0.03  
        self.Ki = 0.000501 
        self.Kd = 0.015  
        
        self.prev_error = 0.0
        self.integral = 0.0

        # --- Koneksi Serial ---
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

        # Timer untuk Watchdog
        rospy.Timer(rospy.Duration(0.02), self.update_system)

        # Thread untuk membaca feedback dari serial
        self.read_thread = threading.Thread(target=self.read_from_serial)
        self.read_thread.daemon = True
        self.read_thread.start()

    def capit_cb(self, msg):
        self.capit_state = msg.data
        rospy.loginfo(f"Capit State: {self.capit_state}")

    # ==================================================
    # BACA SERIAL & TANGKAP DATA YAW DI INDEKS 10
    # ==================================================
    def read_from_serial(self):
        while not rospy.is_shutdown():
            if self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line.startswith("FEEDBACK:"):
                        clean_data = line.replace("FEEDBACK:", "").split(',')
                        data_floats = [float(x) for x in clean_data]
                        
                        # Pastikan panjang array sesuai (11 data: indeks 0 s/d 10)
                        if len(data_floats) >= 11:
                            # Update variabel yaw terkini dari MPU6050
                            self.current_yaw = data_floats[10]
                            
                            # Publish ke ROS topic agar bisa dilihat grafiknya (rqt_plot)
                            self.yaw_pub.publish(self.current_yaw)

                        msg = Float32MultiArray()
                        msg.data = data_floats
                        self.feedback_pub.publish(msg)
                except Exception as e:
                    # Abaikan error parsing sesaat ketika serial baru terkoneksi
                    pass 

    # ==================================================
    # CALLBACK CMD_VEL (KINEMATIKA DIFERENSIAL + PID YAW)
    # ==================================================
    def cmd_cb(self, msg):
        self.last_cmd_time = rospy.get_time()

        # Input kecepatan dari ROS (-1.0 sampai 1.0)
        x = msg.linear.x    # Maju (+) / Mundur (-)
        z = msg.angular.z   # Putar Kiri (+) / Putar Kanan (-)

        z_final = z # Nilai rotasi bawaan dari joy/keyboard

        # --- LOGIKA HEADING HOLD ---
        # 1. Jika ada perintah belok manual dari user (z != 0) atau robot sedang diam
        if abs(z) > 0.05 or abs(x) < 0.01:
            self.is_moving_straight = False
            self.target_yaw = self.current_yaw # Mengikuti ke mana pun robot menghadap
            self.integral = 0.0 # Reset tabungan integral PID
            
        # 2. Jika user HANYA menekan tombol maju/mundur (ingin jalan lurus konstan)
        elif abs(x) >= 0.01 and abs(z) <= 0.05:
            if not self.is_moving_straight:
                # Kunci target yaw di sepersekian detik pertama mulai lurus
                self.target_yaw = self.current_yaw
                self.is_moving_straight = True

            # Hitung Error (Normalisasi matematika untuk loncatan 180/-180 derajat)
            error = self.target_yaw - self.current_yaw
            error = (error + 180.0) % 360.0 - 180.0

            # Kalkulasi PID dengan perlindungan Anti-Windup pada Integral
            self.integral += error
            self.integral = max(min(self.integral, 20.0), -20.0) # Batasi maksimal memori di angka 20

            derivative = error - self.prev_error
            
            z_correction = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
            self.prev_error = error

            # Clamping output koreksi (maksimal pengaruh rotasi 0.5 rad/s agar tidak memutar berlebihan)
            z_correction = max(min(z_correction, 0.5), -0.5)
            
            # Balik arah koreksi (Polaritas) agar sinkron dengan sisi motor yang berlawanan
            z_final = -z_correction 

        # --- KINEMATIKA DIFERENSIAL MURNI ---
        fl = x - z_final  # Front Left
        rl = x - z_final  # Rear Left
        fr = x + z_final  # Front Right
        rr = x + z_final  # Rear Right
        
        # --- PERLINDUNGAN KECEPATAN (ANTI-SURGING MAX SPEED LIMITER) ---
        # Cegah nilai x + z menghasilkan kecepatan melebihi 1.0 (robot jadi membalap diagonal)
        max_wheel_speed = max(abs(fl), abs(fr), abs(rl), abs(rr))
        
        if max_wheel_speed > 1.0:
            fl = fl / max_wheel_speed
            rl = rl / max_wheel_speed
            fr = fr / max_wheel_speed
            rr = rr / max_wheel_speed

        # Logika Motor Tambahan/Lift 
        tl = x
        tr = x
        
        # KONVERSI KE SETPOINT RPM (Maksimal RPM Motor = 120)
        max_rpm = 120.0
        fl_sp = fl * max_rpm
        fr_sp = fr * max_rpm
        rl_sp = rl * max_rpm
        rr_sp = rr * max_rpm

        # KONVERSI UNTUK MOTOR LIFT (Maksimal PWM = 255)
        max_pwm = 255.0
        tl_sp = tl * max_pwm
        tr_sp = tr * max_pwm
        
        # Status tambahan untuk dikirim via string
        proxyCapit = self.capit_state 
        data2 = 255.0
        data3 = 255.0
        data4 = 255.0
        
        # Eksekusi kirim ke ESP32
        self.send_setpoint(fl_sp, fr_sp, rl_sp, rr_sp, 0, 0, proxyCapit, data2, data3, data4)

    def send_setpoint(self, fl, fr, rl, rr, tl, tr, data1, data2, data3, data4):
        data = f"{fl:.1f},{fr:.1f},{rl:.1f},{rr:.1f},{tl:.1f},{tr:.1f},{data1:.1f},{data2:.1f},{data3:.1f},{data4:.1f}\n"
        self.ser.write(data.encode())

    def update_system(self, event):
        # Watchdog: Hentikan robot jika kehilangan koneksi dengan cmd_vel > 0.5 detik
        if rospy.get_time() - self.last_cmd_time > 0.5:
            self.send_setpoint(0, 0, 0, 0, 0, 0, self.capit_state, 0, 0, 0)
            self.is_moving_straight = False
            self.target_yaw = self.current_yaw


if __name__ == '__main__':
    try:
        controller = BaseController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass