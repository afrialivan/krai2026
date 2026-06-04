#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class AutonomousDiffNav:
    def __init__(self):
        rospy.init_node('autonomous_diff_node')

        # ==================================================
        # 1. PARAMETER FISIK ROBOT
        # ==================================================
        self.wheel_radius = 0.05   # Radius roda dalam meter
        self.ticks_per_rev = 330.0 # Jumlah tick per putaran roda
        
        # Lebar robot (jarak antara roda kiri dan kanan)
        self.wheel_base = 0.80     # 80cm = 0.8 meter

        # ==================================================
        # 2. VARIABEL ODOMETRY
        # ==================================================
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.prev_ticks = [0, 0, 0, 0] # [FR, RL, RR, FL]
        self.first_read = True

        # ==================================================
        # 3. DAFTAR TITIK TARGET (WAYPOINTS)
        # ==================================================
        # Format: (X, Y) dalam meter
        self.waypoints = [
            (1.0, 0.0),   # Titik 1: Maju 1 meter
            # (1.0, 1.0),   # Titik 2: Belok kiri, maju 1 meter
            # (2.0, 1.0),   # Titik 3: Belok kanan, maju 1 meter
            (0.0, 0.0)    # Titik 4: Kembali ke awal
        ]
        self.current_wp_idx = 0

        # Publisher & Subscriber
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/motor_feedback', Float32MultiArray, self.encoder_cb)
        
        # Loop Control pada 10Hz
        rospy.Timer(rospy.Duration(0.1), self.control_loop)

    # ==================================================
    # FUNGSI BANTUAN: NORMALISASI SUDUT
    # ==================================================
    def normalize_angle(self, angle):
        # Menjaga sudut error selalu berada di antara -PI sampai PI
        return math.atan2(math.sin(angle), math.cos(angle))

    # ==================================================
    # CALLBACK: MENGHITUNG POSISI (DIFFERENTIAL ODOMETRY)
    # ==================================================
    def encoder_cb(self, msg):
        if len(msg.data) >= 4:
            fr_tick, rl_tick, rr_tick, fl_tick = msg.data[0:4]

            if self.first_read:
                self.prev_ticks = [fr_tick, rl_tick, rr_tick, fl_tick]
                self.first_read = False
                return

            # Hitung delta tick
            d_fr = fr_tick - self.prev_ticks[0]
            d_rl = rl_tick - self.prev_ticks[1]
            d_rr = rr_tick - self.prev_ticks[2]
            d_fl = fl_tick - self.prev_ticks[3]
            self.prev_ticks = [fr_tick, rl_tick, rr_tick, fl_tick]

            # Rata-rata pergerakan sisi Kanan dan Kiri (Untuk 4 Roda Skid-Steer)
            d_right_ticks = (d_fr + d_rr) / 2.0
            d_left_ticks = (d_fl + d_rl) / 2.0

            # Konversi tick ke meter
            dist_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
            dist_right = d_right_ticks * dist_per_tick
            dist_left = d_left_ticks * dist_per_tick

            # Differential Kinematics
            d_center = (dist_right + dist_left) / 2.0
            d_theta = (dist_right - dist_left) / self.wheel_base

            # Update Odometry
            self.x += d_center * math.cos(self.theta + (d_theta / 2.0))
            self.y += d_center * math.sin(self.theta + (d_theta / 2.0))
            self.theta += d_theta
            self.theta = self.normalize_angle(self.theta)

    # ==================================================
    # KENDALI: P-CONTROLLER UNTUK WAYPOINTS
    # ==================================================
    def control_loop(self, event):
        if self.first_read: 
            return

        cmd = Twist()

        # Cek apakah semua titik sudah dicapai
        if self.current_wp_idx >= len(self.waypoints):
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            rospy.loginfo_throttle(5, "Semua waypoints telah dicapai!")
            return

        # Ambil target saat ini
        target_x, target_y = self.waypoints[self.current_wp_idx]

        # Hitung Error Global
        ex = target_x - self.x
        ey = target_y - self.y

        # Jarak ke target (Distance Error)
        distance = math.hypot(ex, ey)

        # Sudut ke target (Heading Error)
        target_theta = math.atan2(ey, ex)
        e_theta = self.normalize_angle(target_theta - self.theta)

        # Jika sudah cukup dekat dengan target (< 10 cm), pindah titik
        if distance < 0.10:
            rospy.loginfo(f"Mencapai Titik {self.current_wp_idx + 1}! Lanjut ke titik berikutnya.")
            self.current_wp_idx += 1
            return

        # --- LOGIKA GERAK DIFFERENTIAL ---
        # Konstanta Proportional
        Kp_linear = 0.5
        Kp_angular = 1.0

        # Jika robot belum menghadap target (error lebih dari ~17 derajat / 0.3 rad)
        # Putar di tempat terlebih dahulu agar lebih presisi
        if abs(e_theta) > 0.3:
            cmd.linear.x = 0.0
            cmd.angular.z = Kp_angular * e_theta
        else:
            # Jika sudah lumayan lurus, maju dan koreksi sedikit arahnya
            cmd.linear.x = Kp_linear * distance
            cmd.angular.z = Kp_angular * e_theta
        
        # Batasi kecepatan maksimum (M/s dan Rad/s)
        cmd.linear.x = max(-0.4, min(0.4, cmd.linear.x))
        cmd.angular.z = max(-0.8, min(0.8, cmd.angular.z))

        rospy.loginfo(f"Menuju Titik {self.current_wp_idx + 1} | Jarak: {distance:.2f}m | Sudut Error: {math.degrees(e_theta):.1f}°")
        
        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    try:
        nav = AutonomousDiffNav()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass