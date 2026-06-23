#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist

# ==============================================================================
# KONFIGURASI ODOMETRI & PERGERAKAN (Dari Referensi Waypoint)
# ==============================================================================
WHEEL_DIAMETER_CM   = 9.5       
TRACK_WIDTH_CM      = 80.0      
PPR                 = 385       
LINEAR_SCALE        = 1.0       
ANGULAR_SCALE       = 0.127     
TURN_TOLERANCE_DEG  = 2.0       
KP_TURN             = 0.02      
MIN_TURN_SPEED      = 0.15      
IDX_LEFT            = 1         
IDX_RIGHT           = 2         

class CapitController:

    def __init__(self):
        rospy.init_node('capit_controller_node')

        # --- Publisher & Subscriber ---
        self.capit_pub = rospy.Publisher('/capit_cmd', Float32, queue_size=10)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/motor_feedback', Float32MultiArray, self.feedback_cb)

        # --- Variabel Odometri ---
        self.wheel_circumference = math.pi * WHEEL_DIAMETER_CM
        self.prev_counts        = None
        self.total_distance_cm  = 0.0
        self.encoder_yaw_deg    = 0.0

        # --- Variabel State Machine ---
        # State: WAITING -> MOVING_BACK -> CLAMPING -> MOVING_FORWARD -> TURNING -> DONE
        self.state = 'WAITING'
        self.start_distance = 0.0
        self.start_yaw = 0.0

        # --- Target Pergerakan ---
        self.target_back_cm = 20.0       # Jarak MUNDUR (20 cm)
        self.target_forward_cm = 30.0    # Jarak MAJU (30 cm)
        
        self.move_speed = 0.5            # Base kecepatan linear 
        self.target_turn_deg = -23.0     # Putar kanan (mines) sebesar 23
        self.turn_speed = 2.0            # Maksimal kecepatan putar

        rospy.loginfo("Node Capit+Gerak aktif. Menunggu deteksi proximity...")
        
    def feedback_cb(self, msg):
        if len(msg.data) < 9:
            return

        # ==========================================
        # 1. UPDATE ODOMETRI DARI ENCODER
        # ==========================================
        counts = {
            'L': msg.data[IDX_LEFT],
            'R': msg.data[IDX_RIGHT],
        }

        if self.prev_counts is None:
            self.prev_counts = counts
            return

        d_tick_L = counts['L'] - self.prev_counts['L']
        d_tick_R = counts['R'] - self.prev_counts['R']

        d_cm_L = (d_tick_L / PPR) * self.wheel_circumference
        d_cm_R = (d_tick_R / PPR) * self.wheel_circumference

        delta_linear = ((d_cm_L + d_cm_R) / 2.0) * LINEAR_SCALE
        self.total_distance_cm += delta_linear

        delta_yaw_rad = ((d_cm_R - d_cm_L) / TRACK_WIDTH_CM) * ANGULAR_SCALE
        self.encoder_yaw_deg += math.degrees(delta_yaw_rad)

        self.prev_counts = counts

        # ==========================================
        # 2. LOGIKA STATE MACHINE
        # ==========================================
        cmd = Twist()
        
        # --- STATE 1: MENUNGGU PROXIMITY ---
        if self.state == 'WAITING':
            current_proximity = msg.data[8]
            if current_proximity == 1.0: 
                rospy.loginfo(f"Objek terdeteksi! Robot mulai mundur {self.target_back_cm} cm...")
                self.start_distance = self.total_distance_cm
                self.state = 'MOVING_BACK'

        # --- STATE 2: MUNDUR 20 CM ---
        elif self.state == 'MOVING_BACK':
            jarak_tempuh = abs(self.total_distance_cm - self.start_distance)
            
            if jarak_tempuh < self.target_back_cm:
                cmd.linear.x = -self.move_speed  # Kecepatan negatif = MUNDUR
            else:
                cmd.linear.x = 0.0
                self.cmd_pub.publish(cmd) # Berhenti sebelum capit
                self.state = 'CLAMPING'
                rospy.loginfo("Mundur selesai. Mengeksekusi capit...")
                self.execute_capit()

        # --- STATE 3: PROSES CAPIT (WAITING DELAY) ---
        elif self.state == 'CLAMPING':
            # Memberikan jeda waktu di dalam callback agar capit benar-benar menutup
            rospy.sleep(1.5) 
            self.start_distance = self.total_distance_cm
            self.state = 'MOVING_FORWARD'
            rospy.loginfo(f"Capit selesai. Mulai maju {self.target_forward_cm} cm...")

        # --- STATE 4: MAJU KEMBALI 30 CM ---
        elif self.state == 'MOVING_FORWARD':
            jarak_tempuh = abs(self.total_distance_cm - self.start_distance)
            
            if jarak_tempuh < self.target_forward_cm:
                cmd.linear.x = self.move_speed   # Kecepatan positif = MAJU
            else:
                cmd.linear.x = 0.0
                self.cmd_pub.publish(cmd) # Berhenti sebelum belok
                self.start_yaw = self.encoder_yaw_deg
                self.state = 'TURNING'
                rospy.loginfo("Maju selesai. Mulai memutar ke kanan...")

        # --- STATE 5: PUTAR KANAN 23 ---
        elif self.state == 'TURNING':
            target_yaw = self.start_yaw + self.target_turn_deg
            error = (target_yaw - self.encoder_yaw_deg + 180) % 360 - 180

            if abs(error) > TURN_TOLERANCE_DEG:
                speed = KP_TURN * abs(error)
                speed = max(MIN_TURN_SPEED, min(speed, abs(self.turn_speed)))
                cmd.angular.z = speed if error > 0 else -speed
            else:
                cmd.angular.z = 0.0
                self.state = 'DONE'
                rospy.loginfo("Selesai! Seluruh urutan aksi berhasil dieksekusi.")
                
            self.cmd_pub.publish(cmd)
            
        # --- STATE 6: SELESAI ---
        elif self.state == 'DONE':
            # Tetap publish nol agar robot tidak hanyut/bergerak sendiri
            self.cmd_pub.publish(cmd)

    def execute_capit(self):
        cmd_msg = Float32()
        cmd_msg.data = 1.0 
        self.capit_pub.publish(cmd_msg)
        rospy.loginfo("Perintah capit (1.0) dikirim ke /capit_cmd.")

    def reset_capit(self):
        self.state = 'WAITING'
        cmd_msg = Float32()
        cmd_msg.data = 0.0 
        self.capit_pub.publish(cmd_msg)
        rospy.loginfo("Sistem direset. Capit terbuka. Menunggu objek baru...")

if __name__ == '__main__':
    try:
        controller = CapitController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass