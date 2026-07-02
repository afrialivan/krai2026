#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Twist
from enum import Enum

class RobotState(Enum):
    INIT = 0
    MAJU_1 = 1
    JEDA_MAJU_1 = 2
    GESER_KANAN = 3
    GESER_KANAN_DELAY = 4
    JEDA_GESER = 5
    MAJU_2 = 6
    CLIMB = 7
    MOTOR_CLIMB = 8            
    CLIMB_2 = 9                
    MAJU_SEBELUM_PUTAR = 10      # Status baru: Maju 2 detik
    PUTAR_KANAN_90 = 11          
    WAIT_FOR_INSTRUCTIONS = 12

class RobotController:
    def __init__(self):
        rospy.init_node('ultrasonic_controller_node', anonymous=True)

        self.state = RobotState.INIT
        self.state_start_time = None

        # --- KONFIGURASI PARAMETER PERULANGAN (LOOP) ---
        self.jumlah_siklus_manjat = 0     
        self.target_maksimal_manjat = 3   

        # --- KONFIGURASI PARAMETER GERAK ---
        self.target_depan_bawah_1 = 40.0 + 5.0
        self.toleransi_bawah = 0.5
        self.target_depan_atas = 50.0  
        self.waktu_tunda_kanan = rospy.Duration(1.2) 
        self.target_depan_bawah_2 = 6.0 

        self.waktu_motor_climb = rospy.Duration(2.0)  
        self.waktu_climb_2 = rospy.Duration(8.0)      
        self.waktu_jeda_sejenak = rospy.Duration(1.5) 
        self.waktu_maju_sebelum_putar = rospy.Duration(0.7) # Durasi maju sebelum putar
        
        self.vel_maju_start = 0.7
        self.vel_maju = 0.7
        self.vel_kanan = -2.5
        self.vel_putar = -1  
        self.target_sudut = 170.0 

        # --- VARIABEL SENSOR ---
        self.jarak_depan_bawah = None
        self.jarak_depan_atas = None
        self.sensor_climb_stop = None
        
        self.current_yaw = None
        self.last_yaw_tracking = None
        self.accumulated_yaw = 0.0

        # --- PUBLISHER & SUBSCRIBER ---
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.climb_pub = rospy.Publisher('/climb', Float32, queue_size=10) 
        self.motor_climb_pub = rospy.Publisher('/motor_climb', Float32, queue_size=10) 
        
        self.sensor_sub = rospy.Subscriber('sensor', Float32MultiArray, self.sensor_callback, queue_size=1)
        self.yaw_sub = rospy.Subscriber('/robot_yaw', Float32, self.yaw_callback, queue_size=1)

        self.rate = rospy.Rate(50)

    def sensor_callback(self, msg):
        if len(msg.data) >= 10:
            self.jarak_depan_bawah = msg.data[0]
            self.sensor_climb_stop = msg.data[6]  
            self.jarak_depan_atas = msg.data[9]
        else:
            rospy.logwarn_throttle(2, "Data array sensor kurang dari 10 index!")

    def yaw_callback(self, msg):
        new_yaw = msg.data
        
        if self.current_yaw is None:
            self.current_yaw = new_yaw
            return

        diff = new_yaw - self.current_yaw
        
        while diff > 180.0: diff -= 360.0
        while diff < -180.0: diff += 360.0

        if abs(diff) < 20.0:
            self.current_yaw = new_yaw
        else:
            rospy.logwarn_throttle(1, f"Anomali MPU diabaikan! Ada lonjakan mendadak sebesar {diff:.2f} derajat.")

    def run(self):
        while not rospy.is_shutdown():
            if self.jarak_depan_bawah is None or self.jarak_depan_atas is None or self.sensor_climb_stop is None or self.current_yaw is None:
                rospy.loginfo_throttle(2, "Menunggu data dari topik sensor dan robot_yaw...")
                self.rate.sleep()
                continue

            if self.state == RobotState.INIT:
                rospy.loginfo("Sistem siap. Memulai gerakan MAJU TAHAP 1.")
                self.state = RobotState.MAJU_1

            cmd = Twist()

            # --- LOGIKA FINITE STATE MACHINE (FSM) ---
            
            if self.state == RobotState.MAJU_1:
                cmd.linear.x = self.vel_maju_start
                if 0.0 < self.jarak_depan_bawah <= (self.target_depan_bawah_1 + self.toleransi_bawah):
                    rospy.loginfo(f"Jarak bawah awal tercapai ({self.jarak_depan_bawah:.2f} cm). BERHENTI SEJENAK.")
                    self.state_start_time = rospy.Time.now()
                    self.state = RobotState.JEDA_MAJU_1

            elif self.state == RobotState.JEDA_MAJU_1:
                cmd.linear.x = 0.0  
                elapsed_time = rospy.Time.now() - self.state_start_time
                if elapsed_time >= self.waktu_jeda_sejenak:
                    rospy.loginfo("Jeda selesai. Mulai GESER KANAN.")
                    self.state = RobotState.GESER_KANAN

            elif self.state == RobotState.GESER_KANAN:
                cmd.linear.y = self.vel_kanan
                if self.jarak_depan_atas > self.target_depan_atas or self.jarak_depan_atas < 0.0:
                    rospy.loginfo(f"Jarak atas lolos ({self.jarak_depan_atas:.2f} cm). Menahan gerakan kanan {self.waktu_tunda_kanan.to_sec()} detik.")
                    self.state_start_time = rospy.Time.now()
                    self.state = RobotState.GESER_KANAN_DELAY

            elif self.state == RobotState.GESER_KANAN_DELAY:
                cmd.linear.y = self.vel_kanan
                elapsed_time = rospy.Time.now() - self.state_start_time
                if elapsed_time >= self.waktu_tunda_kanan:
                    rospy.loginfo("Waktu geser selesai. BERHENTI SEJENAK.")
                    self.state_start_time = rospy.Time.now()
                    self.state = RobotState.JEDA_GESER

            elif self.state == RobotState.JEDA_GESER:
                cmd.linear.y = 0.0  
                elapsed_time = rospy.Time.now() - self.state_start_time
                if elapsed_time >= self.waktu_jeda_sejenak:
                    rospy.loginfo("Jeda selesai. Memulai rangkaian MAJU dan MANJAT.")
                    self.state = RobotState.MAJU_2

            elif self.state == RobotState.MAJU_2:
                cmd.linear.x = self.vel_maju
                if 0.0 < self.jarak_depan_bawah <= (self.target_depan_bawah_2 + self.toleransi_bawah):
                    rospy.loginfo(f"Target 10cm tercapai ({self.jarak_depan_bawah:.2f} cm). Mulai proses CLIMB (Siklus {self.jumlah_siklus_manjat + 1}).")
                    self.state = RobotState.CLIMB

            elif self.state == RobotState.CLIMB:
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                
                climb_msg = Float32()
                climb_msg.data = 2.0
                self.climb_pub.publish(climb_msg)
                
                if self.sensor_climb_stop == 0.0:
                    rospy.loginfo("Sensor index 6 bernilai 0. Lanjut ke MOTOR_CLIMB.")
                    stop_msg = Float32()
                    stop_msg.data = 0.0
                    self.climb_pub.publish(stop_msg)
                    self.state_start_time = rospy.Time.now()
                    self.state = RobotState.MOTOR_CLIMB

            elif self.state == RobotState.MOTOR_CLIMB:
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                
                motor_climb_msg = Float32()
                motor_climb_msg.data = 2.0
                self.motor_climb_pub.publish(motor_climb_msg)
                
                elapsed_time = rospy.Time.now() - self.state_start_time
                if elapsed_time >= self.waktu_motor_climb:
                    rospy.loginfo("Durasi motor_climb selesai. Lanjut ke CLIMB tahap 2.")
                    stop_msg = Float32()
                    stop_msg.data = 0.0
                    self.motor_climb_pub.publish(stop_msg)
                    self.state_start_time = rospy.Time.now()
                    self.state = RobotState.CLIMB_2

            elif self.state == RobotState.CLIMB_2:
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                
                climb2_msg = Float32()
                climb2_msg.data = 1.0
                self.climb_pub.publish(climb2_msg)
                
                elapsed_time = rospy.Time.now() - self.state_start_time
                if elapsed_time >= self.waktu_climb_2:
                    stop_msg = Float32()
                    stop_msg.data = 0.0
                    self.climb_pub.publish(stop_msg)
                    
                    self.jumlah_siklus_manjat += 1
                    
                    if self.jumlah_siklus_manjat < self.target_maksimal_manjat:
                        rospy.loginfo(f"=== Siklus Manjat {self.jumlah_siklus_manjat} dari {self.target_maksimal_manjat} selesai! MENGULANG KEMBALI KE MAJU_2. ===")
                        self.state = RobotState.MAJU_2
                    else:
                        # SETELAH 3 KALI MANJAT, LANJUT KE MAJU 2 DETIK
                        rospy.loginfo("=== SELURUH 3 SIKLUS MANJAT SELESAI. Maju selama 2 detik sebelum rotasi. ===")
                        self.state_start_time = rospy.Time.now()
                        self.state = RobotState.MAJU_SEBELUM_PUTAR

            # 10. MAJU 2 DETIK SEBELUM PUTAR
            elif self.state == RobotState.MAJU_SEBELUM_PUTAR:
                cmd.linear.x = self.vel_maju_start  # Bisa diganti self.vel_maju jika ingin lebih pelan
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0
                
                elapsed_time = rospy.Time.now() - self.state_start_time
                if elapsed_time >= self.waktu_maju_sebelum_putar:
                    rospy.loginfo("Maju 2 detik selesai. Memulai Rotasi 90 Derajat ke Kanan.")
                    # Persiapan tracking sudut sebelum masuk ke state putar
                    self.accumulated_yaw = 0.0
                    self.last_yaw_tracking = self.current_yaw
                    self.state = RobotState.PUTAR_KANAN_90

            # 11. ROTASI 90 DERAJAT KE KANAN
            elif self.state == RobotState.PUTAR_KANAN_90:
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.angular.z = self.vel_putar 
                
                diff = self.current_yaw - self.last_yaw_tracking
                
                while diff > 180.0: diff -= 360.0
                while diff < -180.0: diff += 360.0
                
                self.accumulated_yaw += diff
                self.last_yaw_tracking = self.current_yaw
                
                if abs(self.accumulated_yaw) >= self.target_sudut:
                    rospy.loginfo(f"Rotasi 90 derajat selesai (Total pergeseran terbaca: {abs(self.accumulated_yaw):.2f} derajat). Beralih ke standby.")
                    self.state = RobotState.WAIT_FOR_INSTRUCTIONS

            # 12. DIAM MENUNGGU ARAHAN SELANJUTNYA
            elif self.state == RobotState.WAIT_FOR_INSTRUCTIONS:
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0
                rospy.loginfo_throttle(5, "Robot dalam status standby, semua rangkaian aksi selesai.")

            self.cmd_pub.publish(cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass