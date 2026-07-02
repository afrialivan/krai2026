#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Float32

class HolonomicRobot:
    def __init__(self):
        rospy.init_node('move_complex_node', anonymous=True)
        
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.capit_pub = rospy.Publisher('/capit_cmd', Float32, queue_size=10) 
        
        self.sensor_sub = rospy.Subscriber('/sensor', Float32MultiArray, self.sensor_callback)
        
        self.vel_msg = Twist()
        self.rate = rospy.Rate(50) # Loop 50 Hz
        
        self.distance = 999.0 
        self.prox_state = 1    
        self.trigger_distance = 17.0
        
        self.state = 'NORMAL'
        self.action_start_time = 0.0
        
        self.pause_duration = 0.2 # Waktu berhenti sebelum ganti arah
        self.capit_wait = 3.0     # Waktu tunggu agar capit benar-benar menutup sebelum bergerak

    def sensor_callback(self, msg):
        if len(msg.data) >= 2:
            raw_dist = msg.data[7]
            self.prox_state = msg.data[1]
            
            if raw_dist <= 0:
                self.distance = 999.0
            else:
                self.distance = raw_dist

    def run(self):
        while not rospy.is_shutdown():
            
            # --- STATE 1: NORMAL (Jalan ke Kiri Terus) ---
            if self.state == 'NORMAL':
                if self.prox_state == 0:
                    self.state = 'PAUSE_BEFORE_CAPIT'
                    self.action_start_time = rospy.get_time()
                    rospy.loginfo("Proximity aktif! Berhenti menetralkan momentum...")
                
                elif 0 < self.distance <= self.trigger_distance:
                    self.state = 'PAUSE_BEFORE_FORWARD'
                    self.action_start_time = rospy.get_time()
                    rospy.loginfo(f"Ultrasonik deteksi {self.distance:.1f}cm! Berhenti sebentar...")
                
                else:
                    self.vel_msg.linear.x = 0.0
                    self.vel_msg.linear.y = 2.9  # Kiri
                    self.vel_msg.angular.z = 0.0
                    self.cmd_pub.publish(self.vel_msg)

            # --- STATE 2: JEDA SEBELUM MAJU ---
            elif self.state == 'PAUSE_BEFORE_FORWARD':
                self.vel_msg.linear.x = 0.0
                self.vel_msg.linear.y = 0.0
                self.vel_msg.angular.z = 0.0
                self.cmd_pub.publish(self.vel_msg)
                
                if (rospy.get_time() - self.action_start_time) >= self.pause_duration:
                    self.state = 'FORWARD_STATE'
                    rospy.loginfo("Jeda selesai. Lanjut MAJU.")

            # --- STATE 3: MAJU (Ultrasonik) ---
            elif self.state == 'FORWARD_STATE':
                if self.prox_state == 0:
                    self.state = 'PAUSE_BEFORE_CAPIT'
                    self.action_start_time = rospy.get_time()
                    rospy.loginfo("Proximity aktif saat maju! Berhenti menetralkan momentum...")
                
                elif self.distance > self.trigger_distance:
                    self.state = 'NORMAL'
                    rospy.loginfo("Rintangan hilang. Kembali jalan NORMAL (kiri).")
                
                else:
                    self.vel_msg.linear.x = 0.3  # Maju
                    self.vel_msg.linear.y = 0.0
                    self.vel_msg.angular.z = 0.0
                    self.cmd_pub.publish(self.vel_msg)

            # --- STATE 4: JEDA SEBELUM CAPIT ---
            elif self.state == 'PAUSE_BEFORE_CAPIT':
                self.vel_msg.linear.x = 0.0
                self.vel_msg.linear.y = 0.0
                self.vel_msg.angular.z = 0.0
                self.cmd_pub.publish(self.vel_msg)
                
                if (rospy.get_time() - self.action_start_time) >= self.pause_duration:
                    self.state = 'CAPIT_SEQUENCE'
                    self.action_start_time = rospy.get_time()
                    rospy.loginfo("Jeda selesai. Mulai urutan pergerakan capit (Kiri 0.8 detik).")

            # --- STATE 5: URUTAN CAPIT ---
            elif self.state == 'CAPIT_SEQUENCE':
                self.vel_msg.linear.x = 0.0
                self.vel_msg.linear.y = 2.0 # Manuver kiri
                self.vel_msg.angular.z = 0.0
                self.cmd_pub.publish(self.vel_msg)
                
                elapsed_time = rospy.get_time() - self.action_start_time
                
                if elapsed_time >= 0.8: 
                    # Hentikan robot untuk mencapit
                    self.vel_msg.linear.x = 0.0
                    self.vel_msg.linear.y = 0.0
                    self.vel_msg.angular.z = 0.0
                    self.cmd_pub.publish(self.vel_msg)
                    
                    # Kirim perintah capit
                    cmd_msg = Float32()
                    cmd_msg.data = 1.0
                    self.capit_pub.publish(cmd_msg)
                    rospy.loginfo("Perintah capit dikirim. Menunggu capit menutup rapat...")
                    
                    # Pindah ke state jeda setelah capit
                    self.state = 'PAUSE_AFTER_CAPIT'
                    self.action_start_time = rospy.get_time()

            # --- STATE 6: JEDA SETELAH CAPIT (MENUNGGU CAPIT MENUTUP) ---
            elif self.state == 'PAUSE_AFTER_CAPIT':
                self.vel_msg.linear.x = 0.0
                self.vel_msg.linear.y = 0.0
                self.vel_msg.angular.z = 0.0
                self.cmd_pub.publish(self.vel_msg)
                
                if (rospy.get_time() - self.action_start_time) >= self.capit_wait: # Tunggu 1 detik
                    self.state = 'MOVE_RIGHT_SEQUENCE'
                    self.action_start_time = rospy.get_time()
                    rospy.loginfo("Mulai bergerak ke KANAN selama 2 detik.")

            # --- STATE 7: BERGERAK KANAN (2 DETIK) ---
            elif self.state == 'MOVE_RIGHT_SEQUENCE':
                self.vel_msg.linear.x = 0.0
                self.vel_msg.linear.y = -1.0 # Negatif Y untuk ke kanan secara holonomic
                self.vel_msg.angular.z = 0.0
                self.cmd_pub.publish(self.vel_msg)
                
                if (rospy.get_time() - self.action_start_time) >= 3.0:
                    self.state = 'ROTATE_RIGHT_SEQUENCE'
                    self.action_start_time = rospy.get_time()
                    rospy.loginfo("Mulai BERPUTAR ke KANAN selama 3 detik.")

            # --- STATE 8: BERPUTAR KANAN (3 DETIK) ---
            elif self.state == 'ROTATE_RIGHT_SEQUENCE':
                self.vel_msg.linear.x = 0.0
                self.vel_msg.linear.y = 0.0
                self.vel_msg.angular.z = -0.5 # Negatif Z untuk berputar searah jarum jam (ke kanan)
                self.cmd_pub.publish(self.vel_msg)
                
                if (rospy.get_time() - self.action_start_time) >= 3.0:
                    self.state = 'FINISHED'
                    rospy.loginfo("Semua manuver selesai. Robot Standby.")

            # --- STATE 9: SELESAI ---
            elif self.state == 'FINISHED':
                self.vel_msg.linear.x = 0.0
                self.vel_msg.linear.y = 0.0
                self.vel_msg.angular.z = 0.0
                self.cmd_pub.publish(self.vel_msg)
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        robot = HolonomicRobot()
        robot.run()
    except rospy.ROSInterruptException:
        pass