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
        self.trigger_distance = 20.0
        
        # --- Parameter Ramp & Rem ---
        self.max_forward_speed = 0.3
        self.min_forward_speed = 0.12
        
        self.brake_duration = 0.15      # Waktu rem (detik). Sesuaikan dengan berat robot agar tidak kebablasan mundur.
        self.brake_multiplier = 1.0     # 1.0 berarti tenaga rem sama dengan tenaga gerak terakhir.
        self.brake_vel = Twist()        # Variabel untuk menampung kecepatan rem
        self.next_state_after_brake = ''
        
        self.state = 'NORMAL'
        self.action_start_time = 0.0
        
        self.pause_duration = 0.2
        self.capit_wait = 3.0

    def sensor_callback(self, msg):
        if len(msg.data) >= 2:
            raw_dist = msg.data[10]
            self.prox_state = msg.data[1]
            
            if raw_dist <= 0:
                self.distance = 999.0
            else:
                self.distance = raw_dist

    # Fungsi baru untuk memicu pengereman sebelum masuk state berikutnya
    def apply_brake(self, target_state, log_msg="Mengerem untuk mencegah slip..."):
        rospy.loginfo(log_msg)
        self.next_state_after_brake = target_state
        self.state = 'BRAKING'
        self.action_start_time = rospy.get_time()
        
        # Simpan kecepatan dengan arah berlawanan
        self.brake_vel.linear.x = -self.vel_msg.linear.x * self.brake_multiplier
        self.brake_vel.linear.y = -self.vel_msg.linear.y * self.brake_multiplier
        self.brake_vel.angular.z = -self.vel_msg.angular.z * self.brake_multiplier

    def run(self):
        while not rospy.is_shutdown():
            
            # --- STATE 0: BRAKING (REM SEKEJAP) ---
            if self.state == 'BRAKING':
                self.cmd_pub.publish(self.brake_vel)
                
                # Jika waktu ngerem sudah habis
                if (rospy.get_time() - self.action_start_time) >= self.brake_duration:
                    # Nolkan kecepatan sepenuhnya
                    self.vel_msg.linear.x = 0.0
                    self.vel_msg.linear.y = 0.0
                    self.vel_msg.angular.z = 0.0
                    self.cmd_pub.publish(self.vel_msg)
                    
                    # Lanjut ke state tujuan dan reset timer untuk state tersebut
                    self.state = self.next_state_after_brake
                    self.action_start_time = rospy.get_time()
                    
            # --- STATE 1: NORMAL (Jalan ke Kiri Terus) ---
            elif self.state == 'NORMAL':
                if self.prox_state == 0:
                    self.apply_brake('PAUSE_BEFORE_CAPIT', "Proximity aktif! Rem sejenak sebelum Capit.")
                
                elif 0 < self.distance <= self.trigger_distance:
                    self.apply_brake('PAUSE_BEFORE_FORWARD', f"Ultrasonik {self.distance:.1f}cm! Rem sebelum Maju.")
                
                else:
                    self.vel_msg.linear.x = 0.0
                    self.vel_msg.linear.y = 0.4  # Kiri
                    self.vel_msg.angular.z = 0.0
                    self.cmd_pub.publish(self.vel_msg)

            # --- STATE 2: JEDA SEBELUM MAJU ---
            elif self.state == 'PAUSE_BEFORE_FORWARD':
                # Kecepatan sudah dinolkan oleh state BRAKING
                if (rospy.get_time() - self.action_start_time) >= self.pause_duration:
                    self.state = 'FORWARD_STATE'
                    rospy.loginfo("Jeda selesai. Lanjut MAJU dengan RAMP.")

            # --- STATE 3: MAJU DENGAN RAMP (Ultrasonik) ---
            elif self.state == 'FORWARD_STATE':
                if self.prox_state == 0:
                    self.apply_brake('PAUSE_BEFORE_CAPIT', "Proximity aktif saat maju! Ngerem mundur...")
                
                elif self.distance > self.trigger_distance:
                    self.apply_brake('NORMAL', "Rintangan hilang. Rem sebelum kembali ke pergerakan Kiri.")
                
                else:
                    calc_speed = (self.distance / self.trigger_distance) * self.max_forward_speed
                    ramp_speed = max(self.min_forward_speed, min(self.max_forward_speed, calc_speed))
                    
                    self.vel_msg.linear.x = ramp_speed
                    self.vel_msg.linear.y = 0.0
                    self.vel_msg.angular.z = 0.0
                    self.cmd_pub.publish(self.vel_msg)

            # --- STATE 4: JEDA SEBELUM CAPIT ---
            elif self.state == 'PAUSE_BEFORE_CAPIT':
                if (rospy.get_time() - self.action_start_time) >= self.pause_duration:
                    self.state = 'CAPIT_SEQUENCE'
                    self.action_start_time = rospy.get_time()
                    rospy.loginfo("Mulai manuver penyelarasan Capit (Kiri 0.8 detik).")

            # --- STATE 5: URUTAN CAPIT ---
            elif self.state == 'CAPIT_SEQUENCE':
                self.vel_msg.linear.x = 0.0
                self.vel_msg.linear.y = 0.2 # Manuver kiri
                self.vel_msg.angular.z = 0.0
                self.cmd_pub.publish(self.vel_msg)
                
                elapsed_time = rospy.get_time() - self.action_start_time
                if elapsed_time >= 1.0: 
                    # Rem sebelum betul-betul menjepit
                    self.apply_brake('SEND_CAPIT_CMD', "Rem manuver capit.")

            # --- STATE 5.1: KIRIM COMMAND CAPIT ---
            elif self.state == 'SEND_CAPIT_CMD':
                cmd_msg = Float32()
                cmd_msg.data = 1.0
                self.capit_pub.publish(cmd_msg)
                rospy.loginfo("Perintah capit dikirim. Menunggu capit menutup rapat...")
                
                self.state = 'PAUSE_AFTER_CAPIT'
                self.action_start_time = rospy.get_time()

            # --- STATE 6: JEDA SETELAH CAPIT ---
            elif self.state == 'PAUSE_AFTER_CAPIT':
                if (rospy.get_time() - self.action_start_time) >= self.capit_wait:
                    self.state = 'MOVE_RIGHT_SEQUENCE'
                    self.action_start_time = rospy.get_time()
                    rospy.loginfo("Mulai bergerak ke KANAN selama 3 detik.")

            # --- STATE 7: BERGERAK KANAN (3 DETIK) ---
            elif self.state == 'MOVE_RIGHT_SEQUENCE':
                self.vel_msg.linear.x = 0.0
                self.vel_msg.linear.y = -1.0 # Kanan
                self.vel_msg.angular.z = 0.0
                self.cmd_pub.publish(self.vel_msg)
                
                if (rospy.get_time() - self.action_start_time) >= 3.0:
                    self.apply_brake('ROTATE_RIGHT_SEQUENCE', "Rem gerak kanan.")

            # --- STATE 8: BERPUTAR KANAN (3 DETIK) ---
            elif self.state == 'ROTATE_RIGHT_SEQUENCE':
                self.vel_msg.linear.x = 0.0
                self.vel_msg.linear.y = 0.0
                self.vel_msg.angular.z = -0.5 # Berputar
                self.cmd_pub.publish(self.vel_msg)
                
                if (rospy.get_time() - self.action_start_time) >= 3.0:
                    self.apply_brake('FINISHED', "Rem putaran selesai.")

            # --- STATE 9: SELESAI ---
            elif self.state == 'FINISHED':
                # Kecepatan sudah aman 0 karena melewati state BRAKING
                pass
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        robot = HolonomicRobot()
        robot.run()
    except rospy.ROSInterruptException:
        pass