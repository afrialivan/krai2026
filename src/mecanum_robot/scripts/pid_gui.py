#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import time

class ObstacleAvoidance:
    def init(self):
        rospy.init_node('obstacle_node', anonymous=True)
        rospy.on_shutdown(self.clean_shutdown)

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/ultrasonic_distance', Int32, self.distance_callback)

        self.vel_msg = Twist()
        self.is_turning = False
        self.waktu_maju_bersih = 0.0


        self.last_maneuver_time = 0.0 

        rospy.loginfo("Robot akan maju selama total 10 detik bersih...")
        self.control_loop()

    def distance_callback(self, msg):
        jarak = msg.data
        
        # Jika sedang bermanuver, abaikan data sensor
        if self.is_turning:
            return

        # Jika manuver baru saja selesai kurang dari 2 detik yang lalu, abaikan sensor!
        if time.time() - self.last_maneuver_time < 2.0:
            return

        # Jika lolos proteksi dan mendeteksi objek di radius 55cm atau kurang
        if jarak <= 75 and jarak > 0: 
            rospy.logwarn(f"Objek terdeteksi! Jarak: {jarak} cm")
            self.is_turning = True
            self.avoid_obstacle_maneuver()

    def avoid_obstacle_maneuver(self):
        self.send_twist_command(0.0, 0.0)
        rospy.sleep(0.2)

        self.send_twist_command(0.3, 0.4) 
        rospy.sleep(1.0)
        
        self.send_twist_command(0.4, 0.0) 
        rospy.sleep(6.0)

        self.send_twist_command(0.3, -0.4) 
        rospy.sleep(0.7)
        
        self.send_twist_command(0.4, 0.0) 
        rospy.sleep(3.0)

        self.last_maneuver_time = time.time() 
        self.is_turning = False

    def send_twist_command(self, linear_x, angular_z):
        self.vel_msg.linear.x = linear_x
        self.vel_msg.angular.z = angular_z
        self.velocity_publisher.publish(self.vel_msg)

    def clean_shutdown(self):
        rospy.logwarn("Menerima sinyal shutdown! Menghentikan semua pergerakan motor robot...")
        self.send_twist_command(0.0, 0.0)
        time.sleep(0.2) 

    def control_loop(self):
        hz = 10
        rate = rospy.Rate(hz) 
        durasi_per_loop = 1.0 / hz 

        while not rospy.is_shutdown():
            if self.waktu_maju_bersih >= 7.0:
                rospy.logwarn("Target 10 detik maju lurus terpenuhi! Selesai.")
                rospy.signal_shutdown("Target waktu tercapai")
                break 

            if not self.is_turning:
                self.send_twist_command(0.4, 0.0)
                self.waktu_maju_bersih += durasi_per_loop
            
            rate.sleep()

if name == 'main': # type: ignore
    try:
        ObstacleAvoidance()
    except rospy.ROSInterruptException:
        pass