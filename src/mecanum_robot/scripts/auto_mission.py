#!/usr/bin/env python3

import rospy
import math
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class GotoGoal:
    def __init__(self):
        rospy.init_node('goto_goal_node')
        
        # Publisher & Subscriber
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Posisi Robot Saat Ini
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # Set Target (Ubah koordinat target di sini)
        self.target_x = 2.0  # meter
        self.target_y = 1.0  # meter
        
        # Toleransi Jarak
        self.distance_tolerance = 0.1 # Berhenti jika jarak kurang dari 10 cm

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Mengubah quaternion (orientasi 3D) menjadi sudut Euler/Yaw (2D)
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]

    def normalize_angle(self, angle):
        # Memastikan sudut selalu berada di rentang -Pi hingga Pi
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        rospy.loginfo(f"Menuju target: X={self.target_x}, Y={self.target_y}")
        
        while not rospy.is_shutdown():
            # 1. Hitung jarak ke target
            distance = math.sqrt((self.target_x - self.x)**2 + (self.target_y - self.y)**2)
            
            # 2. Hitung sudut ke target
            angle_to_goal = math.atan2(self.target_y - self.y, self.target_x - self.x)
            
            # 3. Hitung selisih sudut robot saat ini dengan target
            angle_error = self.normalize_angle(angle_to_goal - self.yaw)
            
            cmd = Twist()
            
            # Logika Navigasi P-Controller Sederhana
            if distance >= self.distance_tolerance:
                
                # Jika sudut meleset lebih dari ~11 derajat (0.2 radian), putar di tempat dulu
                if abs(angle_error) > 0.2:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.5 if angle_error > 0 else -0.5
                
                # Jika sudah menghadap target, jalan maju sambil menyesuaikan arah
                else:
                    cmd.linear.x = 0.3 # Kecepatan maju m/s
                    # Koreksi arah perlahan sambil jalan
                    cmd.angular.z = 0.5 * angle_error 
                    
            else:
                # Target Tercapai!
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                rospy.loginfo("Target Berhasil Dicapai!")
                break
                
            self.cmd_pub.publish(cmd)
            rate.sleep()

if __name__ == '__main__':
    try:
        navigator = GotoGoal()
        navigator.run()
    except rospy.ROSInterruptException:
        pass