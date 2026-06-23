#!/usr/bin/env python3

import rospy
import cv2 #type: ignore
import os
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Float32
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO #type: ignore

class IntegratedFollowerNode:
    def __init__(self):
        rospy.init_node('integrated_follower_node', anonymous=True)

        # ==========================================
        # 1. PARAMETER YOLO & KONTROL KAMERA
        # ==========================================
        self.target_object   = rospy.get_param("~target_object", "botol")
        self.max_linear_spd  = rospy.get_param("~linear_speed", -0.1) 
        self.stop_area       = rospy.get_param("~stop_area", 66000)
        self.image_width     = rospy.get_param("~image_width", 640)
        self.center_offset_x = rospy.get_param("~center_offset_x", 0.0)
        
        self.kp_angular      = rospy.get_param("~kp_angular", 0.003)
        self.ki_angular      = rospy.get_param("~ki_angular", 0.0001)
        self.kp_linear       = rospy.get_param("~kp_linear", 0.00005) 

        self.center_x_image   = (self.image_width / 2.0) + self.center_offset_x
        self.integral_error_x = 0.0
        self.last_time        = rospy.get_time()
        self.max_integral     = 500.0
        self.bridge           = CvBridge()

        # ==========================================
        # 2. PARAMETER ODOMETRI & SENSOR MPU6050
        # ==========================================
        self.wheel_diameter  = 9.5    # cm
        self.wheel_circ      = math.pi * self.wheel_diameter
        self.ppr             = 385.0
        self.linear_scale    = 1.0
        
        self.prev_counts       = None
        self.total_distance_cm = 0.0
        self.current_yaw       = 0.0   
        
        # Jarak target maju dan sudut rotasi
        self.forward_target_dist  = rospy.get_param("~forward_target_dist", 50.0) # Robot maju sejauh 50 cm
        self.turn_target_deg      = rospy.get_param("~turn_target_deg", 90.0)     # Mutar 90 derajat
        
        self.max_forward_spd      = abs(rospy.get_param("~approach_speed", 0.3))  # Dipastikan positif untuk laju maju      
        self.kp_forward_linear    = rospy.get_param("~kp_approach_linear", 0.05) 
        
        self.kp_turn              = 0.02   
        self.min_turn_speed       = 0.15   
        self.turn_tolerance_deg   = 2.0    
        
        self.nav_start_dist       = 0.0
        self.nav_start_yaw        = 0.0

        # ==========================================
        # 3. STATE MACHINE
        # ==========================================
        # Urutan: FOLLOWING -> GRABBING -> NAV_FORWARD -> NAV_TURN -> DONE
        self.current_state = "FOLLOWING" 

        # ==========================================
        # 4. MUAT MODEL YOLO
        # ==========================================
        package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        model_path = os.path.join(package_path, 'models', 'best.pt')
        rospy.loginfo(f"Memuat Model YOLO dari: {model_path}")
        try:
            self.model = YOLO(model_path)
            rospy.loginfo("Model YOLO berhasil dimuat. Menyiapkan komunikasi ROS...")
        except Exception as e:
            rospy.logerr(f"Gagal memuat model! Error: {e}")
            rospy.signal_shutdown("Model YOLO tidak ditemukan.")
            return

        # ==========================================
        # 5. SETUP PUBLISHER & SUBSCRIBER
        # ==========================================
        self.cmd_pub   = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.capit_pub = rospy.Publisher('/capit_cmd', Float32, queue_size=10)
        
        rospy.Subscriber('/motor_feedback', Float32MultiArray, self.feedback_cb)
        rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)

    def feedback_cb(self, msg):
        if len(msg.data) < 11:
            return

        counts = {'L': msg.data[1], 'R': msg.data[2]}
        proximity = msg.data[8]
        self.current_yaw = msg.data[10] 

        if self.prev_counts is None:
            self.prev_counts = counts
            return

        d_tick_L = counts['L'] - self.prev_counts['L']
        d_tick_R = counts['R'] - self.prev_counts['R']
        
        d_cm_L = (d_tick_L / self.ppr) * self.wheel_circ
        d_cm_R = (d_tick_R / self.ppr) * self.wheel_circ

        self.total_distance_cm += ((d_cm_L + d_cm_R) / 2.0) * self.linear_scale
        self.prev_counts = counts

        # Transisi dari Mengejar ke Mencapit
        if self.current_state == "FOLLOWING" and proximity == 1.0:
            rospy.loginfo("Sensor Proximity Aktif! Menghentikan gerak dan memulai proses capit.")
            self.current_state = "GRABBING"

    def image_callback(self, data):
        if self.current_state != "FOLLOWING":
            return 

        current_time = rospy.get_time()
        dt = current_time - self.last_time
        self.last_time = current_time
        if dt <= 0: dt = 0.03 

        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            return

        frame = cv2.resize(frame, (self.image_width, int(self.image_width * 0.75)))
        results = self.model(frame, conf=0.25, verbose=False)
        target_found = False
        twist_msg = Twist()
        
        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                if self.model.names[class_id] == self.target_object:
                    target_found = True
                    
                    xyxy = box.xyxy[0].to("cpu").numpy()
                    xmin, ymin, xmax, ymax = xyxy[0], xyxy[1], xyxy[2], xyxy[3]
                    
                    box_center_x = (xmin + xmax) / 2.0
                    box_area = (xmax - xmin) * (ymax - ymin)
                    
                    if box_area < self.stop_area:
                        remaining_area = self.stop_area - box_area
                        calculated_speed = remaining_area * self.kp_linear
                        linear_x = -abs(calculated_speed) # Mundur mendekati objek
                        
                        if linear_x < self.max_linear_spd: 
                            linear_x = self.max_linear_spd
                            
                        min_linear_limit = -0.04
                        if linear_x > min_linear_limit:
                            linear_x = min_linear_limit
                            
                        twist_msg.linear.x = linear_x
                        
                        error_x = self.center_x_image - box_center_x
                        self.integral_error_x += (error_x * dt)
                        
                        if self.integral_error_x > self.max_integral: self.integral_error_x = self.max_integral
                        elif self.integral_error_x < -self.max_integral: self.integral_error_x = -self.max_integral
                        
                        angular_z = (self.kp_angular * error_x) + (self.ki_angular * self.integral_error_x)
                        twist_msg.angular.z = angular_z
                    else:
                        twist_msg.linear.x = 0.0
                        twist_msg.angular.z = 0.0
                        self.integral_error_x = 0.0
                    break
            if target_found: break

        if not target_found:
            self.integral_error_x = 0.0

        self.cmd_pub.publish(twist_msg)

        annotated_frame = results[0].plot()
        cv2.line(annotated_frame, (int(self.center_x_image), 0), (int(self.center_x_image), annotated_frame.shape[0]), (0, 255, 0), 2)
        cv2.imshow("Kamera Integrasi", annotated_frame)
        cv2.waitKey(1)

    def run(self):
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            
            # --- LANGKAH 1: MENCAPIT OBJEK ---
            if self.current_state == "GRABBING":
                self.cmd_pub.publish(Twist()) # Pastikan robot diam
                cap_msg = Float32()
                cap_msg.data = 1.0 
                self.capit_pub.publish(cap_msg)
                
                rospy.loginfo("Objek dicapit! Menunggu 1 detik agar erat...")
                rospy.sleep(1.0) 
                
                rospy.loginfo(f"Lanjut Langkah 2: MAJU membawa objek sejauh {self.forward_target_dist} cm.")
                self.nav_start_dist = self.total_distance_cm
                self.current_state = "NAV_FORWARD"

            # --- LANGKAH 2: NAVIGASI MAJU (Kecepatan Positif) ---
            elif self.current_state == "NAV_FORWARD":
                dist_moved = abs(self.total_distance_cm - self.nav_start_dist)
                remaining_dist = self.forward_target_dist - dist_moved
                
                cmd = Twist()
                if remaining_dist > 0.3:
                    calculated_forward_spd = remaining_dist * self.kp_forward_linear
                    
                    # Logika Maju (Kecepatan Linear HARUS Positif)
                    linear_x = abs(calculated_forward_spd)
                    
                    # Batasi tidak melebih max speed
                    if linear_x > self.max_forward_spd: 
                        linear_x = self.max_forward_spd
                    # Beri tenaga minimum agar tidak macet oleh gesekan
                    if linear_x < 0.06: 
                        linear_x = 0.06

                    cmd.linear.x = linear_x
                    self.cmd_pub.publish(cmd)
                    rospy.loginfo_throttle(0.5, f"[Maju Bawa Objek] Sisa jarak: {remaining_dist:.1f} cm | Speed: {cmd.linear.x:.3f}")
                else:
                    self.cmd_pub.publish(Twist()) 
                    rospy.loginfo(f"[Maju] Selesai. Lanjut Langkah 3: MUTAR sejauh {self.turn_target_deg} derajat.")
                    rospy.sleep(0.5) 
                    
                    self.nav_start_yaw = self.current_yaw 
                    self.current_state = "NAV_TURN"
            
            # --- LANGKAH 3: NAVIGASI BERPUTAR ---
            elif self.current_state == "NAV_TURN":
                target_yaw = self.nav_start_yaw + self.turn_target_deg
                error_deg = (target_yaw - self.current_yaw + 180) % 360 - 180
                
                cmd = Twist()
                if abs(error_deg) > self.turn_tolerance_deg:
                    speed_z = self.kp_turn * abs(error_deg)
                    speed_z = max(self.min_turn_speed, min(speed_z, 1.0))
                    
                    cmd.angular.z = speed_z if error_deg > 0 else -speed_z
                    self.cmd_pub.publish(cmd)
                    rospy.loginfo_throttle(0.5, f"[Mutar MPU] Target: {target_yaw:.1f}° | Aktual: {self.current_yaw:.1f}° | Error: {error_deg:.1f}°")
                else:
                    self.cmd_pub.publish(Twist()) 
                    rospy.loginfo("[Mutar] Selesai mencapai orientasi target.")
                    self.current_state = "DONE"
                
            elif self.current_state == "DONE":
                rospy.loginfo_throttle(5.0, "[Misi Selesai] Robot siap menerima perintah baru.")
                
            rate.sleep()

if __name__ == '__main__':
    try:
        node = IntegratedFollowerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()