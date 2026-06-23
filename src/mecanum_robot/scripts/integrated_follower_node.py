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

        # Fitur Baru untuk Stabilitas:
        self.center_tolerance  = 15.0  
        self.missed_frames     = 0     
        self.max_missed_frames = 5     
        self.last_twist        = Twist()

        # ==========================================
        # 2. PARAMETER ODOMETRI & SENSOR
        # ==========================================
        self.wheel_diameter  = 9.5  # cm
        self.wheel_circ      = math.pi * self.wheel_diameter
        self.ppr             = 385.0
        self.linear_scale    = 1.0
        
        self.track_width_cm     = 80.0
        self.angular_scale      = 0.22
        self.encoder_yaw_deg    = 0.0
        self.turn_tolerance_deg = 2.0
        self.kp_turn            = 0.02
        self.min_turn_speed     = 0.15
        
        self.prev_counts       = None
        self.total_distance_cm = 0.0
        
        self.approach_target_dist = rospy.get_param("~approach_target_dist", 10.0) 
        self.max_approach_spd     = rospy.get_param("~approach_speed", -0.3)        
        self.kp_approach_linear   = rospy.get_param("~kp_approach_linear", 0.05) 
        self.approach_start_dist  = 0.0

        self.post_grab_start_dist = 0.0
        self.post_grab_start_yaw  = 0.0

        self.current_state = "FOLLOWING" 

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

        self.cmd_pub   = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.capit_pub = rospy.Publisher('/capit_cmd', Float32, queue_size=10)
        
        rospy.Subscriber('/motor_feedback', Float32MultiArray, self.feedback_cb)
        rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        rospy.loginfo("Node berjalan penuh. Mencari objek...")

    def feedback_cb(self, msg):
        if len(msg.data) < 9:
            return

        counts = {'L': msg.data[1], 'R': msg.data[2]}
        proximity = msg.data[8]

        if self.prev_counts is None:
            self.prev_counts = counts
            return

        d_tick_L = counts['L'] - self.prev_counts['L']
        d_tick_R = counts['R'] - self.prev_counts['R']
        
        d_cm_L = (d_tick_L / self.ppr) * self.wheel_circ
        d_cm_R = (d_tick_R / self.ppr) * self.wheel_circ

        self.total_distance_cm += ((d_cm_L + d_cm_R) / 2.0) * self.linear_scale
        
        delta_yaw_rad = ((d_cm_R - d_cm_L) / self.track_width_cm) * self.angular_scale
        self.encoder_yaw_deg += math.degrees(delta_yaw_rad)

        self.prev_counts = counts

        if self.current_state == "FOLLOWING" and proximity == 1.0:
            rospy.loginfo(f"Sensor Proximity Aktif! Menghentikan YOLO, mulai MUNDUR sejauh {self.approach_target_dist} cm.")
            self.approach_start_dist = self.total_distance_cm
            self.current_state = "APPROACHING"

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
        results = self.model(frame, conf=0.15, verbose=False)
        
        twist_msg = Twist()
        best_box = None
        max_area = 0
        
        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                if self.model.names[class_id] == self.target_object:
                    xyxy = box.xyxy[0].to("cpu").numpy()
                    xmin, ymin, xmax, ymax = xyxy[0], xyxy[1], xyxy[2], xyxy[3]
                    area = (xmax - xmin) * (ymax - ymin)
                    
                    if area > max_area:
                        max_area = area
                        best_box = xyxy
                        
        if best_box is not None:
            self.missed_frames = 0 
            xmin, ymin, xmax, ymax = best_box
            box_center_x = (xmin + xmax) / 2.0
            box_area = max_area
            
            error_x = self.center_x_image - box_center_x
            
            if abs(error_x) > self.center_tolerance:
                self.integral_error_x += (error_x * dt)
                self.integral_error_x = max(min(self.integral_error_x, self.max_integral), -self.max_integral)
                angular_z = (self.kp_angular * error_x) + (self.ki_angular * self.integral_error_x)
            else:
                angular_z = 0.0
                self.integral_error_x = 0.0 
                
            twist_msg.angular.z = angular_z

            if box_area < self.stop_area:
                remaining_area = self.stop_area - box_area
                calculated_speed = remaining_area * self.kp_linear
                linear_x = -abs(calculated_speed)
                
                if linear_x < self.max_linear_spd: 
                    linear_x = self.max_linear_spd
                    
                min_linear_limit = -0.04
                if linear_x > min_linear_limit:
                    linear_x = min_linear_limit
                    
                twist_msg.linear.x = linear_x
            else:
                twist_msg.linear.x = 0.0

            self.last_twist = twist_msg 
            
        else:
            self.missed_frames += 1
            if self.missed_frames < self.max_missed_frames:
                twist_msg = self.last_twist
            else:
                self.integral_error_x = 0.0
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0

        self.cmd_pub.publish(twist_msg)

        annotated_frame = results[0].plot()
        cv2.line(annotated_frame, (int(self.center_x_image), 0), (int(self.center_x_image), annotated_frame.shape[0]), (0, 255, 0), 2)
        
        if best_box is not None:
            xmin, ymin, xmax, ymax = map(int, best_box)
            cv2.rectangle(annotated_frame, (xmin, ymin), (xmax, ymax), (255, 0, 0), 2)
            cv2.circle(annotated_frame, (int((xmin+xmax)/2), int((ymin+ymax)/2)), 5, (0, 0, 255), -1)

        cv2.imshow("Kamera Integrasi", annotated_frame)
        cv2.waitKey(1)

    def run(self):
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if self.current_state == "APPROACHING":
                dist_moved = abs(self.total_distance_cm - self.approach_start_dist)
                remaining_dist = self.approach_target_dist - dist_moved
                
                cmd = Twist()
                if remaining_dist > 0.3:
                    calculated_approach_spd = remaining_dist * self.kp_approach_linear
                    
                    if self.max_approach_spd < 0:
                        linear_x = -abs(calculated_approach_spd)
                        if linear_x < self.max_approach_spd: 
                            linear_x = self.max_approach_spd
                        if linear_x > -0.06:
                            linear_x = -0.06
                    else:
                        linear_x = abs(calculated_approach_spd)
                        if linear_x > self.max_approach_spd:
                            linear_x = self.max_approach_spd
                        if linear_x < 0.06:
                            linear_x = 0.06

                    cmd.linear.x = linear_x
                    self.cmd_pub.publish(cmd)
                    rospy.loginfo_throttle(0.5, f"Mundur memelan... Sisa: {remaining_dist:.1f} cm | Speed: {cmd.linear.x:.3f}")
                else:
                    self.cmd_pub.publish(cmd)
                    rospy.loginfo(f"Selesai mundur sejauh {self.approach_target_dist} cm. Mengirim perintah capit...")
                    self.current_state = "GRABBING"
                    
            elif self.current_state == "GRABBING":
                cap_msg = Float32()
                cap_msg.data = 1.0 
                self.capit_pub.publish(cap_msg)
                rospy.loginfo("Perintah Capit (1.0) berhasil dikirim!")
                
                rospy.sleep(2.0) 
                
                self.post_grab_start_dist = self.total_distance_cm
                self.current_state = "MOVING_FORWARD"
                
            elif self.current_state == "MOVING_FORWARD":
                dist_moved = abs(self.total_distance_cm - self.post_grab_start_dist)
                target_dist = 30.0
                
                cmd = Twist()
                if dist_moved < target_dist:
                    cmd.linear.x = 0.2  # Kecepatan maju 
                    self.cmd_pub.publish(cmd)
                    rospy.loginfo_throttle(0.5, f"Maju pasca-capit... Jarak: {dist_moved:.1f} / {target_dist} cm")
                else:
                    self.cmd_pub.publish(cmd) 
                    rospy.loginfo("Selesai maju 30cm. Mulai putar kanan...")
                    
                    self.post_grab_start_yaw = self.encoder_yaw_deg
                    self.current_state = "TURNING_RIGHT"
            
            elif self.current_state == "TURNING_RIGHT":
                target_turn = -20.0 
                target_yaw = self.post_grab_start_yaw + target_turn
                error = (target_yaw - self.encoder_yaw_deg + 180) % 360 - 180
                
                cmd = Twist()
                if abs(error) > self.turn_tolerance_deg:
                    speed = self.kp_turn * abs(error)
                    speed = max(self.min_turn_speed, min(speed, 2.0)) 
                    cmd.angular.z = speed if error > 0 else -speed
                    
                    self.cmd_pub.publish(cmd)
                    rospy.loginfo_throttle(0.5, f"Putar kanan... Target: {target_yaw:.1f}° | Saat ini: {self.encoder_yaw_deg:.1f}° | Error: {error:.1f}°")
                else:
                    self.cmd_pub.publish(cmd) # Berhenti
                    rospy.loginfo("Selesai putar kanan. Misi selesai!")
                    self.current_state = "DONE"

            elif self.current_state == "DONE":
                pass
                
            rate.sleep()

if __name__ == '__main__':
    try:
        node = IntegratedFollowerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()