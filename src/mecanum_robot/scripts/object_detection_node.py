#!/usr/bin/env python3
import rospy
import cv2
import os
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Float32
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

class CustomYoloFollower:
    def __init__(self):
        rospy.init_node('yolo_follower_node', anonymous=True)

        # ==========================================
        # 1. PARAMETER TARGET & KALIBRASI YOLO
        # ==========================================
        self.target_object = rospy.get_param("~target_object", "botol")
        self.linear_speed = rospy.get_param("~linear_speed", -0.15)
        self.stop_area = rospy.get_param("~stop_area", 100000)
        self.image_width = rospy.get_param("~image_width", 640)
        self.center_offset_x = rospy.get_param("~center_offset_x", 0.0)

        # --- PARAMETER KONTROL PI ---
        self.kp_angular = rospy.get_param("~kp_angular", 0.003)
        self.ki_angular = rospy.get_param("~ki_angular", 0.0001)
        self.integral_error_x = 0.0
        self.last_time = rospy.get_time()
        self.max_integral = 500.0

        self.bridge = CvBridge()

        # ==========================================
        # 2. PARAMETER ODOMETRI & STATE MACHINE
        # ==========================================
        # Parameter Fisik & Odometri
        self.wheel_circumference = math.pi * 9.5
        self.track_width_cm = 80.0
        self.ppr = 385
        self.linear_scale = 1.0
        self.angular_scale = 0.127
        self.kp_turn = 0.02
        self.min_turn_speed = 0.15
        self.turn_tolerance = 2.0
        
        # Variabel Memori Odometri
        self.prev_counts = None
        self.total_distance_cm = 0.0
        self.encoder_yaw_deg = 0.0

        # State Sequence (FOLLOWING -> MOVING_BACK -> CLAMPING -> MOVING_FORWARD -> TURNING -> DONE)
        self.state = 'FOLLOWING'
        self.start_distance = 0.0
        self.start_yaw = 0.0
        self.clamp_start_time = 0.0

        # Target Navigasi Odometri
        self.target_back_cm = 40.0       # Mundur 20 cm
        self.target_forward_cm = 30.0    # Maju 30 cm
        self.seq_move_speed = 0.5        # Kecepatan saat maju/mundur sequence
        self.target_turn_deg = -23.0     # Belok kanan (mines)
        self.turn_speed = 2.0            # Kecepatan belok maksimum

        # ==========================================
        # 3. PATH MODEL CUSTOM (best.pt)
        # ==========================================
        package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        model_path = os.path.join(package_path, 'models', 'best.pt')
        
        rospy.loginfo(f"Memuat Custom Model dari: {model_path}")
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            rospy.logerr(f"Gagal memuat model best.pt! Error: {e}")
            return

        # ==========================================
        # 4. SETUP PUBLISHER & SUBSCRIBER
        # ==========================================
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.capit_pub = rospy.Publisher('/capit_cmd', Float32, queue_size=10)
        self.twist_msg = Twist()
        
        self.center_x_image = (self.image_width / 2.0) + self.center_offset_x

        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        rospy.Subscriber('/motor_feedback', Float32MultiArray, self.feedback_cb)
        
        rospy.loginfo(f"Node Berjalan. Mencari objek '{self.target_object}'...")

    # ==========================================
    # CALLBACK ODOMETRI & STATE MACHINE
    # ==========================================
    def feedback_cb(self, msg):
        if len(msg.data) < 3:
            return

        # --- Update Odometri ---
        counts = {'L': msg.data[1], 'R': msg.data[2]}

        if self.prev_counts is None:
            self.prev_counts = counts
            return

        d_tick_L = counts['L'] - self.prev_counts['L']
        d_tick_R = counts['R'] - self.prev_counts['R']

        d_cm_L = (d_tick_L / self.ppr) * self.wheel_circumference
        d_cm_R = (d_tick_R / self.ppr) * self.wheel_circumference

        delta_linear = ((d_cm_L + d_cm_R) / 2.0) * self.linear_scale
        self.total_distance_cm += delta_linear

        delta_yaw_rad = ((d_cm_R - d_cm_L) / self.track_width_cm) * self.angular_scale
        self.encoder_yaw_deg += math.degrees(delta_yaw_rad)

        self.prev_counts = counts

        # --- Jalankan Sequence Jika YOLO Sudah Dekat ---
        if self.state != 'FOLLOWING':
            self.run_sequence_machine()

    def run_sequence_machine(self):
        cmd = Twist()

        # STATE: MUNDUR 20 CM
        if self.state == 'MOVING_BACK':
            jarak = abs(self.total_distance_cm - self.start_distance)
            if jarak < self.target_back_cm:
                cmd.linear.x = -self.seq_move_speed
            else:
                cmd.linear.x = 0.0
                self.cmd_pub.publish(cmd)
                
                # Eksekusi capit
                capit_msg = Float32()
                capit_msg.data = 1.0
                self.capit_pub.publish(capit_msg)
                
                self.clamp_start_time = rospy.get_time()
                self.state = 'CLAMPING'
                rospy.loginfo("Mundur selesai. Menutup capit...")

        # STATE: TUNGGU CAPIT (1.5 detik)
        elif self.state == 'CLAMPING':
            if rospy.get_time() - self.clamp_start_time > 1.5:
                self.start_distance = self.total_distance_cm
                self.state = 'MOVING_FORWARD'
                rospy.loginfo(f"Capit selesai. Mulai maju {self.target_forward_cm} cm...")

        # STATE: MAJU 30 CM
        elif self.state == 'MOVING_FORWARD':
            jarak = abs(self.total_distance_cm - self.start_distance)
            if jarak < self.target_forward_cm:
                cmd.linear.x = self.seq_move_speed
            else:
                cmd.linear.x = 0.0
                self.cmd_pub.publish(cmd)
                self.start_yaw = self.encoder_yaw_deg
                self.state = 'TURNING'
                rospy.loginfo("Maju selesai. Mulai memutar ke kanan...")

        # STATE: BELOK KANAN 23 DERAJAT
        elif self.state == 'TURNING':
            target_yaw = self.start_yaw + self.target_turn_deg
            error = (target_yaw - self.encoder_yaw_deg + 180) % 360 - 180

            if abs(error) > self.turn_tolerance:
                speed = self.kp_turn * abs(error)
                speed = max(self.min_turn_speed, min(speed, abs(self.turn_speed)))
                cmd.angular.z = speed if error > 0 else -speed
            else:
                cmd.angular.z = 0.0
                self.state = 'DONE'
                rospy.loginfo("Sequence Selesai! Robot diam di tempat.")

        # STATE: SELESAI
        elif self.state == 'DONE':
            pass

        self.cmd_pub.publish(cmd)

    # ==========================================
    # CALLBACK KAMERA & YOLO
    # ==========================================
    def image_callback(self, data):
        current_time = rospy.get_time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if dt <= 0:
            dt = 0.03 

        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Gagal mengonversi gambar: {e}")
            return

        frame = cv2.resize(frame, (self.image_width, int(self.image_width * 0.75)))
        results = self.model(frame, conf=0.3, verbose=False)
        
        # --- HANYA JALANKAN LOGIKA YOLO JIKA STATE = FOLLOWING ---
        if self.state == 'FOLLOWING':
            target_found = False
            
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    class_id = int(box.cls[0])
                    label = self.model.names[class_id]
                    
                    if label == self.target_object:
                        target_found = True
                        
                        xyxy = box.xyxy[0].to("cpu").numpy()
                        xmin, ymin, xmax, ymax = xyxy[0], xyxy[1], xyxy[2], xyxy[3]
                        
                        box_center_x = (xmin + xmax) / 2.0
                        box_area = (xmax - xmin) * (ymax - ymin)
                        
                        error_x = self.center_x_image - box_center_x
                        self.integral_error_x += (error_x * dt)
                        
                        if self.integral_error_x > self.max_integral:
                            self.integral_error_x = self.max_integral
                        elif self.integral_error_x < -self.max_integral:
                            self.integral_error_x = -self.max_integral
                        
                        angular_z = (self.kp_angular * error_x) + (self.ki_angular * self.integral_error_x)
                        
                        if box_area < self.stop_area:
                            self.twist_msg.linear.x = self.linear_speed
                            self.twist_msg.angular.z = angular_z
                        else:
                            # TRIGGER SEQUENCE: Objek tercapai!
                            self.twist_msg.linear.x = 0.0
                            self.twist_msg.angular.z = 0.0
                            self.cmd_pub.publish(self.twist_msg) # Stop sebentar
                            
                            self.start_distance = self.total_distance_cm
                            self.state = 'MOVING_BACK'
                            rospy.loginfo(f"Objek tercapai! YOLO dimatikan, beralih ke state MUNDUR {self.target_back_cm} cm.")
                        break
                if target_found:
                    break

            # Proteksi jika tidak ada objek (Robot berhenti)
            if not target_found:
                self.twist_msg.linear.x = 0.0
                self.twist_msg.angular.z = 0.0
                self.integral_error_x = 0.0  

            self.cmd_pub.publish(self.twist_msg)
        
        # VISUALISASI KE LAYAR (Tetap berjalan agar kita bisa memonitor kamera)
        # Gunakan result terbaru dari loop luar
        for result in results:
            annotated_frame = result.plot()
        
        cv2.line(annotated_frame, (int(self.center_x_image), 0), (int(self.center_x_image), annotated_frame.shape[0]), (0, 255, 0), 2)
        
        # Tampilkan State Saat Ini di Pojok Kiri Atas
        cv2.putText(annotated_frame, f"State: {self.state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        cv2.imshow("Kamera Belakang - YOLOv8 PI Controller", annotated_frame)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        CustomYoloFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()