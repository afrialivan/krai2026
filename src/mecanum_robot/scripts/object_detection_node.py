#!/usr/bin/env python3
import rospy
import cv2
import os
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

class CustomYoloFollower:
    def __init__(self):
        rospy.init_node('yolo_follower_node', anonymous=True)

        # ==========================================
        # 1. PARAMETER TARGET & KALIBRASI
        # ==========================================
        self.target_object = rospy.get_param("~target_object", "botol")
        self.linear_speed = rospy.get_param("~linear_speed", -0.15)
        self.stop_area = rospy.get_param("~stop_area", 100000)
        self.image_width = rospy.get_param("~image_width", 640)
        self.center_offset_x = rospy.get_param("~center_offset_x", 0.0)

        # --- PARAMETER KONTROL PI ---
        self.kp_angular = rospy.get_param("~kp_angular", 0.003)   # Proportional Gain
        self.ki_angular = rospy.get_param("~ki_angular", 0.0001)  # Integral Gain
        
        # Variabel untuk menampung memori error integral dan waktu
        self.integral_error_x = 0.0
        self.last_time = rospy.get_time()
        self.max_integral = 500.0  # ANTI-WINDUP: Batas maksimal akumulasi error

        self.bridge = CvBridge()

        # ==========================================
        # 2. PATH MODEL CUSTOM (best.pt)
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
        # 3. SETUP PUBLISHER & SUBSCRIBER
        # ==========================================
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist_msg = Twist()
        
        # Titik tengah + offset
        self.center_x_image = (self.image_width / 2.0) + self.center_offset_x

        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        rospy.loginfo(f"Node Berjalan dengan Kontrol PI. Mencari objek '{self.target_object}'...")

    def image_callback(self, data):
        # Hitung jeda waktu (delta time / dt) untuk Integral
        current_time = rospy.get_time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Cegah nilai dt 0 atau tidak masuk akal saat frame pertama
        if dt <= 0:
            dt = 0.03 

        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Gagal mengonversi gambar: {e}")
            return

        frame = cv2.resize(frame, (self.image_width, int(self.image_width * 0.75)))
        results = self.model(frame, conf=0.3, verbose=False)
        
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
                    
                    # 1. Hitung Error Saat Ini
                    error_x = self.center_x_image - box_center_x
                    
                    # 2. Hitung Integral (Akumulasi Error x Waktu)
                    self.integral_error_x += (error_x * dt)
                    
                    # 3. Terapkan Anti-Windup
                    if self.integral_error_x > self.max_integral:
                        self.integral_error_x = self.max_integral
                    elif self.integral_error_x < -self.max_integral:
                        self.integral_error_x = -self.max_integral
                    
                    # 4. KONTROL PI (Hitung kecepatan belok)
                    angular_z = (self.kp_angular * error_x) + (self.ki_angular * self.integral_error_x)
                    
                    if box_area < self.stop_area:
                        self.twist_msg.linear.x = self.linear_speed
                        self.twist_msg.angular.z = angular_z
                    else:
                        # Rem jika sudah dekat
                        self.twist_msg.linear.x = 0.0
                        self.twist_msg.angular.z = 0.0
                        self.integral_error_x = 0.0  # Reset integral saat berhenti
                    break
            if target_found:
                break

        # PROTEKSI UTAMA: Jika tidak ada objek
        if not target_found:
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.0
            self.integral_error_x = 0.0  # Reset integral agar tidak menumpuk saat objek hilang

        self.cmd_pub.publish(self.twist_msg)
        
        # VISUALISASI KE LAYAR
        annotated_frame = result.plot()
        cv2.line(annotated_frame, (int(self.center_x_image), 0), (int(self.center_x_image), annotated_frame.shape[0]), (0, 255, 0), 2)
        
        cv2.imshow("Kamera Belakang - YOLOv8 PI Controller", annotated_frame)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        CustomYoloFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()