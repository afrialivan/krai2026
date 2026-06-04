#!/usr/bin/env python3
import rospy
import cv2  # type: ignore
import json
from ultralytics import YOLO  # type: ignore
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge


class YoloAutonomous:
    def __init__(self):
        rospy.init_node('yolo_detector_node')

        self.model = YOLO('/home/prall/krai_ws/src/mecanum_robot/scripts/best.pt')
        self.bridge = CvBridge()

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.led_pub = rospy.Publisher('/led_cmd', Bool, queue_size=10)
        self.detection_pub = rospy.Publisher('/yolo_detections', String, queue_size=10)

        rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        
        # =========================================================================
        # PARAMETER KENDALI (SILAKAN ATUR DI SINI)
        # =========================================================================
        self.center_x = 683  # Titik tengah horizontal kamera (piksel)
        
        # Batas lebar objek (piksel) untuk menentukan kapan robot harus berhenti.
        # - Makin BESAR angkanya: Robot makin DEKAT dengan objek sebelum berhenti.
        # - Makin KECIL angkanya: Robot berhenti lebih JAUH dari objek.
        self.target_width_threshold = 750  
        
        # Kecepatan mundur robot (Wajib bernilai NEGATIF untuk mundur)
        self.backward_speed = -.3  
        
        # Nilai pembagi sensitivitas belok (PD Sederhana)
        # - Makin KECIL angkanya: Belok makin tajam/responsif.
        # - Makin BESAR angkanya: Belok makin halus/pelan.
        self.turn_sensitivity = 500.0
        # =========================================================================

    def callback(self, data):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            img_h, img_w = cv_img.shape[:2]

            results = self.model.predict(cv_img, conf=0.3, verbose=False)

            move = Twist()
            target_visible = False
            detections = []

            if len(results[0].boxes) > 0:
                target_visible = True
                boxes = results[0].boxes

                for box in boxes:
                    x1, y1, x2, y2 = (
                        float(box.xyxy[0][0]),
                        float(box.xyxy[0][1]),
                        float(box.xyxy[0][2]),
                        float(box.xyxy[0][3]),
                    )

                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0

                    class_id   = int(box.cls[0])
                    class_name = self.model.names[class_id]
                    confidence = float(box.conf[0])

                    detections.append({
                        "class_id":   class_id,
                        "class_name": class_name,
                        "confidence": round(confidence, 3),
                        "cx":         round(cx, 2),
                        "cy":         round(cy, 2),
                        "bbox":       [round(x1,2), round(y1,2), round(x2,2), round(y2,2)],
                    })

                # Gunakan deteksi pertama untuk kendali gerak robot
                primary = detections[0]
                error_x = self.center_x - primary["cx"]

                # 1. Rotasi (Tanda minus dihapus agar arah belok tidak berlawanan saat mundur)
                move.angular.z = error_x / self.turn_sensitivity

                # 2. Logika Mundur Sampai Dekat Target
                w_primary = primary["bbox"][2] - primary["bbox"][0]
                
                if w_primary < self.target_width_threshold:
                    # Jika ukuran objek masih lebih kecil dari batas, robot terus mundur mendekat
                    move.linear.x = self.backward_speed
                    rospy.loginfo(f"Mundur mendekat... Lebar objek: {w_primary:.1f} / Target: {self.target_width_threshold}")
                else:
                    # Jika objek sudah dekat (lebar objek >= target_width_threshold), robot berhenti
                    move.linear.x = 0.0
                    rospy.loginfo("Target Sudah Dekat! Robot Berhenti.")

            # Publish gerakan & LED ke robot
            self.cmd_pub.publish(move)
            self.led_pub.publish(target_visible)

            # Publish data deteksi sebagai JSON string
            detection_msg = String()
            detection_msg.data = json.dumps({
                "frame_width":  img_w,
                "frame_height": img_h,
                "detections":   detections,
            })
            self.detection_pub.publish(detection_msg)

            # Log ringkas ke terminal ROS
            if detections:
                for d in detections:
                    rospy.loginfo(
                        f"[{d['class_name']}] conf={d['confidence']:.2f} "
                        f"cx={d['cx']:.1f} cy={d['cy']:.1f}"
                    )

            # Tampilkan visualisasi pada layar Jetson
            res_plotted = results[0].plot()
            cv2.imshow("Jetson Vision", res_plotted)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"YOLO Error: {e}")


if __name__ == '__main__':
    YoloAutonomous()
    rospy.spin()