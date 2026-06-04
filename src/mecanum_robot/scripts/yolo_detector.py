#!/usr/bin/env python3
import rospy
import cv2 #type: ignore
from ultralytics import YOLO #type: ignore
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from cv_bridge import CvBridge

class YoloAutonomous:
    def __init__(self):
        rospy.init_node('yolo_detector_node')
       
        self.model = YOLO('/home/prall/krai_ws/src/mecanum_robot/scripts/best.pt')
        self.bridge = CvBridge()
        
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.led_pub = rospy.Publisher('/led_cmd', Bool, queue_size=10)
        
        rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.center_x = 320 

    def callback(self, data):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            results = self.model.predict(cv_img, conf=0.5, verbose=False)
            
            move = Twist()
            target_visible = False

            if len(results[0].boxes) > 0:
                target_visible = True
                box = results[0].boxes[0]
                x_mid = (box.xyxy[0][0] + box.xyxy[0][2]) / 2
                w = box.xyxy[0][2] - box.xyxy[0][0]

                # 1. Rotasi (PD Controller Sederhana)
                error_x = self.center_x - x_mid
                move.angular.y = error_x / 500.0 

                # 2. Maju (Berdasarkan lebar target)
                if w < 280: # Sesuaikan dengan ukuran objek 'Fake/Real' Anda
                    move.linear.x = 0.25
                else:
                    move.linear.x = 0.0
                    rospy.loginfo("Target Reached!")

            self.cmd_pub.publish(move)
            self.led_pub.publish(target_visible) # LED nyala jika ada target apa pun

            # Visualisasi
            res_plotted = results[0].plot()
            cv2.imshow("Jetson Vision", res_plotted)
            cv2.waitKey(1)
            
        except Exception as e:
            rospy.logerr(f"YOLO Error: {e}")

if __name__ == '__main__':
    YoloAutonomous()
    rospy.spin()