#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# ===============================================================================
# /usb_cam/image_with_crosshair
# inputkan di rqt_image_view
# ===============================================================================

class ImageCrosshair:
    def __init__(self):
        # Inisialisasi node ROS
        rospy.init_node('crosshair_node', anonymous=True)
        
        # Jembatan antara ROS Image dan OpenCV Mat
        self.bridge = CvBridge()
        
        # Subscribe ke topik kamera asli
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        
        # Publisher untuk mengirim gambar yang sudah diberi garis
        self.image_pub = rospy.Publisher("/usb_cam/image_with_crosshair", Image, queue_size=10)
        
        rospy.loginfo("Node Crosshair berhasil dijalankan...")

    def callback(self, data):
        try:
            # Konversi gambar ROS ke OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Mendapatkan resolusi gambar (tinggi, lebar)
        height, width, _ = cv_image.shape
        
        # Menghitung titik tengah
        center_x = int(width / 2)
        center_y = int(height / 2)
        
        # Pengaturan Garis (Warna BGR dan Ketebalan)
        # (0, 255, 0) berarti warna HIJAU. Silakan ganti sesuai selera.
        color = (0, 255, 0) 
        thickness = 2
        line_length = 20 # Panjang garis crosshair
        
        # Menggambar garis horizontal tengah
        cv2.line(cv_image, (center_x - line_length, center_y), (center_x + line_length, center_y), color, thickness)
        
        # Menggambar garis vertikal tengah
        cv2.line(cv_image, (center_x, center_y - line_length), (center_x, center_y + line_length), color, thickness)

        try:
            # Konversi kembali dari OpenCV ke ROS Image dan Publish
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    try:
        ImageCrosshair()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass