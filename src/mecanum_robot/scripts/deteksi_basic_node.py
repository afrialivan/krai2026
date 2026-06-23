#!/usr/bin/env python3
import rospy
import cv2  # type: ignore
import json
from ultralytics import YOLO  # type: ignore
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class YoloDetector:
    def __init__(self):
        rospy.init_node('yolo_detector_node')

        self.model = YOLO('/home/prall/krai_ws/src/mecanum_robot/models/best.pt')
        self.bridge = CvBridge()

        # Publisher data deteksi (JSON)
        self.detection_pub = rospy.Publisher('/yolo_detections', String, queue_size=10)

        rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        rospy.loginfo("YOLO Detector Node Started")

    def callback(self, data):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            img_h, img_w = cv_img.shape[:2]

            results = self.model.predict(cv_img, conf=0.5, verbose=False)

            detections = []

            if len(results[0].boxes) > 0:
                for box in results[0].boxes:
                    x1, y1, x2, y2 = (
                        float(box.xyxy[0][0]),
                        float(box.xyxy[0][1]),
                        float(box.xyxy[0][2]),
                        float(box.xyxy[0][3]),
                    )

                    # Koordinat tengah
                    cx = round((x1 + x2) / 2.0, 2)
                    cy = round((y1 + y2) / 2.0, 2)

                    class_id   = int(box.cls[0])
                    class_name = self.model.names[class_id]
                    confidence = round(float(box.conf[0]), 3)

                    detections.append({
                        "class_id":   class_id,
                        "class_name": class_name,
                        "confidence": confidence,
                        "cx": cx,
                        "cy": cy,
                        "bbox": [round(x1,2), round(y1,2), round(x2,2), round(y2,2)],
                    })

                    rospy.loginfo(f"[{class_name}] conf={confidence:.2f} | cx={cx} cy={cy}")

            else:
                rospy.loginfo_throttle(3, "Tidak ada objek terdeteksi")

            # Publish hasil deteksi sebagai JSON
            msg = String()
            msg.data = json.dumps({
                "frame_width":  img_w,
                "frame_height": img_h,
                "count":        len(detections),
                "detections":   detections,
            })
            self.detection_pub.publish(msg)

            # Tampilkan visualisasi
            res_plotted = results[0].plot()
            cv2.imshow("YOLO Detection", res_plotted)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Error: {e}")

    def shutdown(self):
        cv2.destroyAllWindows()
        rospy.loginfo("YOLO Detector Node Stopped")


if __name__ == '__main__':
    detector = YoloDetector()
    rospy.on_shutdown(detector.shutdown)
    rospy.spin()