#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray, Float32 # Tambahkan Float32

class CapitController:

    def __init__(self):
        rospy.init_node('capit_controller_node')

        self.proximity_data = None
        self.data_latched = False

        # Publisher baru untuk mengirim perintah capit ke BaseController
        self.capit_pub = rospy.Publisher('/capit_cmd', Float32, queue_size=10)

        # Subscriber 
        rospy.Subscriber('/motor_feedback', Float32MultiArray, self.feedback_cb)

        rospy.loginfo("Node Capit Controller aktif. Menunggu data proximity...")
        
    def feedback_cb(self, msg):
        if len(msg.data) >= 9:
            current_proximity = msg.data[8]
            
            if not self.data_latched:
                if current_proximity == 1.0: 
                    self.proximity_data = current_proximity
                    self.data_latched = True
                    rospy.loginfo(f"Objek terdeteksi! Mengirim perintah capit...")
                    self.execute_capit()

    def execute_capit(self):
        # Kirim nilai 1.0 sebagai perintah untuk menutup capit
        cmd_msg = Float32()
        cmd_msg.data = 1.0 
        self.capit_pub.publish(cmd_msg)
        rospy.loginfo("Perintah capit (1.0) telah dikirim ke /capit_cmd.")

    def reset_capit(self):
        self.data_latched = False
        self.proximity_data = None
        
        # Kirim nilai 0.0 untuk membuka capit kembali
        cmd_msg = Float32()
        cmd_msg.data = 0.0 
        self.capit_pub.publish(cmd_msg)
        rospy.loginfo("Sistem capit direset. Perintah capit (0.0) dikirim.")

if __name__ == '__main__':
    try:
        controller = CapitController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass