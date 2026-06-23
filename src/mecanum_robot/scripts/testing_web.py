#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import random

class KraiRobotBridge:
    def __init__(self):
        # 1. Inisialisasi Node ROS
        rospy.init_node('krai_robot_bridge_node', anonymous=False)
        
        # 2. Variabel Internal (State Robot)
        self.counter_depan = 0
        self.status_gerak = "Stop"
        self.baterai_maks = 100

        # 3. Setup Semua Publisher (ROS -> Web)
        self.pub_depan = rospy.Publisher('/sensor_depan', String, queue_size=10)
        self.pub_kiri = rospy.Publisher('/sensor_kiri', String, queue_size=10)
        self.pub_baterai = rospy.Publisher('/baterai', String, queue_size=10)

        # 4. Setup Semua Subscriber (Web -> ROS)
        rospy.Subscriber('/perintah_gerak', String, self.cb_pergerakan)
        rospy.Subscriber('/mode_robot', String, self.cb_mode_sistem)
        rospy.Subscriber('/kontrol_aktuator', String, self.cb_aktuator)

        rospy.loginfo("🚀 [KRAI BRIDGE] Node berhasil diinisialisasi berbasis OOP!")

    # ==========================================
    # FUNGSI CALLBACK (Sangat mudah ditambah ke bawah)
    # ==========================================
    def cb_pergerakan(self, msg):
        self.status_gerak = msg.data
        rospy.loginfo("[GERAK] Mengubah status ke: %s", self.status_gerak)

    def cb_mode_sistem(self, msg):
        if msg.data == 'Reset':
            self.counter_depan = 0
            rospy.loginfo("[SISTEM] Melakukan reset sensor depan.")

    def cb_aktuator(self, msg):
        rospy.logwarn("[AKTUATOR ACTIVED] Eksekusi perintah: %s !!!", msg.data)

    # ==========================================
    # LOGIKA UTAMA TELEMETRI (LOOPING)
    # ==========================================
    def update_telemetri(self):
        # Simulasi perubahan data data sensor
        if self.status_gerak != "Stop":
            self.counter_depan += 2

        val_kiri = str(random.randint(15, 25))
        val_baterai = str(max(self.baterai_maks - (self.counter_depan // 8), 0))

        # Mengirim ke Web UI
        self.pub_depan.publish(str(self.counter_depan))
        self.pub_kiri.publish(val_kiri)
        self.pub_baterai.publish(val_baterai)


if __name__ == '__main__':
    try:
        # Instansiasi objek class
        robot_bridge = KraiRobotBridge()
        
        # Mengatur frekuensi loop utama (5 kali per detik)
        rate = rospy.Rate(5)
        
        while not rospy.is_shutdown():
            robot_bridge.update_telemetri()
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass