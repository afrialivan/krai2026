#!/usr/bin/env python3

import rospy
import serial # type: ignore
import threading
from std_msgs.msg import Float32MultiArray, Float32 # Tambahkan import Float32

class SensorReceiverNode:
    def __init__(self):
        # anonymous=True agar jika kamu jalankan 2 node yang sama, namanya tidak bentrok
        rospy.init_node('sensor_receiver_node', anonymous=True)

        # Mengambil parameter port dan topic dari terminal
        port_serial = rospy.get_param('~port', '/dev/ttyUSB1')
        baud_rate = rospy.get_param('~baud', 115200)
        topic_name = rospy.get_param('~topic', '/sensor')

        # Publisher
        self.data_pub = rospy.Publisher(topic_name, Float32MultiArray, queue_size=10)
        # Publisher baru khusus untuk yaw
        self.yaw_pub = rospy.Publisher('/robot_yaw', Float32, queue_size=10)

        # Koneksi Serial
        try:
            self.ser = serial.Serial(port_serial, baud_rate, timeout=0.05)
            rospy.loginfo(f"Node Receiver terhubung ke {port_serial} -> Publish ke topic {topic_name} dan /robot_yaw")
        except Exception as e:
            rospy.logerr(f"Gagal membuka port {port_serial}: {e}")
            self.ser = None

        # Thread pembacaan agar tidak memblokir sistem
        if self.ser:
            self.read_thread = threading.Thread(target=self.read_from_serial)
            self.read_thread.daemon = True
            self.read_thread.start()

    def read_from_serial(self):
        rospy.loginfo("Mulai mendengarkan data dari ESP32...")
        while not rospy.is_shutdown():
            if self.ser and self.ser.in_waiting > 0:
                try:
                    # Baca baris data dan bersihkan spasi/enter di ujungnya
                    line = self.ser.readline().decode('utf-8').strip()
                    
                    if line:
                        # Hapus prefix "SENSOR:" jika ESP32 mengirimnya dengan awalan teks
                        clean_line = line.replace("SENSOR:", "")
                        
                        # Pecah berdasarkan koma dan ubah menjadi list of float
                        data_string = clean_line.split(',')
                        data_floats = [float(x) for x in data_string]
                        
                        # Masukkan ke pesan ROS dan publish array utuh
                        msg = Float32MultiArray()
                        msg.data = data_floats
                        self.data_pub.publish(msg)

                        # Pastikan list memiliki setidaknya 4 elemen (index 0,1,2,3) sebelum publish yaw
                        if len(data_floats) >= 4:
                            yaw_msg = Float32()
                            yaw_msg.data = data_floats[3]
                            self.yaw_pub.publish(yaw_msg)
                        
                except ValueError:
                    # Abaikan (pass) jika ada noise serial yang bukan angka
                    pass 
                except Exception as e:
                    rospy.logwarn(f"Error membaca data: {e}")

if __name__ == '__main__':
    try:
        node = SensorReceiverNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass