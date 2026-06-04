# Tambahkan Float32 pada import jika belum ada
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Twist
import rospy
import serial
import threading

class BaseController:

    def __init__(self):
        rospy.init_node('base_controller_node')

        # --- Variabel Status Capit ---
        self.capit_state = 0.0  # Nilai default capit (0.0 = terbuka/mati)

        # Koneksi Serial
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)
        except:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)

        # Publisher
        self.yaw_pub = rospy.Publisher('/robot_yaw', Float32, queue_size=10)
        self.feedback_pub = rospy.Publisher('/motor_feedback', Float32MultiArray, queue_size=10)

        # Subscriber
        rospy.Subscriber("cmd_vel", Twist, self.cmd_cb)
        
        # Subscriber BARU untuk menerima perintah dari capit_controller
        rospy.Subscriber("/capit_cmd", Float32, self.capit_cb)

        self.last_cmd_time = rospy.get_time()

        rospy.Timer(rospy.Duration(0.1), self.update_system)

        self.read_thread = threading.Thread(target=self.read_from_serial)
        self.read_thread.daemon = True
        self.read_thread.start()

    # ==================================================
    # CALLBACK CAPIT (BARU)
    # ==================================================
    def capit_cb(self, msg):
        # Update nilai capit dari node capit_controller
        self.capit_state = msg.data

    # ... (Fungsi read_from_serial tetap sama) ...

    # ==================================================
    # CALLBACK CMD_VEL
    # ==================================================
    def cmd_cb(self, msg):
        self.last_cmd_time = rospy.get_time()

        x = msg.linear.x
        y = msg.linear.y
        z = msg.angular.z

        # KINEMATIKA MECANUM
        fl = x - y - z
        fr = x + y + z
        rl = x - y - z
        rr = x + y + z
        tl = 0
        tr = 0

        # KONVERSI KE SETPOINT RPM
        max_rpm = 60.0
        max_pwm = 255.0
        fl_sp = fl * max_rpm
        fr_sp = fr * max_rpm
        rl_sp = rl * max_rpm
        rr_sp = rr * max_rpm
        tl_sp = tl * max_pwm
        tr_sp = tr * max_pwm
        
        # Ganti hardcode '1' dengan self.capit_state
        proxyCapit = self.capit_state 
        data2 = 255.0
        data3 = 255.0
        data4 = 255.0

        self.send_setpoint(fl_sp, fr_sp, rl_sp, rr_sp, tl_sp, tr_sp, proxyCapit, data2, data3, data4)

    # ... (Fungsi send_setpoint tetap sama) ...

    # ==================================================
    # WATCHDOG
    # ==================================================
    def update_system(self, event):
        # Jika tidak ada cmd_vel masuk selama 0.5 detik, stop robot
        if rospy.get_time() - self.last_cmd_time > 0.5:
            # PENTING: Gunakan self.capit_state agar capit tetap mempertahankan posisinya
            # meski roda robot berhenti (0).
            self.send_setpoint(0, 0, 0, 0, 0, 0, self.capit_state, 0, 0, 0)