#!/usr/bin/env python3
"""
Waypoint Navigator Node
=======================
Navigasi otonom berbasis odometri differential drive (2 encoder).
- Motor Kiri  : msg.data[1]
- Motor Kanan : msg.data[2]

Aksi yang didukung:
  - "move" : maju/mundur dalam satuan cm (speed positif = maju)
  - "turn" : belok dalam derajat (positif = kiri, negatif = kanan)
"""

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


# ==============================================================================
# KONFIGURASI
# ==============================================================================

# --- Fisik Robot ---
WHEEL_DIAMETER_CM   = 9.5       # Diameter roda (cm)
TRACK_WIDTH_CM      = 80.0      # Jarak antar roda kiri-kanan (cm)
PPR                 = 385       # Pulsa per revolusi encoder

# --- Faktor Kalibrasi ---
LINEAR_SCALE        = 1.0       # Koreksi jarak linear (1.0 = tanpa koreksi)
ANGULAR_SCALE       = 0.127     # Koreksi sudut belok

# --- Toleransi ---
TURN_TOLERANCE_DEG  = 2.0       # Toleransi belok (derajat)

# --- P-Controller Belok ---
KP_TURN             = 0.02      # Proportional gain
MIN_TURN_SPEED      = 0.15      # Kecepatan minimum saat mendekati target

# --- Format Data Encoder (index array msg.data) ---
IDX_LEFT            = 1         # index motor kiri
IDX_RIGHT           = 2         # index motor kanan

# --- Daftar Waypoint ---
# Format: {"action": "move"|"turn", "value": <float>, "speed": <float>}
#   move  -> value dalam cm, speed = cmd_vel linear.x
#   turn  -> value dalam derajat, speed = cmd_vel angular.z maksimum
WAYPOINTS = [
    {"action": "move",  "value":  91.0, "speed":  -.6},
    {"action": "move",  "value": 91.0, "speed":  .6},
    # {"action": "turn",  "value":   12.0, "speed":  1}, # mines kanan
    # {"action": "move",  "value":  50.0, "speed":  1},
    # {"action": "turn",  "value":   -12.0, "speed":  1}, # mines kanan
    # {"action": "turn",  "value":  50.0, "speed":  -0.2},
]


# ==============================================================================
# NODE
# ==============================================================================

class WaypointNavigator:

    def __init__(self):
        rospy.init_node('waypoint_navigator_node')

        # Derived constant
        self.wheel_circumference = math.pi * WHEEL_DIAMETER_CM

        # Odometri
        self.prev_counts        = None   # {'L': float, 'R': float}
        self.total_distance_cm  = 0.0    # Akumulasi jarak maju/mundur
        self.encoder_yaw_deg    = 0.0    # Akumulasi sudut (derajat)

        # State mesin waypoint
        self.wp_index           = 0
        self.wp_active          = False
        self.wp_start_distance  = 0.0
        self.wp_start_yaw       = 0.0

        # ROS interface
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/motor_feedback', Float32MultiArray, self._feedback_cb)
        self.rate = rospy.Rate(10)

        rospy.loginfo("[Navigator] Node berhasil diinisialisasi.")

    # ==========================================================================
    # ODOMETRI
    # ==========================================================================

    def _feedback_cb(self, msg: Float32MultiArray):
        """Update odometri dari data encoder ESP32."""
        if len(msg.data) <= max(IDX_LEFT, IDX_RIGHT):
            rospy.logwarn_throttle(5.0,
                f"[Navigator] Data encoder kurang. "
                f"Dibutuhkan >= {max(IDX_LEFT, IDX_RIGHT) + 1} elemen, "
                f"diterima {len(msg.data)}."
            )
            return

        counts = {
            'L': msg.data[IDX_LEFT],
            'R': msg.data[IDX_RIGHT],
        }

        # Abaikan iterasi pertama; hanya simpan nilai awal
        if self.prev_counts is None:
            self.prev_counts = counts
            return

        # Delta tick
        d_tick_L = counts['L'] - self.prev_counts['L']
        d_tick_R = counts['R'] - self.prev_counts['R']

        # Konversi tick -> cm
        d_cm_L = (d_tick_L / PPR) * self.wheel_circumference
        d_cm_R = (d_tick_R / PPR) * self.wheel_circumference

        # Jarak linear (rata-rata 2 roda)
        delta_linear = ((d_cm_L + d_cm_R) / 2.0) * LINEAR_SCALE
        self.total_distance_cm += delta_linear

        # Perubahan yaw dari differential drive
        delta_yaw_rad = ((d_cm_R - d_cm_L) / TRACK_WIDTH_CM) * ANGULAR_SCALE
        self.encoder_yaw_deg += math.degrees(delta_yaw_rad)

        self.prev_counts = counts

    # ==========================================================================
    # NAVIGASI WAYPOINT
    # ==========================================================================

    def _start_waypoint(self, wp: dict):
        """Catat titik referensi saat waypoint baru dimulai."""
        self.wp_start_distance  = self.total_distance_cm
        self.wp_start_yaw       = self.encoder_yaw_deg
        self.wp_active          = True
        rospy.loginfo(
            f"[Navigator] >>> Mulai WP {self.wp_index + 1}/{len(WAYPOINTS)}: "
            f"{wp['action'].upper()} {wp['value']} (speed={wp['speed']})"
        )

    def _advance_waypoint(self):
        """Hentikan robot lalu lanjut ke waypoint berikutnya."""
        self._publish_stop()
        rospy.loginfo(f"[Navigator] <<< WP {self.wp_index + 1} SELESAI.")
        self.wp_index  += 1
        self.wp_active  = False
        rospy.sleep(0.5)    # Jeda singkat antar waypoint

    def _compute_move_cmd(self, wp: dict) -> Twist:
        """Kembalikan Twist untuk aksi 'move'."""
        jarak = abs(self.total_distance_cm - self.wp_start_distance)
        cmd   = Twist()
        if jarak < wp["value"]:
            cmd.linear.x = wp["speed"]
            rospy.loginfo_throttle(1.0,
                f"[Move] Tempuh: {jarak:.1f}/{wp['value']:.1f} cm"
            )
        else:
            self._advance_waypoint()
        return cmd

    def _compute_turn_cmd(self, wp: dict) -> Twist:
        """Kembalikan Twist untuk aksi 'turn' dengan P-controller."""
        target_yaw  = self.wp_start_yaw + wp["value"]
        error       = (target_yaw - self.encoder_yaw_deg + 180) % 360 - 180
        cmd         = Twist()

        if abs(error) > TURN_TOLERANCE_DEG:
            speed = KP_TURN * abs(error)
            speed = max(MIN_TURN_SPEED, min(speed, abs(wp["speed"])))
            cmd.angular.z = speed if error > 0 else -speed
            rospy.loginfo_throttle(0.5,
                f"[Turn] Target: {target_yaw:.1f}° | "
                f"Sekarang: {self.encoder_yaw_deg:.1f}° | "
                f"Error: {error:.1f}°"
            )
        else:
            self._advance_waypoint()
        return cmd

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    # ==========================================================================
    # MAIN LOOP
    # ==========================================================================

    def run(self):
        rospy.loginfo("[Navigator] Menunggu data /motor_feedback pertama...")
        while not rospy.is_shutdown() and self.prev_counts is None:
            self.rate.sleep()

        rospy.loginfo(f"[Navigator] Siap! Total {len(WAYPOINTS)} waypoint.")

        while not rospy.is_shutdown():
            if self.wp_index < len(WAYPOINTS):
                wp = WAYPOINTS[self.wp_index]

                if not self.wp_active:
                    self._start_waypoint(wp)

                if wp["action"] == "move":
                    cmd = self._compute_move_cmd(wp)
                elif wp["action"] == "turn":
                    cmd = self._compute_turn_cmd(wp)
                else:
                    rospy.logwarn(f"[Navigator] Aksi tidak dikenal: '{wp['action']}'")
                    self._advance_waypoint()
                    cmd = Twist()

                self.cmd_pub.publish(cmd)

            else:
                self._publish_stop()
                rospy.loginfo_throttle(5.0,
                    "[Navigator] Semua waypoint selesai. Robot berhenti."
                )

            self.rate.sleep()


# ==============================================================================
# ENTRY POINT
# ==============================================================================

if __name__ == '__main__':
    try:
        WaypointNavigator().run()
    except rospy.ROSInterruptException:
        pass