#!/usr/bin/env python3
import threading
import time

import rclpy
from rclpy.node import Node
import serial

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

# HC-SR04 datasheet-typical values; see bruno_base.ino for the matching
# firmware-side SAFE/WARN thresholds and ECHO_TIMEOUT_US (25ms -> ~4.3m).
SONAR_MIN_RANGE_M = 0.02
SONAR_MAX_RANGE_M = 4.0
SONAR_FOV_RAD = 0.26  # ~15 degrees


class BrunoBaseNode(Node):
    def __init__(self):
        super().__init__('bruno_base_node')

        self.declare_parameter('port', '/dev/esp32')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('max_pwm', 160)
        self.declare_parameter('cmd_vel_timeout_sec', 0.5)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.cmd_vel_timeout_sec = self.get_parameter('cmd_vel_timeout_sec').value

        self._serial_lock = threading.Lock()
        self.ser = serial.Serial(self.port, self.baud, timeout=1.0)
        self.get_logger().info(f'Opened {self.port} @ {self.baud} baud')

        self._last_cmd_time = time.monotonic()
        self._timeout_active = False
        self._send_motor_command(0, 0)

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10)

        self.range_pub_left = self.create_publisher(Range, '/sonar/left', 10)
        self.range_pub_center = self.create_publisher(Range, '/sonar/center', 10)
        self.range_pub_right = self.create_publisher(Range, '/sonar/right', 10)

        self._timeout_timer = self.create_timer(0.1, self._check_cmd_vel_timeout)

        self._stop_reader = threading.Event()
        self._reader_thread = threading.Thread(target=self._serial_reader_loop, daemon=True)
        self._reader_thread.start()

    # -- outbound: /cmd_vel -> "M <left> <right>" -----------------------------

    def _cmd_vel_callback(self, msg: Twist):
        self._last_cmd_time = time.monotonic()
        self._timeout_active = False

        # No wheel-radius/track-width calibration exists for this platform yet
        # (no URDF, open-loop motors - see SESSION_HANDOVER.md), so linear.x and
        # angular.z are treated as normalized commands in roughly [-1, 1] and
        # mixed directly into per-side PWM, scaled by max_pwm.
        left = msg.linear.x - msg.angular.z
        right = msg.linear.x + msg.angular.z

        left_pwm = int(max(-255, min(255, left * self.max_pwm)))
        right_pwm = int(max(-255, min(255, right * self.max_pwm)))

        self._send_motor_command(left_pwm, right_pwm)

    def _check_cmd_vel_timeout(self):
        if self._timeout_active:
            return
        if time.monotonic() - self._last_cmd_time > self.cmd_vel_timeout_sec:
            self.get_logger().warn(
                f'No /cmd_vel for {self.cmd_vel_timeout_sec}s - sending M 0 0')
            self._send_motor_command(0, 0)
            self._timeout_active = True

    def _send_motor_command(self, left_pwm: int, right_pwm: int):
        line = f'M {left_pwm} {right_pwm}\n'
        with self._serial_lock:
            self.ser.write(line.encode('ascii'))

    # -- inbound: "U <left_cm> <mid_cm> <right_cm>" -> Range topics ----------

    def _serial_reader_loop(self):
        while not self._stop_reader.is_set():
            try:
                raw = self.ser.readline()
            except serial.SerialException as exc:
                self.get_logger().error(f'Serial read failed: {exc}')
                time.sleep(0.5)
                continue

            if not raw:
                continue

            line = raw.decode('ascii', errors='replace').strip()
            if not line.startswith('U '):
                continue

            parts = line.split()
            if len(parts) != 4:
                self.get_logger().warn(f'Malformed telemetry line: {line!r}')
                continue

            try:
                left_cm, mid_cm, right_cm = (int(parts[1]), int(parts[2]), int(parts[3]))
            except ValueError:
                self.get_logger().warn(f'Malformed telemetry line: {line!r}')
                continue

            now = self.get_clock().now().to_msg()
            self.range_pub_left.publish(self._make_range_msg(left_cm, now, 'sonar_left_link'))
            self.range_pub_center.publish(self._make_range_msg(mid_cm, now, 'sonar_center_link'))
            self.range_pub_right.publish(self._make_range_msg(right_cm, now, 'sonar_right_link'))

    def _make_range_msg(self, dist_cm: int, stamp, frame_id: str) -> Range:
        msg = Range()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = SONAR_FOV_RAD
        msg.min_range = SONAR_MIN_RANGE_M
        msg.max_range = SONAR_MAX_RANGE_M
        if dist_cm <= 0:
            # -1 (and any other non-positive value) from the firmware means
            # "no echo / out of range" -> report max_range per sensor_msgs/Range
            # convention for an out-of-range reading.
            msg.range = SONAR_MAX_RANGE_M
        else:
            msg.range = min(dist_cm / 100.0, SONAR_MAX_RANGE_M)
        return msg

    def destroy_node(self):
        self._stop_reader.set()
        self._send_motor_command(0, 0)
        self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BrunoBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
