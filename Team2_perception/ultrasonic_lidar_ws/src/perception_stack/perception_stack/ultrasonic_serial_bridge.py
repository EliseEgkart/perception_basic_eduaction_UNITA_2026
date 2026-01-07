#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Float32MultiArray
import serial


class UltrasonicSerialBridge(Node):
    """
    Arduino output (your code):
      d1,d2,d3,d4,d5,d6   (meters, float, 3 decimals)

    ROS2 output:
      /ultra/ranges      Float32MultiArray([d1..d6])  (meters)
      /ultra/detect_idx  UInt8(idx)  (1..6)           (edge-triggered)
    """
    def __init__(self):
        super().__init__('ultrasonic_serial_bridge')

        # --- params ---
        # You confirmed working port was /dev/ttyUSB0 in your other node.
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)

        # detection parameters (meters)
        self.declare_parameter('threshold_m', 0.10)          # 10cm
        self.declare_parameter('event_cooldown_sec', 0.8)    # 이벤트 쿨다운
        self.declare_parameter('log_period_sec', 3.0)        # 3초 상태 로그

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)

        self.threshold_m = float(self.get_parameter('threshold_m').value)
        self.event_cooldown_sec = float(self.get_parameter('event_cooldown_sec').value)
        self.log_period = float(self.get_parameter('log_period_sec').value)

        # --- publishers ---
        self.pub_idx = self.create_publisher(UInt8, '/ultra/detect_idx', 10)
        self.pub_ranges = self.create_publisher(Float32MultiArray, '/ultra/ranges', 10)

        # --- serial ---
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.get_logger().info(f"[UltraBridge] Open serial {port} @ {baud}")
        self.get_logger().info(f"[UltraBridge] threshold_m={self.threshold_m:.3f}, cooldown={self.event_cooldown_sec:.2f}s")

        # --- state tracking for edge + cooldown ---
        self.prev_detect = [False] * 6
        self.last_event_sec = -1e9

        # --- status logging ---
        self.lines_total = 0
        self.frames_total = 0
        self.events_total = 0
        self.last_line = ""
        self.last_rx_sec = -1e9
        self.last_event_idx = None
        self.last_ranges = None

        # timers
        self.timer = self.create_timer(0.01, self._tick)
        self.log_timer = self.create_timer(self.log_period, self._log_status)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _log_status(self):
        age = self._now_sec() - self.last_rx_sec
        self.get_logger().info(
            f"[UltraBridge] status: lines={self.lines_total}, frames={self.frames_total}, events={self.events_total}, "
            f"last_rx_age={age:.2f}s, last_event_idx={self.last_event_idx}, "
            f"last_line='{self.last_line[:80]}'"
        )

    def _tick(self):
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return

            self.lines_total += 1
            self.last_line = line
            self.last_rx_sec = self._now_sec()

            # --- parse CSV: "d1,d2,d3,d4,d5,d6" ---
            parts = line.split(',')
            if len(parts) != 6:
                # Arduino가 헤더/다른 출력이 섞이면 여기서 걸림
                self.get_logger().warn(f"[UltraBridge] Invalid data (need 6 values): {line}")
                return

            vals = []
            for p in parts:
                vals.append(float(p))

            # publish ranges every frame
            msg = Float32MultiArray()
            msg.data = vals
            self.pub_ranges.publish(msg)
            self.frames_total += 1
            self.last_ranges = vals

            # --- detect event: edge + cooldown ---
            now = self._now_sec()
            detected_now = [(0.0 < d < self.threshold_m) for d in vals]

            # rising edges indices where prev was False and now is True
            rising = [i for i in range(6) if detected_now[i] and (not self.prev_detect[i])]

            # update prev
            self.prev_detect = detected_now

            if not rising:
                return

            # cooldown
            if (now - self.last_event_sec) < self.event_cooldown_sec:
                return

            # choose nearest among rising edges (min distance)
            best_i = min(rising, key=lambda i: vals[i])

            idx = best_i + 1  # 1..6
            self.pub_idx.publish(UInt8(data=idx))
            self.last_event_sec = now
            self.events_total += 1
            self.last_event_idx = idx

            self.get_logger().info(f"[UltraBridge] DETECT idx={idx} dist={vals[best_i]:.3f} m (rising edge)")

        except Exception as e:
            self.get_logger().warn(f"[UltraBridge] read/parse error: {e}")


def main():
    rclpy.init()
    n = UltrasonicSerialBridge()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()