#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from pygame import mixer

from ament_index_python.packages import get_package_share_directory


class UltrasonicSpeaker(Node):
    def __init__(self):
        super().__init__('ultrasonic_speaker_node')

        # --- audio init ---
        self.audio_ok = True
        try:
            mixer.init()
        except Exception as e:
            self.audio_ok = False
            self.get_logger().error(f"[UltraSpeaker] mixer.init() failed: {e}")

        # --- params ---
        self.declare_parameter('sound_path', '')          # 비우면 패키지 sounds로 자동
        self.declare_parameter('cooldown_sec', 0.8)
        self.declare_parameter('interrupt', True)
        self.declare_parameter('file_prefix', 'ultra_')   # ultra_1.mp3 ...
        self.declare_parameter('file_suffix', '.mp3')
        self.declare_parameter('log_period_sec', 3.0)     # 3초 상태 로그
        self.declare_parameter('package_name', 'tts_speaker')  # ✅ 기본: tts_speaker

        pkg_name = self.get_parameter('package_name').value
        sound_path_param = self.get_parameter('sound_path').value

        # --- resolve sound_path ---
        if sound_path_param:
            self.sound_path = sound_path_param
        else:
            # default: <pkg_share>/sounds
            try:
                pkg_share = get_package_share_directory(pkg_name)
                self.sound_path = os.path.join(pkg_share, 'sounds')
            except Exception:
                # fallback: cwd/sounds
                self.sound_path = os.path.join(os.getcwd(), 'sounds')

        self.cooldown_sec = float(self.get_parameter('cooldown_sec').value)
        self.interrupt = bool(self.get_parameter('interrupt').value)
        self.file_prefix = self.get_parameter('file_prefix').value
        self.file_suffix = self.get_parameter('file_suffix').value
        self.log_period = float(self.get_parameter('log_period_sec').value)

        # --- state ---
        self.last_play_time = -1e9
        self.last_rx_time = -1e9
        self.last_idx = None

        # --- ros ---
        self.sub = self.create_subscription(UInt8, '/ultra/detect_idx', self.cb, 10)
        self.log_timer = self.create_timer(self.log_period, self._log_status)

        self.get_logger().info(f"[UltraSpeaker] sound_path={self.sound_path}")
        self.get_logger().info(f"[UltraSpeaker] audio_ok={self.audio_ok}, cooldown_sec={self.cooldown_sec}, interrupt={self.interrupt}")

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def is_busy(self):
        try:
            return mixer.music.get_busy()
        except Exception:
            return False

    def stop(self):
        try:
            mixer.music.stop()
        except Exception:
            pass

    def _file_for_idx(self, idx: int) -> str:
        fname = f"{self.file_prefix}{idx}{self.file_suffix}"
        return os.path.join(self.sound_path, fname)

    def _log_status(self):
        rx_age = self.now_sec() - self.last_rx_time
        play_age = self.now_sec() - self.last_play_time
        busy = self.is_busy()
        exists = os.path.isfile(self._file_for_idx(self.last_idx)) if self.last_idx else False

        self.get_logger().info(
            f"[UltraSpeaker] status: audio_ok={self.audio_ok}, busy={busy}, "
            f"last_idx={self.last_idx}, last_rx_age={rx_age:.2f}s, last_play_age={play_age:.2f}s, "
            f"last_file_exists={exists}, sound_path='{self.sound_path}'"
        )

    def play_idx(self, idx: int):
        path = self._file_for_idx(idx)
        if not os.path.isfile(path):
            self.get_logger().warn(f"[UltraSpeaker] file not found: {path}")
            return

        if not self.audio_ok:
            self.get_logger().error("[UltraSpeaker] audio device not initialized (mixer.init failed).")
            return

        try:
            mixer.music.load(path)
            mixer.music.play(0)
            self.last_play_time = self.now_sec()
            self.get_logger().info(f"[UltraSpeaker] PLAY idx={idx} ({os.path.basename(path)})")
        except Exception as e:
            self.get_logger().error(f"[UltraSpeaker] play failed: {e}")

    def cb(self, msg: UInt8):
        idx = int(msg.data)
        self.last_idx = idx
        self.last_rx_time = self.now_sec()

        if idx < 1 or idx > 6:
            return

        # cooldown
        if (self.now_sec() - self.last_play_time) < self.cooldown_sec:
            return

        if self.interrupt:
            self.stop()
            self.play_idx(idx)
        else:
            if not self.is_busy():
                self.play_idx(idx)


def main():
    rclpy.init()
    n = UltrasonicSpeaker() 
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()