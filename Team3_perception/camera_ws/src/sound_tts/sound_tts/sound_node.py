
# ====================
# 편집자 : 이원종, 정규민, 김민서
# 최종 수정일  : 2026-01-06
# 작업 상태 : 진행완료
# 역할 : 신호등 인식 결과 토픽 구독 및 음성 알림(TTS/사운드) 출력 노드 구현
# ====================

# sound_tts/sound_tts/sound_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import os
from playsound import playsound  

class SoundNode(Node):
    def __init__(self):
        super().__init__('sound_node')

        # 구독할 토픽들
        self.topics = {
            'red': '/traffic_light/red',
            'orange': '/traffic_light/orange',
            'green': '/traffic_light/green'
        }

        # 사운드 파일 경로
        self.sounds_dir = os.path.join(os.path.dirname(__file__), '..', 'sound')
        self.color_to_file = {
            'red': 'red.mp3',
            'orange': 'orange.mp3',
            'green': 'green.mp3'
        }

        # 각 토픽별 subscriber 생성
        for color, topic_name in self.topics.items():
            self.create_subscription(
                Bool,
                topic_name,
                lambda msg, c=color: self.listener_callback(msg, c),
                10
            )

        self.get_logger().info('SoundNode initialized!')

    def listener_callback(self, msg: Bool, color: str):
        if msg.data:  # Bool 값이 True일 때만 소리 재생
            sound_file = os.path.join(self.sounds_dir, self.color_to_file[color])
            if os.path.exists(sound_file):
                self.get_logger().info(f'Playing sound for {color}')
                playsound(sound_file)
            else:
                self.get_logger().warn(f'Sound file not found: {sound_file}')

def main(args=None):
    rclpy.init(args=args)
    node = SoundNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
