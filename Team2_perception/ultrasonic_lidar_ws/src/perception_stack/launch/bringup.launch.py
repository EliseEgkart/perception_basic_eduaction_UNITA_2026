from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 라이다 드라이버(네 rplidar_node.py 사용 시)
        Node(
            package='perception_stack',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
        ),

        # 클러스터링 + RViz 마커
        Node(
            package='perception_stack',
            executable='scan_cluster_node',
            name='scan_cluster_node',
            output='screen',
        ),

        # 초음파 시리얼 브릿지
        Node(
            package='perception_stack',
            executable='ultrasonic_serial_bridge',
            name='ultrasonic_serial_bridge',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB0'},
                {'baud': 115200},
                {'topic': '/ultrasonic/ranges_cm'},
            ],
        ),

        # 초음파 감지 -> 음성
        Node(
            package='perception_stack',
            executable='ultrasonic_speaker_node',
            name='ultrasonic_speaker_node',
            output='screen',
            parameters=[
                {'topic': '/ultrasonic/ranges_cm'},
                {'threshold_cm': 40.0},
                {'cooldown_s': 1.5},
            ],
        ),
    ])
