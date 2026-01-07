import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    usb_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('usb_cam'),
                'launch',
                'camera.launch.py',
            )
        )
    )

    rviz_config = os.path.join(
        get_package_share_directory('main_pkg'),
        'config',
        'traffic_light.rviz',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    traffic = Node(
        package='traffic_light_pkg',
        executable='traffic_light_node',
        name='traffic_light_node',
        output='screen',
    )

    sound = Node(
        package='sound_tts',
        executable='sound_node',
        name='sound_node',
        output='screen',
    )

    return LaunchDescription([usb_cam_launch, traffic, sound, rviz])
