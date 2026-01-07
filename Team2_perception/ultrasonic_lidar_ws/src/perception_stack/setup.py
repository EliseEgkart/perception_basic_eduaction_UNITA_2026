from setuptools import setup

package_name = 'perception_stack'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gyun',
    maintainer_email='gyun@todo.todo',
    description='Ultrasonic + LiDAR clustering + speaker nodes',
    license='TODO',
    entry_points={
        'console_scripts': [
            'scan_cluster_node = perception_stack.scan_cluster_node:main',
            'rplidar_node = perception_stack.rplidar_node:main',
            'ultrasonic_serial_bridge = perception_stack.ultrasonic_serial_bridge:main',
            'ultrasonic_speaker_node = perception_stack.ultrasonic_speaker_node:main',
        ],
    },
)
