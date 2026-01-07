from setuptools import find_packages, setup
import os  # ✅ os.path.join 사용하려면 필요
from glob import glob

package_name = 'sound_tts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # add
        (os.path.join('share', package_name, 'sound'), glob('sound/*.mp3')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='noid',
    maintainer_email='noid@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sound_node = sound_tts.sound_node:main',
        ],
    },
)
