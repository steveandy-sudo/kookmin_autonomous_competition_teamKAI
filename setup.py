from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'track_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 파일을 설치
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'assets', 'models'), glob('assets/models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'track_drive = track_drive.track_drive:main',
            'ai_drive_switch = track_drive.ai_drive_switch:main',
            'switchable_cone_ai_driver = track_drive.switchable_cone_ai_driver:main',
            'stop_line_bev_debug = track_drive.stop_line_bev_debug_node:main',
            'traffic_light_debug = track_drive.traffic_light_debug_node:main',
            'school_zone_debug = track_drive.school_zone_debug_node:main',
            'intersection_debug = track_drive.intersection_debug_node:main',
        ],
    },
)
