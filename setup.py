from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'track_drive'
runtime_launch_files = [
    'launch/mission_manager_draft.launch.py',
    'launch/traffic_light_debug.launch.py',
]
runtime_model_files = [
    'assets/models/final.onnx',
]
runtime_rviz_files = [
    'rviz/traffic_light_debug.rviz',
]

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            runtime_launch_files,
        ),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (
            os.path.join('share', package_name, 'docs'),
            ['docs/MISSION_MANAGER_V02.md'],
        ),
        (
            os.path.join('share', package_name, 'rviz'),
            runtime_rviz_files,
        ),
        (
            os.path.join('share', package_name, 'assets', 'models'),
            runtime_model_files,
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Team K.A.I. Mission Manager V0.2 integration package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'traffic_light_debug = track_drive.traffic_light_debug_node:main',
            'mission_manager = track_drive.mission.mission_manager_node:main',
            'mission_start_signal_adapter = track_drive.mission.start_signal_adapter_node:main',
            'mission_drive_policy_adapter = track_drive.integration.drive_policy_adapter_node:main',
            'mission_lane_fallback_adapter = track_drive.integration.lane_fallback_adapter_node:main',
            'mission_camera_cone_adapter = track_drive.integration.camera_cone_adapter_node:main',
            'mission_lidar_cone_adapter = track_drive.integration.lidar_cone_adapter_node:main',
        ],
    },
)
