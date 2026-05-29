from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'cone_il'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='taeyun',
    maintainer_email='daniyun02@gmail.com',
    description='Cone imitation learning nodes for ROS2 Xycar',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cone_data_recorder = cone_il.cone_data_recorder_node:main',
            'cone_keyboard_recorder = cone_il.cone_data_recorder_node:keyboard_main',
            'cone_raw_keyboard_recorder = cone_il.cone_data_recorder_node:raw_keyboard_main',
            'cone_ai_driver = cone_il.cone_ai_driver_node:main',
            'xycar_keyboard_teleop = cone_il.xycar_keyboard_teleop_node:main',
            'xycar_cv_keyboard_teleop = cone_il.xycar_cv_keyboard_teleop_node:main',
        ],
    },
)
