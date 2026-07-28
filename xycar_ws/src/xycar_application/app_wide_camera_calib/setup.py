import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'app_wide_camera_calib'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kai',
    maintainer_email='sunwoo050223@naver.com',
    description='Wide camera fisheye rectification and LiDAR-camera calibration tools.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rectify_wide_camera_node = app_wide_camera_calib.scripts.rectify_wide_camera_node:main',
            'publish_lidar_camera_static_tf = app_wide_camera_calib.scripts.publish_lidar_camera_static_tf:main',
        ],
    },
)
