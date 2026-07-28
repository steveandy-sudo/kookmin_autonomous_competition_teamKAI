import os
from glob import glob
from setuptools import setup

package_name = 'app_wide_camera'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kai',
    maintainer_email='kai@example.com',
    description='Xycar MJPG passthrough compressed camera publisher',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mjpeg_passthrough_node = app_wide_camera.mjpeg_passthrough_node:main',
        ],
    },
)
