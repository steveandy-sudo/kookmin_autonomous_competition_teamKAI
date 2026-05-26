from setuptools import setup

package_name = 'track_drive'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team KAI',
    maintainer_email='teamkai@example.com',
    description='ROS2 starter package for Team KAI autonomous driving simulator task.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'track_drive = track_drive.track_drive:main',
        ],
    },
)
