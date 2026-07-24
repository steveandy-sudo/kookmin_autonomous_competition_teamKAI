from glob import glob
import os

from setuptools import find_packages, setup


package_name = "xycar_gazebo_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob("rviz/*.rviz"),
        ),
        (
            os.path.join("share", package_name, "worlds"),
            glob("worlds/*.sdf"),
        ),
        (
            os.path.join(
                "share",
                package_name,
                "maps",
                "slam_glass_balanced",
            ),
            glob("maps/slam_glass_balanced/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="as",
    maintainer_email="as@example.com",
    description="Bridge real Xycar motor commands to Gazebo Ackermann cmd_vel.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "xycar_motor_bridge = xycar_gazebo_bridge.xycar_motor_bridge:main",
            "xycar_sensor_rviz_republisher = xycar_gazebo_bridge.xycar_sensor_rviz_republisher:main",
            "xycar_slam_world_generator = xycar_gazebo_bridge.slam_world_generator:main",
            "xycar_sim_control = xycar_gazebo_bridge.sim_control:main",
            "xycar_sim_control_gui = xycar_gazebo_bridge.sim_control_gui:main",
        ],
    },
)
