from glob import glob
import os

from setuptools import find_packages, setup


package_name = "xycar_rule_drive"

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="as",
    maintainer_email="as@example.com",
    description="Rule-based Xycar lane follower using KAIEV perception road segments.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lane_rule_driver = xycar_rule_drive.lane_rule_driver:main",
            "keyboard_teleop = xycar_rule_drive.keyboard_teleop:main",
            "kookmin_legacy_camera_driver = xycar_rule_drive.kookmin_legacy_camera_driver:main",
        ],
    },
)
