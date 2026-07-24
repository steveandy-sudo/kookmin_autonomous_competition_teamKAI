from glob import glob
import os

from setuptools import find_packages, setup


package_name = "xycar_map_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "scripts"),
            glob("scripts/*.sh"),
        ),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Team K.A.I.",
    maintainer_email="team.kai@example.com",
    description=(
        "Map waypoint planning, path following, and rule handover for Xycar."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "command_odom_node = xycar_map_nav.command_odom_node:main",
            "waypoint_nav_node = xycar_map_nav.waypoint_nav_node:main",
        ],
    },
)
