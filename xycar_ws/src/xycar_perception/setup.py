from glob import glob
import os

from setuptools import find_packages, setup


package_name = "xycar_perception"

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="as",
    maintainer_email="as@example.com",
    description="Camera-based Xycar perception publisher compatible with KAIEV perception topics.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "camera_perception_node = xycar_perception.camera_perception_node:main",
        ],
    },
)
