import os
from glob import glob

from setuptools import setup


package_name = "xycar_camera"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Team K.A.I.",
    maintainer_email="team.kai@example.com",
    description="Xycar MJPEG passthrough compressed camera publisher",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            (
                "xycar_camera_node = "
                "xycar_camera.mjpeg_passthrough_node:main"
            ),
        ],
    },
)
