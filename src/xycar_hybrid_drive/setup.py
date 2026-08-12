from glob import glob
import os

from setuptools import find_packages, setup


package_name = "xycar_hybrid_drive"

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Team K.A.I.",
    maintainer_email="team.kai@example.com",
    description="Single-node model, lane-rule, and cone-rule Xycar driver.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "hybrid_drive_node = xycar_hybrid_drive.hybrid_drive_node:main",
        ],
    },
)
