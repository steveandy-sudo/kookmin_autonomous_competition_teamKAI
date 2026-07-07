import os
from glob import glob

from setuptools import find_packages, setup


package_name = "il_data_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Team K.A.I.",
    maintainer_email="team.kai@example.com",
    description="Safe data collection tools for Xycar imitation learning.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dataset_recorder_node = il_data_tools.dataset_recorder_node:main",
            "mission_labeler_node = il_data_tools.mission_labeler_node:main",
        ],
    },
)
