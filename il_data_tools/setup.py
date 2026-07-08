from glob import glob
from os.path import isfile
from setuptools import find_packages, setup


package_name = "il_data_tools"
script_files = [path for path in glob("scripts/*") if isfile(path)]

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/scripts", script_files),
    ],
    scripts=script_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Team K.A.I.",
    maintainer_email="team.kai@example.com",
    description="Safe imitation-learning data collection tools for Xycar ROS2.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "il_common_recorder = il_data_tools.common_recorder_node:main",
            "il_mission_labeler = il_data_tools.mission_labeler_node:main",
        ],
    },
)
