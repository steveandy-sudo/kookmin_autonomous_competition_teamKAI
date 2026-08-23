from glob import glob
import os

from setuptools import find_packages, setup


package_name = "lane_bev_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="as",
    maintainer_email="as@example.com",
    description="BEV preview tools adapted from the real Xycar wide-camera lane calibration flow.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "bev_preview_homography = lane_bev_tools.bev_preview_homography:main",
        ],
    },
)
