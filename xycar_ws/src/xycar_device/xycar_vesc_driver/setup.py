from glob import glob
import os

from setuptools import find_packages, setup


package_name = "xycar_vesc_driver"

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
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    test_suite="test",
    zip_safe=True,
    maintainer="xytron",
    maintainer_email="xytron@example.com",
    description="Native ROS 2 Xycar VESC serial driver.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "xycar_vesc_driver = xycar_vesc_driver.driver_node:main",
        ],
    },
)
