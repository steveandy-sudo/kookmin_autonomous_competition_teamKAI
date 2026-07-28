from glob import glob
import os

from setuptools import setup


package_name = "xycar_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="teamkai",
    maintainer_email="teamkai@example.com",
    description="Canonical road-image utilities and real camera calibration.",
    license="MIT",
)
