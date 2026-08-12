from glob import glob
import os

from setuptools import find_packages, setup


package_name = "xycar_dynamics_test"

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
    description="Real Xycar dynamics measurement tools.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "dynamics_test_runner = xycar_dynamics_test.dynamics_test_runner:main",
            "analyze_dynamics_log = xycar_dynamics_test.analyze_dynamics_log:main",
        ],
    },
)
