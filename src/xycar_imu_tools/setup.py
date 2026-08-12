from glob import glob
import os

from setuptools import find_packages, setup


package_name = "xycar_imu_tools"

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
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob("rviz/*.rviz"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="as",
    maintainer_email="as@example.com",
    description="EBIMU serial publisher and IMU-based vehicle calibration tools.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ebimu_serial_publisher = xycar_imu_tools.ebimu_serial_publisher:main",
            "imu_tf_broadcaster = xycar_imu_tools.imu_tf_broadcaster:main",
            "imu_vehicle_spec_calibrator = xycar_imu_tools.imu_vehicle_spec_calibrator:main",
        ],
    },
)
