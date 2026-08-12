from glob import glob
import os

from setuptools import find_packages, setup


package_name = "my_rule"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name), ["requirements.txt"]),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*.yaml"))),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.launch.py"))),
        (os.path.join("share", package_name, "models"), glob(os.path.join("models", "*"))),
        (os.path.join("share", package_name, "rviz"), glob(os.path.join("rviz", "*.rviz"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="teamkai",
    maintainer_email="teamkai@example.com",
    description=(
        "Integrated lane, object-semantic, and LiDAR cone driving stack."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lane_perception_node = my_rule.lane_perception_node:main",
            "object_detection_node = my_rule.object_detection_node:main",
            "cone_node = my_rule.cone_node:main",
            "cone_path_visualizer = my_rule.cone_path_visualizer_node:main",
            "drive_path_visualizer = my_rule.cone_path_visualizer_node:main",
            "bag_progress = my_rule.bag_progress_node:main",
            "compressed_camera_republisher = my_rule.compressed_camera_republisher_node:main",
            "drive_manager = my_rule.drive_manager_node:main",
            "perception_view_node = my_rule.perception_view_node:main",
        ],
    },
)
