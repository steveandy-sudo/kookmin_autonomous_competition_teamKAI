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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="teamkai",
    maintainer_email="teamkai@example.com",
    description=(
        "Object semantics and LiDAR cone command candidates for SLAM control."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "object_detection_node = my_rule.object_detection_node:main",
            "cone_node = my_rule.cone_node:main",
        ],
    },
)
