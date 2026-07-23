from glob import glob
import os

from setuptools import setup


package_name = "my_rule"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*.yaml"))),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.launch.py"))),
        (os.path.join("share", package_name, "models"), glob(os.path.join("models", "*"))),
        (os.path.join("share", package_name, "docs"), glob(os.path.join("docs", "*.md"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="teamkai",
    maintainer_email="teamkai@example.com",
    description="Rule-based autonomous driving node for the Kookmin Xycar final track.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "centerline_tracer = my_rule.centerline_tracer_node:main",
            "lane_node = my_rule.lane_node:main",
            "cone_node = my_rule.cone_node:main",
            "rule_driver = my_rule.rule_driver:main",
        ],
    },
)
