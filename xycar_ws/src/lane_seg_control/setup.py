from glob import glob
from os.path import isfile
from setuptools import setup

package_name = "lane_seg_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/config", glob("config/*.json")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/scripts", glob("scripts/*.sh")),
        (
            "share/" + package_name + "/models",
            [path for path in glob("models/*") if isfile(path)],
        ),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="kai",
    maintainer_email="kai@example.com",
    description=(
        "LR-ASPP MobileNetV3 lane segmentation and canonical road perception"
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lane_seg_unified_viewer = lane_seg_control.unified_viewer_node:main",
            "lane_seg_inference_node = lane_seg_control.lane_seg_inference_node:main",
            (
                "lane_seg_lraspp_inference_node = "
                "lane_seg_control.lraspp_inference_node:main"
            ),
            (
                "export_lraspp_bag_montages = "
                "lane_seg_control.bag_montage_exporter:main"
            ),
            "lane_seg_canonical_adapter = lane_seg_control.canonical_adapter_node:main",
        ],
    },
)
