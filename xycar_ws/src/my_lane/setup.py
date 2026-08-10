from glob import glob
from os.path import isfile
from setuptools import setup

package_name = "my_lane"

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
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
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
            "lane_seg_unified_viewer = my_lane.unified_viewer_node:main",
            "lane_seg_inference_node = my_lane.lane_seg_inference_node:main",
            (
                "lane_seg_lraspp_inference_node = "
                "my_lane.lraspp_inference_node:main"
            ),
            (
                "export_lraspp_bag_montages = "
                "my_lane.bag_montage_exporter:main"
            ),
            (
                "train_row_centerline = "
                "my_lane.train_row_centerline:main"
            ),
            (
                "train_far_centerline = "
                "my_lane.train_far_centerline:main"
            ),
            (
                "steering_compare_viewer = "
                "my_lane.steering_compare_viewer:main"
            ),
            (
                "camera_path_compare_viewer = "
                "my_lane.camera_path_compare_viewer:main"
            ),
            "lane_seg_canonical_adapter = my_lane.canonical_adapter_node:main",
            "bev_path_centerline = my_lane.bev_path_centerline_node:main",
            "compressed_rectifier = my_lane.compressed_rectifier_node:main",
        ],
    },
)
