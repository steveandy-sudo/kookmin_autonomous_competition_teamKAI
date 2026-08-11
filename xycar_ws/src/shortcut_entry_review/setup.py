from glob import glob

from setuptools import setup


package_name = "shortcut_entry_review"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kai",
    maintainer_email="kai@example.com",
    description="Sequence-aware shortcut-entry review and hybrid candidates",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "analyze_shortcut_trigger = "
            "shortcut_entry_review.analyze_shortcut_trigger:main",
            "evaluate_white_yellow_entry = "
            "shortcut_entry_review.evaluate_white_yellow_entry:main",
            "compressed_image_gate = "
            "shortcut_entry_review.compressed_image_gate_node:main",
            "white_yellow_entry_review = "
            "shortcut_entry_review.white_yellow_entry_review_node:main",
            "sequence_entry = "
            "shortcut_entry_review.sequence_entry_node:main",
            "shortcut_candidate_mux = "
            "shortcut_entry_review.shortcut_candidate_mux_node:main",
            "evaluate_sequence_annotations = "
            "shortcut_entry_review.evaluate_sequence_annotations:main",
            "left4_processing_gate = "
            "shortcut_entry_review.left4_processing_gate_node:main",
            "bev_line_annotation = "
            "shortcut_entry_review.bev_line_annotation_node:main",
            "annotation_keyboard = "
            "shortcut_entry_review.annotation_keyboard:main",
            "bag_keyboard_controller = "
            "shortcut_entry_review.bag_keyboard_controller:main",
        ],
    },
)
