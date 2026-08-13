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
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kai",
    maintainer_email="kai@example.com",
    description="Sequence-aware W1 shortcut entry and hybrid candidate",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "compressed_image_gate = "
            "shortcut_entry_review.compressed_image_gate_node:main",
            "sequence_entry = shortcut_entry_review.sequence_entry_node:main",
            "shortcut_candidate_mux = "
            "shortcut_entry_review.shortcut_candidate_mux_node:main",
        ],
    },
)
