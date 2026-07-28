from setuptools import find_packages, setup


package_name = "xycar_run_recorder"

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
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Team K.A.I.",
    maintainer_email="team.kai@example.com",
    description="Compact real-vehicle model-run recording and telemetry tools.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "canonical_compressor = "
            "xycar_run_recorder.canonical_compressor:main",
            "record_model_run = "
            "xycar_run_recorder.record_model_run:main",
            "vehicle_telemetry_bridge = "
            "xycar_run_recorder.vehicle_telemetry_bridge:main",
        ],
    },
)
