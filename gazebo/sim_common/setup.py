"""Setup file for the sim_common package"""

from setuptools import setup  # type: ignore[import-untyped]

PACKAGE_NAME = "sim_common"

setup(
    name=PACKAGE_NAME,
    version="0.0.0",
    packages=[PACKAGE_NAME],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
    ],
    entry_points={
        "console_scripts": [
            "move_joints = sim_common.move_joints:main",
            "joint_gui = sim_common.joint_gui:main",
            "controller_gui = sim_common.controller_gui:main",
            "drivebase = sim_common.drivebase:main",
            "arm = sim_common.arm:main",
        ],
    },
    install_requires=["setuptools", 'evdev; sys_platform == "linux"'],
    zip_safe=True,
    maintainer="Trickfire Robotics",
    maintainer_email="tfrbtcs@uw.edu",
    description="Shared utilities for simulation launch files",
    license="TODO",
)
