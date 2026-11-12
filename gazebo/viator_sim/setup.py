"""Setup file for the viator_sim package"""

from glob import glob

from setuptools import setup

PACKAGE_NAME = "viator_sim"

setup(
    name=PACKAGE_NAME,
    version="0.0.0",
    packages=[PACKAGE_NAME],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        ("share/" + PACKAGE_NAME + "/launch", glob("launch/*.launch.py")),
        ("share/" + PACKAGE_NAME + "/config", glob("config/*")),
    ],
    entry_points={
        "console_scripts": [
            "ublox_gps = viator_sim.ublox_gps:main",
            "zed_camera = viator_sim.zed_camera:main",
        ],
    },
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Trickfire Robotics",
    maintainer_email="tfrbtcs@uw.edu",
    description="Gazebo integration for Viator",
    license="Apache-2.0",
)
