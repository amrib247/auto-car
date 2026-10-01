from glob import glob
import os

from setuptools import find_packages, setup


package_name = "auto_car_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="auto-car-follow",
    maintainer_email="maintainer@example.com",
    description="PD person-following control and serial output for the RC car.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "person_follow_controller = auto_car_control.control_node:main",
            "wasd_controller = auto_car_control.wasd_node:main",
        ],
    },
)