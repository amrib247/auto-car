from glob import glob
import os

from setuptools import find_packages, setup


package_name = "auto_car_sensors"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "models"), glob("models/*.pt")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="auto-car-follow",
    maintainer_email="maintainer@example.com",
    description="Wireless camera and YOLO object detections for the RC car.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "wireless_yolo_sensor = auto_car_sensors.sensor_node:main",
        ],
    },
)