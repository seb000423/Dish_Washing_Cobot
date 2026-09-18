from glob import glob
from setuptools import find_packages, setup

package_name = "ui_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="seb000423",
    maintainer_email="seb000423@gmail.com",
    description="ROS2 support nodes and separately executed Flask HMI for the dishwasher robot.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "gripper_safety = ui_ros.gripper_safety:main",
            "force_gripper_monitor = ui_ros.force_gripper_monitor:main",
            "home_gripper_control = ui_ros.home_gripper_control:main",
        ],
    },
)
