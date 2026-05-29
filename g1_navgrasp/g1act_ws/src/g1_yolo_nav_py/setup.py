import glob
from setuptools import find_packages, setup

package_name = "g1_yolo_nav_py"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/*.launch.py"]),
        ("share/" + package_name + "/config", ["config/*.yaml"]),
        ("share/" + package_name + "/models", glob.glob("*.pt")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="developer",
    maintainer_email="developer@example.com",
    description="宇树 G1 机器人 YOLO 目标识别与路径规划导航功能包",
    license="MIT",
    entry_points={
        "console_scripts": [
            "yolo_detector = g1_yolo_nav_py.yolo_detector:main",
            "spatial_target = g1_yolo_nav_py.spatial_target:main",
            "nav_planner = g1_yolo_nav_py.nav_planner:main",
        ],
    },
)
