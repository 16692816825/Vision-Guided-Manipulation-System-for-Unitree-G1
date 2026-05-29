from setuptools import setup
from glob import glob

package_name = "g1_driver_py"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/params", glob("params/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HunLi",
    maintainer_email="xhunli@qq.com",
    description="宇树 G1 人形机器人驱动包",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "g1_driver = g1_driver_py.driver:main",
        ],
    },
)
