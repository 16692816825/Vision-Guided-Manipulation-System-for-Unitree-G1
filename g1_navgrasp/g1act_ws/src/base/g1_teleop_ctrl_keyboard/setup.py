from setuptools import setup

package_name = "g1_teleop_ctrl_keyboard"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HunLi",
    maintainer_email="xhunli@qq.com",
    description="宇树 G1 键盘控制",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "g1_teleop_keyboard = g1_teleop_ctrl_keyboard.g1_teleop_ctrl_keyboard:main",
        ],
    },
)
