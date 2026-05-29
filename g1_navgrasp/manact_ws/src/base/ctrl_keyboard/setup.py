from setuptools import setup
import os
from glob import glob

package_name = 'ctrl_keyboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 自动安装所有启动文件（可选）
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='unitree@todo.todo',
    description='Unitree G1 Keyboard Control & Auto Walk',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 键盘控制节点
            'ctrl_keyboard = ctrl_keyboard.ctrl_keyboard:main',
            # 自动行走节点（你刚才的自动脚本）
            'auto_walk = ctrl_keyboard.auto_ctrl:main',
        ],
    },
)

