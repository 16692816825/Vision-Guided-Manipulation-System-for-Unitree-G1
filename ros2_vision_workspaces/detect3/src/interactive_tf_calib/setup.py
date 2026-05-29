from setuptools import find_packages, setup

package_name = 'interactive_tf_calib'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/interactive_tf_calib.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='Interactive TF calibration tool: drag a 6DoF marker in RViz to align frames and publish TF, with save-to-JSON service.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interactive_tf_calib_node = interactive_tf_calib.interactive_tf_calib_node:main',
            'lidar_overlay_node = interactive_tf_calib.lidar_overlay_node:main',
        ],
    },
)
