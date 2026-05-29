from setuptools import find_packages, setup

package_name = 'realtime_yolo_pcd_tf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/realtime_yolo_pcd_tf.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='Realtime RGB+Depth YOLO detection with colored pointcloud, markers, and TF.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'realtime_node = realtime_yolo_pcd_tf.realtime_node:main',
        ],
    },
)
