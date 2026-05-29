from setuptools import find_packages, setup
from setuptools.command.develop import develop as _develop


class develop(_develop):
    user_options = _develop.user_options + [
        ('uninstall', None, ''),
        ('editable', None, ''),
        ('build-directory=', None, ''),
        ('script-dir=', None, ''),
    ]
    boolean_options = list(getattr(_develop, 'boolean_options', [])) + ['uninstall', 'editable']

    def initialize_options(self):
        super().initialize_options()
        self.uninstall = False
        self.editable = False
        self.build_directory = None
        self.script_dir = None

package_name = 'yolo_depth_pcd'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/launch',
            ['launch/process_frame_server.launch.py', 'launch/continuous_demo.launch.py', 'launch/realsense_live.launch.py'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='ROS2 node that runs YOLO detection + depth backprojection + pointcloud OBB query on offline dataset frames.',
    license='Apache-2.0',
    tests_require=['pytest'],
    cmdclass={
        'develop': develop,
    },
    entry_points={
        'console_scripts': [
            'process_frame_server = yolo_depth_pcd.process_frame_server:main',
            'process_frame_client = yolo_depth_pcd.process_frame_client:main',
            'live_pipeline_node = yolo_depth_pcd.live_pipeline_node:main',
        ],
    },
)
