import os

from glob import glob
from setuptools import find_packages, setup

package_name = 'smart_traffic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'data'),
            glob('data/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='watan',
    maintainer_email='watan@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'csv_player = smart_traffic.csv_player_node:main',
            'occupancy_grid_node = smart_traffic.occupancy_grid_node:main',
            'occupancy_grid_viewer = smart_traffic.occupancy_grid_viewer:main',
            'collision_detector = smart_traffic.collision_detector:main',
            'yaw_monitor = smart_traffic.yaw_rate_monitor_node:main',
            'hard_brake_detector = smart_traffic.hard_brake_detector_node:main',
            'occupancy_grid_recorder = smart_traffic.occupancy_grid_recorder:main',
        ],
    },
)
