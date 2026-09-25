import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'ugv_nav'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*')),
]

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='udhayan',
    maintainer_email='user@todo.todo',
    description='Perception, planning, and control nodes for the UGV',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bisenet_node = ugv_nav.bisenet_node:main',
            'costmap_builder = ugv_nav.costmap_builder:main',
            'goal_converter = ugv_nav.goal_converter:main',
            'rrt_planner = ugv_nav.rrt_planner:main',
            'path_follower = ugv_nav.path_follower:main',
            'safety_gate = ugv_nav.safety_gate:main',
        ],
    },
)
