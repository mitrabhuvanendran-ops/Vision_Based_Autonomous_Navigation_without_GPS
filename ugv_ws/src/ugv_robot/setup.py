import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'ugv_robot'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
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
    description='Robot description and launch files',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
