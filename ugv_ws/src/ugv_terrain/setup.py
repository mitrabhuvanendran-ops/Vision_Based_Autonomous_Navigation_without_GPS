import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'ugv_terrain'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
]

for d in ['models', 'materials']:
    for filepath in package_files(d):
        install_dir = os.path.join('share', package_name, os.path.dirname(filepath))
        data_files.append((install_dir, [filepath]))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='udhayan',
    maintainer_email='user@todo.todo',
    description='Terrain assets and worlds for outdoor UGV simulation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
