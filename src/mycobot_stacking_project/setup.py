from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mycobot_stacking_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'models', 'yellow_cube'), glob(os.path.join('models', 'yellow_cube', '*.*'))),
        (os.path.join('share', package_name, 'models', 'orange_cube'), glob(os.path.join('models', 'orange_cube', '*.*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wangyukun',
    maintainer_email='wangyukun@todo.todo',
    description='A project to stack cubes using myCobot and MoveIt2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cube_detector = mycobot_stacking_project.cube_detector_node:main',
            'stacking_manager = mycobot_stacking_project.stacking_manager_node:main',
        ],
    },
)
